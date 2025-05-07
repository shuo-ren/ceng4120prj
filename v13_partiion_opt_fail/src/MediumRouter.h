// MediumRouter.h
#pragma once

#include <string>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <map>
#include <queue>
#include <cmath>
#include <limits>
#include <cassert>
#include <algorithm>
#include <chrono>
#include <omp.h>
#include "NetSorter.h"  // Make sure this is included after dataloader.h in your main.cpp

// Define Edge type - pair of node IDs
typedef std::pair<int, int> Edge;

// Define a hash function for Edge to use it in unordered_set/map
struct EdgeHash {
    std::size_t operator()(const Edge& e) const {
        return std::hash<int>{}(e.first) ^ (std::hash<int>{}(e.second) << 1);
    }
};

// Define Usage type - Map from net_id to usage count
typedef std::map<int, int> NetUsageCount;

// Define overall answer structure: Map from Edge to its usage by different nets
typedef std::map<Edge, NetUsageCount> EdgeUsageSolution;

class MediumRouter {
public:
    // Constructor to match the parameters being passed from your factory
    MediumRouter(const NodeMap& initial_nodes, 
                const AdjacencyList& initial_adj,
                const NetList& initial_nets,
                const std::string& out_filename,
                bool debug_flag,
                int num_threads,
                double read_time_sec,
                double time_limit_sec)
        : nodes(initial_nodes),
          adj(initial_adj),
          nets(initial_nets), // Keeps nets as const
          output_filename(out_filename),
          lambda(1.5),  // Default lambda factor for edge reuse penalty
          time_limit_seconds(time_limit_sec),
          start_time(std::chrono::high_resolution_clock::now()),
          debug(debug_flag),
          num_threads(num_threads),
          read_time(read_time_sec),
          time_limit_reached(false) 
    {
        // Initialize node map for quick lookups
        for (const auto& node : nodes) {
            node_map[node.id] = &node;
        }
        
        if (debug) {
            std::cout << "MediumRouter initialized with " << nodes.size() << " nodes and " 
                      << nets.size() << " nets." << std::endl;
            std::cout << "Output file: " << (output_filename.empty() ? "None" : output_filename) << std::endl;
            std::cout << "Lambda factor for edge reuse penalty: " << lambda << std::endl;
            std::cout << "Time limit set to: " << time_limit_seconds << " seconds" << std::endl;
        }
    }

    // Main routing function to be called from outside
    void route() {
        if (debug) {
            std::cout << "Starting routing process..." << std::endl;
        }
        
        global_edge_usage.clear();
        global_node_usage.clear();
        answer.clear();
        
        start_time = std::chrono::high_resolution_clock::now();

        // Create a mutable copy of nets for sorting
        std::vector<Net> mutable_nets = nets;
        
        // Sort nets by bounding box size
        auto start_sort_time = std::chrono::high_resolution_clock::now();
        NetSorter netSorter(nodes, mutable_nets);
        netSorter.sort_nets();
        
        auto end_sort_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> sort_time = end_sort_time - start_sort_time;
        
        if (debug) {
            std::cout << "Nets sorted by bounding box size." << std::endl;
            std::cout << "Sorting time: " << sort_time.count() << " seconds." << std::endl;
        }

        // Set all source and sink usage
        for (size_t i = 0; i < mutable_nets.size(); i++) {
            int source = mutable_nets[i].source;
            global_node_usage[source] = i; // Mark source node as used by this net
            for (int sink : mutable_nets[i].sinks) {
                if (sink > 0) { // Assuming valid sink IDs are positive
                    global_node_usage[sink] = i; // Mark sink node as used by this net
                }
            }
        }

        // Route each net
        for (size_t i = 0; i < mutable_nets.size(); ++i) {
            // Check time limit before processing each net
            if (check_time_limit()) {
                if (debug) {
                    std::cout << "Time limit of " << time_limit_seconds << " seconds reached. Stopping search." << std::endl;
                }
                break;
            }
            route_net(i, mutable_nets);
        }

        // Option to post-process (can be enabled as needed)
        // post_process_routes();
        
        if (debug) {
            std::cout << "Routing process finished." << std::endl;
            if (time_limit_reached) {
                std::cout << "Note: Search was terminated after reaching the " << time_limit_seconds << " second time limit." << std::endl;
            }
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_time = end_time - start_time;
        
        if (debug) {
            std::cout << "=======================================" << std::endl;
            std::cout << "Total elapsed time: " << elapsed_time.count() << " seconds." << std::endl;
            std::cout << "Total edges used: " << answer.size() << std::endl;
            std::cout << "Total nets routed: " << mutable_nets.size() << std::endl;
            std::cout << "Total nodes used: " << global_node_usage.size() << std::endl;
            std::cout << "=======================================" << std::endl;
        }

        write_results_to_file(mutable_nets);
    }

private:
    // Represents a state in the priority queue for A*
    struct AStarNode {
        int node_id;
        double f_cost; // g_cost + h_cost

        // Overload operator> for min-priority queue
        bool operator>(const AStarNode& other) const {
            return f_cost > other.f_cost;
        }
    };

    // --- Core Routing Logic ---
    // Route all sinks for a single net
    void route_net(size_t net_id, const std::vector<Net>& mutable_nets) {
        const Net& current_net = mutable_nets[net_id];
        int source_node_id = current_net.source;
        
        if (debug) {
            std::cout << "  Routing Net " << net_id << " ('" << current_net.name << "') from Source " << source_node_id << std::endl;
        }

        // Track edges used by this specific net so far (for lambda penalty)
        std::unordered_set<Edge, EdgeHash> current_net_edges_used;

        // Iterate through all sinks of the current net
        for (int sink_node_id : current_net.sinks) {
            // Check time limit before processing each sink
            if (check_time_limit()) {
                break;
            }
            
            // Check for valid sink ID
            if (sink_node_id <= 0) {
                break; // Stop processing sinks for this net
            }

            if (debug) {
                std::cout << "    Attempting to route Source " << source_node_id << " -> Sink " << sink_node_id << std::endl;
            }

            // Find the path using A*
            std::vector<int> path = find_path(source_node_id, sink_node_id, net_id, current_net_edges_used);

            if (!path.empty()) {
                if (debug) {
                    std::cout << "      Path found (length " << path.size() << ")" << std::endl;
                }
                // Update global usage and the local usage set for this net
                update_usage(path, net_id, current_net_edges_used);
            } else {
                std::cerr << "      ERROR: Failed to find a path for Net " << net_id << " from " << source_node_id << " to " << sink_node_id << std::endl;
            }
        }
    }

    // A* algorithm implementation for a single source-sink pair
    std::vector<int> find_path(int start_node_id, int end_node_id, size_t net_id, const std::unordered_set<Edge, EdgeHash>& current_net_edges_used) {
        // Priority queue storing {f_cost, node_id}
        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;

        // Map node_id -> cost from start node (g_cost)
        std::unordered_map<int, double> g_cost;
        // Map node_id -> predecessor node_id in the path
        std::unordered_map<int, int> parent;

        // Initialize costs
        g_cost[start_node_id] = 0.0;
        parent[start_node_id] = -1; // Indicate start node has no parent

        // Calculate initial h_cost and push start node to open set
        double h_start = heuristic(start_node_id, end_node_id);
        if (h_start == std::numeric_limits<double>::max()) {
            if (debug) {
                std::cerr << "      ERROR: Cannot calculate heuristic for start/end node. Invalid node ID?" << std::endl;
            }
            return {}; // Return empty path if nodes are invalid
        }
        open_set.push({start_node_id, h_start}); // f_cost = g_cost (0) + h_cost

        std::unordered_set<int> closed_set; // Keep track of visited nodes

        int check_counter = 0; // Counter for periodic time checks

        while (!open_set.empty()) {
            // Periodic time check to avoid excessive overhead
            if (++check_counter >= 1000) { // Check every 1000 iterations
                check_counter = 0;
                if (check_time_limit()) {
                    return {}; // Return empty path if time limit reached
                }
            }
            
            AStarNode current_node_info = open_set.top();
            open_set.pop();
            int current_node_id = current_node_info.node_id;

            // Skip if already processed
            if (closed_set.count(current_node_id)) {
                continue;
            }
            closed_set.insert(current_node_id);

            // Goal check
            if (current_node_id == end_node_id) {
                return reconstruct_path(end_node_id, parent);
            }

            // Explore neighbors using the adjacency list
            if (static_cast<size_t>(current_node_id) >= adj.size()) continue;

            const std::vector<int>& neighbors = adj[current_node_id];
            for (int neighbor_id : neighbors) {
                if (neighbor_id <= 0) {
                    continue; // Skip invalid neighbor
                }

                // Check if neighbor exists
                if (node_map.find(neighbor_id) == node_map.end()) {
                    continue; // Skip invalid neighbor
                }
                
                // Check if already processed
                if (closed_set.count(neighbor_id)) {
                    continue;
                }

                // Calculate cost of traversing the edge
                double edge_cost = get_edge_cost(current_node_id, neighbor_id, net_id, current_net_edges_used);

                // Check if edge is blocked
                if (edge_cost == std::numeric_limits<double>::max()) {
                    continue; // Cannot use this edge
                }
                
                // Calculate tentative g_cost for the neighbor
                double tentative_g_cost = g_cost[current_node_id] + edge_cost;

                // If this path to neighbor is better than any previous one found
                bool neighbor_in_g_cost = g_cost.count(neighbor_id);
                if (!neighbor_in_g_cost || tentative_g_cost < g_cost[neighbor_id]) {
                    // Update path and costs
                    parent[neighbor_id] = current_node_id;
                    g_cost[neighbor_id] = tentative_g_cost;

                    // Calculate f_cost and push to priority queue
                    double h_neighbor = heuristic(neighbor_id, end_node_id);
                    if (h_neighbor == std::numeric_limits<double>::max()) continue;
                    
                    open_set.push({neighbor_id, tentative_g_cost + h_neighbor});
                }
            }
        }

        // If the loop finishes without finding the end node, no path exists
        return {}; // Return empty vector
    }

    // Reconstruct the path from end node using parent map
    std::vector<int> reconstruct_path(int end_node_id, const std::unordered_map<int, int>& parent) {
        std::vector<int> path;
        int current = end_node_id;
        while (current != -1) { // While we haven't reached the start node's parent (-1)
            // Check if current node exists in parent map
            if (path.empty() || parent.count(current)) {
                path.push_back(current);
                current = parent.at(current); // Get the parent
            } else {
                if (debug) {
                    std::cerr << "      ERROR: Path reconstruction failed. Node " << current << " not found in parent map." << std::endl;
                }
                return {}; // Return empty path on error
            }
            // Safety break - avoid infinite loop
            if (path.size() > nodes.size() * 2) {
                if (debug) {
                    std::cerr << "      ERROR: Path reconstruction exceeded max length. Possible cycle?" << std::endl;
                }
                return {};
            }
        }
        std::reverse(path.begin(), path.end()); // Reverse to get path from start to end
        return path;
    }

    // Calculate heuristic cost (Manhattan distance between node centers)
    double heuristic(int node_id, int target_node_id) {
        const Node* current = get_node_by_id(node_id);
        const Node* target = get_node_by_id(target_node_id);

        if (!current || !target) {
            return std::numeric_limits<double>::max();
        }

        // Calculate center points
        double cx1 = (current->begin_x + current->end_x) / 2.0;
        double cy1 = (current->begin_y + current->end_y) / 2.0;
        double cx2 = (target->begin_x + target->end_x) / 2.0;
        double cy2 = (target->begin_y + target->end_y) / 2.0;

        // Manhattan distance
        return std::abs(cx1 - cx2) + std::abs(cy1 - cy2);
    }
    
    // Calculate base distance (Euclidean distance between node centers)
    double get_base_distance(int u_id, int v_id) {
        const Node* u = get_node_by_id(u_id);
        const Node* v = get_node_by_id(v_id);

        if (!u || !v) {
            return std::numeric_limits<double>::max();
        }

        double cx1 = (u->begin_x + u->end_x) / 2.0;
        double cy1 = (u->begin_y + u->end_y) / 2.0;
        double cx2 = (v->begin_x + v->end_x) / 2.0;
        double cy2 = (v->begin_y + v->end_y) / 2.0;
        // Euclidean distance
        return std::sqrt(std::pow(cx1 - cx2, 2) + std::pow(cy1 - cy2, 2));
    }

    // Calculate the actual cost of traversing an edge (u -> v) for a given net
    double get_edge_cost(int u_id, int v_id, size_t net_id, const std::unordered_set<Edge, EdgeHash>& current_net_edges_used) {
        Edge edge = {u_id, v_id}; // Directed edge

        // Check if globally used by another net
        auto global_it = global_edge_usage.find(edge);
        if (global_it != global_edge_usage.end() && global_it->second != net_id) {
            return std::numeric_limits<double>::max();
        }

        // Check if node globally used by another net
        auto global_node_it = global_node_usage.find(v_id);
        if (global_node_it != global_node_usage.end() && global_node_it->second != net_id) {
            return std::numeric_limits<double>::max();
        }

        // Calculate base distance for this edge
        double base_dist = get_base_distance(u_id, v_id);
        if (base_dist == std::numeric_limits<double>::max()) {
            return std::numeric_limits<double>::max();
        }

        // Check if already used by the current net
        if (current_net_edges_used.count(edge)) {
            return base_dist; // Encourage reuse
        } else {
            return base_dist * lambda; // Apply lambda penalty for new edges
        }
    }

    // Get node pointer by ID
    const Node* get_node_by_id(int node_id) const {
        auto it = node_map.find(node_id);
        if (it != node_map.end()) {
            return it->second;
        }
        return nullptr; // Node not found
    }

    // Update global usage map and the tracking set for the current net after finding a path
    void update_usage(const std::vector<int>& path, size_t net_id, std::unordered_set<Edge, EdgeHash>& current_net_edges_used) {
        if (path.size() < 2) return;

        for (size_t i = 0; i < path.size() - 1; ++i) {
            int u_id = path[i];
            int v_id = path[i + 1];
            Edge current_edge = {u_id, v_id};

            // Add to the set tracking edges used by this net
            current_net_edges_used.insert(current_edge);

            // Update global usage map
            if (global_edge_usage.find(current_edge) == global_edge_usage.end()) {
                global_edge_usage[current_edge] = net_id;
                global_node_usage[current_edge.first] = net_id;
                global_node_usage[current_edge.second] = net_id;
            } else {
                // This edge was already used by a previous sink of this same net
                assert(global_edge_usage[current_edge] == net_id);
            }

            // Update the final answer structure
            answer[current_edge][net_id]++; // Increment usage count for this net on this edge
        }
    }

    // Check if time limit has been reached
    bool check_time_limit() {
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = current_time - start_time;
        if (elapsed.count() >= time_limit_seconds) {
            time_limit_reached = true;
            return true;
        }
        return false;
    }

    // Write results to file
    void write_results_to_file(const std::vector<Net>& mutable_nets) {
        auto start_write_time = std::chrono::high_resolution_clock::now();
        
        // Check if a filename was provided
        if (output_filename.empty()) {
            if (debug) {
                std::cout << "No output filename specified. Skipping file writing." << std::endl;
            }
            return;
        }
        
        if (debug) {
            std::cout << "Writing results to file: " << output_filename << std::endl;
        }
        
        // Pre-process: Create a map of net_id to its edges
        std::vector<std::vector<Edge>> net_to_edges(mutable_nets.size());
        
        for (const auto& edge_usage_pair : answer) {
            const Edge& edge = edge_usage_pair.first;
            const NetUsageCount& usage_map = edge_usage_pair.second;
            
            for (const auto& usage : usage_map) {
                size_t net_id = usage.first;
                if (net_id < mutable_nets.size()) {
                    net_to_edges[net_id].push_back(edge);
                }
            }
        }
        
        // Parallelize writing content
        std::vector<std::string> net_contents(mutable_nets.size());
        #pragma omp parallel for schedule(dynamic) num_threads(num_threads)
        for (size_t net_id = 0; net_id < mutable_nets.size(); ++net_id) {
            std::ostringstream net_stream;
            const Net& current_net = mutable_nets[net_id];
            
            // Write net header
            net_stream << current_net.id << " " << current_net.name << "\n";
            
            // Sort edges and write them
            std::vector<Edge>& edges = net_to_edges[net_id];
            std::sort(edges.begin(), edges.end());
            
            for (const auto& edge : edges) {
                net_stream << edge.first << " " << edge.second << "\n";
            }
            
            net_contents[net_id] = net_stream.str();
        }
        
        // Concatenate all content with newlines
        std::string all_content;
        size_t total_size = 0;
        
        // Calculate total size
        for (size_t net_id = 0; net_id < mutable_nets.size(); ++net_id) {
            total_size += net_contents[net_id].size();
            if (net_id < mutable_nets.size() - 1) {
                total_size += 1; // for newline
            }
        }
        
        // Reserve space to avoid reallocations
        all_content.reserve(total_size);
        
        // Concatenate all content
        for (size_t net_id = 0; net_id < mutable_nets.size(); ++net_id) {
            all_content.append(net_contents[net_id]);
            if (net_id < mutable_nets.size() - 1) {
                all_content.append("\n");
            }
        }
        
        // Use fwrite for faster I/O
        FILE* file = fopen(output_filename.c_str(), "wb");
        if (!file) {
            std::cerr << "Error: Could not open output file: " << output_filename << std::endl;
            return;
        }
        
        // Use a large buffer for FILE I/O
        const size_t BUFFER_SIZE = 8 * 1024 * 1024; // 8MB buffer
        char* buffer = new char[BUFFER_SIZE];
        setvbuf(file, buffer, _IOFBF, BUFFER_SIZE);
        
        // Write all content at once
        size_t bytes_written = fwrite(all_content.data(), 1, all_content.size(), file);
        
        if (bytes_written != all_content.size()) {
            std::cerr << "Warning: Not all data was written to the file." << std::endl;
        }
        
        fclose(file);
        delete[] buffer;
        
        if (debug) {
            std::cout << "Finished writing results." << std::endl;
            auto end_write_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> write_time = end_write_time - start_write_time;
            std::cout << "File writing time: " << write_time.count() << " seconds." << std::endl;
        }
    }

    // Member variables
    const NodeMap& nodes;
    const AdjacencyList& adj;
    const NetList& nets;  // Original nets (kept as const)
    std::string output_filename;
    double lambda;  // Penalty factor for using new edges
    double time_limit_seconds;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_time;
    bool debug;
    int num_threads;
    double read_time;
    bool time_limit_reached;

    // Map for efficient node lookup by ID
    std::unordered_map<int, const Node*> node_map;

    // Tracks edge and node usage by nets
    std::map<Edge, size_t> global_edge_usage;
    std::map<int, size_t> global_node_usage;

    // Stores the final routing solution
    EdgeUsageSolution answer;
};