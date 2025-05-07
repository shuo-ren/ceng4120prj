// NetSorter.h
#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <unordered_map>

// Forward declaration - uses the Node and Net types from dataloader.h
// Do NOT redefine these structures here
typedef std::vector<Node> NodeMap;
typedef std::vector<Net> NetList;

class NetSorter {
public:
    // Configuration for Congestion Estimation
    const int GRID_RESOLUTION = 50;

    // Member Variables
    const NodeMap& nodes;
    NetList& nets;
    std::vector<std::vector<int>> congestion_map;
    int map_min_x = 0;
    int map_min_y = 0;
    int grid_dim_x = 0;
    int grid_dim_y = 0;
    
    // Store bounding box values separately
    std::unordered_map<int, double> net_bounding_boxes;

    struct NetScores {
        double congestion_score = 0.0;
        double bounding_box = 0.0;
        size_t fanout = 0;
        int original_index = -1;
    };
    std::vector<NetScores> net_scores;

    // Constructor
    NetSorter(const NodeMap& nodes, NetList& nets)
        : nodes(nodes), nets(nets)
    {
        if (this->nodes.empty() || this->nets.empty()) {
            std::cerr << "Warning: Node list or Net list is empty. Sorting will not be performed." << std::endl;
            return;
        }
        
        // Calculate bounding boxes for all nets
        for (size_t i = 0; i < this->nets.size(); ++i) {
            // Initialize with zero
            net_bounding_boxes[nets[i].id] = 0.0;
            // Calculate bounding box
            calculate_net_bounding_box(i);
        }
        
        estimate_congestion();
        calculate_net_scores();
    }

    // Calculate bounding box for a single net
    void calculate_net_bounding_box(size_t net_idx) {
        const Net& net = nets[net_idx];
        
        // Default zero value in case of issues
        net_bounding_boxes[net.id] = 0.0;
        
        if (nodes.empty() || net.source < 0 || static_cast<size_t>(net.source) >= nodes.size()) {
            return;
        }
        
        int bound_xMin = nodes[net.source].begin_x;
        int bound_xMax = nodes[net.source].end_x;
        int bound_yMin = nodes[net.source].begin_y;
        int bound_yMax = nodes[net.source].end_y;
        
        for (int sink_id : net.sinks) {
            if (sink_id < 0 || static_cast<size_t>(sink_id) >= nodes.size()) continue;
            
            if (nodes[sink_id].begin_x < bound_xMin) {
                bound_xMin = nodes[sink_id].begin_x;
            }
            if (nodes[sink_id].end_x > bound_xMax) {
                bound_xMax = nodes[sink_id].end_x;
            }
            if (nodes[sink_id].begin_y < bound_yMin) {
                bound_yMin = nodes[sink_id].begin_y;
            }
            if (nodes[sink_id].end_y > bound_yMax) {
                bound_yMax = nodes[sink_id].end_y;
            }
        }
        
        // Store the bounding box area in our map
        net_bounding_boxes[net.id] = static_cast<double>((bound_xMax - bound_xMin) * (bound_yMax - bound_yMin));
    }

    // Helper to get Net Bounding Box Coordinates
    void get_net_bounding_box_coords(const Net& net, int& min_x, int& max_x, int& min_y, int& max_y) const {
        if (net.source < 0 || static_cast<size_t>(net.source) >= nodes.size()) {
            // Handle invalid source index
            min_x = max_x = min_y = max_y = 0;
            std::cerr << "Error: Invalid source node index " << net.source << " for net " << net.id << std::endl;
            return;
        }
        
        const Node& source_node = nodes[net.source];
        min_x = std::min(source_node.begin_x, source_node.end_x);
        max_x = std::max(source_node.begin_x, source_node.end_x);
        min_y = std::min(source_node.begin_y, source_node.end_y);
        max_y = std::max(source_node.begin_y, source_node.end_y);

        for (int sink_id : net.sinks) {
            if (sink_id <= 0 || static_cast<size_t>(sink_id) >= nodes.size()) {
                continue; // Skip invalid sink
            }
            
            const Node& sink_node = nodes[sink_id];
            min_x = std::min(min_x, std::min(sink_node.begin_x, sink_node.end_x));
            max_x = std::max(max_x, std::max(sink_node.begin_x, sink_node.end_x));
            min_y = std::min(min_y, std::min(sink_node.begin_y, sink_node.end_y));
            max_y = std::max(max_y, std::max(sink_node.begin_y, sink_node.end_y));
        }
    }

    // Calculate Scores for Sorting
    void calculate_net_scores() {
        net_scores.resize(nets.size());
        for (size_t i = 0; i < nets.size(); ++i) {
            const Net& net = nets[i];
            net_scores[i].original_index = i;
            
            // Ensure bounding box is valid
            double bb = net_bounding_boxes[net.id];
            if (bb <= 0 && !net.sinks.empty()) {
                calculate_net_bounding_box(i);
            } else if (net.sinks.empty() && bb != 0) {
                calculate_net_bounding_box(i);
            }
            
            if (net.sinks.empty()) {
                // Handle nets with no sinks
                net_scores[i].bounding_box = net_bounding_boxes[net.id];
                net_scores[i].fanout = 0;
                net_scores[i].congestion_score = 0;
            } else {
                net_scores[i].bounding_box = net_bounding_boxes[net.id];
                net_scores[i].fanout = net.sinks.size();
                net_scores[i].congestion_score = calculate_single_net_congestion(net);
            }
        }
    }

    // Calculate Congestion Score for a Single Net
    double calculate_single_net_congestion(const Net& net) const {
        if (congestion_map.empty() || grid_dim_x <= 0 || grid_dim_y <= 0 || net.sinks.empty()) {
            return 0.0;
        }

        int net_min_x, net_max_x, net_min_y, net_max_y;
        get_net_bounding_box_coords(net, net_min_x, net_max_x, net_min_y, net_max_y);

        int start_gx = std::max(0, (net_min_x - map_min_x) / GRID_RESOLUTION);
        int end_gx = std::min(grid_dim_x - 1, (net_max_x - map_min_x) / GRID_RESOLUTION);
        int start_gy = std::max(0, (net_min_y - map_min_y) / GRID_RESOLUTION);
        int end_gy = std::min(grid_dim_y - 1, (net_max_y - map_min_y) / GRID_RESOLUTION);
        
        double total_congestion = 0;
        int cell_count = 0;
        for (int gy = start_gy; gy <= end_gy; ++gy) {
            for (int gx = start_gx; gx <= end_gx; ++gx) {
                if (gy >= 0 && gy < grid_dim_y && gx >= 0 && gx < grid_dim_x) {
                    // Subtract 1 because this net itself contributes 1 to its BB cells
                    total_congestion += std::max(0, congestion_map[gy][gx] - 1);
                    cell_count++;
                }
            }
        }
        return (cell_count > 0) ? (total_congestion / cell_count) : 0.0;
    }

    // Congestion Estimation
    void estimate_congestion() {
        // 1. Find overall boundaries
        map_min_x = std::numeric_limits<int>::max();
        int map_max_x = std::numeric_limits<int>::min();
        map_min_y = std::numeric_limits<int>::max();
        int map_max_y = std::numeric_limits<int>::min();

        for (const auto& node : nodes) {
            map_min_x = std::min({map_min_x, node.begin_x, node.end_x});
            map_max_x = std::max({map_max_x, node.begin_x, node.end_x});
            map_min_y = std::min({map_min_y, node.begin_y, node.end_y});
            map_max_y = std::max({map_max_y, node.begin_y, node.end_y});
        }
        
        if (map_min_x >= map_max_x) map_max_x = map_min_x + GRID_RESOLUTION;
        if (map_min_y >= map_max_y) map_max_y = map_min_y + GRID_RESOLUTION;

        // 2. Determine grid dimensions
        grid_dim_x = std::max(1, (map_max_x - map_min_x + GRID_RESOLUTION - 1) / GRID_RESOLUTION);
        grid_dim_y = std::max(1, (map_max_y - map_min_y + GRID_RESOLUTION - 1) / GRID_RESOLUTION);

        // 3. Initialize congestion map
        congestion_map.assign(grid_dim_y, std::vector<int>(grid_dim_x, 0));

        // 4. Add contribution from each net's bounding box
        for (const auto& net : nets) {
            int net_min_x, net_max_x, net_min_y, net_max_y;
            get_net_bounding_box_coords(net, net_min_x, net_max_x, net_min_y, net_max_y);

            int start_gx = std::max(0, (net_min_x - map_min_x) / GRID_RESOLUTION);
            int end_gx = std::min(grid_dim_x - 1, (net_max_x - map_min_x) / GRID_RESOLUTION);
            int start_gy = std::max(0, (net_min_y - map_min_y) / GRID_RESOLUTION);
            int end_gy = std::min(grid_dim_y - 1, (net_max_y - map_min_y) / GRID_RESOLUTION);

            for (int gy = start_gy; gy <= end_gy; ++gy) {
                for (int gx = start_gx; gx <= end_gx; ++gx) {
                    if (gy >= 0 && gy < grid_dim_y && gx >= 0 && gx < grid_dim_x) {
                        congestion_map[gy][gx]++;
                    }
                }
            }
        }
    }

    // Main Sorting Function
    void sort_nets() {
        if (nets.empty() || net_scores.size() != nets.size()) {
            if (!nets.empty()) {
                std::cerr << "Error: Net scores not calculated or size mismatch. Cannot sort." << std::endl;
            }
            return;
        }

        // Create a vector of indices to sort
        std::vector<int> indices(nets.size());
        for (size_t i = 0; i < nets.size(); ++i) indices[i] = static_cast<int>(i);

        // Sort indices based on criteria (prioritize simpler nets)
        std::sort(indices.begin(), indices.end(),
            [this](int idx_a, int idx_b) {
                const NetScores& score_a = net_scores[idx_a];
                const NetScores& score_b = net_scores[idx_b];

                const double tolerance = 1e-9; // For floating point comparisons

                // 1. Primary Sort Criterion: Bounding Box Area (Ascending)
                if (std::abs(score_a.bounding_box - score_b.bounding_box) > tolerance) {
                    return score_a.bounding_box < score_b.bounding_box; // Smaller BB first
                }

                // 2. Secondary Sort Criterion: Fanout (Number of Sinks) (Ascending)
                if (score_a.fanout != score_b.fanout) {
                    return score_a.fanout < score_b.fanout; // Lower fanout first
                }

                // 3. Tertiary Sort Criterion: Congestion Score (Ascending)
                if (std::abs(score_a.congestion_score - score_b.congestion_score) > tolerance) {
                    return score_a.congestion_score < score_b.congestion_score; // Lower congestion first
                }

                // 4. Final Tie-breaker: Net ID (Ascending) for stability
                return nets[idx_a].id < nets[idx_b].id; // Lower ID first
            });

        // Reorder the original nets vector based on the sorted indices
        std::vector<Net> sorted_nets;
        sorted_nets.reserve(nets.size());
        for (int index : indices) {
            if (index >= 0 && static_cast<size_t>(index) < nets.size()) {
                sorted_nets.push_back(std::move(nets[index]));
            } else {
                std::cerr << "Error: Invalid index " << index << " during net reordering." << std::endl;
            }
        }
        
        // Check if sorted_nets size matches original nets size before assignment
        if (sorted_nets.size() == nets.size()) {
            nets = std::move(sorted_nets); // Assign the sorted vector back
        } else {
            std::cerr << "Error: Size mismatch after sorting. Original: " << nets.size() 
                      << ", Sorted: " << sorted_nets.size() << ". Aborting reorder." << std::endl;
        }
    }
};