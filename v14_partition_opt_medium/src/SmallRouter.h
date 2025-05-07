// SmallRouter.h
#pragma once
#include <vector>
#include <string>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <thread>
#include <mutex>
#include <random>

class SmallRouter
{
public:
    SmallRouter(const NodeMap &nodes, const AdjacencyList &adj,
                const NetList &nets, const std::string &outFile,
                bool debug, int threads, double readTime, double timeLimit)
        : nodes(nodes), adj(adj), nets(nets), outFile(outFile),
          debug(debug), threads(threads), readTime(readTime), timeLimit(timeLimit) {}

    void route()
    {
        auto startTime = std::chrono::high_resolution_clock::now();

        // Stores the routing result for each net
        std::unordered_map<int, std::vector<std::pair<int, int>>> result;

        // Global congestion map (node -> net_id)
        std::unordered_map<int, int> congestionMap;

        if (debug)
        {
            std::cout << "Starting SmallRouter for " << nets.size() << " nets\n";
        }

        // Sort nets by complexity (more sinks first)
        std::vector<size_t> netIndices(nets.size());
        for (size_t i = 0; i < nets.size(); ++i)
        {
            netIndices[i] = i;
        }

        std::sort(netIndices.begin(), netIndices.end(), [&](size_t a, size_t b)
                  { return nets[a].sinks.size() > nets[b].sinks.size(); });

        // Initial routing phase
        int totalSuccessful = 0;
        int totalWirelength = 0;

        // First pass: route all nets
        for (size_t idx : netIndices)
        {
            const Net &net = nets[idx];

            if (debug)
            {
                std::cout << "Routing net " << net.id << " with " << net.sinks.size() << " sinks\n";
            }

            std::vector<std::pair<int, int>> connections;
            std::unordered_set<int> netNodes;
            netNodes.insert(net.source);

            bool allSinksRouted = true;
            int netWirelength = 0;

            // Sort sinks by distance (furthest first)
            std::vector<std::pair<int, int>> sortedSinks;
            for (int sink : net.sinks)
            {
                int distance = calculateManhattanDistance(net.source, sink);
                sortedSinks.push_back(std::make_pair(distance, sink));
            }
            std::sort(sortedSinks.begin(), sortedSinks.end(), std::greater<>());

            // Route each sink
            for (const auto &p : sortedSinks)
            {
                int sink = p.second;

                // Find the best starting point
                int startNode = net.source;
                int minDistance = calculateManhattanDistance(startNode, sink);

                for (int node : netNodes)
                {
                    int distance = calculateManhattanDistance(node, sink);
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        startNode = node;
                    }
                }

                // Try to find a path
                std::vector<int> path = findPathSimple(startNode, sink, congestionMap, net.id, netNodes);

                if (path.empty())
                {
                    // Try from source as fallback
                    if (startNode != net.source)
                    {
                        path = findPathSimple(net.source, sink, congestionMap, net.id, netNodes);
                    }

                    if (path.empty())
                    {
                        // Still no path, try more aggressive search
                        path = findPathBFS(net.source, sink, congestionMap, net.id, netNodes);

                        if (path.empty())
                        {
                            allSinksRouted = false;
                            if (debug)
                            {
                                std::cout << "Failed to route sink " << sink << " for net " << net.id << "\n";
                            }
                            break;
                        }
                    }
                }

                // Add path to connections
                for (size_t i = 1; i < path.size(); ++i)
                {
                    connections.push_back(std::make_pair(path[i - 1], path[i]));
                    netNodes.insert(path[i]);
                    netWirelength += nodes[path[i]].length;
                }
            }

            if (allSinksRouted)
            {
                result[net.id] = connections;
                totalSuccessful++;
                totalWirelength += netWirelength;

                // Update congestion map
                for (int node : netNodes)
                {
                    congestionMap[node] = net.id;
                }
            }
            else
            {
                // Rip-up any partial routing for this net
                netNodes.clear();
                connections.clear();
            }
        }

        // If we didn't route all nets, try rip-up and reroute
        if (totalSuccessful < static_cast<int>(nets.size()))
        {
            if (debug)
            {
                std::cout << "Initial routing: " << totalSuccessful << "/" << nets.size()
                          << " nets. Trying rip-up and reroute...\n";
            }

            // Reset and try again with more aggressive settings
            result.clear();
            congestionMap.clear();
            totalSuccessful = 0;
            totalWirelength = 0;

            // Try different net orders
            std::vector<std::vector<size_t>> netOrderings;
            netOrderings.push_back(netIndices); // Original order

            // Reverse order
            std::vector<size_t> reversedIndices = netIndices;
            std::reverse(reversedIndices.begin(), reversedIndices.end());
            netOrderings.push_back(reversedIndices);

            // Random order
            std::vector<size_t> randomIndices = netIndices;
            std::random_device rd;
            std::mt19937 g(rd());
            std::shuffle(randomIndices.begin(), randomIndices.end(), g);
            netOrderings.push_back(randomIndices);

            bool allNetsRouted = false;

            // Try each ordering
            for (const auto &ordering : netOrderings)
            {
                // Reset for this attempt
                std::unordered_map<int, std::vector<std::pair<int, int>>> attemptResult;
                std::unordered_map<int, int> attemptCongestionMap;
                int attemptSuccessful = 0;
                int attemptWirelength = 0;

                bool allRouted = true;

                for (size_t idx : ordering)
                {
                    const Net &net = nets[idx];

                    std::vector<std::pair<int, int>> connections;
                    std::unordered_set<int> netNodes;
                    netNodes.insert(net.source);

                    bool allSinksRouted = true;
                    int netWirelength = 0;

                    // Route each sink
                    for (int sink : net.sinks)
                    {
                        // Very aggressive pathfinding
                        std::vector<int> path = findPathExtreme(net.source, sink, attemptCongestionMap, net.id, netNodes);

                        if (path.empty())
                        {
                            allSinksRouted = false;
                            if (debug)
                            {
                                std::cout << "Still failed to route net " << net.id
                                          << " with extreme search\n";
                            }
                            break;
                        }

                        // Add path to connections
                        for (size_t i = 1; i < path.size(); ++i)
                        {
                            connections.push_back(std::make_pair(path[i - 1], path[i]));
                            netNodes.insert(path[i]);
                            netWirelength += nodes[path[i]].length;
                        }
                    }

                    if (allSinksRouted)
                    {
                        attemptResult[net.id] = connections;
                        attemptSuccessful++;
                        attemptWirelength += netWirelength;

                        // Update congestion map
                        for (int node : netNodes)
                        {
                            attemptCongestionMap[node] = net.id;
                        }
                    }
                    else
                    {
                        allRouted = false;
                        break;
                    }
                }

                // If this attempt routed all nets, use it as the result
                if (allRouted)
                {
                    result = attemptResult;
                    totalSuccessful = attemptSuccessful;
                    totalWirelength = attemptWirelength;
                    allNetsRouted = true;

                    if (debug)
                    {
                        std::cout << "Successfully routed all nets with wirelength: "
                                  << totalWirelength << "\n";
                    }

                    break;
                }
            }

            // If none of the orderings worked, try one-by-one hybrid approach
            if (!allNetsRouted)
            {
                if (debug)
                {
                    std::cout << "Still couldn't route all nets. Trying hybrid approach...\n";
                }

                // Completely reset
                result.clear();
                congestionMap.clear();
                totalSuccessful = 0;
                totalWirelength = 0;

                // Try to route each net with all available methods
                for (size_t i = 0; i < nets.size(); ++i)
                {
                    const Net &net = nets[i];

                    std::vector<std::pair<int, int>> connections;
                    std::unordered_set<int> netNodes;
                    netNodes.insert(net.source);

                    bool routed = false;
                    int netWirelength = 0;

                    // Try multiple methods for each net
                    for (int method = 0; method < 3 && !routed; method++)
                    {
                        connections.clear();
                        netNodes.clear();
                        netNodes.insert(net.source);
                        netWirelength = 0;

                        bool allSinksRouted = true;

                        for (int sink : net.sinks)
                        {
                            std::vector<int> path;

                            // Different methods
                            if (method == 0)
                            {
                                path = findPathBFS(net.source, sink, congestionMap, net.id, netNodes);
                            }
                            else if (method == 1)
                            {
                                path = findPathAStar(net.source, sink, congestionMap, net.id, netNodes);
                            }
                            else
                            {
                                path = findPathExtreme(net.source, sink, congestionMap, net.id, netNodes);
                            }

                            if (path.empty())
                            {
                                allSinksRouted = false;
                                break;
                            }

                            // Add path to connections
                            for (size_t j = 1; j < path.size(); ++j)
                            {
                                connections.push_back(std::make_pair(path[j - 1], path[j]));
                                netNodes.insert(path[j]);
                                netWirelength += nodes[path[j]].length;
                            }
                        }

                        if (allSinksRouted)
                        {
                            routed = true;
                        }
                    }

                    if (routed)
                    {
                        result[net.id] = connections;
                        totalSuccessful++;
                        totalWirelength += netWirelength;

                        // Update congestion map
                        for (int node : netNodes)
                        {
                            congestionMap[node] = net.id;
                        }
                    }
                    else
                    {
                        if (debug)
                        {
                            std::cout << "Failed to route net " << net.id
                                      << " with all methods\n";
                        }
                    }
                }
            }
        }

        if (debug)
        {
            std::cout << "Baseline solution: " << totalSuccessful << "/" << nets.size()
                      << " nets, wirelength: " << totalWirelength << "\n";
        }

        // Calculate remaining time for optimization
        auto baselineTime = std::chrono::high_resolution_clock::now();
        double elapsedSeconds = std::chrono::duration<double>(baselineTime - startTime).count();
        double remainingTime = timeLimit - readTime - elapsedSeconds - 5.0;

        if (debug)
        {
            std::cout << "Remaining time for optimization: " << remainingTime << "s\n";
        }

        // If we have all nets routed and time remaining, optimize for wirelength
        if (totalSuccessful == static_cast<int>(nets.size()) && remainingTime > 10.0)
        {
            if (debug)
            {
                std::cout << "Starting wirelength optimization...\n";
            }

            // Use remaining time for optimization
            int bestWirelength = totalWirelength;
            auto bestResult = result;

            // Multiple optimization passes
            for (int pass = 0; pass < 20; ++pass)
            {
                // Try to optimize each net individually
                for (const auto &entry : result)
                {
                    int netId = entry.first;

                    // Find the corresponding net
                    const Net *currentNet = nullptr;
                    for (const auto &net : nets)
                    {
                        if (net.id == netId)
                        {
                            currentNet = &net;
                            break;
                        }
                    }

                    if (!currentNet)
                        continue;

                    // Remove this net from congestion map
                    for (const auto &connection : entry.second)
                    {
                        congestionMap.erase(connection.first);
                        congestionMap.erase(connection.second);
                    }

                    // Try to re-route this net for better wirelength
                    std::vector<std::pair<int, int>> newConnections;
                    std::unordered_set<int> netNodes;
                    netNodes.insert(currentNet->source);

                    bool improved = true;
                    int newWirelength = 0;

                    // Sort sinks by distance
                    std::vector<std::pair<int, int>> sortedSinks;
                    for (int sink : currentNet->sinks)
                    {
                        int distance = calculateManhattanDistance(currentNet->source, sink);
                        sortedSinks.push_back(std::make_pair(distance, sink));
                    }
                    std::sort(sortedSinks.begin(), sortedSinks.end());

                    // Route each sink
                    for (const auto &p : sortedSinks)
                    {
                        int sink = p.second;

                        // Find best starting point
                        int startNode = currentNet->source;
                        int minDistance = calculateManhattanDistance(startNode, sink);

                        for (int node : netNodes)
                        {
                            int distance = calculateManhattanDistance(node, sink);
                            if (distance < minDistance)
                            {
                                minDistance = distance;
                                startNode = node;
                            }
                        }

                        // Try A* with length optimization
                        std::vector<int> path = findPathOptimized(startNode, sink, congestionMap, netId, netNodes);

                        if (path.empty() && startNode != currentNet->source)
                        {
                            path = findPathOptimized(currentNet->source, sink, congestionMap, netId, netNodes);
                        }

                        if (path.empty())
                        {
                            improved = false;
                            break;
                        }

                        // Add path to connections
                        for (size_t i = 1; i < path.size(); ++i)
                        {
                            newConnections.push_back(std::make_pair(path[i - 1], path[i]));
                            netNodes.insert(path[i]);
                            newWirelength += nodes[path[i]].length;
                        }
                    }

                    // Check if we improved this net
                    int oldWirelength = 0;
                    for (const auto &connection : entry.second)
                    {
                        oldWirelength += nodes[connection.second].length;
                    }

                    if (improved && newWirelength < oldWirelength)
                    {
                        // Update the result
                        result[netId] = newConnections;
                        totalWirelength = totalWirelength - oldWirelength + newWirelength;

                        if (debug)
                        {
                            std::cout << "Optimized net " << netId << ": "
                                      << oldWirelength << " -> " << newWirelength
                                      << " (total: " << totalWirelength << ")\n";
                        }

                        // Update congestion map
                        for (int node : netNodes)
                        {
                            congestionMap[node] = netId;
                        }
                    }
                    else
                    {
                        // Restore congestion map
                        for (const auto &connection : entry.second)
                        {
                            congestionMap[connection.first] = netId;
                            congestionMap[connection.second] = netId;
                        }
                    }
                }

                // Check if we improved overall
                if (totalWirelength < bestWirelength)
                {
                    bestWirelength = totalWirelength;
                    bestResult = result;

                    if (debug)
                    {
                        std::cout << "New best wirelength: " << bestWirelength << "\n";
                    }
                }
            }

            // Use the best result
            result = bestResult;
            totalWirelength = bestWirelength;
        }

        // Write results to output file
        writeResults(result);

        auto endTime = std::chrono::high_resolution_clock::now();
        double totalTime = std::chrono::duration<double>(endTime - startTime).count();

        if (debug)
        {
            std::cout << "Routing completed: " << result.size() << "/" << nets.size()
                      << " nets successfully routed (total wirelength: " << totalWirelength
                      << ") in " << totalTime << "s\n";
        }
    }

private:
    const NodeMap &nodes;
    const AdjacencyList &adj;
    const NetList &nets;
    std::string outFile;
    bool debug;
    int threads;
    double readTime;
    double timeLimit;

    // Simple A* pathfinding
    std::vector<int> findPathSimple(int source, int sink,
                                    const std::unordered_map<int, int> &congestion,
                                    int netId, const std::unordered_set<int> &usedNodes)
    {
        // Priority queue for A* search (cost, nodeId)
        using PQElement = std::pair<int, int>;
        std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq;

        std::unordered_map<int, int> cameFrom;
        std::unordered_map<int, int> gScore;

        // Initialize gScore
        gScore[source] = 0;

        // fScore = gScore + heuristic
        int initialCost = calculateManhattanDistance(source, sink);
        pq.push(std::make_pair(initialCost, source));

        while (!pq.empty())
        {
            auto top = pq.top();
            int current = top.second;
            pq.pop();

            if (current == sink)
            {
                // Reconstruct path
                std::vector<int> path;
                while (current != source)
                {
                    path.push_back(current);
                    current = cameFrom[current];
                }
                path.push_back(source);
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (int neighbor : adj[current])
            {
                // Skip nodes already used by this net (avoid loops)
                if (usedNodes.find(neighbor) != usedNodes.end() && neighbor != sink)
                {
                    continue;
                }

                // Skip nodes used by other nets
                auto it = congestion.find(neighbor);
                if (it != congestion.end() && it->second != netId)
                {
                    continue; // Skip nodes used by other nets
                }

                // Calculate cost
                int nodeCost = nodes[neighbor].length;

                int tentativeGScore = gScore[current] + nodeCost;

                if (!gScore.count(neighbor) || tentativeGScore < gScore[neighbor])
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeGScore;

                    int heuristic = calculateManhattanDistance(neighbor, sink);
                    int fScore = tentativeGScore + heuristic;
                    pq.push(std::make_pair(fScore, neighbor));
                }
            }
        }

        // No path found
        return {};
    }

    // BFS pathfinding - less optimal for wirelength but more reliable for finding any path
    std::vector<int> findPathBFS(int source, int sink,
                                 const std::unordered_map<int, int> &congestion,
                                 int netId, const std::unordered_set<int> &usedNodes)
    {
        std::queue<int> q;
        std::unordered_map<int, int> cameFrom;
        std::unordered_set<int> visited;

        q.push(source);
        visited.insert(source);

        while (!q.empty())
        {
            int current = q.front();
            q.pop();

            if (current == sink)
            {
                // Reconstruct path
                std::vector<int> path;
                while (current != source)
                {
                    path.push_back(current);
                    current = cameFrom[current];
                }
                path.push_back(source);
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (int neighbor : adj[current])
            {
                // Skip visited nodes
                if (visited.find(neighbor) != visited.end())
                {
                    continue;
                }

                // Skip nodes already used by this net (avoid loops)
                if (usedNodes.find(neighbor) != usedNodes.end() && neighbor != sink)
                {
                    continue;
                }

                // Skip nodes used by other nets
                auto it = congestion.find(neighbor);
                if (it != congestion.end() && it->second != netId)
                {
                    continue;
                }

                visited.insert(neighbor);
                cameFrom[neighbor] = current;
                q.push(neighbor);
            }
        }

        // No path found
        return {};
    }

    // A* with optimizations for extreme cases
    std::vector<int> findPathExtreme(int source, int sink,
                                     const std::unordered_map<int, int> &congestion,
                                     int netId, const std::unordered_set<int> &usedNodes)
    {
        // Priority queue for A* search (cost, nodeId)
        using PQElement = std::pair<int, int>;
        std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq;

        std::unordered_map<int, int> cameFrom;
        std::unordered_map<int, int> gScore;

        // Initialize gScore
        gScore[source] = 0;

        // fScore = gScore + heuristic
        int initialCost = calculateManhattanDistance(source, sink);
        pq.push(std::make_pair(initialCost, source));

        int nodesExplored = 0;
        const int explorationLimit = 500000; // Very large limit

        while (!pq.empty() && nodesExplored < explorationLimit)
        {
            auto top = pq.top();
            int current = top.second;
            pq.pop();
            nodesExplored++;

            if (current == sink)
            {
                // Reconstruct path
                std::vector<int> path;
                while (current != source)
                {
                    path.push_back(current);
                    current = cameFrom[current];
                }
                path.push_back(source);
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (int neighbor : adj[current])
            {
                // Skip nodes already used by this net (avoid loops) except near the sink
                if (usedNodes.find(neighbor) != usedNodes.end() &&
                    neighbor != sink &&
                    calculateManhattanDistance(neighbor, sink) > 2)
                {
                    continue;
                }

                // Allow crossing through other nets, but with a high penalty
                int nodeCost = nodes[neighbor].length;

                auto it = congestion.find(neighbor);
                if (it != congestion.end() && it->second != netId)
                {
                    // High penalty but not impossible
                    nodeCost += 1000;
                }

                int tentativeGScore = gScore[current] + nodeCost;

                if (!gScore.count(neighbor) || tentativeGScore < gScore[neighbor])
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeGScore;

                    int heuristic = calculateManhattanDistance(neighbor, sink);
                    int fScore = tentativeGScore + heuristic;
                    pq.push(std::make_pair(fScore, neighbor));
                }
            }
        }

        // No path found
        return {};
    }

    // A* with more focus on wirelength
    std::vector<int> findPathAStar(int source, int sink,
                                   const std::unordered_map<int, int> &congestion,
                                   int netId, const std::unordered_set<int> &usedNodes)
    {
        // Priority queue for A* search (cost, nodeId)
        using PQElement = std::pair<int, int>;
        std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq;

        std::unordered_map<int, int> cameFrom;
        std::unordered_map<int, int> gScore;

        // Initialize gScore
        gScore[source] = 0;

        // fScore = gScore + heuristic
        int initialCost = calculateManhattanDistance(source, sink);
        pq.push(std::make_pair(initialCost, source));

        while (!pq.empty())
        {
            auto top = pq.top();
            int current = top.second;
            pq.pop();

            if (current == sink)
            {
                // Reconstruct path
                std::vector<int> path;
                while (current != source)
                {
                    path.push_back(current);
                    current = cameFrom[current];
                }
                path.push_back(source);
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (int neighbor : adj[current])
            {
                // Skip nodes already used by this net (avoid loops)
                if (usedNodes.find(neighbor) != usedNodes.end() && neighbor != sink)
                {
                    continue;
                }

                // Skip nodes used by other nets
                auto it = congestion.find(neighbor);
                if (it != congestion.end() && it->second != netId)
                {
                    continue;
                }

                // Calculate cost with emphasis on node length
                int nodeCost = nodes[neighbor].length;

                int tentativeGScore = gScore[current] + nodeCost;

                if (!gScore.count(neighbor) || tentativeGScore < gScore[neighbor])
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeGScore;

                    int heuristic = calculateManhattanDistance(neighbor, sink);
                    int fScore = tentativeGScore + heuristic;
                    pq.push(std::make_pair(fScore, neighbor));
                }
            }
        }

        // No path found
        return {};
    }

    // A* specifically optimized for wirelength reduction
    // A* specifically optimized for wirelength reduction
    std::vector<int> findPathOptimized(int source, int sink,
                                       const std::unordered_map<int, int> &congestion,
                                       int netId, const std::unordered_set<int> &usedNodes)
    {
        // Priority queue for A* search (cost, nodeId)
        using PQElement = std::pair<int, int>;
        std::priority_queue<PQElement, std::vector<PQElement>, std::greater<PQElement>> pq;

        std::unordered_map<int, int> cameFrom;
        std::unordered_map<int, int> gScore;

        // Initialize gScore
        gScore[source] = 0;

        // fScore = gScore + heuristic
        int initialCost = nodes[source].length + calculateManhattanDistance(source, sink);
        pq.push(std::make_pair(initialCost, source));

        while (!pq.empty())
        {
            auto top = pq.top();
            int current = top.second;
            pq.pop();

            if (current == sink)
            {
                // Reconstruct path
                std::vector<int> path;
                while (current != source)
                {
                    path.push_back(current);
                    current = cameFrom[current];
                }
                path.push_back(source);
                std::reverse(path.begin(), path.end());
                return path;
            }

            for (int neighbor : adj[current])
            {
                // Skip nodes already used by this net (avoid loops)
                if (usedNodes.find(neighbor) != usedNodes.end() && neighbor != sink)
                {
                    continue;
                }

                // Skip nodes used by other nets
                auto it = congestion.find(neighbor);
                if (it != congestion.end() && it->second != netId)
                {
                    continue;
                }

                // Calculate cost with double emphasis on node length
                int nodeCost = nodes[neighbor].length;

                int tentativeGScore = gScore[current] + nodeCost;

                if (!gScore.count(neighbor) || tentativeGScore < gScore[neighbor])
                {
                    cameFrom[neighbor] = current;
                    gScore[neighbor] = tentativeGScore;

                    // Heuristic weighted more towards wirelength
                    int heuristic = calculateManhattanDistance(neighbor, sink);
                    int fScore = tentativeGScore + heuristic;
                    pq.push(std::make_pair(fScore, neighbor));
                }
            }
        }

        // No path found
        return {};
    }

    // Calculate Manhattan distance between two nodes
    int calculateManhattanDistance(int node1, int node2)
    {
        const auto &n1 = nodes[node1];
        const auto &n2 = nodes[node2];

        // Use center points of nodes
        int x1 = (n1.begin_x + n1.end_x) / 2;
        int y1 = (n1.begin_y + n1.end_y) / 2;
        int x2 = (n2.begin_x + n2.end_x) / 2;
        int y2 = (n2.begin_y + n2.end_y) / 2;

        return std::abs(x2 - x1) + std::abs(y2 - y1);
    }

    // Write routing results to output file in the required format
    void writeResults(const std::unordered_map<int, std::vector<std::pair<int, int>>> &results)
    {
        std::ofstream fout(outFile);
        if (!fout)
            throw std::runtime_error("Cannot open output file: " + outFile);

        // Get net IDs in ascending order
        std::vector<int> netIds;
        for (const auto &p : results)
        {
            int netId = p.first;
            netIds.push_back(netId);
        }
        std::sort(netIds.begin(), netIds.end());

        // Write each net's routing result in ascending net ID order
        for (size_t i = 0; i < netIds.size(); ++i)
        {
            int netId = netIds[i];

            // Find the net name
            std::string netName;
            for (const auto &net : nets)
            {
                if (net.id == netId)
                {
                    netName = net.name;
                    break;
                }
            }

            const auto &connections = results.at(netId);

            // Write net header
            fout << netId << " " << netName << "\n";

            // Write parent-child connections
            for (const auto &connection : connections)
            {
                int parent = connection.first;
                int child = connection.second;
                fout << parent << " " << child << "\n";
            }

            // Add empty line between nets (except after the last net)
            if (i < netIds.size() - 1)
            {
                fout << "\n";
            }
        }

        fout.close();
    }
};