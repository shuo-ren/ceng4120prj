#include "dataloader.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
#include <omp.h> // 添加OpenMP头文件

DataLoader::DataLoader(int num_threads) : num_threads(num_threads) {
    // 确保线程数至少为1
    if (this->num_threads < 1) {
        this->num_threads = 1;
    }
}

void DataLoader::loadDevice(const std::string& filename) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    std::ifstream file(filename);
    if (!file) {
        throw std::runtime_error("Failed to open device file: " + filename);
    }
    
    // 读取节点数量
    std::string line;
    std::getline(file, line);
    size_t nodeCount = std::stoul(line);
    
    std::cout << "Device has " << nodeCount << " nodes" << std::endl;
    
    // 预读取所有节点和邻接表
    std::vector<std::string> node_lines;
    std::vector<std::string> adj_lines;
    
    node_lines.reserve(nodeCount); // 预分配内存
    
    // 读取节点定义行
    std::cout << "Reading node definition lines into memory..." << std::endl;
    for (size_t i = 0; i < nodeCount; ++i) {
        if (std::getline(file, line) && !line.empty()) {
            node_lines.push_back(line);
        }
    }
    
    std::cout << "Read " << node_lines.size() << " node definition lines" << std::endl;
    
    // 读取邻接表行
    std::cout << "Reading adjacency list lines into memory..." << std::endl;
    size_t adj_count = 0;
    while (std::getline(file, line)) {
        if (!line.empty()) {
            adj_lines.push_back(line);
            adj_count++;
            
            // 每百万行输出一次进度
            if (adj_count % 1000000 == 0) {
                std::cout << "  Read " << adj_count << " adjacency lines so far..." << std::endl;
            }
        }
    }
    
    file.close();
    std::cout << "Finished reading " << adj_lines.size() << " adjacency list lines into memory." << std::endl;
    
    // 并行处理节点定义
    std::cout << "Starting parallel node parsing (" << num_threads << " threads)..." << std::endl;
    double start_time_nodes = omp_get_wtime();
    
    nodes.clear();
    nodes.reserve(node_lines.size()); // 预分配空间提高性能
    
    #pragma omp parallel num_threads(num_threads)
    {
        // 每个线程创建自己的局部映射
        NodeMap local_nodes;
        
        #pragma omp for schedule(static)
        for (size_t i = 0; i < node_lines.size(); ++i) {
            std::istringstream iss(node_lines[i]);
            Node node;
            iss >> node.id >> node.type >> node.length 
                >> node.begin_x >> node.begin_y 
                >> node.end_x >> node.end_y;
            
            // 读取可能包含空格的节点名称
            std::string rest;
            if (std::getline(iss, rest)) {
                if (!rest.empty()) {
                    node.name = rest.substr(1); // 跳过前导空格
                }
            }
            
            local_nodes[node.id] = std::move(node);
        }
        
        // 合并局部映射到全局映射
        #pragma omp critical
        {
            for (auto& pair : local_nodes) {
                nodes.insert(std::move(pair));
            }
        }
    }
    
    double end_time_nodes = omp_get_wtime();
    std::cout << "Parallel node parsing finished." << std::endl;
    std::cout << "Node parsing time: " << (end_time_nodes - start_time_nodes) << " seconds." << std::endl;
    
    // 并行处理邻接表
    std::cout << "Starting parallel adjacency list parsing (" << num_threads << " threads)..." << std::endl;
    double start_time_adj = omp_get_wtime();
    
    adjacency.clear();
    adjacency.reserve(adj_lines.size()); // 预分配空间提高性能
    
    #pragma omp parallel num_threads(num_threads)
    {
        // 每个线程创建自己的局部映射
        AdjacencyList local_adjacency;
        
        #pragma omp for schedule(static)
        for (size_t i = 0; i < adj_lines.size(); ++i) {
            std::istringstream iss(adj_lines[i]);
            std::string parent;
            iss >> parent;
            
            std::vector<std::string> children;
            std::string child;
            while (iss >> child) {
                children.push_back(child);
            }
            
            if (!children.empty()) {
                local_adjacency[parent] = std::move(children);
            }
        }
        
        // 合并局部映射到全局映射
        #pragma omp critical
        {
            for (auto& pair : local_adjacency) {
                adjacency.insert(std::move(pair));
            }
        }
    }
    
    double end_time_adj = omp_get_wtime();
    std::cout << "Parallel adjacency list parsing finished." << std::endl;
    std::cout << "Adjacency list parsing time: " << (end_time_adj - start_time_adj) << " seconds." << std::endl;
    
    // 计算连接总数
    size_t connectionCount = 0;
    for (const auto& adj : adjacency) {
        connectionCount += adj.second.size();
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(endTime - startTime).count();
    
    std::cout << "Device loaded in " << duration << " seconds" << std::endl;
    std::cout << "  " << nodes.size() << " nodes" << std::endl;
    std::cout << "  " << connectionCount << " connections" << std::endl;
    
    // 清理预读取的数据以释放内存
    std::vector<std::string>().swap(node_lines);
    std::vector<std::string>().swap(adj_lines);
}

void DataLoader::loadNetlist(const std::string& filename) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    std::ifstream file(filename);
    if (!file) {
        throw std::runtime_error("Failed to open netlist file: " + filename);
    }
    
    // 清除现有网表
    nets.clear();
    
    // 读取网数量或第一个网
    std::string line;
    std::getline(file, line);
    
    // 检查第一行是网数量还是已经是第一个网
    bool hasHeader = false;
    size_t netCount = 0;
    try {
        netCount = std::stoul(line);
        hasHeader = true;
    } catch (const std::exception&) {
        // 第一行已经是网
        file.seekg(0); // 回到开头
    }
    
    // 预读取所有网行
    std::vector<std::string> net_lines;
    while (std::getline(file, line)) {
        if (!line.empty()) {
            net_lines.push_back(line);
        }
    }
    file.close();
    
    if (hasHeader && netCount != net_lines.size()) {
        std::cout << "Warning: Expected " << netCount << " nets but found " 
                  << net_lines.size() << " net definitions" << std::endl;
    }
    
    // 预分配网表大小
    nets.reserve(net_lines.size());
    
    // 使用OpenMP并行处理网行
    std::cout << "Starting parallel netlist parsing (" << num_threads << " threads)..." << std::endl;
    double start_time_nets = omp_get_wtime();
    
    // 使用一个临时向量来存储每个线程的结果
    std::vector<std::vector<Net>> thread_nets(num_threads);
    
    #pragma omp parallel num_threads(num_threads)
    {
        int thread_id = omp_get_thread_num();
        thread_nets[thread_id].reserve(net_lines.size() / num_threads + 1);
        
        #pragma omp for schedule(static)
        for (size_t i = 0; i < net_lines.size(); ++i) {
            std::istringstream iss(net_lines[i]);
            Net net;
            iss >> net.id >> net.name >> net.source;
            
            std::string sink;
            while (iss >> sink) {
                net.sinks.push_back(sink);
            }
            
            thread_nets[thread_id].push_back(std::move(net));
        }
    }
    
    // 按顺序合并所有线程的结果
    for (int i = 0; i < num_threads; ++i) {
        nets.insert(nets.end(), 
                    std::make_move_iterator(thread_nets[i].begin()), 
                    std::make_move_iterator(thread_nets[i].end()));
    }
    
    double end_time_nets = omp_get_wtime();
    std::cout << "Parallel netlist parsing finished." << std::endl;
    std::cout << "Netlist parsing time: " << (end_time_nets - start_time_nets) << " seconds." << std::endl;
    
    // 清理预读取的数据以释放内存
    std::vector<std::string>().swap(net_lines);
    
    auto endTime = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
    
    std::cout << "Netlist loaded in " << duration << " ms" << std::endl;
    std::cout << "  " << nets.size() << " nets" << std::endl;
}