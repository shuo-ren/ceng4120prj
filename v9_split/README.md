# FPGA Router (CENG4120 Final Project)

A multithreaded FPGA routing framework supporting congestion elimination, loop removal, and strategy-based adaptive optimization under time constraints.

## 📂 Project Structure

```
.
├── src/
│   ├── main.cpp                 # Entry point
│   ├── dataloader.h            # Device/netlist file parser
│   ├── router.{h,cpp}          # Core router logic (strategy dispatcher)
│   ├── common/
│   │   ├── a_star.h            # A* pathfinder with occupancy cost
│   │   ├── loop_removal.h      # Loop detection and removal
│   │   └── congestion.h        # Congestion resolution engine
│   └── strategy/
│       ├── strategy.h          # Strategy interface
│       ├── quality.{h,cpp}     # Wirelength-minimizing search (small scale)
│       ├── hybrid.{h,cpp}      # History-based rerouting for long nets
│       └── throughput.{h,cpp}  # Maximize routed nets under budget
└── Makefile
```

## 🛠️ Build

```bash
make
# or manually:
g++ -std=c++17 -O3 -Wall -Isrc -fopenmp $(find src -name '*.cpp') -o fpga_router
```

## 🚀 Usage

```bash
./fpga_router <device> <netlist> <output.route> [--threads N] [--debug]
```

- `--threads`: Number of threads to use (default: 8)
- `--debug`: Enable debug output

## 📥 Input Format

### Device File (`*.device`)

```
<node_count>
<node_id> <type> <length> <begin_x> <begin_y> <end_x> <end_y>
...
<adjacency list>
<node_id> <neighbor_1> <neighbor_2> ...
```

### Netlist File (`*.netlist`)

```
<net_id> <net_name> <source_node> <sink_1> <sink_2> ...
...
```

## 📤 Output Format (`*.route`)

```
<net_id> <net_name>
<src> <dst>
<src> <dst>
...
<empty line>
```

Only successfully routed nets without congestion or loops will be included.

## 🧠 Strategy Selection (Auto-Triggered)

The router will automatically select the optimal routing strategy based on the **netlist size**:

| Net Count        | Strategy           | Description |
|------------------|--------------------|-------------|
| ≤ 100            | `QualityStrategy`  | Multi-try randomized routing to minimize total wirelength |
| 101 – 8000       | `HybridStrategy`   | Rip-up and reroute top 5% longest nets using history-aware A* |
| > 8000           | `ThroughputStrategy` | Focus on maximizing routed nets within remaining time |

## 📋 Routing Pipeline

1. **Data Load**
   - Parses `.device` and `.netlist` into memory (with OpenMP parallel I/O).

2. **First-Round Routing**
   - Parallel A* routing for all nets.
   - Loop removal to ensure tree-like structure per net.
   - Occupancy tracking via `uint8_t occ[]`.

3. **Strategy Phase**
   - A routing budget is computed from total runtime limit:
     ```
     budget = (limit - read_time - ½·read_time - route_time) × 0.95
     ```
   - The chosen strategy reroutes selectively using this budget.

4. **Final Cleanup**
   - Loop check and removal.
   - Congestion elimination: nets contributing most to congestion are removed.

5. **Result Writeback**
   - Output `.route` file with all successfully routed and congestion-free nets.

## 🧪 Benchmark Time Constraints

| Benchmark     | #Nets   | Time Limit (s) |
|---------------|---------|----------------|
| design1       | 21      | 100            |
| design2       | 670     | 100            |
| design3       | 5000    | 100            |
| design4       | 4999    | 100            |
| design5       | 27960   | 250            |

Time limit includes:
- Device/Netlist loading
- Routing and post-processing
- Output writing

## 🔍 Optimization Highlights

- **Multithreaded** first-round A* routing
- **Occupancy-aware** rerouting with history cost (α = 0.6)
- **Loop pruning** using BFS backbone detection
- **Congestion analysis** via node-to-net conflict map
- **Modular strategy pattern** for easy extensibility

## 📌 Notes

- All time constraints are enforced conservatively.
- The tool creates the output directory automatically if it does not exist.
- You may extend the routing strategies in `strategy/` to handle thermal, timing, or power-awareness.

## 👨‍💻 Author

This project was developed as part of the FPGA Routing assignment in CENG4120 at CUHK.  
Feel free to reach out for collaboration or further extensions.
