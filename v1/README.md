# ceng4120prj



Download links
benchmarks (zipped file): https://github.com/ippan-kaishain/CENG4120-2025-Final/releases/download/Released/benchmarks.tar.bz2
Unzip command: tar -xjvf benchmarks.tar.bz2
evaluator: https://github.com/ippan-kaishain/CENG4120-2025-Final/releases/download/Released/eval
Usage: ./eval <device> <netlist> <result>
If you find no permission, try: chmod +x ./eval

device (zipped file): https://github.com/ippan-kaishain/CENG4120-2025-Final/releases/download/Released/xcvu3p.tar.bz2
Unzip command: tar -xjvf xcvu3p.tar.bz2


基本调用方法
bash./fpga_router <device_file> <netlist_file> [--debug] [--cache <cache_file>] [--read-only-cache]
参数说明

<device_file>: FPGA 设备文件路径（必需）
<netlist_file>: 网表文件路径或 "all"（必需）
--debug: 启用调试模式，输出详细信息（可选）
--cache <cache_file>: 指定缓存文件/前缀（可选）
--read-only-cache: 仅从缓存读取，不写入新缓存文件（用于加速测试）（可选）

常用调用示例

处理单个网表文件:
bash./fpga_router xcvu3p.device benchmarks/design1.netlist

处理所有网表文件:
bash./fpga_router xcvu3p.device all

使用缓存并启用调试:
./fpga_router xcvu3p.device benchmarks/design1.netlist --debug --cache cache/mycache

仅读取缓存（测试模式）:
bash./fpga_router xcvu3p.device benchmarks/design1.netlist --cache cache/mycache --read-only-cache

处理所有网表且使用只读缓存:
bash./fpga_router xcvu3p.device all --cache cache/mycache --read-only-cache --debug


这个更新后的调用方法增加了 --read-only-cache 选项，使您可以在测试阶段快速运行程序而不写入新的缓存文件。同时程序会自动创建所需的目录结构，避免目录不存在时的错误。