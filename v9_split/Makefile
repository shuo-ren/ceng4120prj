# Compiler settings
CXX      = g++
CXXFLAGS = -std=c++17 -O3 -Wall -Isrc -fopenmp

# Target binary
TARGET = fpga_router

# Source files
SRC = \
    src/main.cpp \
    src/router.cpp \
    src/common/loop_remove.cpp \
    src/strategy/hybrid.cpp \
    src/strategy/quality.cpp \
    src/strategy/throughput.cpp

# Build rules
all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $@ -lpthread

clean:
	rm -f $(TARGET)
