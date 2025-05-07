CXX      = g++
CXXFLAGS = -std=c++17 -O3 -Wall -Isrc -fopenmp

SOURCES  = main.cpp src/dataloader.cpp
OBJECTS  = $(SOURCES:.cpp=.o)
TARGET   = fpga_router

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) $^ -o $@ -fopenmp

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)
