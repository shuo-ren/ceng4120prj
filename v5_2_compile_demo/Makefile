CXX      = g++
CXXFLAGS = -std=c++17 -O3 -Wall -Isrc -fopenmp
LDFLAGS  = -static -fopenmp

SOURCES  = main.cpp src/dataloader.cpp
OBJECTS  = $(SOURCES:.cpp=.o)
TARGET   = fpga_router

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) $(OBJECTS) -o $(TARGET) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJECTS) $(TARGET)
