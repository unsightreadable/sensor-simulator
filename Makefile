CXX    := g++
TARGET := sensor_simulator
SRC    := sensor_simulator.cpp

UNAME := $(shell uname -s)

ifeq ($(UNAME), Darwin)
  EDGE_FLAGS := -Os -std=c++14 -fno-rtti -fno-exceptions -Wl,-dead_strip
  STRIP_CMD  := strip $(TARGET)
else
  EDGE_FLAGS := -Os -std=c++14 -fno-rtti -fno-exceptions -ffunction-sections -fdata-sections -Wl,--gc-sections -static -s
  STRIP_CMD  := @true
endif

DEV_FLAGS := -O0 -g -std=c++14 -Wall -Wextra

.PHONY: all edge dev clean

all: edge

edge: $(SRC)
	$(CXX) $(EDGE_FLAGS) -o $(TARGET) $(SRC)
	$(STRIP_CMD)
	@echo "built: ./$(TARGET)"
	@ls -lh $(TARGET)

dev: $(SRC)
	$(CXX) $(DEV_FLAGS) -o $(TARGET)_dev $(SRC)
	@echo "built: ./$(TARGET)_dev"

clean:
	rm -f $(TARGET) $(TARGET)_dev
