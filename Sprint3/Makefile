# Compiler and flags
CC = gcc
CFLAGS = -g -Wall -Wextra -fanalyzer -pthread
LIBS = -lm -lrt -lpthread

# Target executable
TARGET = simulation_hybrid

# Source files
SOURCES = main.c shared_memory.c thread_manager.c drone.c utils.c report.c collision.c
OBJECTS = $(SOURCES:.c=.o)

# Header dependencies
HEADERS = shared_memory.h thread_manager.h drone_hybrid.h utils.h report_hybrid.h collision_hybrid.h

# Default target
all: $(TARGET)

# Build the main executable
$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) $(LIBS) -o $(TARGET)

# Individual object file rules
main.o: main.c shared_memory.h thread_manager.h drone.h utils.h report.h
	$(CC) $(CFLAGS) -c main.c -o main.o

shared_memory.o: shared_memory.c shared_memory.h
	$(CC) $(CFLAGS) -c shared_memory.c -o shared_memory.o

thread_manager.o: thread_manager.c thread_manager.h shared_memory.h utils.h collision.h
	$(CC) $(CFLAGS) -c thread_manager.c -o thread_manager.o

drone.o: drone.c drone.h shared_memory.h
	$(CC) $(CFLAGS) -c drone.c -o drone.o

utils.o: utils.c utils.h
	$(CC) $(CFLAGS) -c utils.c -o utils.o

report.o: report.c report.h shared_memory.h
	$(CC) $(CFLAGS) -c report.c -o report.o

collision.o: collision.c collision.h shared_memory.h
	$(CC) $(CFLAGS) -c collision.c -o collision.o

# Clean up build files
clean:
	rm -f *.o $(TARGET) Simulation_Report.txt

# Clean up system resources (shared memory and semaphores)
clean-system:
	@echo "Cleaning up system resources..."
	@ipcs -m | grep $(USER) | awk '{print $$2}' | xargs -r ipcrm -m 2>/dev/null || true
	@ipcs -s | grep $(USER) | awk '{print $$2}' | xargs -r ipcrm -s 2>/dev/null || true
	@echo "System resources cleaned"

# Run the simulation
run: $(TARGET)
	./$(TARGET) $(ARGS)

# Debug run with gdb
debug: $(TARGET)
	gdb --args ./$(TARGET) $(ARGS)

# Install dependencies (Ubuntu/Debian)
install-deps:
	sudo apt-get update
	sudo apt-get install build-essential gdb

# Help target
help:
	@echo "Available targets:"
	@echo "  all          - Build the simulation (default)"
	@echo "  clean        - Remove build files"
	@echo "  clean-system - Clean up shared memory and semaphores"
	@echo "  run          - Run the simulation (use ARGS='file1 file2...')"
	@echo "  debug        - Run with gdb debugger"
	@echo "  install-deps - Install build dependencies"
	@echo "  help         - Show this help message"
	@echo ""
	@echo "Example usage:"
	@echo "  make run ARGS='drone_1_script.txt drone_2_script.txt'"

.PHONY: all clean clean-system run debug install-deps help
