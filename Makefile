# Makefile for Percepto project

BUILD_DIR := build

.PHONY: all build configure clean rebuild test

all: configure

# Only run make -j (assumes cmake already ran)
build:
	@echo "🔧 Running make -j in $(BUILD_DIR)..."
	cd $(BUILD_DIR) && make -j

# Configure CMake and then build
configure:
	@echo "⚙️ Configuring CMake and building..."
	@mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. && make -j

# Clean build directory
clean:
	@echo "🧹 Cleaning build directory..."
	rm -rf $(BUILD_DIR)

# Full clean and rebuild from scratch
rebuild: clean configure

# Run tests with output
test:
	@echo "🧪 Running tests..."
	cd $(BUILD_DIR) && ctest --output-on-failure
