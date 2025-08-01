BUILD_DIR := build

.PHONY: all build configure clean rebuild test format

all: build

build:
	@echo "🔧 Running make -j in $(BUILD_DIR)..."
	@cd $(BUILD_DIR) && make -j

configure:
	@echo "⚙️ Configuring CMake..."
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && cmake -DCMAKE_BUILD_TYPE=Debug .. 

clean:
	@echo "🧹 Cleaning build directory..."
	rm -rf $(BUILD_DIR)

rebuild: clean configure build

test: build
	@echo "🧪 Running tests..."
	@cd $(BUILD_DIR) && ctest --output-on-failure --verbose 

format:
	@echo "🎨 Formatting all C++ source and header files recursively..."
	find src include tests benches -name '*.cpp' -o -name '*.h' | xargs -P $(shell nproc 2>/dev/null || echo 1) clang-format -i