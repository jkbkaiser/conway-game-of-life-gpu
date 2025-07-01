NVCC = nvcc
NVCCFLAGS = -std=c++17 -O2
TARGET = game_of_life
SRC = main.cu
IMG_DIR = images

# Default target: compile and run
all: run

# Compile and run
run: $(TARGET)
	@echo "[RUN] Executing $(TARGET)..."
	@rm -rf $(IMG_DIR)
	@mkdir -p $(IMG_DIR)
	./$(TARGET)

# Compile using nvcc
$(TARGET): $(SRC)
	@echo "[BUILD] Compiling $(SRC) with nvcc..."
	$(NVCC) $(NVCCFLAGS) $< -o $@

# Convert PNG frames into MP4 video
video:
	@echo "[VIDEO] Generating out.mp4 from PNG frames..."
	ffmpeg -framerate 10 -i $(IMG_DIR)/frame_%d.png -pix_fmt yuv420p out.mp4

# Remove all generated files
clean:
	@echo "[CLEAN] Cleaning binary, frames, and video..."
	rm -f $(TARGET)
	rm -rf $(IMG_DIR)
	rm -f out.mp4
