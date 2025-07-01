/*
Conway's Game of Life rules:

1. Survival:
   - A live cell with 2 or 3 live neighbors stays alive.

2. Birth:
   - A dead cell with exactly 3 live neighbors becomes alive.

3. Death:
   - A live cell with fewer than 2 live neighbors dies (underpopulation).
   - A live cell with more than 3 live neighbors dies (overpopulation).
*/

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#include <iostream>
#include <math.h>
#include <filesystem>
#include <string.h>
#include <random>
#include <chrono>

const std::filesystem::path IMG_DIR = "./images";
 
int write_grid_to_img(int *grid, int rows, int cols, int cell_size, int iter)
{
  int n_rows = rows * cell_size;
  int n_cols = cols * cell_size;

  uint8_t* pixels = new uint8_t[n_rows * n_cols * 3];

  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      int alive = grid[row * cols + col];
      uint8_t color = alive ? 255 : 0;

      for (int dy = 0; dy < cell_size; ++dy) {
        for(int dx = 0; dx < cell_size; ++dx) {
          int y = row * cell_size + dy;
          int x = col * cell_size + dx;
          int pixel = (y * n_cols + x) * 3;

          pixels[pixel + 0] = color;
          pixels[pixel + 1] = color;
          pixels[pixel + 2] = color;
        }
      }
    }
  }

  std::filesystem::path filename = IMG_DIR / ("frame_" + std::to_string(iter) + ".png");
  stbi_write_png(filename.c_str(), n_cols, n_rows, 3, pixels, n_cols * 3);
  return 0;
}

__global__
void compute_step_cuda(int *gridA, int *gridB, int rows, int cols)
{
  int index = blockIdx.x * blockDim.x + threadIdx.x;
  int stride = blockDim.x * gridDim.x;

  for (int i = index; i < rows * cols; i+=stride) {
    int row = i / cols;
    int col = i % cols;

    int neighbors = 0;

    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (dx == 0 && dy == 0) continue;

        int x = col + dx;
        int y = row + dy;

        if (x < 0 || x >= cols || y < 0 || y >= rows) continue;

        int alive = gridA[y * cols + x];

        if (alive == 1) {
          ++neighbors;
        }
      }
    }

    int gridIdx = row * cols + col;
    int alive = gridA[gridIdx];

    if (alive) {
      if (neighbors < 2 || 3 < neighbors) {
        gridB[gridIdx] = 0;
      } else {
        gridB[gridIdx] = 1;
      }
    } else  {
      if (neighbors == 3) {
        gridB[gridIdx] = 1;
      } else {
        gridB[gridIdx] = 0;
      }
    }
  }
}

void compute_step_cpu(int *gridA, int *gridB, int rows, int cols)
{
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      int neighbors = 0;

      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) continue;

          int x = col + dx;
          int y = row + dy;

          if (x < 0 || x >= cols || y < 0 || y >= rows) continue;

          int alive = gridA[y * cols + x];

          if (alive == 1) {
            ++neighbors;
          }
        }
      }

      int index = row * cols + col;
      int alive = gridA[index];

      if (alive) {
        if (neighbors < 2 || 3 < neighbors) {
          gridB[index] = 0;
        } else {
          gridB[index] = 1;
        }
      } else  {
        if (neighbors == 3) {
          gridB[index] = 1;
        } else {
          gridB[index] = 0;
        }
      }
    }
  }
}

int main(void)
{
  std::filesystem::create_directory(IMG_DIR);

  bool useGPU = true;
  bool saveImgs = true;
  int scale = 1;

  // 1024 vals
  float aspectRatio = 1.0f;
  float imgSize = 1024;

  int cols = imgSize;
  int rows = imgSize / aspectRatio;

  int iters = 200;

  int *gridA = new int[rows * cols];
  int *gridB = new int[rows * cols];

  std::random_device rd;
  std::mt19937 gen(rd());
  std::bernoulli_distribution dist(0.2);

  for (int i = 0; i < rows * cols; ++i) {
    gridA[i] = dist(gen);
  }

  if (saveImgs) {
    write_grid_to_img(gridA, rows, cols, scale, 0);
  }

  int blockSize = 256;
  int gridSize = (rows * cols + blockSize - 1) / blockSize;

  int *gridADevice;
  int *gridBDevice;
  int gridNumBytes = rows * cols * sizeof(int);

  if (useGPU) {
    cudaMalloc(&gridADevice, gridNumBytes);
    cudaMalloc(&gridBDevice, gridNumBytes);
    cudaMemcpy(gridADevice, gridA, gridNumBytes, cudaMemcpyHostToDevice);
  }

  float avgTime = 0.0f;

  for (int i = 1; i < iters; ++i) {
    float progress = float(i) / iters;
    int barWidth = 50;

    std::cout << "\r" << i << "/" << iters << " [";
    int pos = barWidth * progress;
    for (int j = 0; j < barWidth; ++j) {
        std::cout << (j < pos ? "=" : (j == pos ? ">" : " "));
    }
    std::cout << "] " << int(progress * 100.0) << "%" << std::flush;

    auto start = std::chrono::high_resolution_clock::now();

    // CPU
    if (!useGPU) {
      compute_step_cpu(gridA, gridB, rows, cols);
    }

    // GPU
    if (useGPU) {
      compute_step_cuda<<<gridSize, blockSize>>>(gridADevice, gridBDevice, rows, cols);
      cudaDeviceSynchronize();
      cudaMemcpy(gridB, gridBDevice, gridNumBytes, cudaMemcpyDeviceToHost);
    }

    // Measure time
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;
    avgTime += elapsed.count();

    if (saveImgs) {
      write_grid_to_img(gridB, rows, cols, scale, i);
    }

    if (useGPU) {
      std::swap(gridADevice, gridBDevice);
    } else {
      std::swap(gridA, gridB);
    }
  }

  std::cout << "\nAvg compute time: " << avgTime / iters << " ms" << std::endl;

  if (useGPU) {
    cudaFree(gridADevice);
    cudaFree(gridBDevice);
  }

  delete[] gridA;
  delete[] gridB;

  return 0;
}
