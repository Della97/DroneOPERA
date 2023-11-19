// MatrixFunctions.cpp
#include "MatrixFunctions.h"

std::vector<std::vector<int>> readMatrixFromFile(const std::string& filename) {
    std::ifstream file(filename);
    std::vector<std::vector<int>> matrix;

    if (file.is_open()) {
        int rows, cols;
        file >> rows >> cols; // Assuming the first line of the file contains the dimensions of the matrix.

        matrix.resize(rows, std::vector<int>(cols));

        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                file >> matrix[i][j];
            }
        }

        file.close();
    } else {
        std::cerr << "Unable to open file: " << filename << std::endl;
        // Print more information about the error
        perror("Error");
    }

    return matrix;
}

void printMatrix(const std::vector<std::vector<int>>& matrix) {
    for (const auto& row : matrix) {
        for (int value : row) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }
}