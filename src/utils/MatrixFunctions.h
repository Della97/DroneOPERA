// MatrixFunctions.h
#ifndef MATRIX_FUNCTIONS_H
#define MATRIX_FUNCTIONS_H

#include <iostream>
#include <fstream>
#include <vector>

/**
 * @brief Reads a matrix from a file.
 *
 * This function reads a matrix from a file. The file is expected
 * to have the following format:
 *   <number_of_rows> <number_of_columns>
 *   <row_1_element_1> <row_1_element_2> ... <row_1_element_cols>
 *   <row_2_element_1> <row_2_element_2> ... <row_2_element_cols>
 *   ...
 *   <row_rows_element_1> <row_rows_element_2> ... <row_rows_element_cols>
 *
 * @param filename The name of the file to read the matrix from.
 * @return The matrix read from the file.
 */
std::vector<std::vector<int>> readMatrixFromFile(const std::string& filename);

/**
 * @brief Prints a matrix to the standard output.
 *
 * This function prints a matrix to the standard output.
 *
 * @param matrix The matrix to be printed.
 */
void printMatrix(const std::vector<std::vector<int>>& matrix);

#endif // MATRIX_FUNCTIONS_H