#ifndef AI_TOOLS_H
#define AI_TOOLS_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef float VAR;

#define MAX_VALUE 15
#define MIN_VALUE -15

typedef struct {
	void **mat;
	int matM;
	int matN;
} MAT;




//Classic operations on matrix
//void matrixProduct_8(uint8_t **mat1, uint8_t **mat2, uint8_t **matOut, int sizeMat1_x, int sizeMat1_y, int sizeMat2_x, int sizeMat2_y);
//void matrixProduct_32(uint32_t **mat1, uint32_t **mat2, uint32_t **matOut, int sizeMat1_x, int sizeMat1_y, int sizeMat2_x, int sizeMat2_y);
void matrixProduct_float(MAT *mat1, MAT *mat2, MAT *matOut);
//uint8_t** createMatrix_8(int size_x, int size_y);
//uint32_t** createMatrix_32(int size_x, int size_y);
MAT* createMatrix_float(uint16_t m, uint16_t n);
//void printMatrix_8(uint8_t **mat, int matX, int matY);
//void printMatrix_32(uint32_t **mat, int matX, int matY);
void printMatrix_float(MAT *mat);
void printMatrix(MAT *mat, char* name);
//void fillIncValues_8(uint8_t **mat, int size_x,int size_y);
//void fillRandomValues_8(uint8_t **mat, int size_x,int size_y, int min, int max);
//void fillRandomValues_32(uint32_t **mat, int size_x,int size_y, int min, int max);
//void transposeMatrix_32(uint32_t **mat1, uint32_t**matOut,int mat1X, int mat1Y, int matOutX, int matOutY);
void transposeMatrix_float(MAT *mat1, MAT *matOut);
void freeMatrix(MAT *mat);
//Specific operations on matrix
void sigmoid_matrix(MAT *mat1, MAT *matOut);
VAR sigmoid(VAR x);
void dSigmoid_matrix(MAT *mat1, MAT *matOut);
VAR dSigmoid(VAR x);

void limitMatrix(MAT *mat1, MAT *matOut);
void fillRandomValuesFloat(MAT *mat1, int min, int max);
void matrixSubstract(MAT *mat1, MAT *mat2, MAT *matOut);
void matrixAdd(MAT *mat1, MAT *mat2, MAT *matOut);
void matrixMultiply(MAT *mat1, MAT *mat2, MAT *matOut);

#endif

