#ifndef AI_TOOLS_H
#define AI_TOOLS_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

typedef float VAR;


#define MAX_VALUE 15
#define MIN_VALUE -15
#define NORM_UP 150
#define NORM_DIVIDE 300

typedef struct {
	void **mat;
	int matM;
	int matN;
} MAT;




//Classic operations on matrix
void matrixProduct_float(MAT *mat1, MAT *mat2, MAT *matOut);

MAT* createMatrix_float(uint16_t m, uint16_t n);

void printMatrix_float(MAT *mat);
void printMatrix(MAT *mat, char* name);

void transposeMatrix_float(MAT *mat1, MAT *matOut);
void freeMatrix(MAT *mat);

void sigmoid_matrix(MAT *mat1, MAT *matOut);
VAR sigmoid(VAR x);
void dSigmoid_matrix(MAT *mat1, MAT *matOut);
VAR dSigmoid(VAR x);

void limitMatrix(MAT *mat1, MAT *matOut);
void fillRandomValuesFloat(MAT *mat1, int min, int max);
void matrixSubstract(MAT *mat1, MAT *mat2, MAT *matOut);
void matrixAdd(MAT *mat1, MAT *mat2, MAT *matOut);
void matrixMultiply(MAT *mat1, MAT *mat2, MAT *matOut);

float normalize(float a);

uint16_t float_to_hexa(float input);

#endif

