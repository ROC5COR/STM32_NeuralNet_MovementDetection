#include "ai_tools.h"
#include <math.h>

/*void matrixProduct_8(uint8_t **mat1, uint8_t **mat2, uint8_t **matOut, int sizeMat1_x, int sizeMat1_y, int sizeMat2_x, int sizeMat2_y){
    if(sizeMat1_x != sizeMat2_y){
        matOut = NULL;
        return;
    }

    int i,j,k;
    for(i = 0; i < sizeMat1_y; i++){
        for(j = 0; j < sizeMat2_x; j++){
            matOut[i][j] = 0;
            for(k = 0; k < sizeMat2_y; k++){
                matOut[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}*/
/*void matrixProduct_32(uint32_t **mat1, uint32_t **mat2, uint32_t **matOut, int sizeMat1_x, int sizeMat1_y, int sizeMat2_x, int sizeMat2_y){
    if(sizeMat1_x != sizeMat2_y){
        matOut = NULL;
        return;
    }

    int i,j,k;
    for(i = 0; i < sizeMat1_y; i++){
        for(j = 0; j < sizeMat2_x; j++){
            matOut[i][j] = 0;
            for(k = 0; k < sizeMat2_y; k++){
                matOut[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}*/

void matrixProduct_float(MAT *mat1, MAT *mat2, MAT *matOut){
    if(mat1->matN != mat2->matM){
        printf("Error(Product) : Size won't works(%d,%d) and (%d,%d)\n",mat1->matM,mat1->matN,mat2->matM,mat2->matN);
        matOut = NULL;
        return;
    }

    int i,j,k;
    for(i = 0; i < mat1->matM; i++){
        for(j = 0; j < mat2->matN; j++){
            ((float**)matOut->mat)[i][j] = 0;
            for(k = 0; k < mat2->matM; k++){
                ((float**)matOut->mat)[i][j] += ((float**)mat1->mat)[i][k] * ((float**)mat2->mat)[k][j];
            }
        }
    }
}

/*uint8_t** createMatrix_8(int size_x, int size_y){
    uint8_t **out = NULL;
    out = (uint8_t**)malloc(sizeof(uint8_t*)*size_x);
    if(out == NULL){
        return NULL;
    }
    int i;
    for(i = 0; i < size_y; i++){
        out[i] = (uint8_t*)malloc(sizeof(uint8_t)*size_y);
        if(out[i] == NULL){
            return NULL;
        }
    }
    return out;
}*/

/*uint32_t** createMatrix_32(int size_x, int size_y){
    uint32_t **out = NULL;
    out = (uint32_t**)malloc(sizeof(uint32_t*)*size_x);
    if(out == NULL){
        return NULL;
    }
    int i;
    for(i = 0; i < size_y; i++){
        out[i] = (uint32_t*)malloc(sizeof(uint32_t)*size_y);
        if(out[i] == NULL){
            return NULL;
        }
    }
    return out;
}*/

MAT* createMatrix_float(uint16_t m, uint16_t n){
    MAT *out = (MAT*)malloc(sizeof(MAT));
    VAR **mat = NULL;
    mat = (VAR**)malloc(sizeof(VAR*)*m);
    if(mat == NULL){
        printf("Error while creating matrix\n");
        return NULL;
    }
    int i;
    for(i = 0; i < m; i++){
        mat[i] = (VAR*)malloc(sizeof(VAR)*n);
        if(mat[i] == NULL){
            printf("Error while creating matrix\n");
            return NULL;
        }
        //Code de test (remplissage des données)
        /*int j;
        for(j = 0; j < size_x; j++){
            mat[i][j] = 1.0;
        }*/

    }

    out->mat = (void**)mat;
    out->matM = m;
    out->matN = n;
    return out;
}

/*void printMatrix_8(uint8_t **mat, int matX, int matY){
    int i,j;
    printf("[");
    for(i = 0; i < matY; i++){
        printf("[");
        for(j = 0; j < matX; j++){
            printf(" %d ",mat[i][j]);
        }
        printf("]\n");
    }
    printf("]\n\n");
}*/

/*void printMatrix_32(uint32_t **mat, int matX, int matY){
    int i,j;
    printf("{");
    for(i = 0; i < matY; i++){
        printf("{");
        for(j = 0; j < matX; j++){
        	if(j == matX -1){
        		printf("%d",mat[i][j]);
        	}
        	else{
        		printf("%d,",mat[i][j]);
        	}
        }
        if(i == matY -1){
        	printf("}}\n\n");
        }
        else{
        	printf("},\n");
        }

    }
    //printf("}\n\n");
}*/

void printMatrix_float(MAT *mat){
    int i,j;
    printf("{");
    for(i = 0; i < mat->matM; i++){
        printf("{");
        for(j = 0; j < mat->matN; j++){
            if(j == mat->matN -1){
                printf("%f",((VAR**)mat->mat)[i][j]);
            }
            else{
                printf("%f,",((VAR**)mat->mat)[i][j]);
            }
        }
        if(i == mat->matM -1){
            printf("}}\n\n");
        }
        else{
            printf("},\n");
        }
    }
}

void printMatrix(MAT *mat, char *name){
    printf("%s\n",name);
    printMatrix_float(mat);
}

/*void fillIncValues_8(uint8_t **mat, int size_x,int size_y){
    int i,j;
    uint8_t k = 1;
    for(i = 0; i < size_y; i++){
        for(j = 0; j < size_x; j++){
            mat[i][j] = k;
            k++;
        }
    }
}

void fillRandomValues_8(uint8_t **mat, int size_x,int size_y, int min, int max){
    int i,j;
    for(i = 0; i < size_y; i++){
        for(j = 0; j < size_x; j++){
            mat[i][j] = rand() % max-min  + min;
        }
    }
}

void fillRandomValues_32(uint32_t **mat, int size_x,int size_y, int min, int max){
    int i,j;
    for(i = 0; i < size_y; i++){
        for(j = 0; j < size_x; j++){
            mat[i][j] = rand() % max-min  + min;
        }
    }
}*/

/*void transposeMatrix_32(uint32_t **mat1, uint32_t**matOut, int mat1X, int mat1Y, int matOutX, int matOutY){
	if(mat1 == matOut){
		printf("Warning same matrix passed as parameter\n");
	}
	if(mat1X != matOutY || mat1Y != matOutX){
		printf("Size error\n");
		return;
	}

	int i,j;
	for(i = 0; i < mat1Y; i++){
		for(j = 0; j < mat1X; j++){
			matOut[j][i] = mat1[i][j];
		}
	}
}*/

void transposeMatrix_float(MAT *mat1, MAT *matOut){
    if(mat1 == matOut){
        printf("Warning same matrix passed as parameter\n");
    }
    if(mat1->matM != matOut->matN || mat1->matN != matOut->matM){
        printf("Size error\n");
        return;
    }

    int i,j;
    for(i = 0; i < mat1->matM; i++){
        for(j = 0; j < mat1->matN; j++){
            ((float**)matOut->mat)[j][i] = ((float**)mat1->mat)[i][j];
        }
    }
}

void freeMatrix(MAT *mat){
    int i;
    for(i = 0; i < mat->matM; i++){
        free(mat->mat[i]);
    }
}


//Specific operations

void sigmoid_matrix(MAT *mat1, MAT *matOut){
    int i,j;
    for(i = 0; i < mat1->matM; i++){
        for(j = 0; j < mat1->matN;j++){
            ((float**)matOut->mat)[i][j] = sigmoid(((float**)mat1->mat)[i][j]);
        }
    }
}


VAR sigmoid(VAR x){
    return 1/(1+exp(-x));
}

void dSigmoid_matrix(MAT *mat1, MAT *matOut){
    int i,j;
    for(i = 0; i < mat1->matM; i++){
        for(j = 0; j < mat1->matN;j++){
            ((float**)matOut->mat)[i][j] = dSigmoid(((float**)mat1->mat)[i][j]);
        }
    }
}

VAR dSigmoid(VAR x){
    return exp(x)/pow(exp(x)+1,2);
}

void limitMatrix(MAT *mat1, MAT *matOut){
    int i,j;
    for(i = 0; i < mat1->matM; i++){
        for(j = 0; j < mat1->matN;j++){
            if (((VAR**)mat1->mat)[i][j] > MAX_VALUE){
                ((VAR**)matOut->mat)[i][j] = MAX_VALUE;
            }

            else if (((VAR**)mat1->mat)[i][j] < MIN_VALUE){
                ((VAR**)matOut->mat)[i][j] = MIN_VALUE;
            }
            else{
                ((VAR**)matOut->mat)[i][j] = ((VAR**)mat1->mat)[i][j];
            }

        }
    }
}

void fillRandomValuesFloat(MAT *mat1, int min, int max){
    int i,j;
    for(i = 0; i < mat1->matM; i++){
        for(j = 0; j < mat1->matN; j++){
            ((VAR**)mat1->mat)[i][j] = ((VAR)rand()/(VAR)(RAND_MAX))*(max-min)+min;
        }
    }
}


void matrixSubstract(MAT *mat1, MAT *mat2, MAT *matOut){
    if(mat1->matM != mat2->matM || mat2->matM != matOut->matM ||
        mat1->matN != mat2->matN || mat2->matN != matOut->matN)
    {
        printf("Error in matrixSubstract : bad dimension (%d,%d) (%d,%d) (%d,%d)\n",mat1->matM,mat1->matN,mat2->matM,mat2->matN,matOut->matM,matOut->matN);
        return;
    }

    int i,j;
    for(i = 0; i < mat1->matM; i++){
        for(j = 0; j < mat1->matN; j++){
            ((VAR**)matOut->mat)[i][j] = ((VAR**)mat1->mat)[i][j] - ((VAR**)mat2->mat)[i][j];
        }
    }
}

void matrixAdd(MAT *mat1, MAT *mat2, MAT *matOut){
    if(mat1->matM != mat2->matM || mat2->matM != matOut->matM ||
        mat1->matN != mat2->matN || mat2->matN != matOut->matN)
    {
        printf("Error in matrixAdd : bad dimension\n");
        return;
    }

    int i,j;
    for(i = 0; i < mat1->matM; i++){
        for(j = 0; j < mat1->matN; j++){
            ((VAR**)matOut->mat)[i][j] = ((VAR**)mat1->mat)[i][j] + ((VAR**)mat2->mat)[i][j];
        }
    }
}

void matrixMultiply(MAT *mat1, MAT *mat2, MAT *matOut){
    if(mat1->matM != mat2->matM || mat2->matM != matOut->matM ||
        mat1->matN != mat2->matN || mat2->matN != matOut->matN)
    {
        printf("Error in matrixMultiply : bad dimension\n");
        return;
    }

    int i,j;
    for(i = 0; i < mat1->matM; i++){
        for(j = 0; j < mat1->matN; j++){
            ((VAR**)matOut->mat)[i][j] = ((VAR**)mat1->mat)[i][j] * ((VAR**)mat2->mat)[i][j];
        }
    }
}

uint16_t float_to_hexa(float input){
	if(input > 15){input = 15;}
	if(input < -15){input = -15;}

	int sign = 0x0;
	if(input < 0){
		sign = 0x1;
	}
	uint16_t int_part = floor(input);
	uint16_t frac_part = (input - (float)int_part)*2048;
	//printf("Conversion ent(hexa):%02x ; frac(hexa):%04x\n",int_part,frac_part);
	//printf("Conversion ent(int):%u ; frac(int):%u\n",int_part,frac_part);

	uint16_t total = 0x0000;
	total |= sign<<15;
	total |= int_part<<11;
	total |= frac_part;
	printf("Total (hexa): %05x ; Total(int):%d\n",total,total);

	return total;
}

