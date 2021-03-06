/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stm32l4xx_hal_uart.h"
#include <time.h>
#include "ai_tools.h"
#include "ai_data.h"
#include "adxl345.h"
#include "com_functions.h"


#define ACCELERO_I2C hi2c3
#define SAMPLE_SIZE 120
#define INPUT_SIZE 30
#define OUTPUT_SIZE 4
#define INPUT_SIZE_FILTER 150
#define NB_EPOCHS 300
#define HIDDEN_NEURON 30
#define CIRCLE_SAMPLES 40
#define SQUARE_SAMPLES 40
#define TRIANGLE_SAMPLES 40
#define S_SAMPLES 40

#define TRAINING_
#define TESTING_
#define ACCELERO_
#define LOUKA
#define FILTER_
#define FPGA_COM
#define ACCELERO_

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int __io_putchar(int ch)
{
   HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

int _write(int file, char *ptr, int len)
{
  int DataIdx;
  for(DataIdx = 0; DataIdx < len; DataIdx++){__io_putchar(*ptr++);}
  return len;
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);

/* USER CODE BEGIN PFP */



/* Private function prototypes -----------------------------------------------*/
//extern void initialise_monitor_handles(void);

void fill_matrix_input_1();
void fill_matrix_input_2();
void fill_matrix_input_3();
void fill_static_matrix_inputs();
void fill_static_matrix_outputs();
void fill_matrix_syn1();
void fill_matrix_syn2();
void fill_matrix_square();
void fill_matrix_circle();
void fill_matrix_triangle();
void fill_matrix_S();


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
//Testing matrices
MAT *l1;
MAT *l2;
MAT *syn1;
MAT *syn2;
MAT *input;
MAT *input_filter;

MAT *l1_complete;
MAT *l2_complete;
MAT *l2Error_complete;
MAT *l1Error_complete;
MAT *l1Temp_complete;
MAT *l1Transpose_complete;
MAT *syn2Temp_complete;
MAT *inputsTranspose;

//Training matrices (plus testing matrices)
//MAT *inputs;
MAT *l2Error;
MAT *l1Error;
MAT *l1Temp;
MAT *syn2Transpose;
MAT *l1Transpose;
MAT *inputTranspose;
MAT *syn2Temp;
MAT *syn1Temp;
MAT *output;
uint32_t totalTrainingTime = 0;


float circle[CIRCLE_SAMPLES][INPUT_SIZE];
float square[SQUARE_SAMPLES][INPUT_SIZE];
float triangle[TRIANGLE_SAMPLES][INPUT_SIZE];
float S[S_SAMPLES][INPUT_SIZE];

#ifdef TRAINING
float outputs[SAMPLE_SIZE][OUTPUT_SIZE];
float inputs[SAMPLE_SIZE][INPUT_SIZE];
#endif



void doInit(){
	GPIO_InitTypeDef GPIO_InitStruct;
	  __HAL_RCC_GPIOB_CLK_ENABLE();
	  __HAL_RCC_GPIOA_CLK_ENABLE();
	  __HAL_RCC_GPIOC_CLK_ENABLE();
	  //LED
	  GPIO_InitStruct.Pin = GPIO_PIN_5;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  //button
	  GPIO_InitStruct.Pin = GPIO_PIN_13;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_16);
}

void initTestingMatrix(){
  input = createMatrix_float(1,INPUT_SIZE);
  input_filter = createMatrix_float(1,INPUT_SIZE_FILTER*2);
  l1 = createMatrix_float(1,HIDDEN_NEURON);
  l2 = createMatrix_float(1,OUTPUT_SIZE);
  output = createMatrix_float(1,OUTPUT_SIZE);
}
void freeTestingMatrix(){
    free(input);
    free(l1);
    free(l2);
    free(output);
}
void initTrainingSoloMatrix(){
  //inputs = createMatrix_float(120,20);
  //outputs = createMatrix_float(120,4);
    input = createMatrix_float(1,INPUT_SIZE);
    inputTranspose = createMatrix_float(INPUT_SIZE,1);
    syn1Temp = createMatrix_float(INPUT_SIZE,HIDDEN_NEURON);
    l1 = createMatrix_float(1,HIDDEN_NEURON);
    l1Error = createMatrix_float(1,HIDDEN_NEURON);
    l1Transpose = createMatrix_float(HIDDEN_NEURON,1);
    l1Temp = createMatrix_float(1,HIDDEN_NEURON);
    syn2Transpose = createMatrix_float(OUTPUT_SIZE,HIDDEN_NEURON);
    syn2Temp = createMatrix_float(HIDDEN_NEURON,OUTPUT_SIZE);
    l2 = createMatrix_float(1,OUTPUT_SIZE);
    l2Error = createMatrix_float(1,OUTPUT_SIZE);
    output = createMatrix_float(1,OUTPUT_SIZE);
}
void freeTrainingSoloMatrix(){
    free(input);
    free(inputTranspose);
    free(syn1Temp);
    free(l1Error);
    free(l1);
    free(l1Transpose);
    free(l1Temp);
    free(syn2Transpose);
    free(syn2Temp);
    free(l2);
    free(output);
    free(l2Error);
}
void initTrainingMatrix(int nb_training_sample){
    //inputs = createMatrix_float(nb_training_sample,20);
    inputsTranspose = createMatrix_float(INPUT_SIZE,nb_training_sample);
    syn1Temp = createMatrix_float(INPUT_SIZE,HIDDEN_NEURON);
    l1_complete = createMatrix_float(nb_training_sample,HIDDEN_NEURON);
    l1Error_complete = createMatrix_float(nb_training_sample,HIDDEN_NEURON);
    l1Transpose_complete = createMatrix_float(HIDDEN_NEURON,nb_training_sample);
    l1Temp_complete = createMatrix_float(nb_training_sample,HIDDEN_NEURON);
    syn2Transpose = createMatrix_float(OUTPUT_SIZE,HIDDEN_NEURON);
    syn2Temp = createMatrix_float(HIDDEN_NEURON,OUTPUT_SIZE);
    l2_complete = createMatrix_float(nb_training_sample,OUTPUT_SIZE);
    l2Error_complete = createMatrix_float(nb_training_sample,OUTPUT_SIZE);
    //outputs = createMatrix_float(nb_training_sample,4);
}
void freeTrainingMatrix(){
    //free(inputs);
    free(inputsTranspose);
    free(syn1Temp);
    free(l1_complete);
    free(l1Error_complete);
    free(l1Transpose_complete);
    free(l1Temp_complete);
    free(syn2Transpose);
    free(syn2Temp);
    free(l2_complete);
    free(l2Error_complete);
    //free(outputs);
}

void performComputation(){
    matrixProduct_float(input,syn1,l1);
    limitMatrix(l1,l1);
    //printMatrix(l1,"l1 before sigmoid");
    sigmoid_matrix(l1,l1);
    //printMatrix(l1,"l1 after sigmoid");
    matrixProduct_float(l1,syn2,l2);
    limitMatrix(l2,l2);
    //printMatrix(l2,"l2 before final sigmoid");
    sigmoid_matrix(l2,l2);
}

void train(){
    /*if(inputs->matM != outputs->matM){
        printf("Error : Number of inputs and outputs mismatch");
        return;
    }*/
#ifdef TRAINING
    int i;
    for(i = 0; i < SAMPLE_SIZE;i++){//Iterate through data (inputs and output)
        ((VAR**)input->mat)[0] = inputs[i];

        //input->matN = inputs->matN;
        input->matN = INPUT_SIZE;
        input->matM = 1;

        ((VAR**)output->mat)[0] = outputs[i];
        //output->matN = outputs->matN;
        output->matN = OUTPUT_SIZE;
        output->matM = 1;
        //printMatrix(input,(char*)"input");
        //printMatrix(output,(char*)"output");

        matrixProduct_float(input,syn1,l1);
        limitMatrix(l1,l1);
        sigmoid_matrix(l1,l1);
        matrixProduct_float(l1,syn2,l2);
        limitMatrix(l2,l2);
        sigmoid_matrix(l2,l2);

        matrixSubstract(output,l2,l2Error);// output - l2 => l2Error
        limitMatrix(l2Error,l2Error);
        dSigmoid_matrix(l2,l2); // dSigmoid(l2) => l2
        matrixMultiply(l2Error,l2,l2Error); // l2Error * l2 => L2Error
        limitMatrix(l2Error,l2Error);
        transposeMatrix_float(syn2,syn2Transpose);
        matrixProduct_float(l2Error,syn2Transpose,l1Error);
        limitMatrix(l1Error,l1Error);
        dSigmoid_matrix(l1,l1Temp);
        matrixMultiply(l1Error,l1Temp,l1Error);
        limitMatrix(l1Error,l1Error);

        transposeMatrix_float(l1,l1Transpose);
        matrixProduct_float(l1Transpose,l2Error,syn2Temp);
        limitMatrix(syn2Temp,syn2Temp);
        matrixAdd(syn2,syn2Temp,syn2);
        limitMatrix(syn2,syn2);

        transposeMatrix_float(input,inputTranspose);
        matrixProduct_float(inputTranspose,l1Error,syn1Temp);
        matrixAdd(syn1,syn1Temp,syn1);
        limitMatrix(syn1,syn1);
    }
#endif

}

void printMatData(MAT* mat,char *name){
  int i,j;
  printf("data_of_%s(){\n",name);
    for(i = 0; i < mat->matM; i++){
        for(j = 0; j < mat->matN; j++){
                printf("((float**)%s->mat)[%d][%d]=%f;",name,i,j,((VAR**)mat->mat)[i][j]); 
        }
    } 
    printf("\n}\n");
}

void testInputCom(){
  printf("test ongoing...\n");
  init_com();
  float INPUT[30] = {0};
  //init_INPUT(INPUT);
  for(int i = 0; i < 30; i++){
    INPUT[i] = 0.5;
  }
  /*for(int i = 0; i < 30; i++){
      printf("Value %d : %f\n",i,INPUT[i]);
    }*/
  float OUTPUT[4] = {0};

  printf("initialisations effectues!\n");

  send_verif_false();
  wait_for_FPGA_on_S0();
  HAL_Delay(1000);
  send_STM32_start_request();
  HAL_Delay(1000);

  int i,verif;
  float temp_fpga_element = 0;

  for(i = 0; i < 30;){

      send_input_element(INPUT[i]);
      HAL_Delay(500);
      send_STM32_next_input_request();
      HAL_Delay(500);
      printf("Le FPGA a bien recut l'element %d\n",i);
      wait_for_input_ack_FPGA();
      HAL_Delay(500);
      send_STM32_input_ack();
      printf("Sent STM32 ack\n");
      HAL_Delay(500);
      wait_for_verif_input_req_FPGA();
      HAL_Delay(500);
      printf("Le FPGA a renvoye son element %d\n",i);
      temp_fpga_element = read_fpga_input_element();
      HAL_Delay(500);


      printf("Read from pins : %f\n",temp_fpga_element);
      verif=verif_input_element(INPUT[i], temp_fpga_element);
      if (verif==0)
      {
        send_verif_false();
        printf("l'element renvoye est different\n");

      }else{
        send_verif_OK();
        printf("element du FPGA = element du STM32\n");
        i+=1;
      }
  }
  wait_for_end_input_ack_FPGA();
  for(int j = 0 ; j<4;){
    wait_for_output_req_FPGA();
    temp_fpga_element = read_fpga_input_element();
    send_input_element(temp_fpga_element);
    send_STM32_output_ack();
    HAL_Delay(10);
    verif = FPGA_verification_result();
    if (verif==1){
      OUTPUT[j]= temp_fpga_element;
      j+=1;
    }else {
      printf("le STM32 a renvoye au FPGA un element faux \n");
    }
  }
  wait_for_FPGA_end_cycle();
  printf("fucking com is finished !");

}


float getTestingScore(){
    int successCpt = 0;
    int testCpt = 0;
    //Circle
    for(int sample = 30; sample < 40; sample++){
      for(int i = 0; i < INPUT_SIZE; i++){
        ((float**)input->mat)[0][i] = normalize(circle[sample][i]);
      }
      performComputation();
      if(((float**)l2->mat)[0][3] > ((float**)l2->mat)[0][0] &&
          ((float**)l2->mat)[0][3] > ((float**)l2->mat)[0][1] &&
          ((float**)l2->mat)[0][3] > ((float**)l2->mat)[0][2]){
        successCpt++;
      }
      else{
      }
      testCpt++;
    }
    //Square
    for(int sample = 30; sample < 40; sample++){
      for(int i = 0; i < INPUT_SIZE; i++){
        ((float**)input->mat)[0][i] = normalize(square[sample][i]);
      }
      performComputation();
      if(((float**)l2->mat)[0][2] > ((float**)l2->mat)[0][0] &&
            ((float**)l2->mat)[0][2] > ((float**)l2->mat)[0][1] &&
            ((float**)l2->mat)[0][2] > ((float**)l2->mat)[0][3]){
          successCpt++;
      }

      testCpt++;
    }
    //Triangle
    for(int sample = 30; sample < 40; sample++){
        for(int i = 0; i < INPUT_SIZE; i++){
          ((float**)input->mat)[0][i] = normalize(triangle[sample][i]);
        }
        performComputation();
      if(((float**)l2->mat)[0][1] > ((float**)l2->mat)[0][0] &&
              ((float**)l2->mat)[0][1] > ((float**)l2->mat)[0][2] &&
              ((float**)l2->mat)[0][1] > ((float**)l2->mat)[0][3]){
            successCpt++;
        }
        testCpt++;
    }
    //S
    for(int sample = 30; sample < 40; sample++){
        for(int i = 0; i < INPUT_SIZE; i++){
          ((float**)input->mat)[0][i] = normalize(S[sample][i]);
        }
        performComputation();
      if(((float**)l2->mat)[0][0] > ((float**)l2->mat)[0][1] &&
              ((float**)l2->mat)[0][0] > ((float**)l2->mat)[0][2] &&
              ((float**)l2->mat)[0][0] > ((float**)l2->mat)[0][3]){
          successCpt++;
        }

        testCpt++;
    }
    return (float)successCpt/(float)testCpt;
}


void testAllData(){
	    int successCpt = 0;
	    int testCpt = 0;
	    //Circle
	    for(int sample = 30; sample < 40; sample++){
	      for(int i = 0; i < INPUT_SIZE; i++){
	        ((float**)input->mat)[0][i] = normalize(circle[sample][i]);
	      }
	      printf("Sample(%d)\n",sample);
	      performComputation();
	      printMatrix(input, "Input :");

	      if(((float**)l2->mat)[0][3] > ((float**)l2->mat)[0][0] &&
	          ((float**)l2->mat)[0][3] > ((float**)l2->mat)[0][1] &&
	          ((float**)l2->mat)[0][3] > ((float**)l2->mat)[0][2]){
	        printf("Success !\n");
	        successCpt++;
	      }
	      else{
	        printf("Fail !\n");
	      }
	      printMatrix(l2,(char*)"1: Circle ? : ");
	      testCpt++;
	    }
	    //Square
	    for(int sample = 30; sample < 40; sample++){
	      for(int i = 0; i < INPUT_SIZE; i++){
	        ((float**)input->mat)[0][i] = normalize(square[sample][i]);
	      }
	      printf("Sample(%d)\n",sample);
	      performComputation();
	      if(((float**)l2->mat)[0][2] > ((float**)l2->mat)[0][0] &&
	            ((float**)l2->mat)[0][2] > ((float**)l2->mat)[0][1] &&
	            ((float**)l2->mat)[0][2] > ((float**)l2->mat)[0][3]){
	          printf("Success !\n");
	          successCpt++;
	      }
	      else{
	          printf("Fail !\n");
	      }
	      printMatrix(l2,(char*)"2: Square ? : ");
	      testCpt++;
	    }
	    //Triangle
	    for(int sample = 30; sample < 40; sample++){
	          for(int i = 0; i < INPUT_SIZE; i++){
	            ((float**)input->mat)[0][i] = normalize(triangle[sample][i]);
	          }
	          printf("Sample(%d)\n",sample);
	          performComputation();
	        if(((float**)l2->mat)[0][1] > ((float**)l2->mat)[0][0] &&
	              ((float**)l2->mat)[0][1] > ((float**)l2->mat)[0][2] &&
	              ((float**)l2->mat)[0][1] > ((float**)l2->mat)[0][3]){
	            //printf("Success !\n");
	            successCpt++;
	          }
	          else{
	              printf("Fail !\n");
	          }
	          printMatrix(l2,(char*)"3: Triangle ? : ");
	          testCpt++;
	    }
	    //S
	    for(int sample = 30; sample < 40; sample++){
	          for(int i = 0; i < INPUT_SIZE; i++){
	            ((float**)input->mat)[0][i] = normalize(S[sample][i]);
	            printf("Input(%d) : %f\n",sample,normalize(S[sample][i]));
	          }

	          printf("Sample(%d)\n",sample);
	          performComputation();
	        if(((float**)l2->mat)[0][0] > ((float**)l2->mat)[0][1] &&
	              ((float**)l2->mat)[0][0] > ((float**)l2->mat)[0][2] &&
	              ((float**)l2->mat)[0][0] > ((float**)l2->mat)[0][3]){
	          printf("Success !\n");
	          successCpt++;
	          }
	          else{
	              printf("Fail !\n");
	          }
	          printMatrix(l2,(char*)"Result l2");
	          testCpt++;
	    }
	    printf("%d/%d success\n", successCpt, testCpt);

	    printf("Synthesis : \n");
	#ifdef TESTING
	#ifndef TRAINING
	    printf("\t- TESTING_ONLY\n");
	#endif
	#ifdef TRAINING
	    printf("\t- TRAINING_AND_TESTING\n");
	#endif
	#endif
	#ifndef TESTING
	#ifdef TRAINING
	    printf("\t- TRAINING_ONLY\n");
	#endif
	#endif
	    printf("\t- TRAINING_TIME : %d\n",totalTrainingTime);
	    printf("\t- NB_TRAINING_SAMPLES : %d\n", SAMPLE_SIZE);
	    printf("\t- NB_TESTING_SAMPLES : %d\n", testCpt);
	    printf("\t- INPUT_SIZE : %d\n", INPUT_SIZE);
	    printf("\t- NB_HIDDEN_NEURONS : %d\n", HIDDEN_NEURON);
	    printf("\t- NB_OUTPUT_NEURONS : %d\n", OUTPUT_SIZE);
	    printf("\t- NB_EPOCHS : %d\n", NB_EPOCHS);
	    printf("\t- SUCCESS_RATE : %f%%\n", (float)successCpt/(float)testCpt);

	    //printMatrix(syn2, "SYN2 : ");
}



void generateMif(){
	limitMatrix(syn1,syn1);
	    limitMatrix(syn2,syn2);
	   int syn1_int;
	   int syn2_int;

	   printMatrix(syn1,(char*)"syn1");
	   for(int i = 0; i < HIDDEN_NEURON; i++){
	     printf("%d : ",i);
	     for(int j=16-1; j>=0; j--){
	       if( (((float**)syn1->mat)[j][i]) < 0){
	         syn1_int = - (int)(((float**)syn1->mat)[j][i]*2047) + 32768;

	       }
	       else {
	         syn1_int = (int)(((float**)syn1->mat)[j][i]*2047);

	       }

	       printf("%04X",syn1_int);
	     }
	     printf(";\n");
	  }
	     for(int i = 0; i < HIDDEN_NEURON ; i++){
	         printf("%d : ",i);
	         for(int j=30-1; j>=16; j--){
	           if( (((float**)syn1->mat)[j][i]) < 0){
	             syn1_int = -(int)(((float**)syn1->mat)[j][i]*2047) + 32768;
	           }
	           else {
	             syn1_int = (int)(((float**)syn1->mat)[j][i]*2047);
	           }

	           printf("%04X",syn1_int);
	         }
	     printf(";\n");
	   }
	   printf("End\n");
	   printf("Matrice 2\n");
	     printMatrix(syn2,(char*)"syn2");
	     for(int i = 0; i<=3; i++){
	       printf("%d : ",i);
	       for(int j=15-1; j>=0; j--){
	         if( (((float**)syn2->mat)[j][i]) < 0){
	           syn2_int = -(int)(((float**)syn2->mat)[j][i]*2047) + 32768;
	         }
	         else {
	           syn2_int = (int)(((float**)syn2->mat)[j][i]*2047);
	         }

	         printf("%04X",syn2_int);
	       }
	       printf(";\n");
	    }
	     for(int i = 0; i<=3; i++){
	           printf("%d : ",i);
	           for(int j=30-1; j>=15; j--){
	             if( (((float**)syn2->mat)[j][i]) < 0){
	               syn2_int = -(int)(((float**)syn2->mat)[j][i]*2047) + 32768;
	             }
	             else {
	               syn2_int = (int)(((float**)syn2->mat)[j][i]*2047);
	             }

	             printf("%04X",syn2_int);
	           }
	           printf(";\n");
	        }

	     printf("End\n");
}


int getShape(){

	  if(((float**)l2->mat)[0][3] > ((float**)l2->mat)[0][0] &&
		  ((float**)l2->mat)[0][3] > ((float**)l2->mat)[0][1] &&
		  ((float**)l2->mat)[0][3] > ((float**)l2->mat)[0][2]){
		return 1;
	  }


	  if(((float**)l2->mat)[0][2] > ((float**)l2->mat)[0][0] &&
			((float**)l2->mat)[0][2] > ((float**)l2->mat)[0][1] &&
			((float**)l2->mat)[0][2] > ((float**)l2->mat)[0][3]){
		  return 2;
	  }
		if(((float**)l2->mat)[0][1] > ((float**)l2->mat)[0][0] &&
			  ((float**)l2->mat)[0][1] > ((float**)l2->mat)[0][2] &&
			  ((float**)l2->mat)[0][1] > ((float**)l2->mat)[0][3]){
			return 3;
		  }

		if(((float**)l2->mat)[0][0] > ((float**)l2->mat)[0][1] &&
			  ((float**)l2->mat)[0][0] > ((float**)l2->mat)[0][2] &&
			  ((float**)l2->mat)[0][0] > ((float**)l2->mat)[0][3]){
			return 4;
		  }
		return 0;
}

void fillTrainingDataShape(){
#ifdef TRAINING
	printf("Setting data in inputs matrix...\n");
	    //First matrix filling
	    int cpt = 0;
	    for(int i = 0; i < 30; i++){//Take the 30 first samples
	      for(int j = 0; j < INPUT_SIZE; j++){
	        inputs[cpt][j] = normalize(circle[i][j]);//Formatting data
	        inputs[cpt+1][j] = normalize(square[i][j]);//Formatting data
	        inputs[cpt+2][j] = normalize(triangle[i][j]);//Formatting data
	        inputs[cpt+3][j] = normalize(S[i][j]);//Formatting data
	      }
	      outputs[cpt][0] = 0;
	      outputs[cpt][1] = 0;
	      outputs[cpt][2] = 0;
	      outputs[cpt][3] = 1;

	      outputs[cpt+1][0] = 0;
	      outputs[cpt+1][1] = 0;
	      outputs[cpt+1][2] = 1;
	      outputs[cpt+1][3] = 0;

	      outputs[cpt+2][0] = 0;
	      outputs[cpt+2][1] = 1;
	      outputs[cpt+2][2] = 0;
	      outputs[cpt+2][3] = 0;

	      outputs[cpt+3][0] = 1;
	      outputs[cpt+3][1] = 0;
	      outputs[cpt+3][2] = 0;
	      outputs[cpt+3][3] = 0;
	      cpt = cpt + 4;
	    }
	    printf("Matrix : set\n");
#endif
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C3_Init();

  /* USER CODE BEGIN 2 */
  printf("\n\nSTM32 Neural Net implementation\n");

  doInit();//Global init

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

   printf("Init (limit:[%d,%d]); SystemCoreClock:[%ldMHz]; Sizeof VAR:[%d]; INPUT_SIZE:[%d]; HIDDEN_SIZE:[%d]; OUTPUT_SIZE:[%d]\n",MIN_VALUE,MAX_VALUE, SystemCoreClock/1000000,sizeof(VAR), INPUT_SIZE, HIDDEN_NEURON, OUTPUT_SIZE);

   srand(time(NULL));

   syn1 = createMatrix_float(INPUT_SIZE,HIDDEN_NEURON);
   syn2 = createMatrix_float(HIDDEN_NEURON,OUTPUT_SIZE);

   fill_matrix_circle();//Fill data from function to global array
   fill_matrix_square();
   fill_matrix_triangle();
   fill_matrix_S();


#ifdef TRAINING
  fillRandomValuesFloat(syn1,-1,1);
  fillRandomValuesFloat(syn2,-1,1);
  initTrainingSoloMatrix();
  fillTrainingDataShape(); //fill inputs & outputs matrix with data from shape


#ifdef TESTING
    initTestingMatrix();//Create testing matrices
#endif
   uint32_t startTime = HAL_GetTick();
   for(int epochs = 0; epochs < NB_EPOCHS; epochs++){
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
      train();
      printf("Epochs : %d; Performance : %f\n",epochs,getTestingScore());
    }
    uint32_t endTime = HAL_GetTick();

    printf("StartTime : \t%ld\n",startTime);
    printf("EndTime : \t%ld\n",endTime);
    totalTrainingTime = endTime - startTime;
    freeTrainingSoloMatrix();
#ifdef TESTING
    freeTestingMatrix();
#endif
#endif

#ifdef LOUKA
#ifndef TRAINING
    fillRandomValuesFloat(syn1,-1,1);
    fillRandomValuesFloat(syn2,-1,1);
#endif
    printf("Hola\n");
    //generateMif(); //Generate file for memory of the FPGA
    init_com();
    reset_all_Data_outputs();

    setModeResetOnPort();
    HAL_Delay(1000);
    setMode3OnPort();
    HAL_Delay(1000);
    setMode0OnPort();
    HAL_Delay(1000);

    for(int i = 0; i < INPUT_SIZE; i = i + 2){
    	printf("Iteration no %d\n",i);
    	printf("valeur non nor %f",square[30][i]);
    	setDataOnPort(normalize(square[30][i]));
    	setMode1OnPort();
    	waitForMode1OnPort();
    	printf("Iteration no %d\n",i+1);
    	printf("valeur non nor %f",square[30][i+1]);
		setDataOnPort(normalize(square[30][i+1]));
		setMode2OnPort();
		waitForMode2OnPort();
    }

    printf("Communication Ended !\n");

    //testDataOut();

#endif

#ifdef ACCELERO

    //initTestingMatrix();
    ADXL345_Init(&ACCELERO_I2C);
    HAL_Delay(10);
    printf("Device ID : 0x%02x\n", getDeviceID(&ACCELERO_I2C));
    HAL_Delay(10);
    setDataFormatControl(&ACCELERO_I2C,0x0B);
    HAL_Delay(10);
    setDataRate(&ACCELERO_I2C,ADXL345_3200HZ);
    HAL_Delay(10);
    setPowerControl(&ACCELERO_I2C,MeasurementMode);
    HAL_Delay(10);


#endif


#ifdef TESTING
#ifndef TRAINING
    printf("Loading data of synapses\n");
    fill_matrix_syn1();
    fill_matrix_syn2();
#endif
#endif
#ifdef TESTING
    printf("Testing...\n");
    initTestingMatrix();

    //Testing with new data
    for(int i = 0; i < INPUT_SIZE; i++){
      ((float**)input->mat)[0][i] = normalize(circle[30][i]);
    }
    uint32_t startTimeTesting = HAL_GetTick();
    performComputation();
    uint32_t endTimeTesting = HAL_GetTick();
    printMatrix(l2,(char*)"1: Circle ? : ");
    printf("DeltaTick : %ld\n",endTimeTesting-startTimeTesting);

    testAllData(); //Test all the testing set


    //printf("Test FPGA rapidos\n");
    //init_com();
    //show_fpga_mode()


#endif
#ifdef TRAINING
    //printMatData(syn1,(char*)"syn1");
    //printMatData(syn2,(char*)"syn2");
#endif

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
#ifdef FPGA_COM
    testInputCom();
#endif

while (1)
 {
  if (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13) == GPIO_PIN_RESET){
#ifdef FILTER
    HAL_Delay(100);
    filter_acc();
    performComputation();
    printMatrix(l2,(char*)"Resultats : ");

#endif

  }
  }
freeTestingMatrix();
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C3;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C3 init function */
static void MX_I2C3_Init(void)
{

  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00000E14;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }


    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Digital filter 
    */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}
// User Code
void filter_acc(){
	  printf("Start\n");
	  //uint32_t i2c_startTime = HAL_GetTick();
	 // i2c_startTime = HAL_GetTick();
	  int i = 0;
	  int j = 0;
	  int k = 1;

	  int16_t i2c_data[3] = {0, 0,0};
	  int16_t i2c_data_temp[3] ={0,0,0};

	  float i2c_data_filter[1][INPUT_SIZE_FILTER*2];
	  float i2c_data_pre_output[1][INPUT_SIZE_FILTER*2];
	  float i2c_data_output[1][INPUT_SIZE_FILTER*2];
	  float i2c_data_testing[1][INPUT_SIZE];

	  while(i<INPUT_SIZE_FILTER){
		  while (j<30){
			  getOutput(&ACCELERO_I2C,i2c_data);
			  i2c_data_temp[0]+=i2c_data[0];
			  i2c_data_temp[1]+=i2c_data[1];
			  j++;

		  }

		  i2c_data_filter[0][2*i] = (float)i2c_data_temp[0]/30.0;
		  i2c_data_filter[0][1+2*i] = (float)i2c_data_temp[1]/30.0;

		  //printf("X:%d, Y:%d\n",i2c_data_filter[0][2*i],i2c_data_filter[0][1+2*i]);
		  //((float**)input->mat)[0][2+3*i] =  ((float**)input->mat)[0][2+3*i]/10;
		  i++;
		  j=0;
		  i2c_data_temp[0]=0;
		  i2c_data_temp[1]=0;
		  HAL_Delay(10);

	  }
	   // printf("Fin acquisition\n");
	   // printf("delai : %u\n",HAL_GetTick() - i2c_startTime);
	  //printMatrix(input_pre_filter,(char*)"pre_filtre");
      //i2c_data_pre_filter[0][0]=i2c_data_filter[0][0];
      //i2c_data_pre_filter[0][1]=i2c_data_filter[0][1];
		i2c_data_output[0][0]=i2c_data_filter[0][0];
		i2c_data_output[0][1]=i2c_data_filter[0][1];

		i2c_data_pre_output[0][0]=i2c_data_filter[0][0];
		i2c_data_pre_output[0][1]=i2c_data_filter[0][1];

		i2c_data_testing[0][0]=i2c_data_filter[0][0];
		i2c_data_testing[0][1]=i2c_data_filter[0][1];

		((float**)input->mat)[0][0]=normalize(i2c_data_testing[0][0]);
		((float**)input->mat)[0][1]=normalize(i2c_data_testing[0][1]);

		//printf("suppression offset\n");
		//((float**)input->mat)[0][0]=0;
		//((float**)input->mat)[0][1]=0;
//		((float**)input_filter->mat)[0][0]=i2c_data_filter[0][0];
//		((float**)input_filter->mat)[0][1]=i2c_data_filter[0][1];

		//printf("Data\n");
//		printf("%f\n",((float**)input_filter->mat)[0][0]);
//		printf("%f\n",((float**)input_filter->mat)[0][1]);
		printf("%f\n",i2c_data_output[0][0]);
		printf("%f\n",i2c_data_output[0][1]);

		for(i=1;i<INPUT_SIZE_FILTER;i++){


//		  i2c_data_pre_filter[0][2*i]=i2c_data_filter[0][2*i]*0.1+i2c_data_pre_filter[0][2*(i-1)]*0.9;
//		  i2c_data_pre_filter[0][1+2*i]=i2c_data_filter[0][1+2*i]*0.1+i2c_data_pre_filter[0][1+2*(i-1)]*0.9;
//
//		  i2c_data_output[0][2*i]=i2c_data_filter[0][2*i]-i2c_data_pre_filter[0][2*i];
//		  i2c_data_output[0][1+2*i]=i2c_data_filter[0][1+2*i]-i2c_data_pre_filter[0][1+2*i];

			i2c_data_pre_output[0][2*i]=i2c_data_filter[0][2*i]*0.1+i2c_data_pre_output[0][2*(i-1)]*0.9;
			i2c_data_pre_output[0][1+2*i]=i2c_data_filter[0][1+2*i]*0.1+i2c_data_pre_output[0][1+2*(i-1)]*0.9;

			i2c_data_output[0][2*i]=i2c_data_pre_output[0][2*i]*0.1+i2c_data_output[0][2*(i-1)]*0.9;
			i2c_data_output[0][1+2*i]=i2c_data_pre_output[0][1+2*i]*0.1+i2c_data_output[0][1+2*(i-1)]*0.9;

			if(i%10==0){
				i2c_data_testing[0][2*k]=i2c_data_output[0][2*i];
				i2c_data_testing[0][1+2*k]=i2c_data_output[0][1+2*i];

				((float**)input->mat)[0][2*k]=normalize(i2c_data_testing[0][2*k]);
				((float**)input->mat)[0][2*k+1]=normalize(i2c_data_testing[0][1+2*k]);

//				printf("\n%d\n",i2c_data_testing[0][2*k]);
//				printf("%d\n\n",i2c_data_testing[0][1+2*k]);
//				printf("\n\n%d\n\n",k);
//				printf("\n\n%d\n\n",i);
				k++;

			}

//		  ((float**)input_filter->mat)[0][2*i]=i2c_data_output[0][2*i];
//		  ((float**)input_filter->mat)[0][2*i+1]=i2c_data_output[0][1+2*i];

		  //printf("Data\n");
//		  printf("%f\n",((float**)input_filter->mat)[0][2*i]);
//		  printf("%f\n",((float**)input_filter->mat)[0][1+2*i]);
		  printf("%f\n",i2c_data_output[0][2*i]);
		  printf("%f\n",i2c_data_output[0][1+2*i]);



	  }

			printMatrix(input,(char*)"input : ");

}


void fill_matrix_syn1(){
  ((float**)syn1->mat)[0][0]=0.367188;((float**)syn1->mat)[0][1]=0.308767;((float**)syn1->mat)[0][2]=-0.932239;((float**)syn1->mat)[0][3]=0.426020;((float**)syn1->mat)[0][4]=-0.689382;((float**)syn1->mat)[0][5]=-0.045504;((float**)syn1->mat)[0][6]=-0.303856;((float**)syn1->mat)[0][7]=-0.908460;((float**)syn1->mat)[0][8]=-0.678055;((float**)syn1->mat)[0][9]=0.057900;((float**)syn1->mat)[0][10]=-0.262646;((float**)syn1->mat)[0][11]=-1.152253;((float**)syn1->mat)[0][12]=0.586572;((float**)syn1->mat)[0][13]=-0.121843;((float**)syn1->mat)[0][14]=-0.447770;((float**)syn1->mat)[0][15]=0.665402;((float**)syn1->mat)[0][16]=-0.239576;((float**)syn1->mat)[0][17]=-0.111321;((float**)syn1->mat)[0][18]=-0.057618;((float**)syn1->mat)[0][19]=0.638070;((float**)syn1->mat)[0][20]=-1.917156;((float**)syn1->mat)[0][21]=-0.043421;((float**)syn1->mat)[0][22]=-0.226679;((float**)syn1->mat)[0][23]=-0.098175;((float**)syn1->mat)[0][24]=-1.153359;((float**)syn1->mat)[0][25]=-1.064909;((float**)syn1->mat)[0][26]=0.168795;((float**)syn1->mat)[0][27]=0.064330;((float**)syn1->mat)[0][28]=0.790452;((float**)syn1->mat)[0][29]=0.542309;((float**)syn1->mat)[1][0]=-0.955765;((float**)syn1->mat)[1][1]=-1.337916;((float**)syn1->mat)[1][2]=-1.129155;((float**)syn1->mat)[1][3]=-0.112532;((float**)syn1->mat)[1][4]=-0.405269;((float**)syn1->mat)[1][5]=-0.257340;((float**)syn1->mat)[1][6]=-0.040699;((float**)syn1->mat)[1][7]=-0.034245;((float**)syn1->mat)[1][8]=-0.857229;((float**)syn1->mat)[1][9]=-1.754122;((float**)syn1->mat)[1][10]=-0.498496;((float**)syn1->mat)[1][11]=-0.201936;((float**)syn1->mat)[1][12]=-0.665075;((float**)syn1->mat)[1][13]=0.310980;((float**)syn1->mat)[1][14]=-0.917041;((float**)syn1->mat)[1][15]=-0.953571;((float**)syn1->mat)[1][16]=-1.318214;((float**)syn1->mat)[1][17]=-0.014023;((float**)syn1->mat)[1][18]=0.084096;((float**)syn1->mat)[1][19]=-0.403105;((float**)syn1->mat)[1][20]=-1.478081;((float**)syn1->mat)[1][21]=-1.198099;((float**)syn1->mat)[1][22]=-0.365159;((float**)syn1->mat)[1][23]=-1.030663;((float**)syn1->mat)[1][24]=-0.172533;((float**)syn1->mat)[1][25]=-0.333745;((float**)syn1->mat)[1][26]=-0.630794;((float**)syn1->mat)[1][27]=0.006551;((float**)syn1->mat)[1][28]=-0.220104;((float**)syn1->mat)[1][29]=0.782100;((float**)syn1->mat)[2][0]=-0.601977;((float**)syn1->mat)[2][1]=-0.028436;((float**)syn1->mat)[2][2]=0.689397;((float**)syn1->mat)[2][3]=-0.523525;((float**)syn1->mat)[2][4]=0.386569;((float**)syn1->mat)[2][5]=-0.847309;((float**)syn1->mat)[2][6]=-0.727880;((float**)syn1->mat)[2][7]=-0.779954;((float**)syn1->mat)[2][8]=-0.144809;((float**)syn1->mat)[2][9]=-0.799825;((float**)syn1->mat)[2][10]=0.669304;((float**)syn1->mat)[2][11]=0.038914;((float**)syn1->mat)[2][12]=-0.907612;((float**)syn1->mat)[2][13]=-0.159418;((float**)syn1->mat)[2][14]=-0.253317;((float**)syn1->mat)[2][15]=-0.712453;((float**)syn1->mat)[2][16]=-1.611945;((float**)syn1->mat)[2][17]=0.247788;((float**)syn1->mat)[2][18]=-1.162335;((float**)syn1->mat)[2][19]=-1.533229;((float**)syn1->mat)[2][20]=-1.779274;((float**)syn1->mat)[2][21]=-0.524113;((float**)syn1->mat)[2][22]=-0.213358;((float**)syn1->mat)[2][23]=-0.617440;((float**)syn1->mat)[2][24]=1.799525;((float**)syn1->mat)[2][25]=0.652356;((float**)syn1->mat)[2][26]=-0.050537;((float**)syn1->mat)[2][27]=-0.852286;((float**)syn1->mat)[2][28]=0.678255;((float**)syn1->mat)[2][29]=-1.587747;((float**)syn1->mat)[3][0]=0.095727;((float**)syn1->mat)[3][1]=-0.241958;((float**)syn1->mat)[3][2]=-0.917001;((float**)syn1->mat)[3][3]=-0.172275;((float**)syn1->mat)[3][4]=-0.464736;((float**)syn1->mat)[3][5]=0.421850;((float**)syn1->mat)[3][6]=-0.191414;((float**)syn1->mat)[3][7]=-0.408545;((float**)syn1->mat)[3][8]=-0.698512;((float**)syn1->mat)[3][9]=-0.252260;((float**)syn1->mat)[3][10]=-1.351527;((float**)syn1->mat)[3][11]=0.838643;((float**)syn1->mat)[3][12]=-1.391320;((float**)syn1->mat)[3][13]=0.687120;((float**)syn1->mat)[3][14]=-0.178210;((float**)syn1->mat)[3][15]=-1.543661;((float**)syn1->mat)[3][16]=-0.702914;((float**)syn1->mat)[3][17]=1.712442;((float**)syn1->mat)[3][18]=0.650916;((float**)syn1->mat)[3][19]=-0.319062;((float**)syn1->mat)[3][20]=-1.824418;((float**)syn1->mat)[3][21]=-1.234098;((float**)syn1->mat)[3][22]=0.324038;((float**)syn1->mat)[3][23]=0.593789;((float**)syn1->mat)[3][24]=0.344051;((float**)syn1->mat)[3][25]=-2.276468;((float**)syn1->mat)[3][26]=-1.183547;((float**)syn1->mat)[3][27]=-0.054332;((float**)syn1->mat)[3][28]=0.009721;((float**)syn1->mat)[3][29]=-0.481633;((float**)syn1->mat)[4][0]=1.072602;((float**)syn1->mat)[4][1]=-1.561460;((float**)syn1->mat)[4][2]=0.777062;((float**)syn1->mat)[4][3]=-2.251061;((float**)syn1->mat)[4][4]=0.331770;((float**)syn1->mat)[4][5]=-0.587370;((float**)syn1->mat)[4][6]=-1.339770;((float**)syn1->mat)[4][7]=0.872610;((float**)syn1->mat)[4][8]=-0.644661;((float**)syn1->mat)[4][9]=-0.567903;((float**)syn1->mat)[4][10]=0.509627;((float**)syn1->mat)[4][11]=-0.336188;((float**)syn1->mat)[4][12]=-1.937311;((float**)syn1->mat)[4][13]=0.749832;((float**)syn1->mat)[4][14]=-0.539254;((float**)syn1->mat)[4][15]=2.718366;((float**)syn1->mat)[4][16]=-0.078179;((float**)syn1->mat)[4][17]=-2.851450;((float**)syn1->mat)[4][18]=-2.920056;((float**)syn1->mat)[4][19]=-2.099129;((float**)syn1->mat)[4][20]=-1.949721;((float**)syn1->mat)[4][21]=-0.952331;((float**)syn1->mat)[4][22]=-0.180823;((float**)syn1->mat)[4][23]=-1.976814;((float**)syn1->mat)[4][24]=-1.050941;((float**)syn1->mat)[4][25]=2.862016;((float**)syn1->mat)[4][26]=0.046453;((float**)syn1->mat)[4][27]=-1.315901;((float**)syn1->mat)[4][28]=0.109079;((float**)syn1->mat)[4][29]=-3.119328;((float**)syn1->mat)[5][0]=-0.387531;((float**)syn1->mat)[5][1]=-1.059578;((float**)syn1->mat)[5][2]=0.430480;((float**)syn1->mat)[5][3]=-4.414385;((float**)syn1->mat)[5][4]=-0.568020;((float**)syn1->mat)[5][5]=-0.654049;((float**)syn1->mat)[5][6]=0.370359;((float**)syn1->mat)[5][7]=-0.035327;((float**)syn1->mat)[5][8]=-1.766105;((float**)syn1->mat)[5][9]=-0.683202;((float**)syn1->mat)[5][10]=1.017918;((float**)syn1->mat)[5][11]=0.024146;((float**)syn1->mat)[5][12]=-3.126546;((float**)syn1->mat)[5][13]=2.952804;((float**)syn1->mat)[5][14]=-0.921526;((float**)syn1->mat)[5][15]=3.653866;((float**)syn1->mat)[5][16]=-0.601518;((float**)syn1->mat)[5][17]=-1.858756;((float**)syn1->mat)[5][18]=-2.870618;((float**)syn1->mat)[5][19]=-0.726500;((float**)syn1->mat)[5][20]=-2.604851;((float**)syn1->mat)[5][21]=-1.288929;((float**)syn1->mat)[5][22]=1.678318;((float**)syn1->mat)[5][23]=-0.283737;((float**)syn1->mat)[5][24]=-3.372850;((float**)syn1->mat)[5][25]=1.603343;((float**)syn1->mat)[5][26]=0.649701;((float**)syn1->mat)[5][27]=-1.056852;((float**)syn1->mat)[5][28]=1.805546;((float**)syn1->mat)[5][29]=-2.531156;((float**)syn1->mat)[6][0]=-1.969543;((float**)syn1->mat)[6][1]=-1.402161;((float**)syn1->mat)[6][2]=-2.418344;((float**)syn1->mat)[6][3]=1.267951;((float**)syn1->mat)[6][4]=-0.619805;((float**)syn1->mat)[6][5]=-2.744821;((float**)syn1->mat)[6][6]=0.596029;((float**)syn1->mat)[6][7]=3.455079;((float**)syn1->mat)[6][8]=-2.690616;((float**)syn1->mat)[6][9]=-0.261636;((float**)syn1->mat)[6][10]=-1.898802;((float**)syn1->mat)[6][11]=-2.377505;((float**)syn1->mat)[6][12]=-0.688378;((float**)syn1->mat)[6][13]=-0.303027;((float**)syn1->mat)[6][14]=-0.506037;((float**)syn1->mat)[6][15]=2.978821;((float**)syn1->mat)[6][16]=0.765013;((float**)syn1->mat)[6][17]=-3.694183;((float**)syn1->mat)[6][18]=-0.076130;((float**)syn1->mat)[6][19]=-0.996994;((float**)syn1->mat)[6][20]=-1.379387;((float**)syn1->mat)[6][21]=3.107212;((float**)syn1->mat)[6][22]=-0.269906;((float**)syn1->mat)[6][23]=-1.191081;((float**)syn1->mat)[6][24]=-5.171772;((float**)syn1->mat)[6][25]=1.561285;((float**)syn1->mat)[6][26]=2.267515;((float**)syn1->mat)[6][27]=-0.935116;((float**)syn1->mat)[6][28]=-0.008783;((float**)syn1->mat)[6][29]=-0.412365;((float**)syn1->mat)[7][0]=0.447887;((float**)syn1->mat)[7][1]=-0.522777;((float**)syn1->mat)[7][2]=-1.321189;((float**)syn1->mat)[7][3]=0.439376;((float**)syn1->mat)[7][4]=0.253181;((float**)syn1->mat)[7][5]=-2.970041;((float**)syn1->mat)[7][6]=-2.547755;((float**)syn1->mat)[7][7]=0.253079;((float**)syn1->mat)[7][8]=0.209632;((float**)syn1->mat)[7][9]=0.313251;((float**)syn1->mat)[7][10]=-0.332823;((float**)syn1->mat)[7][11]=-3.199840;((float**)syn1->mat)[7][12]=2.337179;((float**)syn1->mat)[7][13]=-1.600931;((float**)syn1->mat)[7][14]=-0.698240;((float**)syn1->mat)[7][15]=0.153586;((float**)syn1->mat)[7][16]=-0.123056;((float**)syn1->mat)[7][17]=-4.069329;((float**)syn1->mat)[7][18]=-1.239719;((float**)syn1->mat)[7][19]=-3.354282;((float**)syn1->mat)[7][20]=-2.150328;((float**)syn1->mat)[7][21]=1.173372;((float**)syn1->mat)[7][22]=-0.482816;((float**)syn1->mat)[7][23]=-2.091071;((float**)syn1->mat)[7][24]=-0.818587;((float**)syn1->mat)[7][25]=4.266350;((float**)syn1->mat)[7][26]=0.281200;((float**)syn1->mat)[7][27]=-0.620814;((float**)syn1->mat)[7][28]=-0.745517;((float**)syn1->mat)[7][29]=0.246225;((float**)syn1->mat)[8][0]=2.825498;((float**)syn1->mat)[8][1]=-0.053037;((float**)syn1->mat)[8][2]=1.383981;((float**)syn1->mat)[8][3]=-1.090849;((float**)syn1->mat)[8][4]=-0.098445;((float**)syn1->mat)[8][5]=-3.342802;((float**)syn1->mat)[8][6]=-2.260713;((float**)syn1->mat)[8][7]=1.522198;((float**)syn1->mat)[8][8]=-1.258475;((float**)syn1->mat)[8][9]=-0.929618;((float**)syn1->mat)[8][10]=1.526427;((float**)syn1->mat)[8][11]=-3.403196;((float**)syn1->mat)[8][12]=0.296186;((float**)syn1->mat)[8][13]=-0.720893;((float**)syn1->mat)[8][14]=0.466164;((float**)syn1->mat)[8][15]=2.715573;((float**)syn1->mat)[8][16]=-1.313609;((float**)syn1->mat)[8][17]=-5.413654;((float**)syn1->mat)[8][18]=-3.940690;((float**)syn1->mat)[8][19]=-6.356352;((float**)syn1->mat)[8][20]=-1.925303;((float**)syn1->mat)[8][21]=0.450129;((float**)syn1->mat)[8][22]=1.358308;((float**)syn1->mat)[8][23]=-3.373739;((float**)syn1->mat)[8][24]=-0.165479;((float**)syn1->mat)[8][25]=9.008444;((float**)syn1->mat)[8][26]=-0.008324;((float**)syn1->mat)[8][27]=-1.582659;((float**)syn1->mat)[8][28]=0.671118;((float**)syn1->mat)[8][29]=-2.298967;((float**)syn1->mat)[9][0]=-0.133919;((float**)syn1->mat)[9][1]=2.528463;((float**)syn1->mat)[9][2]=-5.042021;((float**)syn1->mat)[9][3]=6.972187;((float**)syn1->mat)[9][4]=-1.845311;((float**)syn1->mat)[9][5]=0.596564;((float**)syn1->mat)[9][6]=-2.506840;((float**)syn1->mat)[9][7]=-2.657983;((float**)syn1->mat)[9][8]=2.310641;((float**)syn1->mat)[9][9]=-1.936961;((float**)syn1->mat)[9][10]=-4.503497;((float**)syn1->mat)[9][11]=-0.818087;((float**)syn1->mat)[9][12]=6.729744;((float**)syn1->mat)[9][13]=-5.365981;((float**)syn1->mat)[9][14]=-1.110796;((float**)syn1->mat)[9][15]=-8.974139;((float**)syn1->mat)[9][16]=-0.092450;((float**)syn1->mat)[9][17]=4.864086;((float**)syn1->mat)[9][18]=4.876764;((float**)syn1->mat)[9][19]=2.698344;((float**)syn1->mat)[9][20]=-1.255061;((float**)syn1->mat)[9][21]=-0.314539;((float**)syn1->mat)[9][22]=-2.981829;((float**)syn1->mat)[9][23]=3.001445;((float**)syn1->mat)[9][24]=4.131298;((float**)syn1->mat)[9][25]=-5.078699;((float**)syn1->mat)[9][26]=-0.230281;((float**)syn1->mat)[9][27]=3.325876;((float**)syn1->mat)[9][28]=-3.354604;((float**)syn1->mat)[9][29]=6.865123;((float**)syn1->mat)[10][0]=0.089985;((float**)syn1->mat)[10][1]=-1.307876;((float**)syn1->mat)[10][2]=1.911754;((float**)syn1->mat)[10][3]=-3.986424;((float**)syn1->mat)[10][4]=0.708376;((float**)syn1->mat)[10][5]=1.235534;((float**)syn1->mat)[10][6]=0.394472;((float**)syn1->mat)[10][7]=-1.505744;((float**)syn1->mat)[10][8]=-0.230167;((float**)syn1->mat)[10][9]=-0.325779;((float**)syn1->mat)[10][10]=0.387642;((float**)syn1->mat)[10][11]=2.807718;((float**)syn1->mat)[10][12]=-2.129199;((float**)syn1->mat)[10][13]=3.559826;((float**)syn1->mat)[10][14]=0.626501;((float**)syn1->mat)[10][15]=-0.056828;((float**)syn1->mat)[10][16]=-2.606408;((float**)syn1->mat)[10][17]=1.988666;((float**)syn1->mat)[10][18]=-2.732751;((float**)syn1->mat)[10][19]=0.354764;((float**)syn1->mat)[10][20]=-2.140369;((float**)syn1->mat)[10][21]=-2.556457;((float**)syn1->mat)[10][22]=0.624088;((float**)syn1->mat)[10][23]=0.168698;((float**)syn1->mat)[10][24]=1.855521;((float**)syn1->mat)[10][25]=0.629758;((float**)syn1->mat)[10][26]=-0.588426;((float**)syn1->mat)[10][27]=-1.781512;((float**)syn1->mat)[10][28]=1.032027;((float**)syn1->mat)[10][29]=-2.416343;((float**)syn1->mat)[11][0]=-0.099707;((float**)syn1->mat)[11][1]=0.496193;((float**)syn1->mat)[11][2]=-1.008909;((float**)syn1->mat)[11][3]=1.638395;((float**)syn1->mat)[11][4]=-0.655428;((float**)syn1->mat)[11][5]=3.947230;((float**)syn1->mat)[11][6]=-1.678989;((float**)syn1->mat)[11][7]=-5.866402;((float**)syn1->mat)[11][8]=3.810836;((float**)syn1->mat)[11][9]=-2.518312;((float**)syn1->mat)[11][10]=-2.152840;((float**)syn1->mat)[11][11]=4.150014;((float**)syn1->mat)[11][12]=5.052128;((float**)syn1->mat)[11][13]=-1.318914;((float**)syn1->mat)[11][14]=-0.708215;((float**)syn1->mat)[11][15]=-9.199465;((float**)syn1->mat)[11][16]=-2.012345;((float**)syn1->mat)[11][17]=7.570587;((float**)syn1->mat)[11][18]=1.466326;((float**)syn1->mat)[11][19]=4.322157;((float**)syn1->mat)[11][20]=-2.141690;((float**)syn1->mat)[11][21]=-3.041611;((float**)syn1->mat)[11][22]=-2.429954;((float**)syn1->mat)[11][23]=3.228515;((float**)syn1->mat)[11][24]=7.400045;((float**)syn1->mat)[11][25]=-7.385340;((float**)syn1->mat)[11][26]=-3.571371;((float**)syn1->mat)[11][27]=1.793165;((float**)syn1->mat)[11][28]=-3.424823;((float**)syn1->mat)[11][29]=1.746420;((float**)syn1->mat)[12][0]=-5.889691;((float**)syn1->mat)[12][1]=0.694841;((float**)syn1->mat)[12][2]=-4.987969;((float**)syn1->mat)[12][3]=3.805990;((float**)syn1->mat)[12][4]=-3.522774;((float**)syn1->mat)[12][5]=4.158938;((float**)syn1->mat)[12][6]=2.402666;((float**)syn1->mat)[12][7]=-1.849437;((float**)syn1->mat)[12][8]=3.816550;((float**)syn1->mat)[12][9]=-1.373761;((float**)syn1->mat)[12][10]=-5.389899;((float**)syn1->mat)[12][11]=3.151396;((float**)syn1->mat)[12][12]=1.662772;((float**)syn1->mat)[12][13]=-0.939691;((float**)syn1->mat)[12][14]=1.092443;((float**)syn1->mat)[12][15]=-8.540187;((float**)syn1->mat)[12][16]=-0.343920;((float**)syn1->mat)[12][17]=8.949526;((float**)syn1->mat)[12][18]=7.533172;((float**)syn1->mat)[12][19]=9.474415;((float**)syn1->mat)[12][20]=-1.424318;((float**)syn1->mat)[12][21]=-0.189127;((float**)syn1->mat)[12][22]=-2.603393;((float**)syn1->mat)[12][23]=6.238516;((float**)syn1->mat)[12][24]=1.807989;((float**)syn1->mat)[12][25]=-14.943494;((float**)syn1->mat)[12][26]=-0.620633;((float**)syn1->mat)[12][27]=2.451829;((float**)syn1->mat)[12][28]=-2.452559;((float**)syn1->mat)[12][29]=4.964991;((float**)syn1->mat)[13][0]=-0.052544;((float**)syn1->mat)[13][1]=-0.447450;((float**)syn1->mat)[13][2]=-0.752218;((float**)syn1->mat)[13][3]=-0.512438;((float**)syn1->mat)[13][4]=-1.363035;((float**)syn1->mat)[13][5]=1.900155;((float**)syn1->mat)[13][6]=-1.345973;((float**)syn1->mat)[13][7]=-2.337425;((float**)syn1->mat)[13][8]=1.989010;((float**)syn1->mat)[13][9]=-2.139271;((float**)syn1->mat)[13][10]=-1.165869;((float**)syn1->mat)[13][11]=0.731574;((float**)syn1->mat)[13][12]=1.227493;((float**)syn1->mat)[13][13]=-1.298626;((float**)syn1->mat)[13][14]=-0.356441;((float**)syn1->mat)[13][15]=-3.365300;((float**)syn1->mat)[13][16]=-2.524584;((float**)syn1->mat)[13][17]=1.402526;((float**)syn1->mat)[13][18]=-0.393090;((float**)syn1->mat)[13][19]=0.504220;((float**)syn1->mat)[13][20]=-2.059527;((float**)syn1->mat)[13][21]=-2.113494;((float**)syn1->mat)[13][22]=-1.545652;((float**)syn1->mat)[13][23]=1.572893;((float**)syn1->mat)[13][24]=3.098589;((float**)syn1->mat)[13][25]=-0.506772;((float**)syn1->mat)[13][26]=-1.856421;((float**)syn1->mat)[13][27]=1.158850;((float**)syn1->mat)[13][28]=-0.077522;((float**)syn1->mat)[13][29]=0.864048;((float**)syn1->mat)[14][0]=-1.830076;((float**)syn1->mat)[14][1]=0.242426;((float**)syn1->mat)[14][2]=-5.188261;((float**)syn1->mat)[14][3]=5.592224;((float**)syn1->mat)[14][4]=-2.490412;((float**)syn1->mat)[14][5]=0.674454;((float**)syn1->mat)[14][6]=-1.284131;((float**)syn1->mat)[14][7]=-0.604127;((float**)syn1->mat)[14][8]=1.590800;((float**)syn1->mat)[14][9]=-1.442553;((float**)syn1->mat)[14][10]=-3.613742;((float**)syn1->mat)[14][11]=-0.338373;((float**)syn1->mat)[14][12]=2.813491;((float**)syn1->mat)[14][13]=-4.063244;((float**)syn1->mat)[14][14]=-0.581114;((float**)syn1->mat)[14][15]=-5.550913;((float**)syn1->mat)[14][16]=0.461094;((float**)syn1->mat)[14][17]=2.965336;((float**)syn1->mat)[14][18]=5.137854;((float**)syn1->mat)[14][19]=2.869709;((float**)syn1->mat)[14][20]=-0.797551;((float**)syn1->mat)[14][21]=1.888424;((float**)syn1->mat)[14][22]=-2.735275;((float**)syn1->mat)[14][23]=1.887664;((float**)syn1->mat)[14][24]=1.156285;((float**)syn1->mat)[14][25]=-7.182751;((float**)syn1->mat)[14][26]=-1.047318;((float**)syn1->mat)[14][27]=2.758986;((float**)syn1->mat)[14][28]=-3.424666;((float**)syn1->mat)[14][29]=4.593750;((float**)syn1->mat)[15][0]=0.848452;((float**)syn1->mat)[15][1]=-0.429965;((float**)syn1->mat)[15][2]=-1.092635;((float**)syn1->mat)[15][3]=1.729243;((float**)syn1->mat)[15][4]=-1.265096;((float**)syn1->mat)[15][5]=-4.050962;((float**)syn1->mat)[15][6]=-2.043085;((float**)syn1->mat)[15][7]=3.167081;((float**)syn1->mat)[15][8]=-2.111465;((float**)syn1->mat)[15][9]=-0.293469;((float**)syn1->mat)[15][10]=-1.213711;((float**)syn1->mat)[15][11]=-5.570889;((float**)syn1->mat)[15][12]=1.128464;((float**)syn1->mat)[15][13]=-1.849904;((float**)syn1->mat)[15][14]=-0.058787;((float**)syn1->mat)[15][15]=4.986012;((float**)syn1->mat)[15][16]=1.076419;((float**)syn1->mat)[15][17]=-7.060944;((float**)syn1->mat)[15][18]=-1.978496;((float**)syn1->mat)[15][19]=-5.210954;((float**)syn1->mat)[15][20]=-1.900289;((float**)syn1->mat)[15][21]=3.372307;((float**)syn1->mat)[15][22]=0.291271;((float**)syn1->mat)[15][23]=-5.410615;((float**)syn1->mat)[15][24]=-2.968846;((float**)syn1->mat)[15][25]=7.390299;((float**)syn1->mat)[15][26]=1.936495;((float**)syn1->mat)[15][27]=-1.582886;((float**)syn1->mat)[15][28]=0.401173;((float**)syn1->mat)[15][29]=0.169722;((float**)syn1->mat)[16][0]=2.350167;((float**)syn1->mat)[16][1]=-0.085043;((float**)syn1->mat)[16][2]=-0.237318;((float**)syn1->mat)[16][3]=2.007858;((float**)syn1->mat)[16][4]=0.043498;((float**)syn1->mat)[16][5]=-0.789451;((float**)syn1->mat)[16][6]=-2.076265;((float**)syn1->mat)[16][7]=-1.176277;((float**)syn1->mat)[16][8]=-0.255692;((float**)syn1->mat)[16][9]=-0.870810;((float**)syn1->mat)[16][10]=-0.013861;((float**)syn1->mat)[16][11]=-3.071907;((float**)syn1->mat)[16][12]=1.634893;((float**)syn1->mat)[16][13]=-2.459786;((float**)syn1->mat)[16][14]=-0.538584;((float**)syn1->mat)[16][15]=-1.038990;((float**)syn1->mat)[16][16]=-0.456531;((float**)syn1->mat)[16][17]=-2.056572;((float**)syn1->mat)[16][18]=-1.405932;((float**)syn1->mat)[16][19]=-4.163850;((float**)syn1->mat)[16][20]=-2.262662;((float**)syn1->mat)[16][21]=-0.648714;((float**)syn1->mat)[16][22]=0.096037;((float**)syn1->mat)[16][23]=-1.735687;((float**)syn1->mat)[16][24]=1.986159;((float**)syn1->mat)[16][25]=5.946536;((float**)syn1->mat)[16][26]=-0.003393;((float**)syn1->mat)[16][27]=-0.124138;((float**)syn1->mat)[16][28]=0.043113;((float**)syn1->mat)[16][29]=-0.317124;((float**)syn1->mat)[17][0]=1.417704;((float**)syn1->mat)[17][1]=-1.509038;((float**)syn1->mat)[17][2]=-0.145751;((float**)syn1->mat)[17][3]=-1.986898;((float**)syn1->mat)[17][4]=0.442144;((float**)syn1->mat)[17][5]=-4.331414;((float**)syn1->mat)[17][6]=-1.100030;((float**)syn1->mat)[17][7]=3.200539;((float**)syn1->mat)[17][8]=-3.380164;((float**)syn1->mat)[17][9]=-1.096738;((float**)syn1->mat)[17][10]=0.816586;((float**)syn1->mat)[17][11]=-3.182842;((float**)syn1->mat)[17][12]=-2.342965;((float**)syn1->mat)[17][13]=-0.787477;((float**)syn1->mat)[17][14]=-1.050159;((float**)syn1->mat)[17][15]=7.207314;((float**)syn1->mat)[17][16]=-0.508491;((float**)syn1->mat)[17][17]=-8.251558;((float**)syn1->mat)[17][18]=-3.551260;((float**)syn1->mat)[17][19]=-7.146995;((float**)syn1->mat)[17][20]=-2.921571;((float**)syn1->mat)[17][21]=2.023108;((float**)syn1->mat)[17][22]=1.294096;((float**)syn1->mat)[17][23]=-5.421794;((float**)syn1->mat)[17][24]=-3.546484;((float**)syn1->mat)[17][25]=9.935602;((float**)syn1->mat)[17][26]=2.493193;((float**)syn1->mat)[17][27]=-3.386883;((float**)syn1->mat)[17][28]=3.130839;((float**)syn1->mat)[17][29]=-2.268148;((float**)syn1->mat)[18][0]=1.641676;((float**)syn1->mat)[18][1]=-1.228229;((float**)syn1->mat)[18][2]=0.867025;((float**)syn1->mat)[18][3]=-0.824113;((float**)syn1->mat)[18][4]=0.124341;((float**)syn1->mat)[18][5]=-0.719394;((float**)syn1->mat)[18][6]=-1.751398;((float**)syn1->mat)[18][7]=0.136312;((float**)syn1->mat)[18][8]=-0.752599;((float**)syn1->mat)[18][9]=-1.205147;((float**)syn1->mat)[18][10]=0.412267;((float**)syn1->mat)[18][11]=0.171665;((float**)syn1->mat)[18][12]=-0.453011;((float**)syn1->mat)[18][13]=0.168113;((float**)syn1->mat)[18][14]=0.477703;((float**)syn1->mat)[18][15]=1.225986;((float**)syn1->mat)[18][16]=-1.254446;((float**)syn1->mat)[18][17]=-0.770435;((float**)syn1->mat)[18][18]=-2.213469;((float**)syn1->mat)[18][19]=-1.704129;((float**)syn1->mat)[18][20]=-1.459524;((float**)syn1->mat)[18][21]=-0.892216;((float**)syn1->mat)[18][22]=-0.656584;((float**)syn1->mat)[18][23]=-1.344858;((float**)syn1->mat)[18][24]=0.608459;((float**)syn1->mat)[18][25]=4.394576;((float**)syn1->mat)[18][26]=-1.089523;((float**)syn1->mat)[18][27]=-0.367250;((float**)syn1->mat)[18][28]=0.405591;((float**)syn1->mat)[18][29]=-0.659980;((float**)syn1->mat)[19][0]=-0.575723;((float**)syn1->mat)[19][1]=-0.923119;((float**)syn1->mat)[19][2]=1.535570;((float**)syn1->mat)[19][3]=-4.760392;((float**)syn1->mat)[19][4]=0.586922;((float**)syn1->mat)[19][5]=-1.708982;((float**)syn1->mat)[19][6]=-0.664923;((float**)syn1->mat)[19][7]=2.148040;((float**)syn1->mat)[19][8]=-1.860240;((float**)syn1->mat)[19][9]=-0.445393;((float**)syn1->mat)[19][10]=1.376825;((float**)syn1->mat)[19][11]=-0.706288;((float**)syn1->mat)[19][12]=-3.472455;((float**)syn1->mat)[19][13]=3.175214;((float**)syn1->mat)[19][14]=0.032646;((float**)syn1->mat)[19][15]=4.989399;((float**)syn1->mat)[19][16]=-1.479388;((float**)syn1->mat)[19][17]=-1.871575;((float**)syn1->mat)[19][18]=-1.739617;((float**)syn1->mat)[19][19]=-2.574401;((float**)syn1->mat)[19][20]=-1.747922;((float**)syn1->mat)[19][21]=-1.196861;((float**)syn1->mat)[19][22]=0.519563;((float**)syn1->mat)[19][23]=-1.163056;((float**)syn1->mat)[19][24]=-3.261311;((float**)syn1->mat)[19][25]=3.569890;((float**)syn1->mat)[19][26]=0.249351;((float**)syn1->mat)[19][27]=-1.322133;((float**)syn1->mat)[19][28]=1.269179;((float**)syn1->mat)[19][29]=-4.137225;((float**)syn1->mat)[20][0]=-0.120774;((float**)syn1->mat)[20][1]=-0.034129;((float**)syn1->mat)[20][2]=-0.032061;((float**)syn1->mat)[20][3]=-1.386310;((float**)syn1->mat)[20][4]=0.466232;((float**)syn1->mat)[20][5]=0.035642;((float**)syn1->mat)[20][6]=-1.152911;((float**)syn1->mat)[20][7]=0.756374;((float**)syn1->mat)[20][8]=-1.432627;((float**)syn1->mat)[20][9]=-1.329962;((float**)syn1->mat)[20][10]=-0.214616;((float**)syn1->mat)[20][11]=-0.265782;((float**)syn1->mat)[20][12]=-1.853217;((float**)syn1->mat)[20][13]=1.085717;((float**)syn1->mat)[20][14]=0.606796;((float**)syn1->mat)[20][15]=1.564911;((float**)syn1->mat)[20][16]=-1.001738;((float**)syn1->mat)[20][17]=-0.966713;((float**)syn1->mat)[20][18]=-1.482537;((float**)syn1->mat)[20][19]=-2.426661;((float**)syn1->mat)[20][20]=-2.400472;((float**)syn1->mat)[20][21]=-0.760581;((float**)syn1->mat)[20][22]=0.680667;((float**)syn1->mat)[20][23]=-0.494950;((float**)syn1->mat)[20][24]=-1.311465;((float**)syn1->mat)[20][25]=1.379417;((float**)syn1->mat)[20][26]=0.526508;((float**)syn1->mat)[20][27]=-0.762714;((float**)syn1->mat)[20][28]=-0.109617;((float**)syn1->mat)[20][29]=-1.699127;((float**)syn1->mat)[21][0]=-1.733528;((float**)syn1->mat)[21][1]=-0.433261;((float**)syn1->mat)[21][2]=-1.261191;((float**)syn1->mat)[21][3]=-1.298950;((float**)syn1->mat)[21][4]=-0.675458;((float**)syn1->mat)[21][5]=-0.907026;((float**)syn1->mat)[21][6]=1.172693;((float**)syn1->mat)[21][7]=0.381859;((float**)syn1->mat)[21][8]=-0.487024;((float**)syn1->mat)[21][9]=-0.249294;((float**)syn1->mat)[21][10]=-2.135659;((float**)syn1->mat)[21][11]=-0.105696;((float**)syn1->mat)[21][12]=-2.130672;((float**)syn1->mat)[21][13]=0.609897;((float**)syn1->mat)[21][14]=0.939165;((float**)syn1->mat)[21][15]=-0.395224;((float**)syn1->mat)[21][16]=0.195496;((float**)syn1->mat)[21][17]=1.599294;((float**)syn1->mat)[21][18]=1.084220;((float**)syn1->mat)[21][19]=1.561423;((float**)syn1->mat)[21][20]=-1.327250;((float**)syn1->mat)[21][21]=0.835368;((float**)syn1->mat)[21][22]=0.430601;((float**)syn1->mat)[21][23]=1.619919;((float**)syn1->mat)[21][24]=-1.524446;((float**)syn1->mat)[21][25]=-4.664369;((float**)syn1->mat)[21][26]=-0.432761;((float**)syn1->mat)[21][27]=-0.691950;((float**)syn1->mat)[21][28]=-0.907928;((float**)syn1->mat)[21][29]=-0.269198;((float**)syn1->mat)[22][0]=-0.047603;((float**)syn1->mat)[22][1]=-0.719766;((float**)syn1->mat)[22][2]=0.770152;((float**)syn1->mat)[22][3]=-1.054050;((float**)syn1->mat)[22][4]=0.315005;((float**)syn1->mat)[22][5]=-1.181283;((float**)syn1->mat)[22][6]=-0.344872;((float**)syn1->mat)[22][7]=0.052878;((float**)syn1->mat)[22][8]=0.042038;((float**)syn1->mat)[22][9]=-1.135541;((float**)syn1->mat)[22][10]=-0.014664;((float**)syn1->mat)[22][11]=-0.636399;((float**)syn1->mat)[22][12]=-1.659669;((float**)syn1->mat)[22][13]=1.193813;((float**)syn1->mat)[22][14]=-0.435199;((float**)syn1->mat)[22][15]=2.175300;((float**)syn1->mat)[22][16]=-0.528866;((float**)syn1->mat)[22][17]=-0.642485;((float**)syn1->mat)[22][18]=-1.276947;((float**)syn1->mat)[22][19]=-0.022137;((float**)syn1->mat)[22][20]=-2.186160;((float**)syn1->mat)[22][21]=-0.336819;((float**)syn1->mat)[22][22]=-0.250905;((float**)syn1->mat)[22][23]=-0.695456;((float**)syn1->mat)[22][24]=-1.148147;((float**)syn1->mat)[22][25]=2.025498;((float**)syn1->mat)[22][26]=-0.304803;((float**)syn1->mat)[22][27]=-1.359773;((float**)syn1->mat)[22][28]=-0.321927;((float**)syn1->mat)[22][29]=-0.989189;((float**)syn1->mat)[23][0]=-1.849989;((float**)syn1->mat)[23][1]=-0.035517;((float**)syn1->mat)[23][2]=-1.110384;((float**)syn1->mat)[23][3]=-0.765550;((float**)syn1->mat)[23][4]=-0.556926;((float**)syn1->mat)[23][5]=0.084838;((float**)syn1->mat)[23][6]=-0.362853;((float**)syn1->mat)[23][7]=0.206316;((float**)syn1->mat)[23][8]=0.468028;((float**)syn1->mat)[23][9]=-2.108485;((float**)syn1->mat)[23][10]=-0.421454;((float**)syn1->mat)[23][11]=-1.116018;((float**)syn1->mat)[23][12]=-0.637785;((float**)syn1->mat)[23][13]=0.794573;((float**)syn1->mat)[23][14]=-0.621264;((float**)syn1->mat)[23][15]=0.531521;((float**)syn1->mat)[23][16]=-0.408146;((float**)syn1->mat)[23][17]=0.619428;((float**)syn1->mat)[23][18]=0.661262;((float**)syn1->mat)[23][19]=2.567806;((float**)syn1->mat)[23][20]=-2.853508;((float**)syn1->mat)[23][21]=0.708856;((float**)syn1->mat)[23][22]=-0.936177;((float**)syn1->mat)[23][23]=0.500358;((float**)syn1->mat)[23][24]=-1.456879;((float**)syn1->mat)[23][25]=-4.419326;((float**)syn1->mat)[23][26]=-0.091629;((float**)syn1->mat)[23][27]=0.578576;((float**)syn1->mat)[23][28]=0.631085;((float**)syn1->mat)[23][29]=-0.029701;((float**)syn1->mat)[24][0]=0.473192;((float**)syn1->mat)[24][1]=-0.894970;((float**)syn1->mat)[24][2]=0.532001;((float**)syn1->mat)[24][3]=-0.501733;((float**)syn1->mat)[24][4]=-0.393707;((float**)syn1->mat)[24][5]=-0.361225;((float**)syn1->mat)[24][6]=-1.046859;((float**)syn1->mat)[24][7]=0.476779;((float**)syn1->mat)[24][8]=-1.345079;((float**)syn1->mat)[24][9]=-1.290049;((float**)syn1->mat)[24][10]=-0.284907;((float**)syn1->mat)[24][11]=0.312864;((float**)syn1->mat)[24][12]=-1.586443;((float**)syn1->mat)[24][13]=-0.167232;((float**)syn1->mat)[24][14]=0.170163;((float**)syn1->mat)[24][15]=0.627591;((float**)syn1->mat)[24][16]=-0.221693;((float**)syn1->mat)[24][17]=-0.876984;((float**)syn1->mat)[24][18]=-0.841598;((float**)syn1->mat)[24][19]=-1.395285;((float**)syn1->mat)[24][20]=-1.388592;((float**)syn1->mat)[24][21]=-0.587615;((float**)syn1->mat)[24][22]=-0.623360;((float**)syn1->mat)[24][23]=-0.353541;((float**)syn1->mat)[24][24]=-0.468200;((float**)syn1->mat)[24][25]=0.696536;((float**)syn1->mat)[24][26]=0.189422;((float**)syn1->mat)[24][27]=-0.772159;((float**)syn1->mat)[24][28]=-0.062385;((float**)syn1->mat)[24][29]=-1.356625;((float**)syn1->mat)[25][0]=-1.259558;((float**)syn1->mat)[25][1]=0.262104;((float**)syn1->mat)[25][2]=-1.116456;((float**)syn1->mat)[25][3]=-0.722563;((float**)syn1->mat)[25][4]=-0.808149;((float**)syn1->mat)[25][5]=-0.086702;((float**)syn1->mat)[25][6]=0.384685;((float**)syn1->mat)[25][7]=-0.410489;((float**)syn1->mat)[25][8]=0.268582;((float**)syn1->mat)[25][9]=-1.551276;((float**)syn1->mat)[25][10]=-0.670757;((float**)syn1->mat)[25][11]=-0.470005;((float**)syn1->mat)[25][12]=-0.938816;((float**)syn1->mat)[25][13]=0.410687;((float**)syn1->mat)[25][14]=0.723485;((float**)syn1->mat)[25][15]=-0.492816;((float**)syn1->mat)[25][16]=0.829642;((float**)syn1->mat)[25][17]=0.925336;((float**)syn1->mat)[25][18]=1.605968;((float**)syn1->mat)[25][19]=1.713753;((float**)syn1->mat)[25][20]=-2.148678;((float**)syn1->mat)[25][21]=-0.325914;((float**)syn1->mat)[25][22]=-1.028803;((float**)syn1->mat)[25][23]=-0.438855;((float**)syn1->mat)[25][24]=-1.618482;((float**)syn1->mat)[25][25]=-4.470872;((float**)syn1->mat)[25][26]=-0.873624;((float**)syn1->mat)[25][27]=-0.820864;((float**)syn1->mat)[25][28]=-0.481692;((float**)syn1->mat)[25][29]=-0.122772;((float**)syn1->mat)[26][0]=-0.339727;((float**)syn1->mat)[26][1]=-1.518319;((float**)syn1->mat)[26][2]=-0.153796;((float**)syn1->mat)[26][3]=-0.984119;((float**)syn1->mat)[26][4]=-0.731682;((float**)syn1->mat)[26][5]=-0.442474;((float**)syn1->mat)[26][6]=-0.419065;((float**)syn1->mat)[26][7]=0.214933;((float**)syn1->mat)[26][8]=-0.276459;((float**)syn1->mat)[26][9]=-0.273199;((float**)syn1->mat)[26][10]=0.067889;((float**)syn1->mat)[26][11]=-0.695629;((float**)syn1->mat)[26][12]=-0.805029;((float**)syn1->mat)[26][13]=-0.866693;((float**)syn1->mat)[26][14]=0.892512;((float**)syn1->mat)[26][15]=1.308354;((float**)syn1->mat)[26][16]=-0.050689;((float**)syn1->mat)[26][17]=-1.156603;((float**)syn1->mat)[26][18]=-0.764221;((float**)syn1->mat)[26][19]=0.238918;((float**)syn1->mat)[26][20]=-2.457925;((float**)syn1->mat)[26][21]=0.081958;((float**)syn1->mat)[26][22]=0.560728;((float**)syn1->mat)[26][23]=0.168427;((float**)syn1->mat)[26][24]=0.119282;((float**)syn1->mat)[26][25]=-0.293148;((float**)syn1->mat)[26][26]=-0.621296;((float**)syn1->mat)[26][27]=-0.490798;((float**)syn1->mat)[26][28]=-0.286257;((float**)syn1->mat)[26][29]=0.046959;((float**)syn1->mat)[27][0]=-2.186720;((float**)syn1->mat)[27][1]=-1.402959;((float**)syn1->mat)[27][2]=-1.502285;((float**)syn1->mat)[27][3]=1.238036;((float**)syn1->mat)[27][4]=-1.680202;((float**)syn1->mat)[27][5]=0.662948;((float**)syn1->mat)[27][6]=1.075899;((float**)syn1->mat)[27][7]=0.560933;((float**)syn1->mat)[27][8]=-0.155564;((float**)syn1->mat)[27][9]=-1.564116;((float**)syn1->mat)[27][10]=-0.752894;((float**)syn1->mat)[27][11]=0.518395;((float**)syn1->mat)[27][12]=-0.969793;((float**)syn1->mat)[27][13]=-0.794859;((float**)syn1->mat)[27][14]=-0.457682;((float**)syn1->mat)[27][15]=-0.136371;((float**)syn1->mat)[27][16]=0.556471;((float**)syn1->mat)[27][17]=0.295884;((float**)syn1->mat)[27][18]=1.190225;((float**)syn1->mat)[27][19]=1.859722;((float**)syn1->mat)[27][20]=-3.170617;((float**)syn1->mat)[27][21]=-0.738850;((float**)syn1->mat)[27][22]=0.437563;((float**)syn1->mat)[27][23]=-0.381854;((float**)syn1->mat)[27][24]=-0.447275;((float**)syn1->mat)[27][25]=-3.274050;((float**)syn1->mat)[27][26]=-0.586139;((float**)syn1->mat)[27][27]=0.592510;((float**)syn1->mat)[27][28]=-0.602001;((float**)syn1->mat)[27][29]=0.218379;((float**)syn1->mat)[28][0]=0.261723;((float**)syn1->mat)[28][1]=-1.306144;((float**)syn1->mat)[28][2]=-0.683360;((float**)syn1->mat)[28][3]=-1.041262;((float**)syn1->mat)[28][4]=0.468364;((float**)syn1->mat)[28][5]=-0.869186;((float**)syn1->mat)[28][6]=-0.634510;((float**)syn1->mat)[28][7]=0.040048;((float**)syn1->mat)[28][8]=0.610836;((float**)syn1->mat)[28][9]=-1.650301;((float**)syn1->mat)[28][10]=-1.209341;((float**)syn1->mat)[28][11]=-0.361227;((float**)syn1->mat)[28][12]=-0.812195;((float**)syn1->mat)[28][13]=0.663304;((float**)syn1->mat)[28][14]=-1.041700;((float**)syn1->mat)[28][15]=0.845613;((float**)syn1->mat)[28][16]=0.331568;((float**)syn1->mat)[28][17]=0.023339;((float**)syn1->mat)[28][18]=0.487650;((float**)syn1->mat)[28][19]=0.177599;((float**)syn1->mat)[28][20]=-1.607250;((float**)syn1->mat)[28][21]=-0.311242;((float**)syn1->mat)[28][22]=-0.214638;((float**)syn1->mat)[28][23]=0.304534;((float**)syn1->mat)[28][24]=-0.185706;((float**)syn1->mat)[28][25]=0.628332;((float**)syn1->mat)[28][26]=0.483433;((float**)syn1->mat)[28][27]=0.420440;((float**)syn1->mat)[28][28]=0.602149;((float**)syn1->mat)[28][29]=-0.121471;((float**)syn1->mat)[29][0]=-0.778251;((float**)syn1->mat)[29][1]=-1.504259;((float**)syn1->mat)[29][2]=-0.335390;((float**)syn1->mat)[29][3]=-0.615908;((float**)syn1->mat)[29][4]=-0.176969;((float**)syn1->mat)[29][5]=-0.795456;((float**)syn1->mat)[29][6]=-0.059924;((float**)syn1->mat)[29][7]=-0.406788;((float**)syn1->mat)[29][8]=-0.610158;((float**)syn1->mat)[29][9]=-0.997281;((float**)syn1->mat)[29][10]=-0.948819;((float**)syn1->mat)[29][11]=-0.604872;((float**)syn1->mat)[29][12]=-1.192476;((float**)syn1->mat)[29][13]=0.502563;((float**)syn1->mat)[29][14]=0.653575;((float**)syn1->mat)[29][15]=0.276369;((float**)syn1->mat)[29][16]=0.559557;((float**)syn1->mat)[29][17]=0.462826;((float**)syn1->mat)[29][18]=-0.015802;((float**)syn1->mat)[29][19]=1.849485;((float**)syn1->mat)[29][20]=-3.260781;((float**)syn1->mat)[29][21]=0.414126;((float**)syn1->mat)[29][22]=-1.327309;((float**)syn1->mat)[29][23]=-0.407530;((float**)syn1->mat)[29][24]=-1.035771;((float**)syn1->mat)[29][25]=-3.107916;((float**)syn1->mat)[29][26]=0.754514;((float**)syn1->mat)[29][27]=-1.020400;((float**)syn1->mat)[29][28]=-0.501975;((float**)syn1->mat)[29][29]=-0.301899;
}
void fill_matrix_syn2(){
  ((float**)syn2->mat)[0][0]=-0.800114;((float**)syn2->mat)[0][1]=0.209362;((float**)syn2->mat)[0][2]=0.851831;((float**)syn2->mat)[0][3]=-1.066391;((float**)syn2->mat)[1][0]=-0.068582;((float**)syn2->mat)[1][1]=0.389377;((float**)syn2->mat)[1][2]=-0.140272;((float**)syn2->mat)[1][3]=0.450001;((float**)syn2->mat)[2][0]=-0.071851;((float**)syn2->mat)[2][1]=-0.571111;((float**)syn2->mat)[2][2]=0.955323;((float**)syn2->mat)[2][3]=-0.296977;((float**)syn2->mat)[3][0]=-0.399850;((float**)syn2->mat)[3][1]=1.719304;((float**)syn2->mat)[3][2]=-3.309920;((float**)syn2->mat)[3][3]=0.687056;((float**)syn2->mat)[4][0]=-0.060588;((float**)syn2->mat)[4][1]=-0.170935;((float**)syn2->mat)[4][2]=0.531065;((float**)syn2->mat)[4][3]=-0.238430;((float**)syn2->mat)[5][0]=-0.629256;((float**)syn2->mat)[5][1]=-0.519016;((float**)syn2->mat)[5][2]=-1.017212;((float**)syn2->mat)[5][3]=1.288048;((float**)syn2->mat)[6][0]=0.376438;((float**)syn2->mat)[6][1]=-0.283385;((float**)syn2->mat)[6][2]=-0.096875;((float**)syn2->mat)[6][3]=0.284448;((float**)syn2->mat)[7][0]=1.000208;((float**)syn2->mat)[7][1]=-1.631526;((float**)syn2->mat)[7][2]=-0.756785;((float**)syn2->mat)[7][3]=-3.001575;((float**)syn2->mat)[8][0]=-1.586440;((float**)syn2->mat)[8][1]=-0.463606;((float**)syn2->mat)[8][2]=-1.962298;((float**)syn2->mat)[8][3]=0.834995;((float**)syn2->mat)[9][0]=0.385110;((float**)syn2->mat)[9][1]=0.386654;((float**)syn2->mat)[9][2]=0.499113;((float**)syn2->mat)[9][3]=0.109583;((float**)syn2->mat)[10][0]=0.065174;((float**)syn2->mat)[10][1]=-0.296654;((float**)syn2->mat)[10][2]=0.894708;((float**)syn2->mat)[10][3]=-0.297915;((float**)syn2->mat)[11][0]=-0.625372;((float**)syn2->mat)[11][1]=-0.945998;((float**)syn2->mat)[11][2]=-0.709218;((float**)syn2->mat)[11][3]=1.048266;((float**)syn2->mat)[12][0]=-1.425803;((float**)syn2->mat)[12][1]=1.773524;((float**)syn2->mat)[12][2]=-2.022777;((float**)syn2->mat)[12][3]=0.508641;((float**)syn2->mat)[13][0]=-0.776931;((float**)syn2->mat)[13][1]=-2.905617;((float**)syn2->mat)[13][2]=0.510952;((float**)syn2->mat)[13][3]=-1.597201;((float**)syn2->mat)[14][0]=-0.272718;((float**)syn2->mat)[14][1]=-0.868688;((float**)syn2->mat)[14][2]=-0.586730;((float**)syn2->mat)[14][3]=-0.458975;((float**)syn2->mat)[15][0]=0.683187;((float**)syn2->mat)[15][1]=-3.044065;((float**)syn2->mat)[15][2]=1.964085;((float**)syn2->mat)[15][3]=-5.776953;((float**)syn2->mat)[16][0]=0.497193;((float**)syn2->mat)[16][1]=0.292206;((float**)syn2->mat)[16][2]=-0.011659;((float**)syn2->mat)[16][3]=-0.110916;((float**)syn2->mat)[17][0]=-3.321600;((float**)syn2->mat)[17][1]=-2.893201;((float**)syn2->mat)[17][2]=-4.379849;((float**)syn2->mat)[17][3]=2.719988;((float**)syn2->mat)[18][0]=-0.518580;((float**)syn2->mat)[18][1]=-0.843381;((float**)syn2->mat)[18][2]=-3.597222;((float**)syn2->mat)[18][3]=1.610090;((float**)syn2->mat)[19][0]=-0.716791;((float**)syn2->mat)[19][1]=-1.882044;((float**)syn2->mat)[19][2]=-2.839560;((float**)syn2->mat)[19][3]=2.696138;((float**)syn2->mat)[20][0]=0.689588;((float**)syn2->mat)[20][1]=0.755386;((float**)syn2->mat)[20][2]=0.705258;((float**)syn2->mat)[20][3]=0.816448;((float**)syn2->mat)[21][0]=1.185207;((float**)syn2->mat)[21][1]=0.557214;((float**)syn2->mat)[21][2]=-0.385211;((float**)syn2->mat)[21][3]=-0.683168;((float**)syn2->mat)[22][0]=0.072935;((float**)syn2->mat)[22][1]=-0.422758;((float**)syn2->mat)[22][2]=0.583041;((float**)syn2->mat)[22][3]=-0.602854;((float**)syn2->mat)[23][0]=-0.743390;((float**)syn2->mat)[23][1]=-0.949910;((float**)syn2->mat)[23][2]=-1.813815;((float**)syn2->mat)[23][3]=1.759687;((float**)syn2->mat)[24][0]=-3.522044;((float**)syn2->mat)[24][1]=-0.235431;((float**)syn2->mat)[24][2]=-2.259983;((float**)syn2->mat)[24][3]=0.636263;((float**)syn2->mat)[25][0]=-2.942738;((float**)syn2->mat)[25][1]=0.496437;((float**)syn2->mat)[25][2]=1.797142;((float**)syn2->mat)[25][3]=-5.946764;((float**)syn2->mat)[26][0]=0.932440;((float**)syn2->mat)[26][1]=-0.292218;((float**)syn2->mat)[26][2]=-0.184990;((float**)syn2->mat)[26][3]=-1.282704;((float**)syn2->mat)[27][0]=-0.506383;((float**)syn2->mat)[27][1]=0.184031;((float**)syn2->mat)[27][2]=-1.253047;((float**)syn2->mat)[27][3]=0.870315;((float**)syn2->mat)[28][0]=-0.178617;((float**)syn2->mat)[28][1]=-1.200417;((float**)syn2->mat)[28][2]=0.704726;((float**)syn2->mat)[28][3]=-1.849402;((float**)syn2->mat)[29][0]=-1.478717;((float**)syn2->mat)[29][1]=0.010647;((float**)syn2->mat)[29][2]=-4.338893;((float**)syn2->mat)[29][3]=0.391917;
}

void fill_matrix_input_1(){
//(1, 20) out : 0,0,0,1
    ((float**)input->mat)[0][0] = 0.406667; ((float**)input->mat)[0][1] = 0.661333; ((float**)input->mat)[0][2] = 0.378667; ((float**)input->mat)[0][3] = 0.733333; ((float**)input->mat)[0][4] = 0.556;    ((float**)input->mat)[0][5] = 0.706667; ((float**)input->mat)[0][6] = 0.568;    ((float**)input->mat)[0][7] = 0.697333; ((float**)input->mat)[0][8] = 0.534667; ((float**)input->mat)[0][9] = 0.690667; ((float**)input->mat)[0][10] = 0.525333;    ((float**)input->mat)[0][11] = 0.676;   ((float**)input->mat)[0][12] = 0.525333;    ((float**)input->mat)[0][13] = 0.662667;    ((float**)input->mat)[0][14] = 0.409333;    ((float**)input->mat)[0][15] = 0.717333;    ((float**)input->mat)[0][16] = 0.452;   ((float**)input->mat)[0][17] = 0.698667;    ((float**)input->mat)[0][18] = 0.502667;    ((float**)input->mat)[0][19] = 0.685333;
}
void fill_matrix_input_1_1(){
//(1, 20)
  ((float**)input->mat)[0][0] = 0.445333; ((float**)input->mat)[0][1] = 0.648;  ((float**)input->mat)[0][2] = 0.424;  ((float**)input->mat)[0][3] = 0.7;  ((float**)input->mat)[0][4] = 0.513333; ((float**)input->mat)[0][5] = 0.688;  ((float**)input->mat)[0][6] = 0.552;  ((float**)input->mat)[0][7] = 0.681333; ((float**)input->mat)[0][8] = 0.544;  ((float**)input->mat)[0][9] = 0.674667; ((float**)input->mat)[0][10] = 0.516; ((float**)input->mat)[0][11] = 0.674667;  ((float**)input->mat)[0][12] = 0.517333;  ((float**)input->mat)[0][13] = 0.661333;  ((float**)input->mat)[0][14] = 0.508; ((float**)input->mat)[0][15] = 0.666667;  ((float**)input->mat)[0][16] = 0.433333;  ((float**)input->mat)[0][17] = 0.657333;  ((float**)input->mat)[0][18] = 0.437333;  ((float**)input->mat)[0][19] = 0.674667;
}
void fill_matrix_input_2(){
//(1, 20) out : 0,0,1,0
    ((float**)input->mat)[0][0] = 0.410667; ((float**)input->mat)[0][1] = 0.672;    ((float**)input->mat)[0][2] = 0.461333; ((float**)input->mat)[0][3] = 0.553333; ((float**)input->mat)[0][4] = 0.502667; ((float**)input->mat)[0][5] = 0.630667; ((float**)input->mat)[0][6] = 0.568;    ((float**)input->mat)[0][7] = 0.696;    ((float**)input->mat)[0][8] = 0.532;    ((float**)input->mat)[0][9] = 0.806667; ((float**)input->mat)[0][10] = 0.445333;    ((float**)input->mat)[0][11] = 0.769333;    ((float**)input->mat)[0][12] = 0.428;   ((float**)input->mat)[0][13] = 0.608;   ((float**)input->mat)[0][14] = 0.472;   ((float**)input->mat)[0][15] = 0.572;   ((float**)input->mat)[0][16] = 0.474667;    ((float**)input->mat)[0][17] = 0.629333;    ((float**)input->mat)[0][18] = 0.484;   ((float**)input->mat)[0][19] = 0.748;
}
void fill_matrix_input_3(){
//(1, 20)
  ((float**)input->mat)[0][0] = 0.450667; ((float**)input->mat)[0][1] = 0.626667; ((float**)input->mat)[0][2] = 0.533333; ((float**)input->mat)[0][3] = 0.572;  ((float**)input->mat)[0][4] = 0.558667; ((float**)input->mat)[0][5] = 0.885333; ((float**)input->mat)[0][6] = 0.429333; ((float**)input->mat)[0][7] = 0.8;  ((float**)input->mat)[0][8] = 0.370667; ((float**)input->mat)[0][9] = 0.584;  ((float**)input->mat)[0][10] = 0.502667;  ((float**)input->mat)[0][11] = 0.544; ((float**)input->mat)[0][12] = 0.614667;  ((float**)input->mat)[0][13] = 0.769333;  ((float**)input->mat)[0][14] = 0.505333;  ((float**)input->mat)[0][15] = 0.864; ((float**)input->mat)[0][16] = 0.425333;  ((float**)input->mat)[0][17] = 0.649333;  ((float**)input->mat)[0][18] = 0.444; ((float**)input->mat)[0][19] = 0.574667;
}
void fill_matrix_input_4(){
//(1, 20)
  ((float**)input->mat)[0][0] = 0.5;  ((float**)input->mat)[0][1] = 0.716;  ((float**)input->mat)[0][2] = 0.582667; ((float**)input->mat)[0][3] = 0.708;  ((float**)input->mat)[0][4] = 0.5;  ((float**)input->mat)[0][5] = 0.641333; ((float**)input->mat)[0][6] = 0.414667; ((float**)input->mat)[0][7] = 0.616;  ((float**)input->mat)[0][8] = 0.414667; ((float**)input->mat)[0][9] = 0.585333; ((float**)input->mat)[0][10] = 0.44;  ((float**)input->mat)[0][11] = 0.686667;  ((float**)input->mat)[0][12] = 0.518667;  ((float**)input->mat)[0][13] = 0.710667;  ((float**)input->mat)[0][14] = 0.541333;  ((float**)input->mat)[0][15] = 0.693333;  ((float**)input->mat)[0][16] = 0.576; ((float**)input->mat)[0][17] = 0.658667;  ((float**)input->mat)[0][18] = 0.402667;  ((float**)input->mat)[0][19] = 0.734667;
}
void fill_static_matrix_inputs(){
//(102, 20)
  #ifdef TRAINING
  inputs[0][0] = 0.433333;  inputs[0][1] = 0.596; inputs[0][2] = 0.558667;  inputs[0][3] = 0.589333;  inputs[0][4] = 0.576; inputs[0][5] = 0.738667;  inputs[0][6] = 0.561333;  inputs[0][7] = 0.802667;  inputs[0][8] = 0.461333;  inputs[0][9] = 0.805333;  inputs[0][10] = 0.425333; inputs[0][11] = 0.622667; inputs[0][12] = 0.432;  inputs[0][13] = 0.578667; inputs[0][14] = 0.46; inputs[0][15] = 0.573333; inputs[0][16] = 0.461333; inputs[0][17] = 0.661333; inputs[0][18] = 0.493333; inputs[0][19] = 0.821333; inputs[1][0] = 0.392; inputs[1][1] = 0.653333;  inputs[1][2] = 0.432; inputs[1][3] = 0.676; inputs[1][4] = 0.544; inputs[1][5] = 0.702667;  inputs[1][6] = 0.564; inputs[1][7] = 0.678667;  inputs[1][8] = 0.542667;  inputs[1][9] = 0.689333;  inputs[1][10] = 0.548;  inputs[1][11] = 0.676;  inputs[1][12] = 0.518667; inputs[1][13] = 0.682667; inputs[1][14] = 0.42; inputs[1][15] = 0.66; inputs[1][16] = 0.4;  inputs[1][17] = 0.633333; inputs[1][18] = 0.473333; inputs[1][19] = 0.654667; inputs[2][0] = 0.485333;  inputs[2][1] = 0.685333;  inputs[2][2] = 0.56;  inputs[2][3] = 0.749333;  inputs[2][4] = 0.496; inputs[2][5] = 0.702667;  inputs[2][6] = 0.434667;  inputs[2][7] = 0.566667;  inputs[2][8] = 0.396; inputs[2][9] = 0.546667;  inputs[2][10] = 0.457333; inputs[2][11] = 0.710667; inputs[2][12] = 0.553333; inputs[2][13] = 0.742667; inputs[2][14] = 0.616;  inputs[2][15] = 0.677333; inputs[2][16] = 0.286667; inputs[2][17] = 0.685333; inputs[2][18] = 0.454667; inputs[2][19] = 0.677333; inputs[3][0] = 0.470667;  inputs[3][1] = 0.666667;  inputs[3][2] = 0.457333;  inputs[3][3] = 0.582667;  inputs[3][4] = 0.508; inputs[3][5] = 0.648; inputs[3][6] = 0.585333;  inputs[3][7] = 0.705333;  inputs[3][8] = 0.525333;  inputs[3][9] = 0.834667;  inputs[3][10] = 0.406667; inputs[3][11] = 0.712;  inputs[3][12] = 0.406667; inputs[3][13] = 0.556;  inputs[3][14] = 0.564;  inputs[3][15] = 0.613333; inputs[3][16] = 0.562667; inputs[3][17] = 0.725333; inputs[3][18] = 0.464;  inputs[3][19] = 0.789333; inputs[4][0] = 0.46;  inputs[4][1] = 0.641333;  inputs[4][2] = 0.488; inputs[4][3] = 0.573333;  inputs[4][4] = 0.565333;  inputs[4][5] = 0.704; inputs[4][6] = 0.586667;  inputs[4][7] = 0.828; inputs[4][8] = 0.402667;  inputs[4][9] = 0.769333;  inputs[4][10] = 0.376;  inputs[4][11] = 0.54; inputs[4][12] = 0.485333; inputs[4][13] = 0.604;  inputs[4][14] = 0.606667; inputs[4][15] = 0.678667; inputs[4][16] = 0.502667; inputs[4][17] = 0.914667; inputs[4][18] = 0.44; inputs[4][19] = 0.734667; inputs[5][0] = 0.558667;  inputs[5][1] = 0.736; inputs[5][2] = 0.530667;  inputs[5][3] = 0.685333;  inputs[5][4] = 0.432; inputs[5][5] = 0.592; inputs[5][6] = 0.442667;  inputs[5][7] = 0.570667;  inputs[5][8] = 0.42;  inputs[5][9] = 0.694667;  inputs[5][10] = 0.504;  inputs[5][11] = 0.704;  inputs[5][12] = 0.541333; inputs[5][13] = 0.696;  inputs[5][14] = 0.582667; inputs[5][15] = 0.686667; inputs[5][16] = 0.404;  inputs[5][17] = 0.708;  inputs[5][18] = 0.485333; inputs[5][19] = 0.684;  inputs[6][0] = 0.392; inputs[6][1] = 0.645333;  inputs[6][2] = 0.373333;  inputs[6][3] = 0.652; inputs[6][4] = 0.533333;  inputs[6][5] = 0.704; inputs[6][6] = 0.565333;  inputs[6][7] = 0.657333;  inputs[6][8] = 0.530667;  inputs[6][9] = 0.674667;  inputs[6][10] = 0.584;  inputs[6][11] = 0.676;  inputs[6][12] = 0.541333; inputs[6][13] = 0.648;  inputs[6][14] = 0.410667; inputs[6][15] = 0.681333; inputs[6][16] = 0.488;  inputs[6][17] = 0.693333; inputs[6][18] = 0.517333; inputs[6][19] = 0.685333; inputs[7][0] = 0.490667;  inputs[7][1] = 0.656; inputs[7][2] = 0.605333;  inputs[7][3] = 0.633333;  inputs[7][4] = 0.493333;  inputs[7][5] = 0.718667;  inputs[7][6] = 0.369333;  inputs[7][7] = 0.774667;  inputs[7][8] = 0.48;  inputs[7][9] = 0.618667;  inputs[7][10] = 0.517333; inputs[7][11] = 0.558667; inputs[7][12] = 0.453333; inputs[7][13] = 0.616;  inputs[7][14] = 0.478667; inputs[7][15] = 0.676;  inputs[7][16] = 0.532;  inputs[7][17] = 0.688;  inputs[7][18] = 0.684;  inputs[7][19] = 0.677333; inputs[8][0] = 0.457333;  inputs[8][1] = 0.62;  inputs[8][2] = 0.530667;  inputs[8][3] = 0.589333;  inputs[8][4] = 0.549333;  inputs[8][5] = 0.725333;  inputs[8][6] = 0.481333;  inputs[8][7] = 0.821333;  inputs[8][8] = 0.414667;  inputs[8][9] = 0.685333;  inputs[8][10] = 0.428;  inputs[8][11] = 0.533333; inputs[8][12] = 0.6;  inputs[8][13] = 0.626667; inputs[8][14] = 0.529333; inputs[8][15] = 0.808;  inputs[8][16] = 0.469333; inputs[8][17] = 0.909333; inputs[8][18] = 0.437333; inputs[8][19] = 0.576;  inputs[9][0] = 0.477333;  inputs[9][1] = 0.638667;  inputs[9][2] = 0.530667;  inputs[9][3] = 0.589333;  inputs[9][4] = 0.601333;  inputs[9][5] = 0.745333;  inputs[9][6] = 0.493333;  inputs[9][7] = 0.884; inputs[9][8] = 0.386667;  inputs[9][9] = 0.633333;  inputs[9][10] = 0.477333; inputs[9][11] = 0.526667; inputs[9][12] = 0.625333; inputs[9][13] = 0.616;  inputs[9][14] = 0.492;  inputs[9][15] = 0.765333; inputs[9][16] = 0.424;  inputs[9][17] = 0.742667; inputs[9][18] = 0.445333; inputs[9][19] = 0.678667; inputs[10][0] = 0.54; inputs[10][1] = 0.718667; inputs[10][2] = 0.556;  inputs[10][3] = 0.66; inputs[10][4] = 0.418667; inputs[10][5] = 0.62; inputs[10][6] = 0.428;  inputs[10][7] = 0.604;  inputs[10][8] = 0.417333; inputs[10][9] = 0.650667; inputs[10][10] = 0.492; inputs[10][11] = 0.697333;  inputs[10][12] = 0.536; inputs[10][13] = 0.717333;  inputs[10][14] = 0.581333;  inputs[10][15] = 0.701333;  inputs[10][16] = 0.464; inputs[10][17] = 0.722667;  inputs[10][18] = 0.437333;  inputs[10][19] = 0.690667;  inputs[11][0] = 0.517333; inputs[11][1] = 0.701333; inputs[11][2] = 0.578667; inputs[11][3] = 0.693333; inputs[11][4] = 0.428;  inputs[11][5] = 0.658667; inputs[11][6] = 0.448;  inputs[11][7] = 0.552;  inputs[11][8] = 0.428;  inputs[11][9] = 0.614667; inputs[11][10] = 0.498667;  inputs[11][11] = 0.712; inputs[11][12] = 0.573333;  inputs[11][13] = 0.701333;  inputs[11][14] = 0.502667;  inputs[11][15] = 0.7; inputs[11][16] = 0.473333;  inputs[11][17] = 0.688; inputs[11][18] = 0.498667;  inputs[11][19] = 0.697333;  inputs[12][0] = 0.462667; inputs[12][1] = 0.650667; inputs[12][2] = 0.468;  inputs[12][3] = 0.552;  inputs[12][4] = 0.588;  inputs[12][5] = 0.681333; inputs[12][6] = 0.573333; inputs[12][7] = 0.796;  inputs[12][8] = 0.408;  inputs[12][9] = 0.746667; inputs[12][10] = 0.376; inputs[12][11] = 0.552; inputs[12][12] = 0.493333;  inputs[12][13] = 0.597333;  inputs[12][14] = 0.562667;  inputs[12][15] = 0.688; inputs[12][16] = 0.568; inputs[12][17] = 0.761333;  inputs[12][18] = 0.489333;  inputs[12][19] = 0.846667;  inputs[13][0] = 0.382667; inputs[13][1] = 0.648;  inputs[13][2] = 0.321333; inputs[13][3] = 0.749333; inputs[13][4] = 0.616;  inputs[13][5] = 0.772;  inputs[13][6] = 0.641333; inputs[13][7] = 0.713333; inputs[13][8] = 0.570667; inputs[13][9] = 0.674667; inputs[13][10] = 0.445333;  inputs[13][11] = 0.712; inputs[13][12] = 0.329333;  inputs[13][13] = 0.746667;  inputs[13][14] = 0.449333;  inputs[13][15] = 0.697333;  inputs[13][16] = 0.477333;  inputs[13][17] = 0.685333;  inputs[13][18] = 0.494667;  inputs[13][19] = 0.690667;  inputs[14][0] = 0.521333; inputs[14][1] = 0.698667; inputs[14][2] = 0.589333; inputs[14][3] = 0.741333; inputs[14][4] = 0.490667; inputs[14][5] = 0.672;  inputs[14][6] = 0.425333; inputs[14][7] = 0.542667; inputs[14][8] = 0.418667; inputs[14][9] = 0.594667; inputs[14][10] = 0.468; inputs[14][11] = 0.757333;  inputs[14][12] = 0.536; inputs[14][13] = 0.726667;  inputs[14][14] = 0.601333;  inputs[14][15] = 0.674667;  inputs[14][16] = 0.474667;  inputs[14][17] = 0.732; inputs[14][18] = 0.477333;  inputs[14][19] = 0.697333;  inputs[15][0] = 0.569333; inputs[15][1] = 0.789333; inputs[15][2] = 0.465333; inputs[15][3] = 0.726667; inputs[15][4] = 0.538667; inputs[15][5] = 0.562667; inputs[15][6] = 0.445333; inputs[15][7] = 0.592;  inputs[15][8] = 0.376;  inputs[15][9] = 0.629333; inputs[15][10] = 0.478667;  inputs[15][11] = 0.722667;  inputs[15][12] = 0.581333;  inputs[15][13] = 0.724; inputs[15][14] = 0.596; inputs[15][15] = 0.684; inputs[15][16] = 0.416; inputs[15][17] = 0.708; inputs[15][18] = 0.409333;  inputs[15][19] = 0.678667;  inputs[16][0] = 0.428;  inputs[16][1] = 0.674667; inputs[16][2] = 0.474667; inputs[16][3] = 0.684;  inputs[16][4] = 0.597333; inputs[16][5] = 0.676;  inputs[16][6] = 0.562667; inputs[16][7] = 0.684;  inputs[16][8] = 0.552;  inputs[16][9] = 0.665333; inputs[16][10] = 0.606667;  inputs[16][11] = 0.696; inputs[16][12] = 0.494667;  inputs[16][13] = 0.674667;  inputs[16][14] = 0.384; inputs[16][15] = 0.681333;  inputs[16][16] = 0.481333;  inputs[16][17] = 0.674667;  inputs[16][18] = 0.493333;  inputs[16][19] = 0.668; inputs[17][0] = 0.476;  inputs[17][1] = 0.672;  inputs[17][2] = 0.457333; inputs[17][3] = 0.565333; inputs[17][4] = 0.541333; inputs[17][5] = 0.722667; inputs[17][6] = 0.534667; inputs[17][7] = 0.825333; inputs[17][8] = 0.434667; inputs[17][9] = 0.801333; inputs[17][10] = 0.384; inputs[17][11] = 0.602667;  inputs[17][12] = 0.48;  inputs[17][13] = 0.589333;  inputs[17][14] = 0.593333;  inputs[17][15] = 0.64;  inputs[17][16] = 0.546667;  inputs[17][17] = 0.8; inputs[17][18] = 0.438667;  inputs[17][19] = 0.829333;  inputs[18][0] = 0.390667; inputs[18][1] = 0.674667; inputs[18][2] = 0.461333; inputs[18][3] = 0.714667; inputs[18][4] = 0.641333; inputs[18][5] = 0.705333; inputs[18][6] = 0.605333; inputs[18][7] = 0.696;  inputs[18][8] = 0.54; inputs[18][9] = 0.689333; inputs[18][10] = 0.570667;  inputs[18][11] = 0.653333;  inputs[18][12] = 0.54;  inputs[18][13] = 0.693333;  inputs[18][14] = 0.413333;  inputs[18][15] = 0.694667;  inputs[18][16] = 0.478667;  inputs[18][17] = 0.665333;  inputs[18][18] = 0.485333;  inputs[18][19] = 0.664; inputs[19][0] = 0.450667; inputs[19][1] = 0.550667; inputs[19][2] = 0.546667; inputs[19][3] = 0.566667; inputs[19][4] = 0.533333; inputs[19][5] = 0.816;  inputs[19][6] = 0.517333; inputs[19][7] = 0.857333; inputs[19][8] = 0.46; inputs[19][9] = 0.704;  inputs[19][10] = 0.424; inputs[19][11] = 0.598667;  inputs[19][12] = 0.464; inputs[19][13] = 0.573333;  inputs[19][14] = 0.484; inputs[19][15] = 0.556; inputs[19][16] = 0.534667;  inputs[19][17] = 0.596; inputs[19][18] = 0.476; inputs[19][19] = 0.753333;  inputs[20][0] = 0.474667; inputs[20][1] = 0.646667; inputs[20][2] = 0.488;  inputs[20][3] = 0.582667; inputs[20][4] = 0.581333; inputs[20][5] = 0.684;  inputs[20][6] = 0.526667; inputs[20][7] = 0.753333; inputs[20][8] = 0.445333; inputs[20][9] = 0.733333; inputs[20][10] = 0.36;  inputs[20][11] = 0.610667;  inputs[20][12] = 0.450667;  inputs[20][13] = 0.572; inputs[20][14] = 0.586667;  inputs[20][15] = 0.629333;  inputs[20][16] = 0.512; inputs[20][17] = 0.862667;  inputs[20][18] = 0.449333;  inputs[20][19] = 0.730667;  inputs[21][0] = 0.532;  inputs[21][1] = 0.682667; inputs[21][2] = 0.456;  inputs[21][3] = 0.612;  inputs[21][4] = 0.424;  inputs[21][5] = 0.572;  inputs[21][6] = 0.418667; inputs[21][7] = 0.625333; inputs[21][8] = 0.477333; inputs[21][9] = 0.732;  inputs[21][10] = 0.578667;  inputs[21][11] = 0.698667;  inputs[21][12] = 0.610667;  inputs[21][13] = 0.669333;  inputs[21][14] = 0.388; inputs[21][15] = 0.721333;  inputs[21][16] = 0.476; inputs[21][17] = 0.669333;  inputs[21][18] = 0.506667;  inputs[21][19] = 0.692; inputs[22][0] = 0.448;  inputs[22][1] = 0.568;  inputs[22][2] = 0.538667; inputs[22][3] = 0.697333; inputs[22][4] = 0.557333; inputs[22][5] = 0.786667; inputs[22][6] = 0.464;  inputs[22][7] = 0.778667; inputs[22][8] = 0.441333; inputs[22][9] = 0.610667; inputs[22][10] = 0.478667;  inputs[22][11] = 0.585333;  inputs[22][12] = 0.488; inputs[22][13] = 0.569333;  inputs[22][14] = 0.501333;  inputs[22][15] = 0.677333;  inputs[22][16] = 0.485333;  inputs[22][17] = 0.722667;  inputs[22][18] = 0.501333;  inputs[22][19] = 0.670667;  inputs[23][0] = 0.554667; inputs[23][1] = 0.724;  inputs[23][2] = 0.512;  inputs[23][3] = 0.690667; inputs[23][4] = 0.393333; inputs[23][5] = 0.617333; inputs[23][6] = 0.410667; inputs[23][7] = 0.582667; inputs[23][8] = 0.424;  inputs[23][9] = 0.666667; inputs[23][10] = 0.549333;  inputs[23][11] = 0.726667;  inputs[23][12] = 0.577333;  inputs[23][13] = 0.689333;  inputs[23][14] = 0.56;  inputs[23][15] = 0.678667;  inputs[23][16] = 0.416; inputs[23][17] = 0.690667;  inputs[23][18] = 0.489333;  inputs[23][19] = 0.68;  inputs[24][0] = 0.478667; inputs[24][1] = 0.641333; inputs[24][2] = 0.497333; inputs[24][3] = 0.597333; inputs[24][4] = 0.570667; inputs[24][5] = 0.690667; inputs[24][6] = 0.481333; inputs[24][7] = 0.804;  inputs[24][8] = 0.410667; inputs[24][9] = 0.728;  inputs[24][10] = 0.4; inputs[24][11] = 0.554667;  inputs[24][12] = 0.498667;  inputs[24][13] = 0.581333;  inputs[24][14] = 0.586667;  inputs[24][15] = 0.682667;  inputs[24][16] = 0.504; inputs[24][17] = 0.802667;  inputs[24][18] = 0.432; inputs[24][19] = 0.802667;  inputs[25][0] = 0.46; inputs[25][1] = 0.629333; inputs[25][2] = 0.529333; inputs[25][3] = 0.598667; inputs[25][4] = 0.590667; inputs[25][5] = 0.732;  inputs[25][6] = 0.493333; inputs[25][7] = 0.838667; inputs[25][8] = 0.394667; inputs[25][9] = 0.634667; inputs[25][10] = 0.436; inputs[25][11] = 0.572; inputs[25][12] = 0.534667;  inputs[25][13] = 0.650667;  inputs[25][14] = 0.562667;  inputs[25][15] = 0.72;  inputs[25][16] = 0.508; inputs[25][17] = 0.786667;  inputs[25][18] = 0.445333;  inputs[25][19] = 0.762667;  inputs[26][0] = 0.397333; inputs[26][1] = 0.617333; inputs[26][2] = 0.445333; inputs[26][3] = 0.52; inputs[26][4] = 0.529333; inputs[26][5] = 0.710667; inputs[26][6] = 0.581333; inputs[26][7] = 0.748;  inputs[26][8] = 0.457333; inputs[26][9] = 0.786667; inputs[26][10] = 0.425333;  inputs[26][11] = 0.692; inputs[26][12] = 0.473333;  inputs[26][13] = 0.589333;  inputs[26][14] = 0.472; inputs[26][15] = 0.546667;  inputs[26][16] = 0.577333;  inputs[26][17] = 0.765333;  inputs[26][18] = 0.492; inputs[26][19] = 0.76;  inputs[27][0] = 0.476;  inputs[27][1] = 0.702667; inputs[27][2] = 0.409333; inputs[27][3] = 0.574667; inputs[27][4] = 0.518667; inputs[27][5] = 0.554667; inputs[27][6] = 0.606667; inputs[27][7] = 0.677333; inputs[27][8] = 0.58; inputs[27][9] = 0.748;  inputs[27][10] = 0.498667;  inputs[27][11] = 0.781333;  inputs[27][12] = 0.436; inputs[27][13] = 0.722667;  inputs[27][14] = 0.438667;  inputs[27][15] = 0.577333;  inputs[27][16] = 0.445333;  inputs[27][17] = 0.589333;  inputs[27][18] = 0.462667;  inputs[27][19] = 0.566667;  inputs[28][0] = 0.506667; inputs[28][1] = 0.704;  inputs[28][2] = 0.557333; inputs[28][3] = 0.745333; inputs[28][4] = 0.474667; inputs[28][5] = 0.669333; inputs[28][6] = 0.454667; inputs[28][7] = 0.628;  inputs[28][8] = 0.393333; inputs[28][9] = 0.581333; inputs[28][10] = 0.441333;  inputs[28][11] = 0.681333;  inputs[28][12] = 0.542667;  inputs[28][13] = 0.726667;  inputs[28][14] = 0.690667;  inputs[28][15] = 0.688; inputs[28][16] = 0.322667;  inputs[28][17] = 0.776; inputs[28][18] = 0.444; inputs[28][19] = 0.714667;  inputs[29][0] = 0.429333; inputs[29][1] = 0.666667; inputs[29][2] = 0.462667; inputs[29][3] = 0.549333; inputs[29][4] = 0.530667; inputs[29][5] = 0.572;  inputs[29][6] = 0.572;  inputs[29][7] = 0.76; inputs[29][8] = 0.518667; inputs[29][9] = 0.802667; inputs[29][10] = 0.473333;  inputs[29][11] = 0.724; inputs[29][12] = 0.412; inputs[29][13] = 0.570667;  inputs[29][14] = 0.454667;  inputs[29][15] = 0.582667;  inputs[29][16] = 0.430667;  inputs[29][17] = 0.561333;  inputs[29][18] = 0.481333;  inputs[29][19] = 0.769333;  inputs[30][0] = 0.550667; inputs[30][1] = 0.717333; inputs[30][2] = 0.512;  inputs[30][3] = 0.696;  inputs[30][4] = 0.452;  inputs[30][5] = 0.609333; inputs[30][6] = 0.434667; inputs[30][7] = 0.588;  inputs[30][8] = 0.42; inputs[30][9] = 0.7;  inputs[30][10] = 0.532; inputs[30][11] = 0.72;  inputs[30][12] = 0.593333;  inputs[30][13] = 0.701333;  inputs[30][14] = 0.422667;  inputs[30][15] = 0.730667;  inputs[30][16] = 0.46;  inputs[30][17] = 0.7; inputs[30][18] = 0.506667;  inputs[30][19] = 0.7; inputs[31][0] = 0.553333; inputs[31][1] = 0.725333; inputs[31][2] = 0.513333; inputs[31][3] = 0.721333; inputs[31][4] = 0.445333; inputs[31][5] = 0.594667; inputs[31][6] = 0.432;  inputs[31][7] = 0.529333; inputs[31][8] = 0.442667; inputs[31][9] = 0.674667; inputs[31][10] = 0.525333;  inputs[31][11] = 0.733333;  inputs[31][12] = 0.562667;  inputs[31][13] = 0.704; inputs[31][14] = 0.468; inputs[31][15] = 0.676; inputs[31][16] = 0.458667;  inputs[31][17] = 0.674667;  inputs[31][18] = 0.502667;  inputs[31][19] = 0.692; inputs[32][0] = 0.437333; inputs[32][1] = 0.753333; inputs[32][2] = 0.409333; inputs[32][3] = 0.518667; inputs[32][4] = 0.473333; inputs[32][5] = 0.586667; inputs[32][6] = 0.584;  inputs[32][7] = 0.741333; inputs[32][8] = 0.514667; inputs[32][9] = 0.886667; inputs[32][10] = 0.412; inputs[32][11] = 0.784; inputs[32][12] = 0.444; inputs[32][13] = 0.618667;  inputs[32][14] = 0.492; inputs[32][15] = 0.557333;  inputs[32][16] = 0.425333;  inputs[32][17] = 0.565333;  inputs[32][18] = 0.518667;  inputs[32][19] = 0.621333;  inputs[33][0] = 0.446667; inputs[33][1] = 0.653333; inputs[33][2] = 0.448;  inputs[33][3] = 0.706667; inputs[33][4] = 0.544;  inputs[33][5] = 0.717333; inputs[33][6] = 0.534667; inputs[33][7] = 0.698667; inputs[33][8] = 0.537333; inputs[33][9] = 0.644;  inputs[33][10] = 0.598667;  inputs[33][11] = 0.677333;  inputs[33][12] = 0.628; inputs[33][13] = 0.684; inputs[33][14] = 0.3; inputs[33][15] = 0.764; inputs[33][16] = 0.46;  inputs[33][17] = 0.704; inputs[33][18] = 0.485333;  inputs[33][19] = 0.676; inputs[34][0] = 0.412;  inputs[34][1] = 0.662667; inputs[34][2] = 0.488;  inputs[34][3] = 0.556;  inputs[34][4] = 0.54; inputs[34][5] = 0.606667; inputs[34][6] = 0.58; inputs[34][7] = 0.722667; inputs[34][8] = 0.533333; inputs[34][9] = 0.825333; inputs[34][10] = 0.449333;  inputs[34][11] = 0.758667;  inputs[34][12] = 0.393333;  inputs[34][13] = 0.642667;  inputs[34][14] = 0.458667;  inputs[34][15] = 0.588; inputs[34][16] = 0.477333;  inputs[34][17] = 0.594667;  inputs[34][18] = 0.5; inputs[34][19] = 0.621333;  inputs[35][0] = 0.432;  inputs[35][1] = 0.608;  inputs[35][2] = 0.333333; inputs[35][3] = 0.692;  inputs[35][4] = 0.458667; inputs[35][5] = 0.713333; inputs[35][6] = 0.626667; inputs[35][7] = 0.662667; inputs[35][8] = 0.565333; inputs[35][9] = 0.718667; inputs[35][10] = 0.554667;  inputs[35][11] = 0.688; inputs[35][12] = 0.616; inputs[35][13] = 0.662667;  inputs[35][14] = 0.589333;  inputs[35][15] = 0.658667;  inputs[35][16] = 0.381333;  inputs[35][17] = 0.702667;  inputs[35][18] = 0.461333;  inputs[35][19] = 0.684; inputs[36][0] = 0.477333; inputs[36][1] = 0.632;  inputs[36][2] = 0.481333; inputs[36][3] = 0.594667; inputs[36][4] = 0.581333; inputs[36][5] = 0.717333; inputs[36][6] = 0.506667; inputs[36][7] = 0.828;  inputs[36][8] = 0.396;  inputs[36][9] = 0.706667; inputs[36][10] = 0.416; inputs[36][11] = 0.537333;  inputs[36][12] = 0.562667;  inputs[36][13] = 0.637333;  inputs[36][14] = 0.598667;  inputs[36][15] = 0.705333;  inputs[36][16] = 0.458667;  inputs[36][17] = 0.838667;  inputs[36][18] = 0.433333;  inputs[36][19] = 0.724; inputs[37][0] = 0.477333; inputs[37][1] = 0.649333; inputs[37][2] = 0.522667; inputs[37][3] = 0.589333; inputs[37][4] = 0.530667; inputs[37][5] = 0.742667; inputs[37][6] = 0.504;  inputs[37][7] = 0.848;  inputs[37][8] = 0.424;  inputs[37][9] = 0.685333; inputs[37][10] = 0.430667;  inputs[37][11] = 0.554667;  inputs[37][12] = 0.570667;  inputs[37][13] = 0.628; inputs[37][14] = 0.56;  inputs[37][15] = 0.732; inputs[37][16] = 0.488; inputs[37][17] = 0.845333;  inputs[37][18] = 0.452; inputs[37][19] = 0.758667;  inputs[38][0] = 0.397333; inputs[38][1] = 0.538667; inputs[38][2] = 0.561333; inputs[38][3] = 0.664;  inputs[38][4] = 0.598667; inputs[38][5] = 0.817333; inputs[38][6] = 0.474667; inputs[38][7] = 0.822667; inputs[38][8] = 0.401333; inputs[38][9] = 0.608;  inputs[38][10] = 0.438667;  inputs[38][11] = 0.554667;  inputs[38][12] = 0.469333;  inputs[38][13] = 0.609333;  inputs[38][14] = 0.522667;  inputs[38][15] = 0.677333;  inputs[38][16] = 0.501333;  inputs[38][17] = 0.745333;  inputs[38][18] = 0.504; inputs[38][19] = 0.690667;  inputs[39][0] = 0.476;  inputs[39][1] = 0.632;  inputs[39][2] = 0.497333; inputs[39][3] = 0.562667; inputs[39][4] = 0.545333; inputs[39][5] = 0.808;  inputs[39][6] = 0.508;  inputs[39][7] = 0.808;  inputs[39][8] = 0.418667; inputs[39][9] = 0.64; inputs[39][10] = 0.44;  inputs[39][11] = 0.530667;  inputs[39][12] = 0.558667;  inputs[39][13] = 0.624; inputs[39][14] = 0.552; inputs[39][15] = 0.784; inputs[39][16] = 0.498667;  inputs[39][17] = 0.814667;  inputs[39][18] = 0.453333;  inputs[39][19] = 0.641333;  inputs[40][0] = 0.554667; inputs[40][1] = 0.628;  inputs[40][2] = 0.624;  inputs[40][3] = 0.698667; inputs[40][4] = 0.556;  inputs[40][5] = 0.772;  inputs[40][6] = 0.478667; inputs[40][7] = 0.822667; inputs[40][8] = 0.409333; inputs[40][9] = 0.669333; inputs[40][10] = 0.448; inputs[40][11] = 0.586667;  inputs[40][12] = 0.493333;  inputs[40][13] = 0.593333;  inputs[40][14] = 0.433333;  inputs[40][15] = 0.465333;  inputs[40][16] = 0.449333;  inputs[40][17] = 0.853333;  inputs[40][18] = 0.492; inputs[40][19] = 0.718667;  inputs[41][0] = 0.392;  inputs[41][1] = 0.632;  inputs[41][2] = 0.394667; inputs[41][3] = 0.694667; inputs[41][4] = 0.490667; inputs[41][5] = 0.725333; inputs[41][6] = 0.616;  inputs[41][7] = 0.726667; inputs[41][8] = 0.602667; inputs[41][9] = 0.694667; inputs[41][10] = 0.573333;  inputs[41][11] = 0.669333;  inputs[41][12] = 0.582667;  inputs[41][13] = 0.662667;  inputs[41][14] = 0.417333;  inputs[41][15] = 0.657333;  inputs[41][16] = 0.421333;  inputs[41][17] = 0.645333;  inputs[41][18] = 0.485333;  inputs[41][19] = 0.664; inputs[42][0] = 0.4;  inputs[42][1] = 0.705333; inputs[42][2] = 0.630667; inputs[42][3] = 0.722667; inputs[42][4] = 0.669333; inputs[42][5] = 0.697333; inputs[42][6] = 0.576;  inputs[42][7] = 0.68; inputs[42][8] = 0.569333; inputs[42][9] = 0.642667; inputs[42][10] = 0.658667;  inputs[42][11] = 0.662667;  inputs[42][12] = 0.317333;  inputs[42][13] = 0.688; inputs[42][14] = 0.445333;  inputs[42][15] = 0.656; inputs[42][16] = 0.484; inputs[42][17] = 0.668; inputs[42][18] = 0.490667;  inputs[42][19] = 0.674667;  inputs[43][0] = 0.466667; inputs[43][1] = 0.652;  inputs[43][2] = 0.504;  inputs[43][3] = 0.568;  inputs[43][4] = 0.58; inputs[43][5] = 0.632;  inputs[43][6] = 0.512;  inputs[43][7] = 0.838667; inputs[43][8] = 0.416;  inputs[43][9] = 0.736;  inputs[43][10] = 0.388; inputs[43][11] = 0.592; inputs[43][12] = 0.545333;  inputs[43][13] = 0.609333;  inputs[43][14] = 0.625333;  inputs[43][15] = 0.754667;  inputs[43][16] = 0.52;  inputs[43][17] = 0.829333;  inputs[43][18] = 0.44;  inputs[43][19] = 0.7; inputs[44][0] = 0.470667; inputs[44][1] = 0.657333; inputs[44][2] = 0.478667; inputs[44][3] = 0.605333; inputs[44][4] = 0.501333; inputs[44][5] = 0.592;  inputs[44][6] = 0.570667; inputs[44][7] = 0.754667; inputs[44][8] = 0.472;  inputs[44][9] = 0.849333; inputs[44][10] = 0.368; inputs[44][11] = 0.658667;  inputs[44][12] = 0.437333;  inputs[44][13] = 0.493333;  inputs[44][14] = 0.596; inputs[44][15] = 0.692; inputs[44][16] = 0.530667;  inputs[44][17] = 0.772; inputs[44][18] = 0.352; inputs[44][19] = 0.88;  inputs[45][0] = 0.474667; inputs[45][1] = 0.664;  inputs[45][2] = 0.517333; inputs[45][3] = 0.597333; inputs[45][4] = 0.524;  inputs[45][5] = 0.701333; inputs[45][6] = 0.56; inputs[45][7] = 0.717333; inputs[45][8] = 0.496;  inputs[45][9] = 0.869333; inputs[45][10] = 0.358667;  inputs[45][11] = 0.662667;  inputs[45][12] = 0.438667;  inputs[45][13] = 0.578667;  inputs[45][14] = 0.562667;  inputs[45][15] = 0.604; inputs[45][16] = 0.556; inputs[45][17] = 0.741333;  inputs[45][18] = 0.497333;  inputs[45][19] = 0.853333;  inputs[46][0] = 0.445333; inputs[46][1] = 0.629333; inputs[46][2] = 0.48; inputs[46][3] = 0.616;  inputs[46][4] = 0.577333; inputs[46][5] = 0.625333; inputs[46][6] = 0.517333; inputs[46][7] = 0.838667; inputs[46][8] = 0.428;  inputs[46][9] = 0.76; inputs[46][10] = 0.366667;  inputs[46][11] = 0.568; inputs[46][12] = 0.474667;  inputs[46][13] = 0.553333;  inputs[46][14] = 0.586667;  inputs[46][15] = 0.741333;  inputs[46][16] = 0.504; inputs[46][17] = 0.845333;  inputs[46][18] = 0.438667;  inputs[46][19] = 0.636; inputs[47][0] = 0.496;  inputs[47][1] = 0.717333; inputs[47][2] = 0.36; inputs[47][3] = 0.666667; inputs[47][4] = 0.476;  inputs[47][5] = 0.533333; inputs[47][6] = 0.54; inputs[47][7] = 0.638667; inputs[47][8] = 0.604;  inputs[47][9] = 0.762667; inputs[47][10] = 0.494667;  inputs[47][11] = 0.849333;  inputs[47][12] = 0.450667;  inputs[47][13] = 0.72;  inputs[47][14] = 0.453333;  inputs[47][15] = 0.558667;  inputs[47][16] = 0.481333;  inputs[47][17] = 0.582667;  inputs[47][18] = 0.470667;  inputs[47][19] = 0.589333;  inputs[48][0] = 0.445333; inputs[48][1] = 0.648;  inputs[48][2] = 0.424;  inputs[48][3] = 0.7;  inputs[48][4] = 0.513333; inputs[48][5] = 0.688;  inputs[48][6] = 0.552;  inputs[48][7] = 0.681333; inputs[48][8] = 0.544;  inputs[48][9] = 0.674667; inputs[48][10] = 0.516; inputs[48][11] = 0.674667;  inputs[48][12] = 0.517333;  inputs[48][13] = 0.661333;  inputs[48][14] = 0.508; inputs[48][15] = 0.666667;  inputs[48][16] = 0.433333;  inputs[48][17] = 0.657333;  inputs[48][18] = 0.437333;  inputs[48][19] = 0.674667;  inputs[49][0] = 0.454667; inputs[49][1] = 0.554667; inputs[49][2] = 0.526667; inputs[49][3] = 0.632;  inputs[49][4] = 0.577333; inputs[49][5] = 0.754667; inputs[49][6] = 0.488;  inputs[49][7] = 0.82; inputs[49][8] = 0.444;  inputs[49][9] = 0.792;  inputs[49][10] = 0.46;  inputs[49][11] = 0.654667;  inputs[49][12] = 0.496; inputs[49][13] = 0.597333;  inputs[49][14] = 0.48;  inputs[49][15] = 0.586667;  inputs[49][16] = 0.472; inputs[49][17] = 0.605333;  inputs[49][18] = 0.524; inputs[49][19] = 0.764; inputs[50][0] = 0.54; inputs[50][1] = 0.716;  inputs[50][2] = 0.549333; inputs[50][3] = 0.746667; inputs[50][4] = 0.485333; inputs[50][5] = 0.6;  inputs[50][6] = 0.416;  inputs[50][7] = 0.561333; inputs[50][8] = 0.422667; inputs[50][9] = 0.624;  inputs[50][10] = 0.513333;  inputs[50][11] = 0.733333;  inputs[50][12] = 0.598667;  inputs[50][13] = 0.714667;  inputs[50][14] = 0.476; inputs[50][15] = 0.682667;  inputs[50][16] = 0.44;  inputs[50][17] = 0.710667;  inputs[50][18] = 0.488; inputs[50][19] = 0.710667;  inputs[51][0] = 0.490667; inputs[51][1] = 0.638667; inputs[51][2] = 0.542667; inputs[51][3] = 0.721333; inputs[51][4] = 0.513333; inputs[51][5] = 0.697333; inputs[51][6] = 0.481333; inputs[51][7] = 0.645333; inputs[51][8] = 0.421333; inputs[51][9] = 0.608;  inputs[51][10] = 0.444; inputs[51][11] = 0.578667;  inputs[51][12] = 0.429333;  inputs[51][13] = 0.712; inputs[51][14] = 0.556; inputs[51][15] = 0.728; inputs[51][16] = 0.617333;  inputs[51][17] = 0.682667;  inputs[51][18] = 0.426667;  inputs[51][19] = 0.690667;  inputs[52][0] = 0.478667; inputs[52][1] = 0.642667; inputs[52][2] = 0.484;  inputs[52][3] = 0.632;  inputs[52][4] = 0.490667; inputs[52][5] = 0.612;  inputs[52][6] = 0.544;  inputs[52][7] = 0.604;  inputs[52][8] = 0.537333; inputs[52][9] = 0.781333; inputs[52][10] = 0.382667;  inputs[52][11] = 0.801333;  inputs[52][12] = 0.401333;  inputs[52][13] = 0.590667;  inputs[52][14] = 0.5; inputs[52][15] = 0.569333;  inputs[52][16] = 0.622667;  inputs[52][17] = 0.681333;  inputs[52][18] = 0.521333;  inputs[52][19] = 0.848; inputs[53][0] = 0.457333; inputs[53][1] = 0.601333; inputs[53][2] = 0.569333; inputs[53][3] = 0.564;  inputs[53][4] = 0.576;  inputs[53][5] = 0.757333; inputs[53][6] = 0.473333; inputs[53][7] = 0.890667; inputs[53][8] = 0.370667; inputs[53][9] = 0.604;  inputs[53][10] = 0.445333;  inputs[53][11] = 0.525333;  inputs[53][12] = 0.493333;  inputs[53][13] = 0.604; inputs[53][14] = 0.590667;  inputs[53][15] = 0.817333;  inputs[53][16] = 0.428; inputs[53][17] = 0.8; inputs[53][18] = 0.434667;  inputs[53][19] = 0.617333;  inputs[54][0] = 0.5;  inputs[54][1] = 0.685333; inputs[54][2] = 0.576;  inputs[54][3] = 0.765333; inputs[54][4] = 0.497333; inputs[54][5] = 0.709333; inputs[54][6] = 0.445333; inputs[54][7] = 0.546667; inputs[54][8] = 0.412;  inputs[54][9] = 0.524;  inputs[54][10] = 0.404; inputs[54][11] = 0.752; inputs[54][12] = 0.530667;  inputs[54][13] = 0.758667;  inputs[54][14] = 0.632; inputs[54][15] = 0.684; inputs[54][16] = 0.413333;  inputs[54][17] = 0.724; inputs[54][18] = 0.434667;  inputs[54][19] = 0.688; inputs[55][0] = 0.488;  inputs[55][1] = 0.685333; inputs[55][2] = 0.54; inputs[55][3] = 0.741333; inputs[55][4] = 0.514667; inputs[55][5] = 0.74; inputs[55][6] = 0.502667; inputs[55][7] = 0.568;  inputs[55][8] = 0.444;  inputs[55][9] = 0.542667; inputs[55][10] = 0.378667;  inputs[55][11] = 0.625333;  inputs[55][12] = 0.492; inputs[55][13] = 0.756; inputs[55][14] = 0.617333;  inputs[55][15] = 0.712; inputs[55][16] = 0.393333;  inputs[55][17] = 0.684; inputs[55][18] = 0.408; inputs[55][19] = 0.713333;  inputs[56][0] = 0.532;  inputs[56][1] = 0.72; inputs[56][2] = 0.578667; inputs[56][3] = 0.712;  inputs[56][4] = 0.44; inputs[56][5] = 0.598667; inputs[56][6] = 0.389333; inputs[56][7] = 0.537333; inputs[56][8] = 0.406667; inputs[56][9] = 0.696;  inputs[56][10] = 0.541333;  inputs[56][11] = 0.732; inputs[56][12] = 0.658667;  inputs[56][13] = 0.674667;  inputs[56][14] = 0.321333;  inputs[56][15] = 0.758667;  inputs[56][16] = 0.444; inputs[56][17] = 0.701333;  inputs[56][18] = 0.390667;  inputs[56][19] = 0.725333;  inputs[57][0] = 0.548;  inputs[57][1] = 0.697333; inputs[57][2] = 0.488;  inputs[57][3] = 0.644;  inputs[57][4] = 0.437333; inputs[57][5] = 0.589333; inputs[57][6] = 0.454667; inputs[57][7] = 0.58; inputs[57][8] = 0.46; inputs[57][9] = 0.733333; inputs[57][10] = 0.553333;  inputs[57][11] = 0.724; inputs[57][12] = 0.572; inputs[57][13] = 0.692; inputs[57][14] = 0.432; inputs[57][15] = 0.717333;  inputs[57][16] = 0.472; inputs[57][17] = 0.702667;  inputs[57][18] = 0.406667;  inputs[57][19] = 0.7; inputs[58][0] = 0.429333; inputs[58][1] = 0.609333; inputs[58][2] = 0.38; inputs[58][3] = 0.682667; inputs[58][4] = 0.537333; inputs[58][5] = 0.7;  inputs[58][6] = 0.564;  inputs[58][7] = 0.688;  inputs[58][8] = 0.528;  inputs[58][9] = 0.68; inputs[58][10] = 0.532; inputs[58][11] = 0.670667;  inputs[58][12] = 0.530667;  inputs[58][13] = 0.66;  inputs[58][14] = 0.414667;  inputs[58][15] = 0.666667;  inputs[58][16] = 0.373333;  inputs[58][17] = 0.678667;  inputs[58][18] = 0.468; inputs[58][19] = 0.698667;  inputs[59][0] = 0.493333; inputs[59][1] = 0.690667; inputs[59][2] = 0.604;  inputs[59][3] = 0.756;  inputs[59][4] = 0.490667; inputs[59][5] = 0.662667; inputs[59][6] = 0.401333; inputs[59][7] = 0.594667; inputs[59][8] = 0.428;  inputs[59][9] = 0.592;  inputs[59][10] = 0.445333;  inputs[59][11] = 0.738667;  inputs[59][12] = 0.522667;  inputs[59][13] = 0.717333;  inputs[59][14] = 0.588; inputs[59][15] = 0.694667;  inputs[59][16] = 0.469333;  inputs[59][17] = 0.694667;  inputs[59][18] = 0.456; inputs[59][19] = 0.678667;  inputs[60][0] = 0.5;  inputs[60][1] = 0.682667; inputs[60][2] = 0.592;  inputs[60][3] = 0.738667; inputs[60][4] = 0.514667; inputs[60][5] = 0.645333; inputs[60][6] = 0.386667; inputs[60][7] = 0.618667; inputs[60][8] = 0.412;  inputs[60][9] = 0.536;  inputs[60][10] = 0.473333;  inputs[60][11] = 0.753333;  inputs[60][12] = 0.537333;  inputs[60][13] = 0.74;  inputs[60][14] = 0.62;  inputs[60][15] = 0.652; inputs[60][16] = 0.384; inputs[60][17] = 0.717333;  inputs[60][18] = 0.470667;  inputs[60][19] = 0.696; inputs[61][0] = 0.437333; inputs[61][1] = 0.761333; inputs[61][2] = 0.428;  inputs[61][3] = 0.512;  inputs[61][4] = 0.542667; inputs[61][5] = 0.566667; inputs[61][6] = 0.634667; inputs[61][7] = 0.753333; inputs[61][8] = 0.532;  inputs[61][9] = 0.869333; inputs[61][10] = 0.44;  inputs[61][11] = 0.782667;  inputs[61][12] = 0.425333;  inputs[61][13] = 0.594667;  inputs[61][14] = 0.434667;  inputs[61][15] = 0.554667;  inputs[61][16] = 0.424; inputs[61][17] = 0.613333;  inputs[61][18] = 0.48;  inputs[61][19] = 0.793333;  inputs[62][0] = 0.574667; inputs[62][1] = 0.733333; inputs[62][2] = 0.548;  inputs[62][3] = 0.681333; inputs[62][4] = 0.437333; inputs[62][5] = 0.612;  inputs[62][6] = 0.426667; inputs[62][7] = 0.590667; inputs[62][8] = 0.428;  inputs[62][9] = 0.674667; inputs[62][10] = 0.508; inputs[62][11] = 0.728; inputs[62][12] = 0.564; inputs[62][13] = 0.714667;  inputs[62][14] = 0.585333;  inputs[62][15] = 0.693333;  inputs[62][16] = 0.416; inputs[62][17] = 0.717333;  inputs[62][18] = 0.485333;  inputs[62][19] = 0.689333;  inputs[63][0] = 0.434667; inputs[63][1] = 0.658667; inputs[63][2] = 0.372;  inputs[63][3] = 0.68; inputs[63][4] = 0.505333; inputs[63][5] = 0.726667; inputs[63][6] = 0.642667; inputs[63][7] = 0.689333; inputs[63][8] = 0.56; inputs[63][9] = 0.694667; inputs[63][10] = 0.533333;  inputs[63][11] = 0.708; inputs[63][12] = 0.557333;  inputs[63][13] = 0.674667;  inputs[63][14] = 0.502667;  inputs[63][15] = 0.641333;  inputs[63][16] = 0.413333;  inputs[63][17] = 0.701333;  inputs[63][18] = 0.452; inputs[63][19] = 0.645333;  inputs[64][0] = 0.425333; inputs[64][1] = 0.604;  inputs[64][2] = 0.508;  inputs[64][3] = 0.608;  inputs[64][4] = 0.577333; inputs[64][5] = 0.68; inputs[64][6] = 0.558667; inputs[64][7] = 0.84; inputs[64][8] = 0.442667; inputs[64][9] = 0.768;  inputs[64][10] = 0.392; inputs[64][11] = 0.588; inputs[64][12] = 0.476; inputs[64][13] = 0.588; inputs[64][14] = 0.572; inputs[64][15] = 0.661333;  inputs[64][16] = 0.561333;  inputs[64][17] = 0.833333;  inputs[64][18] = 0.465333;  inputs[64][19] = 0.862667;  inputs[65][0] = 0.405333; inputs[65][1] = 0.64; inputs[65][2] = 0.528;  inputs[65][3] = 0.533333; inputs[65][4] = 0.62; inputs[65][5] = 0.678667; inputs[65][6] = 0.604;  inputs[65][7] = 0.845333; inputs[65][8] = 0.493333; inputs[65][9] = 0.888;  inputs[65][10] = 0.416; inputs[65][11] = 0.692; inputs[65][12] = 0.434667;  inputs[65][13] = 0.548; inputs[65][14] = 0.497333;  inputs[65][15] = 0.533333;  inputs[65][16] = 0.486667;  inputs[65][17] = 0.824; inputs[65][18] = 0.52;  inputs[65][19] = 0.824; inputs[66][0] = 0.369333; inputs[66][1] = 0.669333; inputs[66][2] = 0.317333; inputs[66][3] = 0.676;  inputs[66][4] = 0.477333; inputs[66][5] = 0.718667; inputs[66][6] = 0.549333; inputs[66][7] = 0.730667; inputs[66][8] = 0.574667; inputs[66][9] = 0.690667; inputs[66][10] = 0.566667;  inputs[66][11] = 0.677333;  inputs[66][12] = 0.552; inputs[66][13] = 0.688; inputs[66][14] = 0.425333;  inputs[66][15] = 0.685333;  inputs[66][16] = 0.377333;  inputs[66][17] = 0.694667;  inputs[66][18] = 0.492; inputs[66][19] = 0.688; inputs[67][0] = 0.454667; inputs[67][1] = 0.656;  inputs[67][2] = 0.461333; inputs[67][3] = 0.553333; inputs[67][4] = 0.544;  inputs[67][5] = 0.685333; inputs[67][6] = 0.529333; inputs[67][7] = 0.792;  inputs[67][8] = 0.397333; inputs[67][9] = 0.777333; inputs[67][10] = 0.376; inputs[67][11] = 0.512; inputs[67][12] = 0.524; inputs[67][13] = 0.585333;  inputs[67][14] = 0.62;  inputs[67][15] = 0.709333;  inputs[67][16] = 0.528; inputs[67][17] = 0.812; inputs[67][18] = 0.410667;  inputs[67][19] = 0.78;  inputs[68][0] = 0.513333; inputs[68][1] = 0.682667; inputs[68][2] = 0.402667; inputs[68][3] = 0.588;  inputs[68][4] = 0.418667; inputs[68][5] = 0.692;  inputs[68][6] = 0.468;  inputs[68][7] = 0.696;  inputs[68][8] = 0.646667; inputs[68][9] = 0.702667; inputs[68][10] = 0.584; inputs[68][11] = 0.688; inputs[68][12] = 0.521333;  inputs[68][13] = 0.650667;  inputs[68][14] = 0.509333;  inputs[68][15] = 0.648; inputs[68][16] = 0.344; inputs[68][17] = 0.68;  inputs[68][18] = 0.418667;  inputs[68][19] = 0.657333;  inputs[69][0] = 0.350667; inputs[69][1] = 0.669333; inputs[69][2] = 0.268;  inputs[69][3] = 0.657333; inputs[69][4] = 0.513333; inputs[69][5] = 0.701333; inputs[69][6] = 0.614667; inputs[69][7] = 0.716;  inputs[69][8] = 0.568;  inputs[69][9] = 0.697333; inputs[69][10] = 0.546667;  inputs[69][11] = 0.669333;  inputs[69][12] = 0.526667;  inputs[69][13] = 0.696; inputs[69][14] = 0.344; inputs[69][15] = 0.684; inputs[69][16] = 0.468; inputs[69][17] = 0.674667;  inputs[69][18] = 0.492; inputs[69][19] = 0.661333;  inputs[70][0] = 0.461333; inputs[70][1] = 0.630667; inputs[70][2] = 0.494667; inputs[70][3] = 0.56; inputs[70][4] = 0.530667; inputs[70][5] = 0.714667; inputs[70][6] = 0.509333; inputs[70][7] = 0.786667; inputs[70][8] = 0.425333; inputs[70][9] = 0.774667; inputs[70][10] = 0.392; inputs[70][11] = 0.542667;  inputs[70][12] = 0.488; inputs[70][13] = 0.592; inputs[70][14] = 0.577333;  inputs[70][15] = 0.690667;  inputs[70][16] = 0.513333;  inputs[70][17] = 0.810667;  inputs[70][18] = 0.432; inputs[70][19] = 0.792; inputs[71][0] = 0.573333; inputs[71][1] = 0.618667; inputs[71][2] = 0.488;  inputs[71][3] = 0.786667; inputs[71][4] = 0.368;  inputs[71][5] = 0.765333; inputs[71][6] = 0.498667; inputs[71][7] = 0.581333; inputs[71][8] = 0.536;  inputs[71][9] = 0.502667; inputs[71][10] = 0.406667;  inputs[71][11] = 0.652; inputs[71][12] = 0.465333;  inputs[71][13] = 0.697333;  inputs[71][14] = 0.565333;  inputs[71][15] = 0.694667;  inputs[71][16] = 0.630667;  inputs[71][17] = 0.733333;  inputs[71][18] = 0.370667;  inputs[71][19] = 0.717333;  inputs[72][0] = 0.470667; inputs[72][1] = 0.618667; inputs[72][2] = 0.481333; inputs[72][3] = 0.592;  inputs[72][4] = 0.538667; inputs[72][5] = 0.766667; inputs[72][6] = 0.498667; inputs[72][7] = 0.84; inputs[72][8] = 0.402667; inputs[72][9] = 0.682667; inputs[72][10] = 0.434667;  inputs[72][11] = 0.526667;  inputs[72][12] = 0.524; inputs[72][13] = 0.618667;  inputs[72][14] = 0.598667;  inputs[72][15] = 0.777333;  inputs[72][16] = 0.484; inputs[72][17] = 0.876; inputs[72][18] = 0.464; inputs[72][19] = 0.592; inputs[73][0] = 0.46; inputs[73][1] = 0.670667; inputs[73][2] = 0.474667; inputs[73][3] = 0.524;  inputs[73][4] = 0.610667; inputs[73][5] = 0.76; inputs[73][6] = 0.496;  inputs[73][7] = 0.853333; inputs[73][8] = 0.369333; inputs[73][9] = 0.610667; inputs[73][10] = 0.442667;  inputs[73][11] = 0.537333;  inputs[73][12] = 0.62;  inputs[73][13] = 0.688; inputs[73][14] = 0.565333;  inputs[73][15] = 0.830667;  inputs[73][16] = 0.444; inputs[73][17] = 0.801333;  inputs[73][18] = 0.466667;  inputs[73][19] = 0.510667;  inputs[74][0] = 0.488;  inputs[74][1] = 0.628;  inputs[74][2] = 0.513333; inputs[74][3] = 0.656;  inputs[74][4] = 0.586667; inputs[74][5] = 0.793333; inputs[74][6] = 0.545333; inputs[74][7] = 0.848;  inputs[74][8] = 0.436;  inputs[74][9] = 0.750667; inputs[74][10] = 0.453333;  inputs[74][11] = 0.569333;  inputs[74][12] = 0.461333;  inputs[74][13] = 0.554667;  inputs[74][14] = 0.466667;  inputs[74][15] = 0.565333;  inputs[74][16] = 0.478667;  inputs[74][17] = 0.828; inputs[74][18] = 0.498667;  inputs[74][19] = 0.729333;  inputs[75][0] = 0.458667; inputs[75][1] = 0.632;  inputs[75][2] = 0.485333; inputs[75][3] = 0.590667; inputs[75][4] = 0.568;  inputs[75][5] = 0.746667; inputs[75][6] = 0.481333; inputs[75][7] = 0.809333; inputs[75][8] = 0.406667; inputs[75][9] = 0.688;  inputs[75][10] = 0.421333;  inputs[75][11] = 0.561333;  inputs[75][12] = 0.505333;  inputs[75][13] = 0.552; inputs[75][14] = 0.569333;  inputs[75][15] = 0.674667;  inputs[75][16] = 0.534667;  inputs[75][17] = 0.764; inputs[75][18] = 0.394667;  inputs[75][19] = 0.824; inputs[76][0] = 0.441333; inputs[76][1] = 0.696;  inputs[76][2] = 0.414667; inputs[76][3] = 0.536;  inputs[76][4] = 0.52; inputs[76][5] = 0.62; inputs[76][6] = 0.573333; inputs[76][7] = 0.76; inputs[76][8] = 0.517333; inputs[76][9] = 0.830667; inputs[76][10] = 0.445333;  inputs[76][11] = 0.724; inputs[76][12] = 0.44;  inputs[76][13] = 0.586667;  inputs[76][14] = 0.473333;  inputs[76][15] = 0.554667;  inputs[76][16] = 0.488; inputs[76][17] = 0.525333;  inputs[76][18] = 0.497333;  inputs[76][19] = 0.770667;  inputs[77][0] = 0.493333; inputs[77][1] = 0.641333; inputs[77][2] = 0.545333; inputs[77][3] = 0.690667; inputs[77][4] = 0.538667; inputs[77][5] = 0.709333; inputs[77][6] = 0.469333; inputs[77][7] = 0.668;  inputs[77][8] = 0.445333; inputs[77][9] = 0.541333; inputs[77][10] = 0.457333;  inputs[77][11] = 0.585333;  inputs[77][12] = 0.449333;  inputs[77][13] = 0.766667;  inputs[77][14] = 0.557333;  inputs[77][15] = 0.706667;  inputs[77][16] = 0.604; inputs[77][17] = 0.669333;  inputs[77][18] = 0.392; inputs[77][19] = 0.744; inputs[78][0] = 0.376;  inputs[78][1] = 0.476;  inputs[78][2] = 0.530667; inputs[78][3] = 0.588;  inputs[78][4] = 0.568;  inputs[78][5] = 0.76; inputs[78][6] = 0.494667; inputs[78][7] = 0.832;  inputs[78][8] = 0.445333; inputs[78][9] = 0.632;  inputs[78][10] = 0.452; inputs[78][11] = 0.550667;  inputs[78][12] = 0.458667;  inputs[78][13] = 0.570667;  inputs[78][14] = 0.489333;  inputs[78][15] = 0.676; inputs[78][16] = 0.497333;  inputs[78][17] = 0.762667;  inputs[78][18] = 0.493333;  inputs[78][19] = 0.686667;  inputs[79][0] = 0.445333; inputs[79][1] = 0.692;  inputs[79][2] = 0.461333; inputs[79][3] = 0.548;  inputs[79][4] = 0.537333; inputs[79][5] = 0.594667; inputs[79][6] = 0.582667; inputs[79][7] = 0.738667; inputs[79][8] = 0.546667; inputs[79][9] = 0.78; inputs[79][10] = 0.477333;  inputs[79][11] = 0.769333;  inputs[79][12] = 0.42;  inputs[79][13] = 0.616; inputs[79][14] = 0.453333;  inputs[79][15] = 0.577333;  inputs[79][16] = 0.477333;  inputs[79][17] = 0.590667;  inputs[79][18] = 0.442667;  inputs[79][19] = 0.504; inputs[80][0] = 0.424;  inputs[80][1] = 0.586667; inputs[80][2] = 0.352;  inputs[80][3] = 0.721333; inputs[80][4] = 0.48; inputs[80][5] = 0.704;  inputs[80][6] = 0.556;  inputs[80][7] = 0.685333; inputs[80][8] = 0.548;  inputs[80][9] = 0.666667; inputs[80][10] = 0.521333;  inputs[80][11] = 0.657333;  inputs[80][12] = 0.536; inputs[80][13] = 0.633333;  inputs[80][14] = 0.568; inputs[80][15] = 0.648; inputs[80][16] = 0.385333;  inputs[80][17] = 0.684; inputs[80][18] = 0.457333;  inputs[80][19] = 0.688; inputs[81][0] = 0.546667; inputs[81][1] = 0.716;  inputs[81][2] = 0.58; inputs[81][3] = 0.701333; inputs[81][4] = 0.388;  inputs[81][5] = 0.661333; inputs[81][6] = 0.402667; inputs[81][7] = 0.577333; inputs[81][8] = 0.406667; inputs[81][9] = 0.650667; inputs[81][10] = 0.482667;  inputs[81][11] = 0.72;  inputs[81][12] = 0.572; inputs[81][13] = 0.709333;  inputs[81][14] = 0.584; inputs[81][15] = 0.688; inputs[81][16] = 0.42;  inputs[81][17] = 0.744; inputs[81][18] = 0.446667;  inputs[81][19] = 0.689333;  inputs[82][0] = 0.522667; inputs[82][1] = 0.728;  inputs[82][2] = 0.54; inputs[82][3] = 0.694667; inputs[82][4] = 0.466667; inputs[82][5] = 0.616;  inputs[82][6] = 0.428;  inputs[82][7] = 0.589333; inputs[82][8] = 0.441333; inputs[82][9] = 0.586667; inputs[82][10] = 0.481333;  inputs[82][11] = 0.736; inputs[82][12] = 0.554667;  inputs[82][13] = 0.737333;  inputs[82][14] = 0.570667;  inputs[82][15] = 0.674667;  inputs[82][16] = 0.466667;  inputs[82][17] = 0.677333;  inputs[82][18] = 0.494667;  inputs[82][19] = 0.709333;  inputs[83][0] = 0.44; inputs[83][1] = 0.734667; inputs[83][2] = 0.454667; inputs[83][3] = 0.529333; inputs[83][4] = 0.598667; inputs[83][5] = 0.661333; inputs[83][6] = 0.602667; inputs[83][7] = 0.725333; inputs[83][8] = 0.550667; inputs[83][9] = 0.826667; inputs[83][10] = 0.454667;  inputs[83][11] = 0.757333;  inputs[83][12] = 0.418667;  inputs[83][13] = 0.597333;  inputs[83][14] = 0.457333;  inputs[83][15] = 0.566667;  inputs[83][16] = 0.488; inputs[83][17] = 0.545333;  inputs[83][18] = 0.481333;  inputs[83][19] = 0.773333;  inputs[84][0] = 0.472;  inputs[84][1] = 0.618667; inputs[84][2] = 0.502667; inputs[84][3] = 0.561333; inputs[84][4] = 0.573333; inputs[84][5] = 0.796;  inputs[84][6] = 0.46; inputs[84][7] = 0.868;  inputs[84][8] = 0.410667; inputs[84][9] = 0.645333; inputs[84][10] = 0.453333;  inputs[84][11] = 0.509333;  inputs[84][12] = 0.548; inputs[84][13] = 0.626667;  inputs[84][14] = 0.497333;  inputs[84][15] = 0.857333;  inputs[84][16] = 0.432; inputs[84][17] = 0.674667;  inputs[84][18] = 0.449333;  inputs[84][19] = 0.578667;  inputs[85][0] = 0.472;  inputs[85][1] = 0.598667; inputs[85][2] = 0.554667; inputs[85][3] = 0.612;  inputs[85][4] = 0.616;  inputs[85][5] = 0.866667; inputs[85][6] = 0.418667; inputs[85][7] = 0.814667; inputs[85][8] = 0.36; inputs[85][9] = 0.597333; inputs[85][10] = 0.464; inputs[85][11] = 0.54;  inputs[85][12] = 0.538667;  inputs[85][13] = 0.724; inputs[85][14] = 0.610667;  inputs[85][15] = 0.777333;  inputs[85][16] = 0.473333;  inputs[85][17] = 0.776; inputs[85][18] = 0.441333;  inputs[85][19] = 0.694667;  inputs[86][0] = 0.462667; inputs[86][1] = 0.618667; inputs[86][2] = 0.365333; inputs[86][3] = 0.690667; inputs[86][4] = 0.686667; inputs[86][5] = 0.746667; inputs[86][6] = 0.593333; inputs[86][7] = 0.697333; inputs[86][8] = 0.537333; inputs[86][9] = 0.669333; inputs[86][10] = 0.561333;  inputs[86][11] = 0.649333;  inputs[86][12] = 0.488; inputs[86][13] = 0.685333;  inputs[86][14] = 0.417333;  inputs[86][15] = 0.682667;  inputs[86][16] = 0.450667;  inputs[86][17] = 0.68;  inputs[86][18] = 0.481333;  inputs[86][19] = 0.684; inputs[87][0] = 0.534667; inputs[87][1] = 0.769333; inputs[87][2] = 0.546667; inputs[87][3] = 0.725333; inputs[87][4] = 0.453333; inputs[87][5] = 0.573333; inputs[87][6] = 0.389333; inputs[87][7] = 0.541333; inputs[87][8] = 0.424;  inputs[87][9] = 0.632;  inputs[87][10] = 0.513333;  inputs[87][11] = 0.744; inputs[87][12] = 0.562667;  inputs[87][13] = 0.724; inputs[87][14] = 0.586667;  inputs[87][15] = 0.654667;  inputs[87][16] = 0.44;  inputs[87][17] = 0.689333;  inputs[87][18] = 0.478667;  inputs[87][19] = 0.716; inputs[88][0] = 0.462667; inputs[88][1] = 0.630667; inputs[88][2] = 0.478667; inputs[88][3] = 0.541333; inputs[88][4] = 0.586667; inputs[88][5] = 0.762667; inputs[88][6] = 0.510667; inputs[88][7] = 0.876;  inputs[88][8] = 0.389333; inputs[88][9] = 0.725333; inputs[88][10] = 0.404; inputs[88][11] = 0.526667;  inputs[88][12] = 0.578667;  inputs[88][13] = 0.594667;  inputs[88][14] = 0.624; inputs[88][15] = 0.733333;  inputs[88][16] = 0.489333;  inputs[88][17] = 0.801333;  inputs[88][18] = 0.4; inputs[88][19] = 0.802667;  inputs[89][0] = 0.493333; inputs[89][1] = 0.688;  inputs[89][2] = 0.596;  inputs[89][3] = 0.754667; inputs[89][4] = 0.526667; inputs[89][5] = 0.674667; inputs[89][6] = 0.422667; inputs[89][7] = 0.598667; inputs[89][8] = 0.416;  inputs[89][9] = 0.554667; inputs[89][10] = 0.470667;  inputs[89][11] = 0.748; inputs[89][12] = 0.573333;  inputs[89][13] = 0.730667;  inputs[89][14] = 0.576; inputs[89][15] = 0.696; inputs[89][16] = 0.413333;  inputs[89][17] = 0.68;  inputs[89][18] = 0.402667;  inputs[89][19] = 0.713333;  inputs[90][0] = 0.377333; inputs[90][1] = 0.698667; inputs[90][2] = 0.552;  inputs[90][3] = 0.753333; inputs[90][4] = 0.594667; inputs[90][5] = 0.712;  inputs[90][6] = 0.608;  inputs[90][7] = 0.704;  inputs[90][8] = 0.530667; inputs[90][9] = 0.68; inputs[90][10] = 0.574667;  inputs[90][11] = 0.684; inputs[90][12] = 0.496; inputs[90][13] = 0.752; inputs[90][14] = 0.393333;  inputs[90][15] = 0.709333;  inputs[90][16] = 0.494667;  inputs[90][17] = 0.688; inputs[90][18] = 0.501333;  inputs[90][19] = 0.682667;  inputs[91][0] = 0.441333; inputs[91][1] = 0.590667; inputs[91][2] = 0.530667; inputs[91][3] = 0.632;  inputs[91][4] = 0.588;  inputs[91][5] = 0.717333; inputs[91][6] = 0.538667; inputs[91][7] = 0.816;  inputs[91][8] = 0.497333; inputs[91][9] = 0.798667; inputs[91][10] = 0.445333;  inputs[91][11] = 0.618667;  inputs[91][12] = 0.422667;  inputs[91][13] = 0.601333;  inputs[91][14] = 0.445333;  inputs[91][15] = 0.589333;  inputs[91][16] = 0.498667;  inputs[91][17] = 0.772; inputs[91][18] = 0.468; inputs[91][19] = 0.784; inputs[92][0] = 0.442667; inputs[92][1] = 0.638667; inputs[92][2] = 0.384;  inputs[92][3] = 0.657333; inputs[92][4] = 0.548;  inputs[92][5] = 0.712;  inputs[92][6] = 0.578667; inputs[92][7] = 0.688;  inputs[92][8] = 0.578667; inputs[92][9] = 0.665333; inputs[92][10] = 0.550667;  inputs[92][11] = 0.682667;  inputs[92][12] = 0.582667;  inputs[92][13] = 0.650667;  inputs[92][14] = 0.5; inputs[92][15] = 0.697333;  inputs[92][16] = 0.394667;  inputs[92][17] = 0.74;  inputs[92][18] = 0.454667;  inputs[92][19] = 0.66;  inputs[93][0] = 0.501333; inputs[93][1] = 0.689333; inputs[93][2] = 0.570667; inputs[93][3] = 0.716;  inputs[93][4] = 0.498667; inputs[93][5] = 0.674667; inputs[93][6] = 0.424;  inputs[93][7] = 0.6;  inputs[93][8] = 0.424;  inputs[93][9] = 0.574667; inputs[93][10] = 0.445333;  inputs[93][11] = 0.706667;  inputs[93][12] = 0.533333;  inputs[93][13] = 0.742667;  inputs[93][14] = 0.556; inputs[93][15] = 0.684; inputs[93][16] = 0.593333;  inputs[93][17] = 0.689333;  inputs[93][18] = 0.44;  inputs[93][19] = 0.674667;  inputs[94][0] = 0.434667; inputs[94][1] = 0.569333; inputs[94][2] = 0.521333; inputs[94][3] = 0.648;  inputs[94][4] = 0.556;  inputs[94][5] = 0.690667; inputs[94][6] = 0.508;  inputs[94][7] = 0.816;  inputs[94][8] = 0.445333; inputs[94][9] = 0.790667; inputs[94][10] = 0.446667;  inputs[94][11] = 0.638667;  inputs[94][12] = 0.449333;  inputs[94][13] = 0.561333;  inputs[94][14] = 0.453333;  inputs[94][15] = 0.557333;  inputs[94][16] = 0.482667;  inputs[94][17] = 0.813333;  inputs[94][18] = 0.496; inputs[94][19] = 0.741333;  inputs[95][0] = 0.450667; inputs[95][1] = 0.749333; inputs[95][2] = 0.384;  inputs[95][3] = 0.526667; inputs[95][4] = 0.522667; inputs[95][5] = 0.590667; inputs[95][6] = 0.566667; inputs[95][7] = 0.724;  inputs[95][8] = 0.568;  inputs[95][9] = 0.752;  inputs[95][10] = 0.488; inputs[95][11] = 0.830667;  inputs[95][12] = 0.410667;  inputs[95][13] = 0.716; inputs[95][14] = 0.449333;  inputs[95][15] = 0.590667;  inputs[95][16] = 0.485333;  inputs[95][17] = 0.573333;  inputs[95][18] = 0.457333;  inputs[95][19] = 0.557333;  inputs[96][0] = 0.416;  inputs[96][1] = 0.674667; inputs[96][2] = 0.372;  inputs[96][3] = 0.701333; inputs[96][4] = 0.464;  inputs[96][5] = 0.682667; inputs[96][6] = 0.58; inputs[96][7] = 0.722667; inputs[96][8] = 0.534667; inputs[96][9] = 0.684;  inputs[96][10] = 0.564; inputs[96][11] = 0.662667;  inputs[96][12] = 0.549333;  inputs[96][13] = 0.674667;  inputs[96][14] = 0.290667;  inputs[96][15] = 0.701333;  inputs[96][16] = 0.44;  inputs[96][17] = 0.678667;  inputs[96][18] = 0.513333;  inputs[96][19] = 0.676; inputs[97][0] = 0.576;  inputs[97][1] = 0.76; inputs[97][2] = 0.477333; inputs[97][3] = 0.717333; inputs[97][4] = 0.526667; inputs[97][5] = 0.502667; inputs[97][6] = 0.36; inputs[97][7] = 0.573333; inputs[97][8] = 0.397333; inputs[97][9] = 0.741333; inputs[97][10] = 0.577333;  inputs[97][11] = 0.706667;  inputs[97][12] = 0.629333;  inputs[97][13] = 0.685333;  inputs[97][14] = 0.408; inputs[97][15] = 0.681333;  inputs[97][16] = 0.392; inputs[97][17] = 0.676; inputs[97][18] = 0.481333;  inputs[97][19] = 0.710667;  inputs[98][0] = 0.481333; inputs[98][1] = 0.573333; inputs[98][2] = 0.52; inputs[98][3] = 0.66; inputs[98][4] = 0.544;  inputs[98][5] = 0.714667; inputs[98][6] = 0.498667; inputs[98][7] = 0.789333; inputs[98][8] = 0.457333; inputs[98][9] = 0.757333; inputs[98][10] = 0.396; inputs[98][11] = 0.621333;  inputs[98][12] = 0.456; inputs[98][13] = 0.564; inputs[98][14] = 0.521333;  inputs[98][15] = 0.646667;  inputs[98][16] = 0.48;  inputs[98][17] = 0.606667;  inputs[98][18] = 0.417333;  inputs[98][19] = 0.824; inputs[99][0] = 0.453333; inputs[99][1] = 0.605333; inputs[99][2] = 0.589333; inputs[99][3] = 0.577333; inputs[99][4] = 0.537333; inputs[99][5] = 0.785333; inputs[99][6] = 0.426667; inputs[99][7] = 0.822667; inputs[99][8] = 0.34; inputs[99][9] = 0.533333; inputs[99][10] = 0.508; inputs[99][11] = 0.553333;  inputs[99][12] = 0.578667;  inputs[99][13] = 0.726667;  inputs[99][14] = 0.486667;  inputs[99][15] = 0.781333;  inputs[99][16] = 0.416; inputs[99][17] = 0.778667;  inputs[99][18] = 0.470667;  inputs[99][19] = 0.513333;  inputs[100][0] = 0.462667;  inputs[100][1] = 0.648; inputs[100][2] = 0.474667;  inputs[100][3] = 0.573333;  inputs[100][4] = 0.577333;  inputs[100][5] = 0.686667;  inputs[100][6] = 0.525333;  inputs[100][7] = 0.784; inputs[100][8] = 0.416; inputs[100][9] = 0.778667;  inputs[100][10] = 0.386667; inputs[100][11] = 0.569333; inputs[100][12] = 0.52; inputs[100][13] = 0.564;  inputs[100][14] = 0.561333; inputs[100][15] = 0.705333; inputs[100][16] = 0.474667; inputs[100][17] = 0.749333; inputs[100][18] = 0.402667; inputs[100][19] = 0.748;  inputs[101][0] = 0.378667;  inputs[101][1] = 0.633333;  inputs[101][2] = 0.353333;  inputs[101][3] = 0.733333;  inputs[101][4] = 0.449333;  inputs[101][5] = 0.718667;  inputs[101][6] = 0.562667;  inputs[101][7] = 0.710667;  inputs[101][8] = 0.585333;  inputs[101][9] = 0.665333;  inputs[101][10] = 0.562667; inputs[101][11] = 0.670667; inputs[101][12] = 0.529333; inputs[101][13] = 0.646667; inputs[101][14] = 0.24; inputs[101][15] = 0.632;  inputs[101][16] = 0.369333; inputs[101][17] = 0.636;  inputs[101][18] = 0.461333; inputs[101][19] = 0.677333;
  #endif
}
void fill_static_matrix_outputs(){
//(102, 4)
  #ifdef TRAINING
  outputs[0][0] = 0;  outputs[0][1] = 0;  outputs[0][2] = 1;  outputs[0][3] = 0;  outputs[1][0] = 0;  outputs[1][1] = 0;  outputs[1][2] = 0;  outputs[1][3] = 1;  outputs[2][0] = 1;  outputs[2][1] = 0;  outputs[2][2] = 0;  outputs[2][3] = 0;  outputs[3][0] = 0;  outputs[3][1] = 1;  outputs[3][2] = 0;  outputs[3][3] = 0;  outputs[4][0] = 0;  outputs[4][1] = 1;  outputs[4][2] = 0;  outputs[4][3] = 0;  outputs[5][0] = 1;  outputs[5][1] = 0;  outputs[5][2] = 0;  outputs[5][3] = 0;  outputs[6][0] = 0;  outputs[6][1] = 0;  outputs[6][2] = 0;  outputs[6][3] = 1;  outputs[7][0] = 1;  outputs[7][1] = 0;  outputs[7][2] = 0;  outputs[7][3] = 0;  outputs[8][0] = 0;  outputs[8][1] = 1;  outputs[8][2] = 0;  outputs[8][3] = 0;  outputs[9][0] = 0;  outputs[9][1] = 1;  outputs[9][2] = 0;  outputs[9][3] = 0;  outputs[10][0] = 1; outputs[10][1] = 0; outputs[10][2] = 0; outputs[10][3] = 0; outputs[11][0] = 1; outputs[11][1] = 0; outputs[11][2] = 0; outputs[11][3] = 0; outputs[12][0] = 0; outputs[12][1] = 1; outputs[12][2] = 0; outputs[12][3] = 0; outputs[13][0] = 0; outputs[13][1] = 0; outputs[13][2] = 0; outputs[13][3] = 1; outputs[14][0] = 1; outputs[14][1] = 0; outputs[14][2] = 0; outputs[14][3] = 0; outputs[15][0] = 1; outputs[15][1] = 0; outputs[15][2] = 0; outputs[15][3] = 0; outputs[16][0] = 0; outputs[16][1] = 0; outputs[16][2] = 0; outputs[16][3] = 1; outputs[17][0] = 0; outputs[17][1] = 1; outputs[17][2] = 0; outputs[17][3] = 0; outputs[18][0] = 0; outputs[18][1] = 0; outputs[18][2] = 0; outputs[18][3] = 1; outputs[19][0] = 0; outputs[19][1] = 0; outputs[19][2] = 1; outputs[19][3] = 0; outputs[20][0] = 0; outputs[20][1] = 1; outputs[20][2] = 0; outputs[20][3] = 0; outputs[21][0] = 1; outputs[21][1] = 0; outputs[21][2] = 0; outputs[21][3] = 0; outputs[22][0] = 0; outputs[22][1] = 0; outputs[22][2] = 1; outputs[22][3] = 0; outputs[23][0] = 1; outputs[23][1] = 0; outputs[23][2] = 0; outputs[23][3] = 0; outputs[24][0] = 0; outputs[24][1] = 1; outputs[24][2] = 0; outputs[24][3] = 0; outputs[25][0] = 0; outputs[25][1] = 1; outputs[25][2] = 0; outputs[25][3] = 0; outputs[26][0] = 0; outputs[26][1] = 0; outputs[26][2] = 1; outputs[26][3] = 0; outputs[27][0] = 0; outputs[27][1] = 0; outputs[27][2] = 1; outputs[27][3] = 0; outputs[28][0] = 1; outputs[28][1] = 0; outputs[28][2] = 0; outputs[28][3] = 0; outputs[29][0] = 0; outputs[29][1] = 0; outputs[29][2] = 1; outputs[29][3] = 0; outputs[30][0] = 1; outputs[30][1] = 0; outputs[30][2] = 0; outputs[30][3] = 0; outputs[31][0] = 1; outputs[31][1] = 0; outputs[31][2] = 0; outputs[31][3] = 0; outputs[32][0] = 0; outputs[32][1] = 0; outputs[32][2] = 1; outputs[32][3] = 0; outputs[33][0] = 0; outputs[33][1] = 0; outputs[33][2] = 0; outputs[33][3] = 1; outputs[34][0] = 0; outputs[34][1] = 0; outputs[34][2] = 1; outputs[34][3] = 0; outputs[35][0] = 0; outputs[35][1] = 0; outputs[35][2] = 0; outputs[35][3] = 1; outputs[36][0] = 0; outputs[36][1] = 1; outputs[36][2] = 0; outputs[36][3] = 0; outputs[37][0] = 0; outputs[37][1] = 1; outputs[37][2] = 0; outputs[37][3] = 0; outputs[38][0] = 0; outputs[38][1] = 0; outputs[38][2] = 1; outputs[38][3] = 0; outputs[39][0] = 0; outputs[39][1] = 1; outputs[39][2] = 0; outputs[39][3] = 0; outputs[40][0] = 0; outputs[40][1] = 0; outputs[40][2] = 1; outputs[40][3] = 0; outputs[41][0] = 0; outputs[41][1] = 0; outputs[41][2] = 0; outputs[41][3] = 1; outputs[42][0] = 0; outputs[42][1] = 0; outputs[42][2] = 0; outputs[42][3] = 1; outputs[43][0] = 0; outputs[43][1] = 1; outputs[43][2] = 0; outputs[43][3] = 0; outputs[44][0] = 0; outputs[44][1] = 1; outputs[44][2] = 0; outputs[44][3] = 0; outputs[45][0] = 0; outputs[45][1] = 1; outputs[45][2] = 0; outputs[45][3] = 0; outputs[46][0] = 0; outputs[46][1] = 1; outputs[46][2] = 0; outputs[46][3] = 0; outputs[47][0] = 0; outputs[47][1] = 0; outputs[47][2] = 1; outputs[47][3] = 0; outputs[48][0] = 0; outputs[48][1] = 0; outputs[48][2] = 0; outputs[48][3] = 1; outputs[49][0] = 0; outputs[49][1] = 0; outputs[49][2] = 1; outputs[49][3] = 0; outputs[50][0] = 1; outputs[50][1] = 0; outputs[50][2] = 0; outputs[50][3] = 0; outputs[51][0] = 1; outputs[51][1] = 0; outputs[51][2] = 0; outputs[51][3] = 0; outputs[52][0] = 0; outputs[52][1] = 1; outputs[52][2] = 0; outputs[52][3] = 0; outputs[53][0] = 0; outputs[53][1] = 1; outputs[53][2] = 0; outputs[53][3] = 0; outputs[54][0] = 1; outputs[54][1] = 0; outputs[54][2] = 0; outputs[54][3] = 0; outputs[55][0] = 1; outputs[55][1] = 0; outputs[55][2] = 0; outputs[55][3] = 0; outputs[56][0] = 1; outputs[56][1] = 0; outputs[56][2] = 0; outputs[56][3] = 0; outputs[57][0] = 1; outputs[57][1] = 0; outputs[57][2] = 0; outputs[57][3] = 0; outputs[58][0] = 0; outputs[58][1] = 0; outputs[58][2] = 0; outputs[58][3] = 1; outputs[59][0] = 1; outputs[59][1] = 0; outputs[59][2] = 0; outputs[59][3] = 0; outputs[60][0] = 1; outputs[60][1] = 0; outputs[60][2] = 0; outputs[60][3] = 0; outputs[61][0] = 0; outputs[61][1] = 0; outputs[61][2] = 1; outputs[61][3] = 0; outputs[62][0] = 1; outputs[62][1] = 0; outputs[62][2] = 0; outputs[62][3] = 0; outputs[63][0] = 0; outputs[63][1] = 0; outputs[63][2] = 0; outputs[63][3] = 1; outputs[64][0] = 0; outputs[64][1] = 1; outputs[64][2] = 0; outputs[64][3] = 0; outputs[65][0] = 0; outputs[65][1] = 0; outputs[65][2] = 1; outputs[65][3] = 0; outputs[66][0] = 0; outputs[66][1] = 0; outputs[66][2] = 0; outputs[66][3] = 1; outputs[67][0] = 0; outputs[67][1] = 1; outputs[67][2] = 0; outputs[67][3] = 0; outputs[68][0] = 0; outputs[68][1] = 0; outputs[68][2] = 0; outputs[68][3] = 1; outputs[69][0] = 0; outputs[69][1] = 0; outputs[69][2] = 0; outputs[69][3] = 1; outputs[70][0] = 0; outputs[70][1] = 1; outputs[70][2] = 0; outputs[70][3] = 0; outputs[71][0] = 1; outputs[71][1] = 0; outputs[71][2] = 0; outputs[71][3] = 0; outputs[72][0] = 0; outputs[72][1] = 1; outputs[72][2] = 0; outputs[72][3] = 0; outputs[73][0] = 0; outputs[73][1] = 1; outputs[73][2] = 0; outputs[73][3] = 0; outputs[74][0] = 0; outputs[74][1] = 0; outputs[74][2] = 1; outputs[74][3] = 0; outputs[75][0] = 0; outputs[75][1] = 1; outputs[75][2] = 0; outputs[75][3] = 0; outputs[76][0] = 0; outputs[76][1] = 0; outputs[76][2] = 1; outputs[76][3] = 0; outputs[77][0] = 1; outputs[77][1] = 0; outputs[77][2] = 0; outputs[77][3] = 0; outputs[78][0] = 0; outputs[78][1] = 0; outputs[78][2] = 1; outputs[78][3] = 0; outputs[79][0] = 0; outputs[79][1] = 0; outputs[79][2] = 1; outputs[79][3] = 0; outputs[80][0] = 0; outputs[80][1] = 0; outputs[80][2] = 0; outputs[80][3] = 1; outputs[81][0] = 1; outputs[81][1] = 0; outputs[81][2] = 0; outputs[81][3] = 0; outputs[82][0] = 1; outputs[82][1] = 0; outputs[82][2] = 0; outputs[82][3] = 0; outputs[83][0] = 0; outputs[83][1] = 0; outputs[83][2] = 1; outputs[83][3] = 0; outputs[84][0] = 0; outputs[84][1] = 1; outputs[84][2] = 0; outputs[84][3] = 0; outputs[85][0] = 0; outputs[85][1] = 1; outputs[85][2] = 0; outputs[85][3] = 0; outputs[86][0] = 0; outputs[86][1] = 0; outputs[86][2] = 0; outputs[86][3] = 1; outputs[87][0] = 1; outputs[87][1] = 0; outputs[87][2] = 0; outputs[87][3] = 0; outputs[88][0] = 0; outputs[88][1] = 1; outputs[88][2] = 0; outputs[88][3] = 0; outputs[89][0] = 1; outputs[89][1] = 0; outputs[89][2] = 0; outputs[89][3] = 0; outputs[90][0] = 0; outputs[90][1] = 0; outputs[90][2] = 0; outputs[90][3] = 1; outputs[91][0] = 0; outputs[91][1] = 0; outputs[91][2] = 1; outputs[91][3] = 0; outputs[92][0] = 0; outputs[92][1] = 0; outputs[92][2] = 0; outputs[92][3] = 1; outputs[93][0] = 1; outputs[93][1] = 0; outputs[93][2] = 0; outputs[93][3] = 0; outputs[94][0] = 0; outputs[94][1] = 0; outputs[94][2] = 1; outputs[94][3] = 0; outputs[95][0] = 0; outputs[95][1] = 0; outputs[95][2] = 1; outputs[95][3] = 0; outputs[96][0] = 0; outputs[96][1] = 0; outputs[96][2] = 0; outputs[96][3] = 1; outputs[97][0] = 1; outputs[97][1] = 0; outputs[97][2] = 0; outputs[97][3] = 0; outputs[98][0] = 0; outputs[98][1] = 0; outputs[98][2] = 1; outputs[98][3] = 0; outputs[99][0] = 0; outputs[99][1] = 1; outputs[99][2] = 0; outputs[99][3] = 0; outputs[100][0] = 0;  outputs[100][1] = 1;  outputs[100][2] = 0;  outputs[100][3] = 0;  outputs[101][0] = 0;  outputs[101][1] = 0;  outputs[101][2] = 0;  outputs[101][3] = 1;
  #endif
}
void fill_matrix_circle(){
  
  circle[0][0]=4.027636;circle[0][1]=20.036720;circle[0][2]=21.374531;circle[0][3]=13.819603;circle[0][4]=-15.183036;circle[0][5]=8.259226;circle[0][6]=-39.628671;circle[0][7]=40.112691;circle[0][8]=-5.257105;circle[0][9]=52.359172;circle[0][10]=7.488636;circle[0][11]=30.522470;circle[0][12]=-10.730234;circle[0][13]=20.895870;circle[0][14]=-11.463509;circle[0][15]=21.379651;circle[0][16]=-10.041735;circle[0][17]=22.435019;circle[0][18]=-9.363231;circle[0][19]=22.803003;circle[0][20]=-8.855651;circle[0][21]=22.931312;circle[0][22]=-8.298347;circle[0][23]=22.976050;circle[0][24]=-8.104027;circle[0][25]=22.991649;circle[0][26]=-8.036272;circle[0][27]=22.997088;circle[0][28]=-8.012647;circle[0][29]=22.998985;circle[1][0]=-6.328865;circle[1][1]=15.940137;circle[1][2]=-6.021133;circle[1][3]=25.318457;circle[1][4]=9.927491;circle[1][5]=16.848542;circle[1][6]=-13.120889;circle[1][7]=4.817339;circle[1][8]=-51.453486;circle[1][9]=10.787010;circle[1][10]=-14.915039;circle[1][11]=23.126648;circle[1][12]=12.838208;circle[1][13]=12.201853;circle[1][14]=-17.743613;circle[1][15]=6.150011;circle[1][16]=-20.085746;circle[1][17]=12.857930;circle[1][18]=-14.191405;circle[1][19]=18.161036;circle[1][20]=-10.810131;circle[1][21]=20.110114;circle[1][22]=-9.631154;circle[1][23]=22.514062;circle[1][24]=-9.220070;circle[1][25]=23.481886;circle[1][26]=-9.076734;circle[1][27]=23.819345;circle[1][28]=-9.026755;circle[1][29]=23.937009;circle[2][0]=-5.526285;circle[2][1]=16.202694;circle[2][2]=-4.216869;circle[2][3]=21.590569;circle[2][4]=2.566133;circle[2][5]=14.520363;circle[2][6]=-12.858424;circle[2][7]=11.685608;circle[2][8]=-38.410094;circle[2][9]=32.895236;circle[2][10]=4.455548;circle[2][11]=50.154204;circle[2][12]=28.453256;circle[2][13]=35.212172;circle[2][14]=1.184816;circle[2][15]=22.678558;circle[2][16]=-5.570279;circle[2][17]=22.116068;circle[2][18]=-6.583276;circle[2][19]=22.384370;circle[2][20]=-6.854697;circle[2][21]=22.785343;circle[2][22]=-6.949336;circle[2][23]=23.493713;circle[2][24]=-6.982335;circle[2][25]=25.738691;circle[2][26]=-6.993840;circle[2][27]=26.560209;circle[2][28]=-6.997852;circle[2][29]=26.846654;circle[3][0]=-5.556262;circle[3][1]=16.539647;circle[3][2]=-3.661766;circle[3][3]=23.182266;circle[3][4]=5.761946;circle[3][5]=16.492908;circle[3][6]=-24.990258;circle[3][7]=10.566300;circle[3][8]=-35.140279;circle[3][9]=31.100845;circle[3][10]=2.890209;circle[3][11]=43.187574;circle[3][12]=12.506159;circle[3][13]=22.430648;circle[3][14]=-11.297348;circle[3][15]=11.239439;circle[3][16]=-11.895824;circle[3][17]=16.641658;circle[3][18]=-10.561033;circle[3][19]=20.600221;circle[3][20]=-8.506207;circle[3][21]=22.684952;circle[3][22]=-7.525182;circle[3][23]=24.010030;circle[3][24]=-7.248730;circle[3][25]=25.064329;circle[3][26]=-7.086727;circle[3][27]=25.673752;circle[3][28]=-7.030240;circle[3][29]=25.886244;circle[4][0]=-5.604092;circle[4][1]=17.152226;circle[4][2]=-6.694217;circle[4][3]=23.025912;circle[4][4]=-1.391431;circle[4][5]=15.060035;circle[4][6]=-5.543960;circle[4][7]=14.457036;circle[4][8]=-46.088140;circle[4][9]=30.098844;circle[4][10]=-32.695980;circle[4][11]=52.668194;circle[4][12]=10.347218;circle[4][13]=40.107831;circle[4][14]=-10.813556;circle[4][15]=19.345209;circle[4][16]=-16.323694;circle[4][17]=19.070971;circle[4][18]=-12.212159;circle[4][19]=21.636481;circle[4][20]=-10.120011;circle[4][21]=23.175892;circle[4][22]=-8.981014;circle[4][23]=23.712651;circle[4][24]=-8.342058;circle[4][25]=23.899808;circle[4][26]=-8.119268;circle[4][27]=23.965065;circle[4][28]=-8.041586;circle[4][29]=23.987819;circle[5][0]=-6.125795;circle[5][1]=17.908826;circle[5][2]=-5.877238;circle[5][3]=24.896137;circle[5][4]=-4.117069;circle[5][5]=23.779356;circle[5][6]=-7.424483;circle[5][7]=24.695916;circle[5][8]=-33.916710;circle[5][9]=27.598101;circle[5][10]=-39.431289;circle[5][11]=38.070930;circle[5][12]=-3.647704;circle[5][13]=37.479316;circle[5][14]=30.515609;circle[5][15]=25.304088;circle[5][16]=1.142607;circle[5][17]=18.205628;circle[5][18]=-7.477280;circle[5][19]=20.249032;circle[5][20]=-8.550849;circle[5][21]=21.389475;circle[5][22]=-8.843391;circle[5][23]=22.308826;circle[5][24]=-8.945394;circle[5][25]=23.227562;circle[5][26]=-8.980960;circle[5][27]=23.730667;circle[5][28]=-8.993361;circle[5][29]=23.906090;circle[6][0]=-6.360692;circle[6][1]=17.947898;circle[6][2]=-6.266351;circle[6][3]=23.788060;circle[6][4]=1.342474;circle[6][5]=21.417450;circle[6][6]=-11.478011;circle[6][7]=16.497520;circle[6][8]=-38.960735;circle[6][9]=35.672790;circle[6][10]=-25.595195;circle[6][11]=41.760546;circle[6][12]=15.605265;circle[6][13]=36.174077;circle[6][14]=6.172327;circle[6][15]=19.667379;circle[6][16]=-11.839213;circle[6][17]=14.766579;circle[6][18]=-11.323576;circle[6][19]=18.017293;circle[6][20]=-9.810181;circle[6][21]=21.223895;circle[6][22]=-9.282493;circle[6][23]=22.380711;circle[6][24]=-9.098499;circle[6][25]=22.784067;circle[6][26]=-9.034345;circle[6][27]=23.195709;circle[6][28]=-9.011975;circle[6][29]=23.719561;circle[7][0]=-6.068842;circle[7][1]=16.742716;circle[7][2]=6.324619;circle[7][3]=22.734196;circle[7][4]=7.340179;circle[7][5]=16.570095;circle[7][6]=-31.375675;circle[7][7]=10.550061;circle[7][8]=-46.471250;circle[7][9]=25.463722;circle[7][10]=-24.549322;circle[7][11]=43.953733;circle[7][12]=12.645399;circle[7][13]=40.189865;circle[7][14]=-0.467867;circle[7][15]=26.920795;circle[7][16]=-11.173786;circle[7][17]=23.736626;circle[7][18]=-10.430681;circle[7][19]=23.256846;circle[7][20]=-9.498848;circle[7][21]=23.558115;circle[7][22]=-9.073937;circle[7][23]=24.255434;circle[7][24]=-7.723137;circle[7][25]=24.610768;circle[7][26]=-7.523142;circle[7][27]=24.864283;circle[7][28]=-7.833730;circle[7][29]=24.952678;circle[8][0]=-5.935795;circle[8][1]=19.133985;circle[8][2]=4.240080;circle[8][3]=22.412809;circle[8][4]=10.798227;circle[8][5]=7.736704;circle[8][6]=-28.848318;circle[8][7]=11.005264;circle[8][8]=-36.887794;circle[8][9]=32.964020;circle[8][10]=3.414709;circle[8][11]=42.071616;circle[8][12]=29.949059;circle[8][13]=40.774288;circle[8][14]=-3.429257;circle[8][15]=29.289619;circle[8][16]=-9.173478;circle[8][17]=27.147019;circle[8][18]=-9.142277;circle[8][19]=26.399941;circle[8][20]=-9.049609;circle[8][21]=26.139451;circle[8][22]=-8.746298;circle[8][23]=26.048623;circle[8][24]=-8.260218;circle[8][25]=26.016954;circle[8][26]=-8.090732;circle[8][27]=26.005911;circle[8][28]=-8.031636;circle[8][29]=26.002061;circle[9][0]=-4.943683;circle[9][1]=17.195273;circle[9][2]=-4.304058;circle[9][3]=22.393993;circle[9][4]=3.414681;circle[9][5]=18.604565;circle[9][6]=-13.027430;circle[9][7]=16.187313;circle[9][8]=-45.793402;circle[9][9]=23.762427;circle[9][10]=-28.123725;circle[9][11]=37.318709;circle[9][12]=8.850592;circle[9][13]=36.405216;circle[9][14]=12.113966;circle[9][15]=19.395903;circle[9][16]=-8.002843;circle[9][17]=18.275872;circle[9][18]=-10.192908;circle[9][19]=21.029198;circle[9][20]=-9.454683;circle[9][21]=22.312824;circle[9][22]=-9.058538;circle[9][23]=23.169907;circle[9][24]=-8.369089;circle[9][25]=24.332267;circle[9][26]=-8.128694;circle[9][27]=25.418498;circle[9][28]=-8.044873;circle[9][29]=25.797243;circle[10][0]=-5.945672;circle[10][1]=18.140308;circle[10][2]=-3.313622;circle[10][3]=23.828238;circle[10][4]=7.348186;circle[10][5]=15.669094;circle[10][6]=-16.473175;circle[10][7]=8.592889;circle[10][8]=-41.210830;circle[10][9]=24.227604;circle[10][10]=-29.759446;circle[10][11]=46.899570;circle[10][12]=10.638665;circle[10][13]=38.191351;circle[10][14]=1.073089;circle[10][15]=17.869720;circle[10][16]=-12.020919;circle[10][17]=18.046009;circle[10][18]=-11.416122;circle[10][19]=20.762710;circle[10][20]=-10.493771;circle[10][21]=22.563805;circle[10][22]=-9.650464;circle[10][23]=24.150551;circle[10][24]=-9.226803;circle[10][25]=25.694078;circle[10][26]=-9.079081;circle[10][27]=27.157233;circle[10][28]=-9.027574;circle[10][29]=27.706145;circle[11][0]=-5.716285;circle[11][1]=18.106385;circle[11][2]=-3.569939;circle[11][3]=25.057431;circle[11][4]=13.703711;circle[11][5]=21.931843;circle[11][6]=-16.014211;circle[11][7]=11.851596;circle[11][8]=-34.598435;circle[11][9]=29.049495;circle[11][10]=-18.495143;circle[11][11]=44.176871;circle[11][12]=15.348453;circle[11][13]=33.797911;circle[11][14]=0.937417;circle[11][15]=13.951890;circle[11][16]=-13.182863;circle[11][17]=15.251493;circle[11][18]=-11.923029;circle[11][19]=20.051223;circle[11][20]=-10.399519;circle[11][21]=22.623147;circle[11][22]=-9.487982;circle[11][23]=23.519921;circle[11][24]=-9.170149;circle[11][25]=23.932607;circle[11][26]=-8.590768;circle[11][27]=24.627823;circle[11][28]=-8.205988;circle[11][29]=24.970230;circle[12][0]=-6.025795;circle[12][1]=18.001798;circle[12][2]=0.429450;circle[12][3]=20.989611;circle[12][4]=6.702070;circle[12][5]=9.009264;circle[12][6]=-25.728640;circle[12][7]=19.583427;circle[12][8]=-25.158489;circle[12][9]=37.557534;circle[12][10]=5.623676;circle[12][11]=48.061208;circle[12][12]=14.382011;circle[12][13]=34.426441;circle[12][14]=-10.196320;circle[12][15]=22.227413;circle[12][16]=-16.374893;circle[12][17]=23.650617;circle[12][18]=-12.780359;circle[12][19]=23.878178;circle[12][20]=-10.869451;circle[12][21]=23.957523;circle[12][22]=-9.651837;circle[12][23]=24.975451;circle[12][24]=-8.956282;circle[12][25]=25.642762;circle[12][26]=-8.333435;circle[12][27]=25.875439;circle[12][28]=-7.847703;circle[12][29]=26.056568;circle[13][0]=-6.259718;circle[13][1]=17.908826;circle[13][2]=-3.692965;circle[13][3]=24.769037;circle[13][4]=10.307888;circle[13][5]=15.383659;circle[13][6]=-18.228026;circle[13][7]=5.602600;circle[13][8]=-55.616656;circle[13][9]=17.825499;circle[13][10]=-27.228831;circle[13][11]=30.290511;circle[13][12]=14.804710;circle[13][13]=24.210142;circle[13][14]=0.356212;circle[13][15]=9.745106;circle[13][16]=-12.414899;circle[13][17]=14.167640;circle[13][18]=-11.062984;circle[13][19]=18.535915;circle[13][20]=-9.719318;circle[13][21]=21.178918;circle[13][22]=-9.250811;circle[13][23]=22.833587;circle[13][24]=-8.565749;circle[13][25]=24.061856;circle[13][26]=-7.675561;circle[13][27]=25.141448;circle[13][28]=-7.316554;circle[13][29]=25.700642;circle[14][0]=-5.681895;circle[14][1]=18.889965;circle[14][2]=-2.474207;circle[14][3]=25.265549;circle[14][4]=18.367301;circle[14][5]=19.044242;circle[14][6]=-12.423253;circle[14][7]=4.528720;circle[14][8]=-51.866426;circle[14][9]=10.937965;circle[14][10]=-33.422750;circle[14][11]=32.482148;circle[14][12]=12.502758;circle[14][13]=29.079906;circle[14][14]=7.316450;circle[14][15]=15.606872;circle[14][16]=-6.852700;circle[14][17]=15.663943;circle[14][18]=-8.941346;circle[14][19]=20.449313;circle[14][20]=-9.061337;circle[14][21]=23.383655;circle[14][22]=-9.021387;circle[14][23]=25.087737;circle[14][24]=-9.007457;circle[14][25]=25.681914;circle[14][26]=-8.534041;circle[14][27]=25.889090;circle[14][28]=-8.186209;circle[14][29]=25.961328;circle[15][0]=-5.513216;circle[15][1]=19.023888;circle[15][2]=-1.946670;circle[15][3]=24.737898;circle[15][4]=13.047926;circle[15][5]=16.242110;circle[15][6]=-14.533608;circle[15][7]=7.074474;circle[15][8]=-39.873934;circle[15][9]=37.949981;circle[15][10]=-28.383863;circle[15][11]=56.382115;circle[15][12]=0.574816;circle[15][13]=43.774201;circle[15][14]=2.396342;circle[15][15]=21.060095;circle[15][16]=-9.371024;circle[15][17]=20.419884;circle[15][18]=-9.819432;circle[15][19]=22.100369;circle[15][20]=-9.324460;circle[15][21]=22.686318;circle[15][22]=-9.113132;circle[15][23]=22.890626;circle[15][24]=-9.039447;circle[15][25]=22.961864;circle[15][26]=-8.669854;circle[15][27]=22.986703;circle[15][28]=-8.233564;circle[15][29]=23.517067;circle[16][0]=-5.604092;circle[16][1]=17.867875;circle[16][2]=-5.876932;circle[16][3]=24.867009;circle[16][4]=8.152522;circle[16][5]=22.975477;circle[16][6]=-11.725002;circle[16][7]=13.251904;circle[16][8]=-57.549063;circle[16][9]=13.608882;circle[16][10]=-47.500197;circle[16][11]=25.809478;circle[16][12]=5.616211;circle[16][13]=35.017541;circle[16][14]=30.932355;circle[16][15]=15.435379;circle[16][16]=-1.009473;circle[16][17]=7.637663;circle[16][18]=-8.808173;circle[16][19]=13.214410;circle[16][20]=-9.053645;circle[16][21]=17.551248;circle[16][22]=-9.018705;circle[16][23]=20.141395;circle[16][24]=-9.006522;circle[16][25]=21.351944;circle[16][26]=-9.002274;circle[16][27]=21.774037;circle[16][28]=-9.000793;circle[16][29]=21.921212;circle[17][0]=-4.900636;circle[17][1]=16.452716;circle[17][2]=1.137374;circle[17][3]=16.690686;circle[17][4]=0.044908;circle[17][5]=3.156810;circle[17][6]=-28.483892;circle[17][7]=13.841306;circle[17][8]=-30.564018;circle[17][9]=33.773561;circle[17][10]=-3.142695;circle[17][11]=48.706758;circle[17][12]=21.508471;circle[17][13]=24.736292;circle[17][14]=-7.710006;circle[17][15]=11.121519;circle[17][16]=-12.879281;circle[17][17]=14.680735;circle[17][18]=-11.393154;circle[17][19]=18.063498;circle[17][20]=-9.964060;circle[17][21]=19.976105;circle[17][22]=-9.336147;circle[17][23]=20.642990;circle[17][24]=-8.927207;circle[17][25]=21.344077;circle[17][26]=-8.323297;circle[17][27]=21.771294;circle[17][28]=-8.112727;circle[17][29]=21.920255;circle[18][0]=-6.259718;circle[18][1]=16.962226;circle[18][2]=-2.000735;circle[18][3]=22.980667;circle[18][4]=5.950324;circle[18][5]=14.468366;circle[18][6]=-29.923647;circle[18][7]=10.567709;circle[18][8]=-46.910289;circle[18][9]=22.891110;circle[18][10]=-11.968282;circle[18][11]=52.478503;circle[18][12]=17.989967;circle[18][13]=39.280745;circle[18][14]=-3.620522;circle[18][15]=20.800291;circle[18][16]=-8.208525;circle[18][17]=19.832451;circle[18][18]=-9.104351;circle[18][19]=20.592901;circle[18][20]=-9.036385;circle[18][21]=21.267563;circle[18][22]=-9.012687;circle[18][23]=21.744615;circle[18][24]=-8.594914;circle[18][25]=21.910953;circle[18][26]=-8.207434;circle[18][27]=22.437510;circle[18][28]=-8.072328;circle[18][29]=22.803872;circle[19][0]=-5.137142;circle[19][1]=15.157139;circle[19][2]=1.491597;circle[19][3]=16.927583;circle[19][4]=6.969112;circle[19][5]=9.699807;circle[19][6]=-20.979334;circle[19][7]=9.004338;circle[19][8]=-41.270240;circle[19][9]=17.706420;circle[19][10]=-15.049223;circle[19][11]=29.375143;circle[19][12]=23.723933;circle[19][13]=28.643525;circle[19][14]=4.274510;circle[19][15]=13.090229;circle[19][16]=-8.739281;circle[19][17]=13.874217;circle[19][18]=-9.458326;circle[19][19]=18.564889;circle[19][20]=-9.159808;circle[19][21]=20.802251;circle[19][22]=-9.055722;circle[19][23]=21.582371;circle[19][24]=-9.019429;circle[19][25]=22.315382;circle[19][26]=-8.735774;circle[19][27]=23.412610;circle[19][28]=-8.256549;circle[19][29]=23.795190;circle[20][0]=-5.367236;circle[20][1]=16.925247;circle[20][2]=-2.334780;circle[20][3]=22.208786;circle[20][4]=10.998557;circle[20][5]=9.415882;circle[20][6]=-34.403450;circle[20][7]=8.176886;circle[20][8]=-66.954545;circle[20][9]=26.458513;circle[20][10]=-23.677979;circle[20][11]=46.060586;circle[20][12]=19.490953;circle[20][13]=30.451708;circle[20][14]=-2.767573;circle[20][15]=14.436383;circle[20][16]=-14.194160;circle[20][17]=14.933655;circle[20][18]=-12.605695;circle[20][19]=17.499391;circle[20][20]=-10.619339;circle[20][21]=20.219327;circle[20][22]=-7.852475;circle[20][23]=21.948651;circle[20][24]=-6.645918;circle[20][25]=22.942927;circle[20][26]=-6.225218;circle[20][27]=22.980100;circle[20][28]=-6.078529;circle[20][29]=23.644383;circle[21][0]=-6.328865;circle[21][1]=15.827067;circle[21][2]=2.305176;circle[21][3]=20.943062;circle[21][4]=1.118178;circle[21][5]=12.579829;circle[21][6]=-48.614725;circle[21][7]=17.340883;circle[21][8]=-57.110227;circle[21][9]=36.432406;circle[21][10]=-5.992612;circle[21][11]=44.959199;circle[21][12]=9.503842;circle[21][13]=18.067162;circle[21][14]=-16.976799;circle[21][15]=13.615654;circle[21][16]=-16.800476;circle[21][17]=18.121945;circle[21][18]=-12.582586;circle[21][19]=20.609064;circle[21][20]=-10.287913;circle[21][21]=21.515010;circle[21][22]=-8.980508;circle[21][23]=21.830895;circle[21][24]=-8.341882;circle[21][25]=21.941037;circle[21][26]=-8.119207;circle[21][27]=21.979441;circle[21][28]=-8.041565;circle[21][29]=21.992831;circle[22][0]=-6.838375;circle[22][1]=17.355296;circle[22][2]=3.759553;circle[22][3]=19.726715;circle[22][4]=-5.993542;circle[22][5]=7.371820;circle[22][6]=-44.444555;circle[22][7]=8.441140;circle[22][8]=-45.996857;circle[22][9]=26.380409;circle[22][10]=9.852748;circle[22][11]=36.014442;circle[22][12]=27.471235;circle[22][13]=13.688277;circle[22][14]=2.403081;circle[22][15]=11.101410;circle[22][16]=-5.023992;circle[22][17]=17.509833;circle[22][18]=-7.613652;circle[22][19]=19.783054;circle[22][20]=-8.516610;circle[22][21]=20.575677;circle[22][22]=-8.831452;circle[22][23]=20.852048;circle[22][24]=-8.941231;circle[22][25]=20.948412;circle[22][26]=-8.979509;circle[22][27]=22.042844;circle[22][28]=-8.992855;circle[22][29]=22.666260;circle[23][0]=-5.513216;circle[23][1]=16.017944;circle[23][2]=-5.151320;circle[23][3]=23.564830;circle[23][4]=6.216232;circle[23][5]=19.613816;circle[23][6]=-10.883112;circle[23][7]=9.728204;circle[23][8]=-43.080559;circle[23][9]=14.199273;circle[23][10]=-51.878348;circle[23][11]=40.025526;circle[23][12]=-24.023987;circle[23][13]=51.363840;circle[23][14]=10.713489;circle[23][15]=41.186020;circle[23][16]=21.061141;circle[23][17]=17.221349;circle[23][18]=-10.302955;circle[23][19]=13.796360;circle[23][20]=-13.783386;circle[23][21]=17.524543;circle[23][22]=-11.540690;circle[23][23]=19.788183;circle[23][24]=-9.797646;circle[23][25]=20.577466;circle[23][26]=-8.626801;circle[23][27]=20.852671;circle[23][28]=-8.218552;circle[23][29]=20.948630;circle[24][0]=-6.026672;circle[24][1]=17.143103;circle[24][2]=-1.162129;circle[24][3]=21.834317;circle[24][4]=4.963143;circle[24][5]=9.006103;circle[24][6]=-24.277065;circle[24][7]=2.086339;circle[24][8]=-56.084860;circle[24][9]=12.234892;circle[24][10]=-50.200913;circle[24][11]=40.710887;circle[24][12]=-10.855595;circle[24][13]=56.920473;circle[24][14]=15.740566;circle[24][15]=26.245446;circle[24][16]=-22.827530;circle[24][17]=15.665305;circle[24][18]=-25.305327;circle[24][19]=17.375154;circle[24][20]=-18.155647;circle[24][21]=19.084773;circle[24][22]=-13.736831;circle[24][23]=20.024780;circle[24][24]=-11.683274;circle[24][25]=20.659962;circle[24][26]=-10.177411;circle[24][27]=20.881436;circle[24][28]=-8.888835;circle[24][29]=20.958659;circle[25][0]=-6.125795;circle[25][1]=16.702204;circle[25][2]=-4.714899;circle[25][3]=20.760059;circle[25][4]=-0.786781;circle[25][5]=10.187999;circle[25][6]=-21.914355;circle[25][7]=4.207716;circle[25][8]=-38.439450;circle[25][9]=23.782365;circle[25][10]=-9.024719;circle[25][11]=50.884524;circle[25][12]=23.822126;circle[25][13]=56.931268;circle[25][14]=17.165485;circle[25][15]=26.981909;circle[25][16]=-15.857078;circle[25][17]=15.967471;circle[25][18]=-17.022633;circle[25][19]=17.803450;circle[25][20]=-14.340126;circle[25][21]=19.153111;circle[25][22]=-12.376037;circle[25][23]=20.226411;circle[25][24]=-11.070284;circle[25][25]=20.730266;circle[25][26]=-10.373185;circle[25][27]=20.905950;circle[25][28]=-10.130122;circle[25][29]=20.967207;circle[26][0]=-5.925795;circle[26][1]=14.333349;circle[26][2]=2.386492;circle[26][3]=15.190041;circle[26][4]=3.868996;circle[26][5]=-0.505528;circle[26][6]=-41.302016;circle[26][7]=-3.527341;circle[26][8]=-37.850749;circle[26][9]=26.310323;circle[26][10]=-2.617193;circle[26][11]=47.109290;circle[26][12]=27.504508;circle[26][13]=35.661410;circle[26][14]=-1.190319;circle[26][15]=18.091638;circle[26][16]=-13.511712;circle[26][17]=18.499521;circle[26][18]=-11.445419;circle[26][19]=19.476815;circle[26][20]=-9.852665;circle[26][21]=20.007577;circle[26][22]=-9.297306;circle[26][23]=21.266543;circle[26][24]=-9.103664;circle[26][25]=21.744259;circle[26][26]=-9.036145;circle[26][27]=21.910829;circle[26][28]=-9.012603;circle[26][29]=21.968908;circle[27][0]=-5.773916;circle[27][1]=17.253394;circle[27][2]=5.202808;circle[27][3]=19.475082;circle[27][4]=6.619545;circle[27][5]=4.857966;circle[27][6]=-40.204322;circle[27][7]=11.677673;circle[27][8]=-43.310248;circle[27][9]=43.814044;circle[27][10]=4.431896;circle[27][11]=51.352378;circle[27][12]=35.505871;circle[27][13]=39.304720;circle[27][14]=1.924934;circle[27][15]=18.901407;circle[27][16]=-7.004069;circle[27][17]=19.343393;circle[27][18]=-8.955383;circle[27][19]=20.422377;circle[27][20]=-9.114062;circle[27][21]=21.639128;circle[27][22]=-9.039771;circle[27][23]=22.935003;circle[27][24]=-9.013867;circle[27][25]=23.628659;circle[27][26]=-9.004835;circle[27][27]=23.870521;circle[27][28]=-9.001686;circle[27][29]=24.606175;circle[28][0]=-6.125795;circle[28][1]=16.195747;circle[28][2]=-2.650446;circle[28][3]=22.266927;circle[28][4]=16.500683;circle[28][5]=11.824925;circle[28][6]=-29.331431;circle[28][7]=14.624514;circle[28][8]=-45.000082;circle[28][9]=38.088263;circle[28][10]=-8.062265;circle[28][11]=51.109593;circle[28][12]=38.718738;circle[28][13]=39.734441;circle[28][14]=10.337771;circle[28][15]=22.513680;circle[28][16]=-7.912455;circle[28][17]=20.368390;circle[28][18]=-9.609570;circle[28][19]=22.601854;circle[28][20]=-9.863865;circle[28][21]=24.163818;circle[28][22]=-8.770420;circle[28][23]=25.277974;circle[28][24]=-8.268629;circle[28][25]=25.748245;circle[28][26]=-8.093665;circle[28][27]=25.912219;circle[28][28]=-8.032659;circle[28][29]=25.969392;circle[29][0]=-4.432077;circle[29][1]=16.071088;circle[29][2]=-5.104721;circle[29][3]=23.013002;circle[29][4]=11.180601;circle[29][5]=15.858398;circle[29][6]=-1.893438;circle[29][7]=4.054432;circle[29][8]=-36.751999;circle[29][9]=12.806616;circle[29][10]=-38.570754;circle[29][11]=32.771898;circle[29][12]=-11.096312;circle[29][13]=37.056832;circle[29][14]=22.804876;circle[29][15]=19.466124;circle[29][16]=-10.606156;circle[29][17]=6.025530;circle[29][18]=-17.207858;circle[29][19]=11.898653;circle[29][20]=-13.187948;circle[29][21]=17.407493;circle[29][22]=-11.011569;circle[29][23]=20.398692;circle[29][24]=-10.495391;circle[29][25]=21.902658;circle[29][26]=-9.651029;circle[29][27]=23.268702;circle[29][28]=-9.227000;circle[29][29]=23.745012;circle[30][0]=-6.639251;circle[30][1]=16.130137;circle[30][2]=-2.644545;circle[30][3]=22.773467;circle[30][4]=17.906735;circle[30][5]=8.441285;circle[30][6]=-23.700376;circle[30][7]=6.968075;circle[30][8]=-37.121978;circle[30][9]=43.021734;circle[30][10]=5.613776;circle[30][11]=52.011481;circle[30][12]=36.236062;circle[30][13]=32.803742;circle[30][14]=6.973673;circle[30][15]=13.135347;circle[30][16]=-5.405371;circle[30][17]=15.709990;circle[30][18]=-8.126952;circle[30][19]=20.608600;circle[30][20]=-8.695587;circle[30][21]=22.817492;circle[30][22]=-8.793858;circle[30][23]=23.858685;circle[30][24]=-8.276801;circle[30][25]=24.602048;circle[30][26]=-8.096515;circle[30][27]=24.861243;circle[30][28]=-8.033653;circle[30][29]=24.951618;circle[31][0]=-8.252226;circle[31][1]=14.003235;circle[31][2]=-4.577198;circle[31][3]=21.103847;circle[31][4]=11.673376;circle[31][5]=17.510753;circle[31][6]=-11.636405;circle[31][7]=9.383439;circle[31][8]=-50.035683;circle[31][9]=14.129203;circle[31][10]=-37.674861;circle[31][11]=31.062394;circle[31][12]=12.449903;circle[31][13]=29.750361;circle[31][14]=15.447297;circle[31][15]=10.927214;circle[31][16]=-5.351846;circle[31][17]=9.012703;circle[31][18]=-9.220932;circle[31][19]=15.647821;circle[31][20]=-9.728356;circle[31][21]=20.436454;circle[31][22]=-9.634284;circle[31][23]=22.675680;circle[31][24]=-8.877261;circle[31][25]=24.635548;circle[31][26]=-8.305882;circle[31][27]=26.175567;circle[31][28]=-8.106654;circle[31][29]=26.712538;circle[32][0]=-6.125795;circle[32][1]=17.807853;circle[32][2]=-1.866312;circle[32][3]=24.985129;circle[32][4]=9.092125;circle[32][5]=13.375155;circle[32][6]=-29.281405;circle[32][7]=3.767664;circle[32][8]=-45.270046;circle[32][9]=25.057679;circle[32][10]=-8.148035;circle[32][11]=42.975650;circle[32][12]=26.527042;circle[32][13]=30.508197;circle[32][14]=4.040852;circle[32][15]=10.783962;circle[32][16]=-14.172937;circle[32][17]=14.181838;circle[32][18]=-13.142380;circle[32][19]=17.932590;circle[32][20]=-10.526147;circle[32][21]=20.461251;circle[32][22]=-9.532135;circle[32][23]=21.463471;circle[32][24]=-8.841644;circle[32][25]=21.740024;circle[32][26]=-8.103463;circle[32][27]=22.560674;circle[32][28]=-7.384754;circle[32][29]=23.190716;circle[33][0]=-5.513216;circle[33][1]=18.411308;circle[33][2]=-7.169333;circle[33][3]=24.565952;circle[33][4]=-0.976770;circle[33][5]=20.581283;circle[33][6]=-10.759418;circle[33][7]=8.365274;circle[33][8]=-44.864367;circle[33][9]=15.451300;circle[33][10]=-34.794273;circle[33][11]=34.523043;circle[33][12]=-0.202896;circle[33][13]=43.814455;circle[33][14]=14.153329;circle[33][15]=25.979972;circle[33][16]=-23.974462;circle[33][17]=13.589285;circle[33][18]=-25.183535;circle[33][19]=16.653513;circle[33][20]=-17.117959;circle[33][21]=20.135795;circle[33][22]=-12.137979;circle[33][23]=22.108551;circle[33][24]=-10.094146;circle[33][25]=24.643136;circle[33][26]=-9.110505;circle[33][27]=25.255891;circle[33][28]=-8.387209;circle[33][29]=25.089224;circle[34][0]=-6.177375;circle[34][1]=17.807853;circle[34][2]=10.417276;circle[34][3]=16.355644;circle[34][4]=6.015266;circle[34][5]=8.401979;circle[34][6]=-34.833282;circle[34][7]=30.152853;circle[34][8]=-29.000979;circle[34][9]=55.098923;circle[34][10]=10.094275;circle[34][11]=65.227560;circle[34][12]=17.137835;circle[34][13]=37.684045;circle[34][14]=-10.924459;circle[34][15]=26.188011;circle[34][16]=-12.490027;circle[34][17]=23.050759;circle[34][18]=-10.868219;circle[34][19]=21.815055;circle[34][20]=-9.378077;circle[34][21]=22.345024;circle[34][22]=-8.480506;circle[34][23]=22.771624;circle[34][24]=-8.511442;circle[34][25]=23.952070;circle[34][26]=-8.829650;circle[34][27]=25.285931;circle[34][28]=-8.596703;circle[34][29]=25.751020;circle[35][0]=-5.897842;circle[35][1]=16.924273;circle[35][2]=-0.667219;circle[35][3]=23.669728;circle[35][4]=19.434919;circle[35][5]=12.383894;circle[35][6]=-33.829118;circle[35][7]=8.426259;circle[35][8]=-77.955219;circle[35][9]=44.812272;circle[35][10]=-5.324335;circle[35][11]=66.916307;circle[35][12]=17.771131;circle[35][13]=21.177364;circle[35][14]=-10.341950;circle[35][15]=13.553733;circle[35][16]=-11.601207;circle[35][17]=17.421502;circle[35][18]=-9.906985;circle[35][19]=19.752255;circle[35][20]=-8.746713;circle[35][21]=20.607985;circle[35][22]=-8.260363;circle[35][23]=21.514634;circle[35][24]=-8.090783;circle[35][25]=21.830763;circle[35][26]=-8.031654;circle[35][27]=21.940991;circle[35][28]=-8.011037;circle[35][29]=21.979425;circle[36][0]=-6.125795;circle[36][1]=17.152226;circle[36][2]=-5.796062;circle[36][3]=22.634529;circle[36][4]=2.022267;circle[36][5]=16.615649;circle[36][6]=-23.619356;circle[36][7]=6.883505;circle[36][8]=-67.058297;circle[36][9]=24.726881;circle[36][10]=-32.667794;circle[36][11]=37.366911;circle[36][12]=16.965621;circle[36][13]=26.316347;circle[36][14]=11.219294;circle[36][15]=7.880148;circle[36][16]=-13.502294;circle[36][17]=10.617818;circle[36][18]=-13.038160;circle[36][19]=16.898860;circle[36][20]=-11.059341;circle[36][21]=20.933922;circle[36][22]=-9.947666;circle[36][23]=23.030925;circle[36][24]=-9.330431;circle[36][25]=24.313426;circle[36][26]=-8.771314;circle[36][27]=24.760606;circle[36][28]=-8.268941;circle[36][29]=24.916529;circle[37][0]=-6.542618;circle[37][1]=18.944643;circle[37][2]=0.288360;circle[37][3]=18.844577;circle[37][4]=5.172988;circle[37][5]=4.236609;circle[37][6]=-25.443193;circle[37][7]=13.146635;circle[37][8]=-34.684824;circle[37][9]=31.864850;circle[37][10]=-7.501911;circle[37][11]=43.852519;circle[37][12]=10.160883;circle[37][13]=50.160287;circle[37][14]=14.237482;circle[37][15]=15.352787;circle[37][16]=-15.765881;circle[37][17]=2.658212;circle[37][18]=-18.765131;circle[37][19]=11.328464;circle[37][20]=-14.425127;circle[37][21]=16.955891;circle[37][22]=-12.004268;circle[37][23]=20.621607;circle[37][24]=-10.698845;circle[37][25]=22.822027;circle[37][26]=-10.243672;circle[37][27]=23.589266;circle[37][28]=-9.472384;circle[37][29]=23.388227;circle[38][0]=-6.738375;circle[38][1]=16.539647;circle[38][2]=-4.484105;circle[38][3]=22.299108;circle[38][4]=2.398585;circle[38][5]=17.922399;circle[38][6]=-17.615634;circle[38][7]=5.202812;circle[38][8]=-48.718889;circle[38][9]=13.421044;circle[38][10]=-42.013951;circle[38][11]=36.787348;circle[38][12]=-29.237336;circle[38][13]=62.344442;circle[38][14]=0.323327;circle[38][15]=41.053560;circle[38][16]=-12.553887;circle[38][17]=19.482052;circle[38][18]=-25.602290;circle[38][19]=18.225233;circle[38][20]=-19.199724;circle[38][21]=20.222499;circle[38][22]=-13.146487;circle[38][23]=21.180224;circle[38][24]=-10.445791;circle[38][25]=21.324951;circle[38][26]=-8.934583;circle[38][27]=21.764625;circle[38][28]=-8.325869;circle[38][29]=21.917930;circle[39][0]=-6.467375;circle[39][1]=16.972103;circle[39][2]=-5.418288;circle[39][3]=23.032246;circle[39][4]=0.269751;circle[39][5]=20.576452;circle[39][6]=-1.292333;circle[39][7]=1.307062;circle[39][8]=-31.554320;circle[39][9]=4.452800;circle[39][10]=-59.179276;circle[39][11]=20.278335;circle[39][12]=-39.863571;circle[39][13]=35.335412;circle[39][14]=11.942583;circle[39][15]=28.302512;circle[39][16]=-9.561393;circle[39][17]=7.699709;circle[39][18]=-18.596141;circle[39][19]=10.197508;circle[39][20]=-14.513655;circle[39][21]=16.983255;circle[39][22]=-11.473814;circle[39][23]=20.465051;circle[39][24]=-9.862566;circle[39][25]=22.525628;circle[39][26]=-9.300758;circle[39][27]=23.485918;circle[39][28]=-9.104868;circle[39][29]=23.820751;
  
}
void fill_matrix_square(){
  
  square[0][0]=-6.360692;square[0][1]=12.864170;square[0][2]=-8.270052;square[0][3]=17.892211;square[0][4]=37.437548;square[0][5]=13.055435;square[0][6]=-4.027858;square[0][7]=-1.363185;square[0][8]=-17.938258;square[0][9]=-2.722960;square[0][10]=-3.310085;square[0][11]=1.251742;square[0][12]=-10.558362;square[0][13]=35.939898;square[0][14]=-41.693429;square[0][15]=25.638275;square[0][16]=3.672102;square[0][17]=25.398355;square[0][18]=7.512275;square[0][19]=47.704226;square[0][20]=-1.770535;square[0][21]=12.894786;square[0][22]=-5.909709;square[0][23]=6.416047;square[0][24]=-7.271160;square[0][25]=12.354559;square[0][26]=-7.745869;square[0][27]=16.646456;square[0][28]=-7.911390;square[0][29]=18.830692;square[1][0]=-6.125795;square[1][1]=14.692785;square[1][2]=-8.015248;square[1][3]=20.103453;square[1][4]=5.261540;square[1][5]=26.153799;square[1][6]=11.543213;square[1][7]=16.974955;square[1][8]=-21.971124;square[1][9]=14.274578;square[1][10]=-16.940598;square[1][11]=6.985455;square[1][12]=-17.593109;square[1][13]=3.578225;square[1][14]=-16.884234;square[1][15]=29.468048;square[1][16]=-39.692189;square[1][17]=30.796603;square[1][18]=-42.470919;square[1][19]=31.163580;square[1][20]=-10.581676;square[1][21]=37.493583;square[1][22]=-9.934750;square[1][23]=54.003133;square[1][24]=-9.666668;square[1][25]=25.483951;square[1][26]=-9.271195;square[1][27]=17.530471;square[1][28]=-8.572857;square[1][29]=18.487177;square[2][0]=-6.125795;square[2][1]=14.701908;square[2][2]=-8.205248;square[2][3]=20.757956;square[2][4]=18.453896;square[2][5]=23.477417;square[2][6]=-1.594159;square[2][7]=20.970169;square[2][8]=-17.782832;square[2][9]=12.718093;square[2][10]=-9.565392;square[2][11]=4.293337;square[2][12]=-8.823260;square[2][13]=28.841910;square[2][14]=-22.202750;square[2][15]=31.415608;square[2][16]=-36.070199;square[2][17]=21.334920;square[2][18]=-6.691649;square[2][19]=23.008418;square[2][20]=-3.971457;square[2][21]=32.820284;square[2][22]=-4.593986;square[2][23]=15.999263;square[2][24]=-8.115040;square[2][25]=8.301647;square[2][26]=-8.821052;square[2][27]=12.931027;square[2][28]=-8.937605;square[2][29]=17.366841;square[3][0]=-6.168842;square[3][1]=16.060990;square[3][2]=-8.616328;square[3][3]=21.781072;square[3][4]=5.046191;square[3][5]=23.479080;square[3][6]=-13.838799;square[3][7]=22.957726;square[3][8]=-23.907046;square[3][9]=18.176418;square[3][10]=-15.197988;square[3][11]=5.253320;square[3][12]=-6.985420;square[3][13]=31.987631;square[3][14]=-32.818041;square[3][15]=29.621648;square[3][16]=-3.427235;square[3][17]=26.045457;square[3][18]=0.746023;square[3][19]=48.887082;square[3][20]=-4.952352;square[3][21]=23.050394;square[3][22]=-6.937351;square[3][23]=15.986962;square[3][24]=-7.629477;square[3][25]=17.400185;square[3][26]=-7.870807;square[3][27]=18.442179;square[3][28]=-7.954953;square[3][29]=19.418079;square[4][0]=-6.394475;square[4][1]=16.673570;square[4][2]=2.768431;square[4][3]=22.055183;square[4][4]=5.702412;square[4][5]=27.280593;square[4][6]=-24.267741;square[4][7]=22.831520;square[4][8]=-17.290233;square[4][9]=12.223409;square[4][10]=-13.560574;square[4][11]=27.923630;square[4][12]=-16.353069;square[4][13]=37.739472;square[4][14]=-29.364455;square[4][15]=32.332051;square[4][16]=-4.928847;square[4][17]=30.711056;square[4][18]=-1.604318;square[4][19]=39.858658;square[4][20]=-8.373093;square[4][21]=14.621977;square[4][22]=-10.173066;square[4][23]=9.536760;square[4][24]=-10.060344;square[4][25]=13.726311;square[4][26]=-10.021041;square[4][27]=16.978416;square[4][28]=-10.007336;square[4][29]=18.295117;square[5][0]=-5.604092;square[5][1]=15.927067;square[5][2]=3.363105;square[5][3]=23.198376;square[5][4]=10.933597;square[5][5]=21.691894;square[5][6]=-27.226054;square[5][7]=18.365459;square[5][8]=-27.134207;square[5][9]=3.834024;square[5][10]=-20.971315;square[5][11]=21.439166;square[5][12]=-26.368339;square[5][13]=35.216680;square[5][14]=-36.182034;square[5][15]=24.541751;square[5][16]=-3.270314;square[5][17]=20.614466;square[5][18]=-1.638289;square[5][19]=36.620438;square[5][20]=-1.076662;square[5][21]=11.317177;square[5][22]=-2.738883;square[5][23]=9.060109;square[5][24]=-4.229809;square[5][25]=13.973703;square[5][26]=-5.952305;square[5][27]=18.117170;square[5][28]=-6.634691;square[5][29]=20.646141;square[6][0]=-6.259718;square[6][1]=15.405364;square[6][2]=-7.983265;square[6][3]=21.429898;square[6][4]=13.913460;square[6][5]=20.146586;square[6][6]=-26.782443;square[6][7]=21.476651;square[6][8]=-27.999253;square[6][9]=8.883136;square[6][10]=-17.355227;square[6][11]=6.956781;square[6][12]=-17.176190;square[6][13]=32.768104;square[6][14]=-42.327033;square[6][15]=28.769324;square[6][16]=2.070125;square[6][17]=31.524303;square[6][18]=7.508875;square[6][19]=49.044073;square[6][20]=-3.418599;square[6][21]=34.080276;square[6][22]=-9.718996;square[6][23]=13.172184;square[6][24]=-9.384299;square[6][25]=15.261600;square[6][26]=-8.482675;square[6][27]=17.928758;square[6][28]=-8.168298;square[6][29]=19.729124;square[7][0]=-6.542595;square[7][1]=15.927067;square[7][2]=6.414268;square[7][3]=22.253765;square[7][4]=11.065982;square[7][5]=22.383182;square[7][6]=-25.447651;square[7][7]=19.080541;square[7][8]=-20.136827;square[7][9]=2.300517;square[7][10]=-17.228219;square[7][11]=22.102821;square[7][12]=-31.408470;square[7][13]=28.887430;square[7][14]=-13.877806;square[7][15]=28.018397;square[7][16]=19.776894;square[7][17]=44.229821;square[7][18]=5.890340;square[7][19]=54.341858;square[7][20]=-6.089459;square[7][21]=21.025631;square[7][22]=-7.985157;square[7][23]=16.531674;square[7][24]=-8.033567;square[7][25]=17.376619;square[7][26]=-8.011704;square[7][27]=18.955665;square[7][28]=-8.004081;square[7][29]=19.635863;square[8][0]=-6.984491;square[8][1]=15.214488;square[8][2]=9.850844;square[8][3]=20.434678;square[8][4]=8.697607;square[8][5]=19.852248;square[8][6]=-18.267172;square[8][7]=17.499380;square[8][8]=-13.408141;square[8][9]=2.532033;square[8][10]=-13.495381;square[8][11]=26.489781;square[8][12]=-36.548728;square[8][13]=29.377307;square[8][14]=-24.372005;square[8][15]=19.208160;square[8][16]=-0.670543;square[8][17]=43.680873;square[8][18]=-2.190663;square[8][19]=37.950098;square[8][20]=-6.104028;square[8][21]=17.192970;square[8][22]=-7.338915;square[8][23]=16.906161;square[8][24]=-7.769494;square[8][25]=19.110456;square[8][26]=-7.919628;square[8][27]=20.341157;square[8][28]=-7.971976;square[8][29]=20.770276;square[9][0]=-5.146752;square[9][1]=13.989329;square[9][2]=11.237966;square[9][3]=17.445619;square[9][4]=-0.698409;square[9][5]=13.834908;square[9][6]=-25.567701;square[9][7]=14.907302;square[9][8]=-15.752022;square[9][9]=8.966150;square[9][10]=-10.477048;square[9][11]=32.462732;square[9][12]=-26.848622;square[9][13]=37.405813;square[9][14]=-7.175406;square[9][15]=36.779064;square[9][16]=3.470299;square[9][17]=48.219726;square[9][18]=-8.716744;square[9][19]=23.396656;square[9][20]=-10.996023;square[9][21]=8.248977;square[9][22]=-10.003392;square[9][23]=12.876240;square[9][24]=-9.659371;square[9][25]=17.274287;square[9][26]=-8.886008;square[9][27]=19.049603;square[9][28]=-8.308932;square[9][29]=20.190320;square[10][0]=-5.557236;square[10][1]=14.233349;square[10][2]=6.419494;square[10][3]=22.343768;square[10][4]=-15.114310;square[10][5]=33.515233;square[10][6]=-27.047004;square[10][7]=24.348939;square[10][8]=-14.274090;square[10][9]=19.931265;square[10][10]=-22.875790;square[10][11]=41.136577;square[10][12]=-42.992377;square[10][13]=32.977218;square[10][14]=-7.405207;square[10][15]=27.449115;square[10][16]=10.385520;square[10][17]=42.617914;square[10][18]=10.408610;square[10][19]=39.353959;square[10][20]=2.979503;square[10][21]=19.570669;square[10][22]=-1.324586;square[10][23]=18.653875;square[10][24]=-3.067141;square[10][25]=19.530635;square[10][26]=-3.674732;square[10][27]=20.358046;square[10][28]=-3.696586;square[10][29]=20.776164;square[11][0]=-5.604092;square[11][1]=15.870114;square[11][2]=7.447753;square[11][3]=20.900105;square[11][4]=-3.876405;square[11][5]=15.836084;square[11][6]=-31.996863;square[11][7]=7.195996;square[11][8]=-21.254667;square[11][9]=-4.211828;square[11][10]=-20.918037;square[11][11]=20.500600;square[11][12]=-51.508022;square[11][13]=20.219323;square[11][14]=-15.651956;square[11][15]=21.725090;square[11][16]=1.130838;square[11][17]=45.503477;square[11][18]=-0.546788;square[11][19]=35.023025;square[11][20]=-4.098583;square[11][21]=20.198775;square[11][22]=-5.337017;square[11][23]=19.559368;square[11][24]=-5.768832;square[11][25]=20.036361;square[11][26]=-5.919397;square[11][27]=20.664000;square[11][28]=-5.971895;square[11][29]=20.882844;square[12][0]=-6.670444;square[12][1]=15.756067;square[12][2]=9.196508;square[12][3]=22.690793;square[12][4]=7.080215;square[12][5]=13.902951;square[12][6]=-9.296126;square[12][7]=7.376008;square[12][8]=-9.704990;square[12][9]=-3.965473;square[12][10]=-14.617641;square[12][11]=18.096518;square[12][12]=-33.170072;square[12][13]=14.440512;square[12][14]=-6.413942;square[12][15]=20.349715;square[12][16]=-1.196345;square[12][17]=42.210367;square[12][18]=0.656654;square[12][19]=16.877963;square[12][20]=-3.027647;square[12][21]=9.321584;square[12][22]=-4.312283;square[12][23]=14.181879;square[12][24]=-5.228767;square[12][25]=17.788584;square[12][26]=-5.731088;square[12][27]=19.228927;square[12][28]=-5.906236;square[12][29]=19.731143;square[13][0]=-6.025795;square[13][1]=14.360329;square[13][2]=6.190560;square[13][3]=18.183920;square[13][4]=6.603373;square[13][5]=15.921291;square[13][6]=-26.757672;square[13][7]=9.179909;square[13][8]=-21.157152;square[13][9]=4.357832;square[13][10]=-20.205341;square[13][11]=23.383156;square[13][12]=-45.530680;square[13][13]=24.745748;square[13][14]=-8.686974;square[13][15]=27.576973;square[13][16]=-0.936302;square[13][17]=35.661463;square[13][18]=-1.820180;square[13][19]=11.723290;square[13][20]=-3.891265;square[13][21]=12.042361;square[13][22]=-4.613408;square[13][23]=16.797391;square[13][24]=-4.865204;square[13][25]=18.883319;square[13][26]=-4.952999;square[13][27]=19.610637;square[13][28]=-4.983612;square[13][29]=19.864238;square[14][0]=-6.025795;square[14][1]=15.370604;square[14][2]=-6.954990;square[14][3]=19.636116;square[14][4]=6.141072;square[14][5]=22.633463;square[14][6]=-13.628948;square[14][7]=22.274630;square[14][8]=-28.898389;square[14][9]=9.639888;square[14][10]=-20.199476;square[14][11]=11.399186;square[14][12]=-16.482239;square[14][13]=35.520369;square[14][14]=-32.604244;square[14][15]=29.433482;square[14][16]=-3.767682;square[14][17]=28.080860;square[14][18]=-3.189818;square[14][19]=44.056157;square[14][20]=-5.167305;square[14][21]=26.156332;square[14][22]=-7.012300;square[14][23]=14.885524;square[14][24]=-7.655610;square[14][25]=16.546479;square[14][26]=-7.879919;square[14][27]=18.897920;square[14][28]=-7.958130;square[14][29]=20.267050;square[15][0]=-4.624942;square[15][1]=14.873657;square[15][2]=10.211795;square[15][3]=23.936194;square[15][4]=6.439155;square[15][5]=27.456716;square[15][6]=-14.742159;square[15][7]=19.787349;square[15][8]=-11.693525;square[15][9]=13.756177;square[15][10]=-12.932325;square[15][11]=31.961927;square[15][12]=-23.418734;square[15][13]=30.329965;square[15][14]=3.963749;square[15][15]=28.059129;square[15][16]=4.195697;square[15][17]=49.915086;square[15][18]=-0.303823;square[15][19]=34.079383;square[15][20]=-6.458015;square[15][21]=17.483971;square[15][22]=-8.113665;square[15][23]=17.141220;square[15][24]=-8.690954;square[15][25]=18.578631;square[15][26]=-8.892242;square[15][27]=19.504399;square[15][28]=-8.962427;square[15][29]=20.236705;square[16][0]=-6.125795;square[16][1]=16.805394;square[16][2]=-1.899336;square[16][3]=21.864336;square[16][4]=12.758089;square[16][5]=17.294404;square[16][6]=-22.203957;square[16][7]=14.196032;square[16][8]=-14.935947;square[16][9]=8.991488;square[16][10]=-13.790260;square[16][11]=36.485532;square[16][12]=-48.797809;square[16][13]=28.208053;square[16][14]=-10.790495;square[16][15]=24.841565;square[16][16]=-1.099038;square[16][17]=37.969034;square[16][18]=-5.564536;square[16][19]=31.389080;square[16][20]=-8.601510;square[16][21]=16.066175;square[16][22]=-8.861055;square[16][23]=16.633014;square[16][24]=-8.982533;square[16][25]=18.035793;square[16][26]=-9.123528;square[16][27]=17.899187;square[16][28]=-8.521368;square[16][29]=18.616170;square[17][0]=-5.824942;square[17][1]=15.357534;square[17][2]=2.604402;square[17][3]=19.702771;square[17][4]=-2.354115;square[17][5]=22.963109;square[17][6]=-22.839531;square[17][7]=18.800881;square[17][8]=-15.459121;square[17][9]=1.652962;square[17][10]=-14.318728;square[17][11]=33.272229;square[17][12]=-31.772073;square[17][13]=33.443191;square[17][14]=-10.514785;square[17][15]=25.137448;square[17][16]=-1.600811;square[17][17]=35.309510;square[17][18]=-1.216411;square[17][19]=28.103540;square[17][20]=-6.286030;square[17][21]=13.263039;square[17][22]=-8.053697;square[17][23]=15.718968;square[17][24]=-8.018723;square[17][25]=18.126975;square[17][26]=-8.006528;square[17][27]=19.690816;square[17][28]=-8.002276;square[17][29]=20.543516;square[18][0]=-6.548375;square[18][1]=15.061464;square[18][2]=-8.272593;square[18][3]=20.363815;square[18][4]=17.937460;square[18][5]=20.123309;square[18][6]=-11.712936;square[18][7]=18.873151;square[18][8]=-11.746699;square[18][9]=6.145194;square[18][10]=-15.914329;square[18][11]=36.726125;square[18][12]=-35.902774;square[18][13]=35.030242;square[18][14]=-19.615876;square[18][15]=23.118125;square[18][16]=-4.277537;square[18][17]=36.040723;square[18][18]=-13.616250;square[18][19]=37.746528;square[18][20]=-13.699596;square[18][21]=16.936574;square[18][22]=-10.888797;square[18][23]=16.424247;square[18][24]=-9.658583;square[18][25]=19.024212;square[18][26]=-9.229634;square[18][27]=20.311085;square[18][28]=-8.558365;square[18][29]=20.759790;square[19][0]=-6.168842;square[19][1]=15.264534;square[19][2]=-7.704625;square[19][3]=20.954131;square[19][4]=22.356386;square[19][5]=21.932779;square[19][6]=-15.064042;square[19][7]=16.315090;square[19][8]=-13.252086;square[19][9]=2.691504;square[19][10]=-6.396028;square[19][11]=24.892937;square[19][12]=-26.389481;square[19][13]=32.620213;square[19][14]=-17.346747;square[19][15]=28.777373;square[19][16]=7.679436;square[19][17]=37.860341;square[19][18]=0.399240;square[19][19]=26.131098;square[19][20]=-6.141947;square[19][21]=2.032877;square[19][22]=-8.003459;square[19][23]=7.413470;square[19][24]=-8.381528;square[19][25]=14.717115;square[19][26]=-8.133030;square[19][27]=18.157972;square[19][28]=-8.046385;square[19][29]=19.826284;square[20][0]=-5.513216;square[20][1]=15.626214;square[20][2]=-5.545164;square[20][3]=21.741563;square[20][4]=2.071441;square[20][5]=29.716235;square[20][6]=-36.314015;square[20][7]=27.850007;square[20][8]=-29.720317;square[20][9]=8.384884;square[20][10]=-20.638459;square[20][11]=27.265713;square[20][12]=-36.544555;square[20][13]=31.964320;square[20][14]=1.857413;square[20][15]=32.573997;square[20][16]=10.056682;square[20][17]=54.642572;square[20][18]=3.162822;square[20][19]=31.264712;square[20][20]=-1.280974;square[20][21]=15.824079;square[20][22]=-3.051934;square[20][23]=17.462033;square[20][24]=-2.982390;square[20][25]=19.345119;square[20][26]=-3.563392;square[20][27]=21.074300;square[20][28]=-3.847764;square[20][29]=21.677228;square[21][0]=-6.028011;square[21][1]=15.405364;square[21][2]=-7.863732;square[21][3]=21.554557;square[21][4]=15.015254;square[21][5]=27.805668;square[21][6]=-12.780094;square[21][7]=21.712768;square[21][8]=-12.878692;square[21][9]=8.088662;square[21][10]=-13.837791;square[21][11]=36.010794;square[21][12]=-28.521289;square[21][13]=35.701695;square[21][14]=-26.021683;square[21][15]=29.789749;square[21][16]=-1.852978;square[21][17]=34.564486;square[21][18]=-1.807021;square[21][19]=37.057344;square[21][20]=-5.941616;square[21][21]=18.847036;square[21][22]=-6.902286;square[21][23]=17.702962;square[21][24]=-6.314608;square[21][25]=18.340329;square[21][26]=-6.109697;square[21][27]=19.421309;square[21][28]=-6.038249;square[21][29]=19.798223;square[22][0]=-5.915818;square[22][1]=16.339767;square[22][2]=-3.768253;square[22][3]=25.009864;square[22][4]=33.225282;square[22][5]=27.147443;square[22][6]=-2.793381;square[22][7]=18.710643;square[22][8]=-10.117406;square[22][9]=7.011134;square[22][10]=-7.935013;square[22][11]=18.397073;square[22][12]=-19.764493;square[22][13]=28.734363;square[22][14]=-40.219156;square[22][15]=26.003159;square[22][16]=-5.775697;square[22][17]=35.160335;square[22][18]=-2.333008;square[22][19]=47.804145;square[22][20]=-5.936022;square[22][21]=19.018391;square[22][22]=-7.924419;square[22][23]=18.375942;square[22][24]=-7.973647;square[22][25]=20.433076;square[22][26]=-7.990811;square[22][27]=21.453647;square[22][28]=-7.996796;square[22][29]=21.538499;square[23][0]=-6.125795;square[23][1]=16.017944;square[23][2]=-6.035738;square[23][3]=22.329472;square[23][4]=12.003096;square[23][5]=23.207701;square[23][6]=-20.377076;square[23][7]=23.583897;square[23][8]=-12.819495;square[23][9]=10.719730;square[23][10]=-15.249198;square[23][11]=36.130924;square[23][12]=-36.903051;square[23][13]=35.648307;square[23][14]=-4.408436;square[23][15]=34.574676;square[23][16]=0.225769;square[23][17]=40.107292;square[23][18]=-1.213439;square[23][19]=30.658861;square[23][20]=-5.997692;square[23][21]=16.185275;square[23][22]=-6.689259;square[23][23]=17.175163;square[23][24]=-6.891651;square[23][25]=19.293599;square[23][26]=-6.962221;square[23][27]=20.405015;square[23][28]=-6.986827;square[23][29]=20.892542;square[24][0]=-6.026672;square[24][1]=17.195273;square[24][2]=5.252078;square[24][3]=22.482043;square[24][4]=4.709883;square[24][5]=22.752198;square[24][6]=-23.474722;square[24][7]=18.842920;square[24][8]=-15.322345;square[24][9]=8.252677;square[24][10]=-14.266545;square[24][11]=32.157551;square[24][12]=-41.637050;square[24][13]=26.367089;square[24][14]=-23.909042;square[24][15]=24.405953;square[24][16]=-8.421252;square[24][17]=44.009343;square[24][18]=-3.880048;square[24][19]=32.491094;square[24][20]=-3.958175;square[24][21]=18.561195;square[24][22]=-3.985417;square[24][23]=18.878517;square[24][24]=-3.994915;square[24][25]=20.223863;square[24][26]=-3.998227;square[24][27]=21.950232;square[24][28]=-3.999382;square[24][29]=22.633969;square[25][0]=-5.513216;square[25][1]=16.071088;square[25][2]=5.797388;square[25][3]=22.438982;square[25][4]=3.478405;square[25][5]=18.944902;square[25][6]=-24.253729;square[25][7]=13.224373;square[25][8]=-16.800882;square[25][9]=2.638394;square[25][10]=-15.347292;square[25][11]=22.425928;square[25][12]=-41.235642;square[25][13]=20.406152;square[25][14]=-9.882230;square[25][15]=23.845723;square[25][16]=0.171937;square[25][17]=43.527453;square[25][18]=-1.468670;square[25][19]=25.703167;square[25][20]=-3.850490;square[25][21]=16.480506;square[25][22]=-5.008701;square[25][23]=18.407847;square[25][24]=-5.654355;square[25][25]=19.444851;square[25][26]=-5.879481;square[25][27]=20.457753;square[25][28]=-5.957978;square[25][29]=21.332633;square[26][0]=-6.371911;square[26][1]=16.929784;square[26][2]=-7.892384;square[26][3]=21.826153;square[26][4]=19.707428;square[26][5]=13.170448;square[26][6]=-17.454750;square[26][7]=13.348289;square[26][8]=-19.012177;square[26][9]=4.161823;square[26][10]=-18.200879;square[26][11]=23.116122;square[26][12]=-31.675151;square[26][13]=31.472923;square[26][14]=-4.163167;square[26][15]=27.941576;square[26][16]=11.849203;square[26][17]=38.166483;square[26][18]=5.337519;square[26][19]=31.524669;square[26][20]=-2.249921;square[26][21]=13.278584;square[26][22]=-4.692428;square[26][23]=16.266941;square[26][24]=-5.544078;square[26][25]=19.962264;square[26][26]=-5.841030;square[26][27]=21.289485;square[26][28]=-5.944571;square[26][29]=21.752259;square[27][0]=-6.125795;square[27][1]=15.927067;square[27][2]=9.992640;square[27][3]=25.068403;square[27][4]=18.737458;square[27][5]=24.043794;square[27][6]=-21.444421;square[27][7]=17.274604;square[27][8]=-15.190346;square[27][9]=8.576170;square[27][10]=-11.817271;square[27][11]=29.719383;square[27][12]=-35.042750;square[27][13]=26.489452;square[27][14]=-14.490626;square[27][15]=23.427385;square[27][16]=3.307411;square[27][17]=33.793080;square[27][18]=-3.142733;square[27][19]=40.841344;square[27][20]=-5.957892;square[27][21]=21.187147;square[27][22]=-6.636639;square[27][23]=19.823869;square[27][24]=-6.873304;square[27][25]=19.938587;square[27][26]=-6.955824;square[27][27]=20.078587;square[27][28]=-6.984597;square[27][29]=20.778723;square[28][0]=-5.513216;square[28][1]=16.552716;square[28][2]=2.764588;square[28][3]=31.275132;square[28][4]=26.125636;square[28][5]=35.671119;square[28][6]=-15.385820;square[28][7]=25.761944;square[28][8]=-14.866240;square[28][9]=7.540824;square[28][10]=-15.877806;square[28][11]=28.989253;square[28][12]=-34.473044;square[28][13]=31.192963;square[28][14]=-31.476721;square[28][15]=33.961078;square[28][16]=-0.485698;square[28][17]=37.865837;square[28][18]=1.297608;square[28][19]=41.736913;square[28][20]=-5.069815;square[28][21]=15.793478;square[28][22]=-6.978308;square[28][23]=14.816782;square[28][24]=-7.643758;square[28][25]=18.520445;square[28][26]=-7.354083;square[28][27]=20.135433;square[28][28]=-7.123461;square[28][29]=20.698544;square[29][0]=-5.604092;square[29][1]=16.942249;square[29][2]=-0.468130;square[29][3]=20.731360;square[29][4]=10.877102;square[29][5]=17.065961;square[29][6]=-16.303906;square[29][7]=14.878435;square[29][8]=-10.103425;square[29][9]=10.831196;square[29][10]=-16.820782;square[29][11]=32.188574;square[29][12]=-43.566724;square[29][13]=31.724332;square[29][14]=-3.135653;square[29][15]=26.614838;square[29][16]=6.561848;square[29][17]=36.747806;square[29][18]=-0.996579;square[29][19]=14.376099;square[29][20]=-5.678589;square[29][21]=11.785426;square[29][22]=-7.190574;square[29][23]=16.797373;square[29][24]=-7.717771;square[29][25]=20.185956;square[29][26]=-7.901593;square[29][27]=21.367482;square[29][28]=-7.965687;square[29][29]=21.779455;square[30][0]=-6.068842;square[30][1]=16.808326;square[30][2]=-5.975999;square[30][3]=23.256383;square[30][4]=8.421963;square[30][5]=25.639670;square[30][6]=-20.146650;square[30][7]=19.621512;square[30][8]=-11.569360;square[30][9]=7.881931;square[30][10]=-18.126861;square[30][11]=21.632271;square[30][12]=-27.201829;square[30][13]=29.950241;square[30][14]=-29.691684;square[30][15]=33.126492;square[30][16]=-4.282480;square[30][17]=35.475960;square[30][18]=-6.604371;square[30][19]=46.828025;square[30][20]=-9.299535;square[30][21]=17.464158;square[30][22]=-9.234060;square[30][23]=14.778394;square[30][24]=-9.081612;square[30][25]=17.489275;square[30][26]=-9.028456;square[30][27]=20.002633;square[30][28]=-8.087601;square[30][29]=21.303561;square[31][0]=-5.716285;square[31][1]=16.371716;square[31][2]=2.579728;square[31][3]=18.432529;square[31][4]=11.500830;square[31][5]=18.537276;square[31][6]=-20.434665;square[31][7]=15.495865;square[31][8]=-17.657059;square[31][9]=-0.396364;square[31][10]=-13.635556;square[31][11]=13.992823;square[31][12]=-30.797647;square[31][13]=20.420294;square[31][14]=-29.214477;square[31][15]=20.239028;square[31][16]=4.153313;square[31][17]=30.134345;square[31][18]=0.977133;square[31][19]=45.347636;square[31][20]=-4.011820;square[31][21]=18.555528;square[31][22]=-5.958086;square[31][23]=17.195377;square[31][24]=-6.636707;square[31][25]=18.839326;square[31][26]=-6.873328;square[31][27]=19.695298;square[31][28]=-6.955832;square[31][29]=20.545079;square[32][0]=-6.218113;square[32][1]=16.311694;square[32][2]=11.775181;square[32][3]=22.308013;square[32][4]=-1.802207;square[32][5]=10.726769;square[32][6]=-25.382013;square[32][7]=10.502145;square[32][8]=-17.104755;square[32][9]=6.976997;square[32][10]=-15.000672;square[32][11]=28.774581;square[32][12]=-43.014944;square[32][13]=25.410637;square[32][14]=-11.682322;square[32][15]=31.174152;square[32][16]=5.002471;square[32][17]=40.027747;square[32][18]=0.612967;square[32][19]=27.691070;square[32][20]=-3.194138;square[32][21]=10.956031;square[32][22]=-5.021656;square[32][23]=14.371453;square[32][24]=-5.658873;square[32][25]=18.568238;square[32][26]=-5.881056;square[32][27]=20.152097;square[32][28]=-5.958527;square[32][29]=20.704354;square[33][0]=-5.616285;square[33][1]=16.430644;square[33][2]=1.248868;square[33][3]=21.331772;square[33][4]=0.392223;square[33][5]=14.790994;square[33][6]=-26.010779;square[33][7]=13.889283;square[33][8]=-16.052832;square[33][9]=2.562319;square[33][10]=-10.974903;square[33][11]=26.393984;square[33][12]=-27.471539;square[33][13]=27.247448;square[33][14]=-10.880535;square[33][15]=25.555378;square[33][16]=1.397377;square[33][17]=39.627120;square[33][18]=-1.494969;square[33][19]=25.787143;square[33][20]=-7.693024;square[33][21]=12.230605;square[33][22]=-9.095607;square[33][23]=15.714231;square[33][24]=-9.033336;square[33][25]=19.000803;square[33][26]=-9.011624;square[33][27]=20.954245;square[33][28]=-8.482350;square[33][29]=21.635368;square[34][0]=-6.269816;square[34][1]=16.240644;square[34][2]=-1.137639;square[34][3]=22.755801;square[34][4]=12.662878;square[34][5]=23.992333;square[34][6]=-20.597087;square[34][7]=18.490078;square[34][8]=-17.940216;square[34][9]=5.345192;square[34][10]=-17.066855;square[34][11]=28.069845;square[34][12]=-40.116598;square[34][13]=24.753204;square[34][14]=-32.909885;square[34][15]=25.389235;square[34][16]=-6.355645;square[34][17]=32.998189;square[34][18]=-4.796740;square[34][19]=35.673738;square[34][20]=-6.059855;square[34][21]=12.947615;square[34][22]=-6.572192;square[34][23]=14.031912;square[34][24]=-6.199511;square[34][25]=17.750696;square[34][26]=-6.682145;square[34][27]=19.828296;square[34][28]=-6.889171;square[34][29]=20.591452;square[35][0]=-5.935795;square[35][1]=15.737067;square[35][2]=16.886006;square[35][3]=25.290347;square[35][4]=-0.218538;square[35][5]=25.646236;square[35][6]=-18.558127;square[35][7]=16.474229;square[35][8]=-18.759563;square[35][9]=5.990094;square[35][10]=-16.853996;square[35][11]=27.918046;square[35][12]=-45.857131;square[35][13]=27.538038;square[35][14]=-21.783547;square[35][15]=22.108255;square[35][16]=-6.129062;square[35][17]=45.253914;square[35][18]=-3.836638;square[35][19]=33.967692;square[35][20]=-4.214039;square[35][21]=19.805178;square[35][22]=-4.725952;square[35][23]=19.377877;square[35][24]=-4.904445;square[35][25]=20.352612;square[35][26]=-4.966682;square[35][27]=20.774270;square[35][28]=-4.988383;square[35][29]=20.921293;square[36][0]=-7.294877;square[36][1]=16.286623;square[36][2]=2.683316;square[36][3]=22.523155;square[36][4]=25.216383;square[36][5]=26.980154;square[36][6]=-18.134148;square[36][7]=24.039196;square[36][8]=-15.519315;square[36][9]=12.840864;square[36][10]=-13.348512;square[36][11]=26.685123;square[36][12]=-24.865259;square[36][13]=27.219442;square[36][14]=-19.985131;square[36][15]=23.887085;square[36][16]=7.399614;square[36][17]=31.730432;square[36][18]=4.770162;square[36][19]=36.718457;square[36][20]=-3.353610;square[36][21]=14.620163;square[36][22]=-6.379904;square[36][23]=13.846904;square[36][24]=-7.435107;square[36][25]=17.421636;square[36][26]=-7.803034;square[36][27]=19.100980;square[36][28]=-7.931322;square[36][29]=19.686531;square[37][0]=-5.513216;square[37][1]=15.314488;square[37][2]=13.366702;square[37][3]=19.796058;square[37][4]=12.111718;square[37][5]=20.336178;square[37][6]=-28.796848;square[37][7]=18.013187;square[37][8]=-25.541188;square[37][9]=-2.066947;square[37][10]=-22.973967;square[37][11]=13.373361;square[37][12]=-23.461156;square[37][13]=20.981894;square[37][14]=-40.875075;square[37][15]=19.290208;square[37][16]=8.842752;square[37][17]=25.747899;square[37][18]=9.413734;square[37][19]=45.299728;square[37][20]=4.234265;square[37][21]=18.565780;square[37][22]=-1.780211;square[37][23]=12.745226;square[37][24]=-3.877329;square[37][25]=15.816650;square[37][26]=-4.608549;square[37][27]=18.092123;square[37][28]=-4.863509;square[37][29]=19.986086;square[38][0]=-5.657236;square[38][1]=15.401555;square[38][2]=9.582529;square[38][3]=17.094327;square[38][4]=-11.058439;square[38][5]=11.591671;square[38][6]=-29.048934;square[38][7]=8.629664;square[38][8]=-20.776279;square[38][9]=2.071091;square[38][10]=-18.014570;square[38][11]=29.653338;square[38][12]=-38.837076;square[38][13]=34.878993;square[38][14]=-20.952893;square[38][15]=34.321177;square[38][16]=4.797399;square[38][17]=38.134162;square[38][18]=-6.065566;square[38][19]=33.345262;square[38][20]=-11.029680;square[38][21]=14.463069;square[38][22]=-10.015127;square[38][23]=15.057792;square[38][24]=-9.353953;square[38][25]=17.586695;square[38][26]=-8.510836;square[38][27]=18.507211;square[38][28]=-5.727800;square[38][29]=17.569093;square[39][0]=-6.289402;square[39][1]=14.016811;square[39][2]=-5.859973;square[39][3]=18.726434;square[39][4]=16.057697;square[39][5]=21.051865;square[39][6]=-18.844027;square[39][7]=19.080841;square[39][8]=-17.044384;square[39][9]=5.575983;square[39][10]=-16.739592;square[39][11]=24.693056;square[39][12]=-25.433819;square[39][13]=32.045466;square[39][14]=-37.076794;square[39][15]=26.930377;square[39][16]=3.282509;square[39][17]=33.227901;square[39][18]=6.658231;square[39][19]=50.516633;square[39][20]=0.390743;square[39][21]=27.113261;square[39][22]=-3.771686;square[39][23]=20.846774;square[39][24]=-5.223035;square[39][25]=20.213463;square[39][26]=-5.729089;square[39][27]=20.074430;square[39][28]=-5.905539;square[39][29]=20.025952;
  
}
void fill_matrix_triangle(){
 
  triangle[0][0]=-5.556262;triangle[0][1]=15.357534;triangle[0][2]=-2.604831;triangle[0][3]=17.173349;triangle[0][4]=13.465347;triangle[0][5]=-5.780742;triangle[0][6]=-14.620628;triangle[0][7]=19.983417;triangle[0][8]=-27.713552;triangle[0][9]=31.258031;triangle[0][10]=-31.530247;triangle[0][11]=30.708467;triangle[0][12]=8.009335;triangle[0][13]=30.844769;triangle[0][14]=12.872251;triangle[0][15]=48.161278;triangle[0][16]=-5.777360;triangle[0][17]=35.958423;triangle[0][18]=-20.657010;triangle[0][19]=11.110685;triangle[0][20]=-15.539658;triangle[0][21]=13.183993;triangle[0][22]=-10.710705;triangle[0][23]=17.251975;triangle[0][24]=-8.945164;triangle[0][25]=19.451333;triangle[0][26]=-8.329558;triangle[0][27]=20.460013;triangle[0][28]=-8.114910;triangle[0][29]=21.463040;triangle[1][0]=-4.900636;triangle[1][1]=15.405364;triangle[1][2]=-5.590789;triangle[1][3]=20.326027;triangle[1][4]=5.699889;triangle[1][5]=11.095366;triangle[1][6]=8.424461;triangle[1][7]=8.112746;triangle[1][8]=-13.836118;triangle[1][9]=33.407908;triangle[1][10]=-38.259015;triangle[1][11]=36.642663;triangle[1][12]=-43.255325;triangle[1][13]=34.990533;triangle[1][14]=4.484899;triangle[1][15]=37.074601;triangle[1][16]=15.112454;triangle[1][17]=48.386792;triangle[1][18]=-2.974680;triangle[1][19]=33.617313;triangle[1][20]=-16.142383;triangle[1][21]=12.719436;triangle[1][22]=-12.852862;triangle[1][23]=15.940706;triangle[1][24]=-9.465341;triangle[1][25]=19.679833;triangle[1][26]=-7.859611;triangle[1][27]=22.454909;triangle[1][28]=-7.299728;triangle[1][29]=23.805160;triangle[2][0]=-5.716285;triangle[2][1]=17.755682;triangle[2][2]=-5.125238;triangle[2][3]=19.436989;triangle[2][4]=6.159397;triangle[2][5]=7.819762;triangle[2][6]=-7.489072;triangle[2][7]=16.042860;triangle[2][8]=-26.097588;triangle[2][9]=39.803884;triangle[2][10]=-45.332161;triangle[2][11]=32.731247;triangle[2][12]=-7.942079;triangle[2][13]=19.711734;triangle[2][14]=18.620422;triangle[2][15]=28.138591;triangle[2][16]=17.069574;triangle[2][17]=43.840280;triangle[2][18]=-2.774680;triangle[2][19]=17.979959;triangle[2][20]=-10.727649;triangle[2][21]=12.218477;triangle[2][22]=-9.469901;triangle[2][23]=16.883816;triangle[2][24]=-8.512523;triangle[2][25]=19.908675;triangle[2][26]=-8.178706;triangle[2][27]=22.184233;triangle[2][28]=-7.449731;triangle[2][29]=23.637881;triangle[3][0]=-5.323216;triangle[3][1]=17.149294;triangle[3][2]=-3.471694;triangle[3][3]=23.432961;triangle[3][4]=2.247782;triangle[3][5]=15.047677;triangle[3][6]=-16.908743;triangle[3][7]=25.228693;triangle[3][8]=-26.978230;triangle[3][9]=31.723113;triangle[3][10]=-36.816301;triangle[3][11]=31.683999;triangle[3][12]=4.437212;triangle[3][13]=35.917675;triangle[3][14]=16.814801;triangle[3][15]=47.856370;triangle[3][16]=10.503519;triangle[3][17]=38.907766;triangle[3][18]=-2.810175;triangle[3][19]=17.935916;triangle[3][20]=-6.570741;triangle[3][21]=18.298154;triangle[3][22]=-7.501648;triangle[3][23]=20.730885;triangle[3][24]=-7.826236;triangle[3][25]=22.398809;triangle[3][26]=-7.939412;triangle[3][27]=23.441699;triangle[3][28]=-7.978874;triangle[3][29]=23.805333;triangle[4][0]=-6.216672;triangle[4][1]=19.123888;triangle[4][2]=-3.054120;triangle[4][3]=25.707904;triangle[4][4]=16.955304;triangle[4][5]=10.810616;triangle[4][6]=-6.288007;triangle[4][7]=28.108890;triangle[4][8]=-27.591434;triangle[4][9]=38.860552;triangle[4][10]=-20.512064;triangle[4][11]=31.754452;triangle[4][12]=13.291632;triangle[4][13]=31.903362;triangle[4][14]=20.288386;triangle[4][15]=46.921391;triangle[4][16]=11.475575;triangle[4][17]=37.674003;triangle[4][18]=-2.599512;triangle[4][19]=16.106753;triangle[4][20]=-6.116966;triangle[4][21]=16.487659;triangle[4][22]=-7.243427;triangle[4][23]=20.039224;triangle[4][24]=-7.084878;triangle[4][25]=21.784879;triangle[4][26]=-7.029595;triangle[4][27]=22.576313;triangle[4][28]=-7.010319;triangle[4][29]=22.852270;triangle[5][0]=-5.445285;triangle[5][1]=18.453475;triangle[5][2]=-3.779907;triangle[5][3]=13.477156;triangle[5][4]=5.893466;triangle[5][5]=8.112871;triangle[5][6]=-14.661351;triangle[5][7]=25.420242;triangle[5][8]=-40.497905;triangle[5][9]=37.257967;triangle[5][10]=-39.119215;triangle[5][11]=31.419992;triangle[5][12]=3.270520;triangle[5][13]=25.923420;triangle[5][14]=11.918876;triangle[5][15]=38.484827;triangle[5][16]=0.618249;triangle[5][17]=26.920270;triangle[5][18]=-13.330855;triangle[5][19]=10.949926;triangle[5][20]=-11.546121;triangle[5][21]=15.120646;triangle[5][22]=-8.767897;triangle[5][23]=20.062318;triangle[5][24]=-7.616428;triangle[5][25]=22.627015;triangle[5][26]=-7.214935;triangle[5][27]=23.521270;triangle[5][28]=-7.074943;triangle[5][29]=23.833077;triangle[6][0]=-5.647139;triangle[6][1]=17.554829;triangle[6][2]=-1.390705;triangle[6][3]=18.395210;triangle[6][4]=6.391166;triangle[6][5]=4.794507;triangle[6][6]=-12.532568;triangle[6][7]=29.400451;triangle[6][8]=-29.720461;triangle[6][9]=39.385511;triangle[6][10]=-38.194775;triangle[6][11]=28.213354;triangle[6][12]=1.577520;triangle[6][13]=32.084885;triangle[6][14]=12.798482;triangle[6][15]=44.562282;triangle[6][16]=1.487403;triangle[6][17]=29.774587;triangle[6][18]=-10.380103;triangle[6][19]=14.383584;triangle[6][20]=-9.784506;triangle[6][21]=17.981853;triangle[6][22]=-9.273540;triangle[6][23]=21.637051;triangle[6][24]=-8.573674;triangle[6][25]=23.176091;triangle[6][26]=-8.200028;triangle[6][27]=23.712721;triangle[6][28]=-8.069745;triangle[6][29]=23.899832;triangle[7][0]=-5.513216;triangle[7][1]=17.152226;triangle[7][2]=-4.270910;triangle[7][3]=16.229762;triangle[7][4]=-0.139888;triangle[7][5]=9.965607;triangle[7][6]=-13.885663;triangle[7][7]=29.596009;triangle[7][8]=-40.302090;triangle[7][9]=38.746378;triangle[7][10]=-46.894604;triangle[7][11]=30.777384;triangle[7][12]=-3.841999;triangle[7][13]=27.026881;triangle[7][14]=8.373731;triangle[7][15]=40.508574;triangle[7][16]=3.861042;triangle[7][17]=31.927527;triangle[7][18]=-15.068394;triangle[7][19]=13.592688;triangle[7][20]=-13.591028;triangle[7][21]=15.821590;triangle[7][22]=-10.368535;triangle[7][23]=19.763933;triangle[7][24]=-8.825857;triangle[7][25]=21.220332;triangle[7][26]=-8.287959;triangle[7][27]=22.249850;triangle[7][28]=-8.000405;triangle[7][29]=23.260142;triangle[8][0]=-5.513216;triangle[8][1]=17.095273;triangle[8][2]=-0.657803;triangle[8][3]=26.150457;triangle[8][4]=9.884485;triangle[8][5]=15.859338;triangle[8][6]=-4.353103;triangle[8][7]=14.456414;triangle[8][8]=-37.815428;triangle[8][9]=27.752227;triangle[8][10]=-43.070426;triangle[8][11]=32.212349;triangle[8][12]=4.456406;triangle[8][13]=31.652965;triangle[8][14]=13.350580;triangle[8][15]=44.823279;triangle[8][16]=5.336666;triangle[8][17]=32.139846;triangle[8][18]=-8.833367;triangle[8][19]=13.217752;triangle[8][20]=-9.577880;triangle[8][21]=16.152968;triangle[8][22]=-9.148350;triangle[8][23]=19.739761;triangle[8][24]=-9.051727;triangle[8][25]=21.924483;triangle[8][26]=-8.918036;triangle[8][27]=23.276312;triangle[8][28]=-8.320099;triangle[8][29]=23.747666;triangle[9][0]=-5.659332;triangle[9][1]=17.187123;triangle[9][2]=3.034028;triangle[9][3]=14.933127;triangle[9][4]=6.072108;triangle[9][5]=6.076274;triangle[9][6]=-29.117978;triangle[9][7]=26.931995;triangle[9][8]=-53.713699;triangle[9][9]=27.912483;triangle[9][10]=-40.497520;triangle[9][11]=22.620969;triangle[9][12]=-1.213568;triangle[9][13]=38.261355;triangle[9][14]=13.097083;triangle[9][15]=43.440079;triangle[9][16]=-6.656111;triangle[9][17]=17.250398;triangle[9][18]=-11.247910;triangle[9][19]=13.078732;triangle[9][20]=-9.594586;triangle[9][21]=17.336554;triangle[9][22]=-8.555998;triangle[9][23]=20.109406;triangle[9][24]=-8.193864;triangle[9][25]=21.611790;triangle[9][26]=-8.067596;triangle[9][27]=22.705961;triangle[9][28]=-8.023569;triangle[9][29]=23.548797;triangle[10][0]=-4.900636;triangle[10][1]=15.927067;triangle[10][2]=-1.487491;triangle[10][3]=18.242312;triangle[10][4]=-0.025679;triangle[10][5]=10.245109;triangle[10][6]=-9.777235;triangle[10][7]=24.073462;triangle[10][8]=-23.535617;triangle[10][9]=35.184709;triangle[10][10]=-44.772387;triangle[10][11]=26.966014;triangle[10][12]=-7.849391;triangle[10][13]=26.694545;triangle[10][14]=8.909931;triangle[10][15]=43.909012;triangle[10][16]=-3.794672;triangle[10][17]=49.141998;triangle[10][18]=-23.932554;triangle[10][19]=13.633314;triangle[10][20]=-19.490349;triangle[10][21]=13.200850;triangle[10][22]=-12.907908;triangle[10][23]=17.460923;triangle[10][24]=-10.362603;triangle[10][25]=19.766000;triangle[10][26]=-9.285110;triangle[10][27]=21.091434;triangle[10][28]=-7.835511;triangle[10][29]=21.683203;triangle[11][0]=-4.900636;triangle[11][1]=17.195273;triangle[11][2]=1.290657;triangle[11][3]=16.674708;triangle[11][4]=6.510212;triangle[11][5]=5.062271;triangle[11][6]=-18.690788;triangle[11][7]=25.785603;triangle[11][8]=-39.995565;triangle[11][9]=31.283187;triangle[11][10]=-35.753604;triangle[11][11]=25.246374;triangle[11][12]=5.273072;triangle[11][13]=28.455183;triangle[11][14]=14.898918;triangle[11][15]=48.355861;triangle[11][16]=-4.122759;triangle[11][17]=35.905222;triangle[11][18]=-16.226008;triangle[11][19]=17.554410;triangle[11][20]=-13.225920;triangle[11][21]=18.258001;triangle[11][22]=-10.312350;triangle[11][23]=20.526885;triangle[11][24]=-8.806266;triangle[11][25]=21.486357;triangle[11][26]=-8.010128;triangle[11][27]=22.010904;triangle[11][28]=-6.700888;triangle[11][29]=22.655123;triangle[12][0]=-5.103706;triangle[12][1]=16.876639;triangle[12][2]=1.381717;triangle[12][3]=18.615460;triangle[12][4]=3.533240;triangle[12][5]=8.072603;triangle[12][6]=-16.416085;triangle[12][7]=29.152317;triangle[12][8]=-41.642683;triangle[12][9]=34.508701;triangle[12][10]=-17.454611;triangle[12][11]=35.560314;triangle[12][12]=18.352244;triangle[12][13]=34.638036;triangle[12][14]=22.303764;triangle[12][15]=43.831600;triangle[12][16]=0.394652;triangle[12][17]=24.744451;triangle[12][18]=-7.840164;triangle[12][19]=18.055222;triangle[12][20]=-8.907971;triangle[12][21]=20.468377;triangle[12][22]=-8.967912;triangle[12][23]=22.526788;triangle[12][24]=-8.988811;triangle[12][25]=23.486323;triangle[12][26]=-8.896099;triangle[12][27]=23.820892;triangle[12][28]=-7.902940;triangle[12][29]=23.937549;triangle[13][0]=-4.900636;triangle[13][1]=17.152226;triangle[13][2]=-6.919319;triangle[13][3]=24.217615;triangle[13][4]=-7.623190;triangle[13][5]=26.029842;triangle[13][6]=-7.868614;triangle[13][7]=26.661727;triangle[13][8]=-0.587446;triangle[13][9]=16.631138;triangle[13][10]=0.687483;triangle[13][11]=14.675872;triangle[13][12]=-14.425487;triangle[13][13]=42.294403;triangle[13][14]=-34.986744;triangle[13][15]=35.514774;triangle[13][16]=-5.093524;triangle[13][17]=29.432934;triangle[13][18]=10.821889;triangle[13][19]=42.854681;triangle[13][20]=2.880151;triangle[13][21]=42.235174;triangle[13][22]=-9.722391;triangle[13][23]=17.178788;triangle[13][24]=-9.555175;triangle[13][25]=17.253774;triangle[13][26]=-8.624045;triangle[13][27]=20.814985;triangle[13][28]=-8.217591;triangle[13][29]=22.889454;triangle[14][0]=-4.900636;triangle[14][1]=17.764806;triangle[14][2]=-2.834333;triangle[14][3]=20.622334;triangle[14][4]=7.503402;triangle[14][5]=8.260943;triangle[14][6]=-7.116313;triangle[14][7]=18.799152;triangle[14][8]=-26.930273;triangle[14][9]=29.007228;triangle[14][10]=-25.357140;triangle[14][11]=23.121488;triangle[14][12]=-2.336912;triangle[14][13]=26.704013;triangle[14][14]=2.510724;triangle[14][15]=44.856853;triangle[14][16]=-5.636662;triangle[14][17]=29.268546;triangle[14][18]=-10.693917;triangle[14][19]=17.804753;triangle[14][20]=-10.223743;triangle[14][21]=20.857706;triangle[14][22]=-8.904990;triangle[14][23]=22.904350;triangle[14][24]=-7.664229;triangle[14][25]=23.617970;triangle[14][26]=-7.231602;triangle[14][27]=23.866795;triangle[14][28]=-7.080755;triangle[14][29]=24.297454;triangle[15][0]=-5.935795;triangle[15][1]=16.536714;triangle[15][2]=-0.780567;triangle[15][3]=12.873898;triangle[15][4]=3.992125;triangle[15][5]=6.107852;triangle[15][6]=-15.303996;triangle[15][7]=28.267839;triangle[15][8]=-41.799519;triangle[15][9]=32.621963;triangle[15][10]=-27.004861;triangle[15][11]=30.340899;triangle[15][12]=5.053357;triangle[15][13]=40.731740;triangle[15][14]=10.417920;triangle[15][15]=49.346951;triangle[15][16]=-5.285886;triangle[15][17]=15.228756;triangle[15][18]=-8.388583;triangle[15][19]=13.574384;triangle[15][20]=-8.377302;triangle[15][21]=18.629258;triangle[15][22]=-8.131557;triangle[15][23]=21.814957;triangle[15][24]=-7.394550;triangle[15][25]=23.238123;triangle[15][26]=-7.137571;triangle[15][27]=23.734350;triangle[15][28]=-7.569671;triangle[15][29]=25.602101;triangle[16][0]=-5.212362;triangle[16][1]=17.617853;triangle[16][2]=-5.904111;triangle[16][3]=22.945559;triangle[16][4]=12.922350;triangle[16][5]=14.294840;triangle[16][6]=-5.813218;triangle[16][7]=33.616509;triangle[16][8]=-26.023503;triangle[16][9]=40.112631;triangle[16][10]=-29.995478;triangle[16][11]=27.856805;triangle[16][12]=6.430310;triangle[16][13]=30.678426;triangle[16][14]=11.979848;triangle[16][15]=45.963146;triangle[16][16]=2.700537;triangle[16][17]=30.130133;triangle[16][18]=-9.685043;triangle[16][19]=12.627240;triangle[16][20]=-9.702176;triangle[16][21]=16.128069;triangle[16][22]=-8.900934;triangle[16][23]=19.952584;triangle[16][24]=-8.314136;triangle[16][25]=21.630010;triangle[16][26]=-8.109532;triangle[16][27]=23.234894;triangle[16][28]=-8.038192;triangle[16][29]=24.384545;triangle[17][0]=-5.824942;triangle[17][1]=17.430170;triangle[17][2]=-2.612646;triangle[17][3]=27.069392;triangle[17][4]=8.992442;triangle[17][5]=14.477960;triangle[17][6]=-14.829769;triangle[17][7]=23.932561;triangle[17][8]=-37.132020;triangle[17][9]=30.792426;triangle[17][10]=-36.405774;triangle[17][11]=32.502126;triangle[17][12]=3.368719;triangle[17][13]=39.122820;triangle[17][14]=7.041241;triangle[17][15]=45.682089;triangle[17][16]=-7.517656;triangle[17][17]=27.021507;triangle[17][18]=-18.024310;triangle[17][19]=11.480192;triangle[17][20]=-14.060370;triangle[17][21]=15.666860;triangle[17][22]=-10.664442;triangle[17][23]=18.957687;triangle[17][24]=-8.277712;triangle[17][25]=20.387889;triangle[17][26]=-7.445511;triangle[17][27]=22.007425;triangle[17][28]=-7.155340;triangle[17][29]=23.122469;triangle[18][0]=-5.897842;triangle[18][1]=17.446172;triangle[18][2]=-5.473912;triangle[18][3]=21.944946;triangle[18][4]=7.936118;triangle[18][5]=7.724650;triangle[18][6]=-11.229940;triangle[18][7]=19.242841;triangle[18][8]=-31.200398;triangle[18][9]=35.312697;triangle[18][10]=-48.974169;triangle[18][11]=28.721128;triangle[18][12]=-5.107145;triangle[18][13]=23.422344;triangle[18][14]=7.491142;triangle[18][15]=32.077530;triangle[18][16]=3.221562;triangle[18][17]=38.061621;triangle[18][18]=-11.958617;triangle[18][19]=15.447095;triangle[18][20]=-13.899359;triangle[18][21]=16.429439;triangle[18][22]=-10.360273;triangle[18][23]=20.396606;triangle[18][24]=-8.822976;triangle[18][25]=22.092253;triangle[18][26]=-8.286954;triangle[18][27]=22.683488;triangle[18][28]=-8.100055;triangle[18][29]=22.889639;triangle[19][0]=-4.900636;triangle[19][1]=17.764806;triangle[19][2]=-4.216117;triangle[19][3]=9.388961;triangle[19][4]=8.369251;triangle[19][5]=-10.738762;triangle[19][6]=-13.110785;triangle[19][7]=17.804140;triangle[19][8]=-36.407384;triangle[19][9]=32.385901;triangle[19][10]=-24.555031;triangle[19][11]=28.399772;triangle[19][12]=17.085887;triangle[19][13]=32.210592;triangle[19][14]=23.564877;triangle[19][15]=46.033349;triangle[19][16]=2.424182;triangle[19][17]=27.450139;triangle[19][18]=-15.402099;triangle[19][19]=11.957972;triangle[19][20]=-13.707383;triangle[19][21]=15.472130;triangle[19][22]=-11.192685;triangle[19][23]=19.482061;triangle[19][24]=-9.113220;triangle[19][25]=21.122049;triangle[19][26]=-8.388156;triangle[19][27]=22.215580;triangle[19][28]=-7.791442;triangle[19][29]=23.296023;triangle[20][0]=-4.900636;triangle[20][1]=17.420906;triangle[20][2]=-4.977997;triangle[20][3]=24.840788;triangle[20][4]=5.223910;triangle[20][5]=15.032287;triangle[20][6]=-16.956918;triangle[20][7]=26.932292;triangle[20][8]=-36.183347;triangle[20][9]=35.272336;triangle[20][10]=-45.172983;triangle[20][11]=25.004440;triangle[20][12]=-5.649666;triangle[20][13]=26.604624;triangle[20][14]=2.396716;triangle[20][15]=35.998769;triangle[20][16]=-4.341360;triangle[20][17]=20.700194;triangle[20][18]=-13.940578;triangle[20][19]=6.798626;triangle[20][20]=-12.517277;triangle[20][21]=12.575037;triangle[20][22]=-10.012117;triangle[20][23]=18.030893;triangle[20][24]=-8.701582;triangle[20][25]=20.806058;triangle[20][26]=-8.244626;triangle[20][27]=22.235020;triangle[20][28]=-8.085296;triangle[20][29]=23.254971;triangle[21][0]=-5.513216;triangle[21][1]=17.196247;triangle[21][2]=-2.131425;triangle[21][3]=12.349633;triangle[21][4]=6.248511;triangle[21][5]=5.752436;triangle[21][6]=-19.348438;triangle[21][7]=40.876763;triangle[21][8]=-43.618927;triangle[21][9]=39.768082;triangle[21][10]=-37.243259;triangle[21][11]=28.601528;triangle[21][12]=-1.137708;triangle[21][13]=30.533930;triangle[21][14]=17.068878;triangle[21][15]=46.145465;triangle[21][16]=1.440493;triangle[21][17]=22.579043;triangle[21][18]=-9.884854;triangle[21][19]=9.703209;triangle[21][20]=-9.519937;triangle[21][21]=14.685943;triangle[21][22]=-9.181291;triangle[21][23]=19.449746;triangle[21][24]=-9.063212;triangle[21][25]=21.210781;triangle[21][26]=-8.612531;triangle[21][27]=22.376138;triangle[21][28]=-8.213576;triangle[21][29]=22.782473;triangle[22][0]=-4.900636;triangle[22][1]=16.808326;triangle[22][2]=-6.919319;triangle[22][3]=23.446383;triangle[22][4]=-7.623190;triangle[22][5]=25.570930;triangle[22][6]=-7.868614;triangle[22][7]=25.850393;triangle[22][8]=-3.794617;triangle[22][9]=18.562050;triangle[22][10]=0.102065;triangle[22][11]=11.209945;triangle[22][12]=-15.322680;triangle[22][13]=29.432609;triangle[22][14]=-31.634315;triangle[22][15]=26.397309;triangle[22][16]=-1.171754;triangle[22][17]=30.290016;triangle[22][18]=12.597279;triangle[22][19]=47.758800;triangle[22][20]=0.259907;triangle[22][21]=32.061559;triangle[22][22]=-11.521658;triangle[22][23]=15.087383;triangle[22][24]=-10.212160;triangle[22][25]=17.846606;triangle[22][26]=-8.771332;triangle[22][27]=20.470012;triangle[22][28]=-8.268947;triangle[22][29]=21.466526;triangle[23][0]=-5.556262;triangle[23][1]=18.033485;triangle[23][2]=-6.940684;triangle[23][3]=23.919733;triangle[23][4]=4.054707;triangle[23][5]=7.260134;triangle[23][6]=0.004444;triangle[23][7]=11.362452;triangle[23][8]=-18.883910;triangle[23][9]=27.263815;triangle[23][10]=-34.205640;triangle[23][11]=25.267186;triangle[23][12]=-4.850126;triangle[23][13]=16.528584;triangle[23][14]=12.486379;triangle[23][15]=33.277253;triangle[23][16]=13.949563;triangle[23][17]=41.838463;triangle[23][18]=-8.303883;triangle[23][19]=16.947150;triangle[23][20]=-9.161002;triangle[23][21]=15.984357;triangle[23][22]=-8.404817;triangle[23][23]=20.303647;triangle[23][24]=-8.141151;triangle[23][25]=22.059840;triangle[23][26]=-8.049216;triangle[23][27]=23.241719;triangle[23][28]=-8.017161;triangle[23][29]=23.735604;triangle[24][0]=-5.700283;triangle[24][1]=16.952347;triangle[24][2]=-7.849460;triangle[24][3]=23.496600;triangle[24][4]=-1.575224;triangle[24][5]=19.879198;triangle[24][6]=3.101893;triangle[24][7]=9.737546;triangle[24][8]=-15.759203;triangle[24][9]=38.146761;triangle[24][10]=-35.260347;triangle[24][11]=36.714578;triangle[24][12]=-16.472144;triangle[24][13]=25.474505;triangle[24][14]=9.167181;triangle[24][15]=25.060312;triangle[24][16]=11.409354;triangle[24][17]=41.624262;triangle[24][18]=-5.587972;triangle[24][19]=32.698326;triangle[24][20]=-13.322714;triangle[24][21]=18.024510;triangle[24][22]=-10.927952;triangle[24][23]=19.154237;triangle[24][24]=-9.020914;triangle[24][25]=20.878125;triangle[24][26]=-8.355971;triangle[24][27]=22.221406;triangle[24][28]=-8.124119;triangle[24][29]=22.728521;triangle[25][0]=-6.168842;triangle[25][1]=17.243103;triangle[25][2]=-8.051578;triangle[25][3]=24.249302;triangle[25][4]=4.090709;triangle[25][5]=13.230980;triangle[25][6]=-5.028477;triangle[25][7]=19.799385;triangle[25][8]=-33.219010;triangle[25][9]=41.887787;triangle[25][10]=-50.603603;triangle[25][11]=35.535771;triangle[25][12]=-8.777358;triangle[25][13]=29.473123;triangle[25][14]=3.343928;triangle[25][15]=35.492343;triangle[25][16]=-2.138574;triangle[25][17]=41.577840;triangle[25][18]=-17.703390;triangle[25][19]=16.170156;triangle[25][20]=-17.135611;triangle[25][21]=14.200374;triangle[25][22]=-12.086862;triangle[25][23]=17.927526;triangle[25][24]=-10.076322;triangle[25][25]=19.928694;triangle[25][26]=-8.965780;triangle[25][27]=21.277780;triangle[25][28]=-8.146747;triangle[25][29]=21.748178;triangle[26][0]=-4.900636;triangle[26][1]=17.243103;triangle[26][2]=0.981109;triangle[26][3]=5.343951;triangle[26][4]=-1.125669;triangle[26][5]=3.836630;triangle[26][6]=-24.818587;triangle[26][7]=35.184937;triangle[26][8]=-50.397390;triangle[26][9]=31.757721;triangle[26][10]=-9.585741;triangle[26][11]=23.471310;triangle[26][12]=13.242527;triangle[26][13]=40.124947;triangle[26][14]=3.141045;triangle[26][15]=29.630873;triangle[26][16]=-11.833098;triangle[26][17]=13.105896;triangle[26][18]=-10.597223;triangle[26][19]=16.062708;triangle[26][20]=-8.905596;triangle[26][21]=19.891052;triangle[26][22]=-8.315762;triangle[26][23]=21.915977;triangle[26][24]=-8.110099;triangle[26][25]=22.622025;triangle[26][26]=-8.038389;triangle[26][27]=23.519530;triangle[26][28]=-8.013386;triangle[26][29]=23.832470;triangle[27][0]=-6.125795;triangle[27][1]=17.152226;triangle[27][2]=-5.312136;triangle[27][3]=20.101012;triangle[27][4]=10.355836;triangle[27][5]=1.673467;triangle[27][6]=-13.798050;triangle[27][7]=21.301809;triangle[27][8]=-43.402216;triangle[27][9]=35.031163;triangle[27][10]=-27.407235;triangle[27][11]=24.474892;triangle[27][12]=8.028668;triangle[27][13]=37.280463;triangle[27][14]=12.181677;triangle[27][15]=48.425871;triangle[27][16]=-6.850810;triangle[27][17]=23.954341;triangle[27][18]=-9.956990;triangle[27][19]=19.917436;triangle[27][20]=-8.151570;triangle[27][21]=21.488136;triangle[27][22]=-8.052849;triangle[27][23]=23.124167;triangle[27][24]=-8.018427;triangle[27][25]=23.694616;triangle[27][26]=-8.006425;triangle[27][27]=23.893519;triangle[27][28]=-7.480537;triangle[27][29]=24.484576;triangle[28][0]=-4.900636;triangle[28][1]=16.808326;triangle[28][2]=-6.919319;triangle[28][3]=23.446383;triangle[28][4]=2.235962;triangle[28][5]=15.597694;triangle[28][6]=2.292142;triangle[28][7]=12.558368;triangle[28][8]=-26.133279;triangle[28][9]=44.371277;triangle[28][10]=-43.132893;triangle[28][11]=40.449009;triangle[28][12]=-5.586719;triangle[28][13]=31.550042;triangle[28][14]=11.198894;triangle[28][15]=37.208328;triangle[28][16]=1.942374;triangle[28][17]=29.820824;triangle[28][18]=-9.093588;triangle[28][19]=15.748808;triangle[28][20]=-9.442804;triangle[28][21]=17.989319;triangle[28][22]=-9.154396;triangle[28][23]=20.418799;triangle[28][24]=-9.053835;triangle[28][25]=21.448669;triangle[28][26]=-9.018771;triangle[28][27]=21.807763;triangle[28][28]=-8.537986;triangle[28][29]=21.932971;triangle[29][0]=-5.604092;triangle[29][1]=18.865081;triangle[29][2]=-6.975387;triangle[29][3]=25.556172;triangle[29][4]=8.043245;triangle[29][5]=11.623890;triangle[29][6]=-2.073450;triangle[29][7]=5.165438;triangle[29][8]=-23.734616;triangle[29][9]=30.096044;triangle[29][10]=-41.765061;triangle[29][11]=27.055436;triangle[29][12]=-0.406804;triangle[29][13]=26.192145;triangle[29][14]=21.842633;triangle[29][15]=40.648388;triangle[29][16]=18.518320;triangle[29][17]=51.261923;triangle[29][18]=-3.935946;triangle[29][19]=17.037088;triangle[29][20]=-9.599830;triangle[29][21]=14.305884;triangle[29][22]=-9.247890;triangle[29][23]=17.993177;triangle[29][24]=-9.086434;triangle[29][25]=20.521119;triangle[29][26]=-9.030138;triangle[29][27]=21.484346;triangle[29][28]=-9.010508;triangle[29][29]=22.091203;triangle[30][0]=-5.647139;triangle[30][1]=16.268647;triangle[30][2]=0.658903;triangle[30][3]=15.449871;triangle[30][4]=10.484359;triangle[30][5]=7.370741;triangle[30][6]=-22.684475;triangle[30][7]=32.846873;triangle[30][8]=-56.481777;triangle[30][9]=32.553086;triangle[30][10]=-9.944996;triangle[30][11]=26.851695;triangle[30][12]=14.257856;triangle[30][13]=37.098550;triangle[30][14]=2.461775;triangle[30][15]=31.276948;triangle[30][16]=-13.305916;triangle[30][17]=6.330921;triangle[30][18]=-12.539450;triangle[30][19]=11.435603;triangle[30][20]=-10.885451;triangle[30][21]=17.543596;triangle[30][22]=-9.899228;triangle[30][23]=20.446148;triangle[30][24]=-9.313541;triangle[30][25]=21.458205;triangle[30][26]=-9.109325;triangle[30][27]=21.811088;triangle[30][28]=-9.038119;triangle[30][29]=22.034130;triangle[31][0]=-5.824942;triangle[31][1]=17.336973;triangle[31][2]=-6.012367;triangle[31][3]=18.484301;triangle[31][4]=7.051802;triangle[31][5]=0.691029;triangle[31][6]=-19.130282;triangle[31][7]=37.636716;triangle[31][8]=-54.244169;triangle[31][9]=44.010710;triangle[31][10]=-29.775108;triangle[31][11]=35.372668;triangle[31][12]=11.592882;triangle[31][13]=39.132147;triangle[31][14]=9.043044;triangle[31][15]=33.788074;triangle[31][16]=-10.501693;triangle[31][17]=7.574724;triangle[31][18]=-11.518396;triangle[31][19]=11.830549;triangle[31][20]=-9.878110;triangle[31][21]=17.451687;triangle[31][22]=-9.035178;triangle[31][23]=20.332313;triangle[31][24]=-8.360944;triangle[31][25]=21.418513;triangle[31][26]=-8.125854;triangle[31][27]=22.206758;triangle[31][28]=-8.043882;triangle[31][29]=22.723414;triangle[32][0]=-5.657236;triangle[32][1]=17.355296;triangle[32][2]=5.143314;triangle[32][3]=12.659142;triangle[32][4]=2.706452;triangle[32][5]=8.292370;triangle[32][6]=-22.224507;triangle[32][7]=31.718424;triangle[32][8]=-50.028390;triangle[32][9]=32.163381;triangle[32][10]=-11.313337;triangle[32][11]=30.028618;triangle[32][12]=7.497914;triangle[32][13]=35.368302;triangle[32][14]=13.888021;triangle[32][15]=41.527133;triangle[32][16]=-3.012361;triangle[32][17]=17.617931;triangle[32][18]=-9.577349;triangle[32][19]=11.523918;triangle[32][20]=-9.240051;triangle[32][21]=16.421340;triangle[32][22]=-9.083701;triangle[32][23]=20.016099;triangle[32][24]=-8.377863;triangle[32][25]=21.308257;triangle[32][26]=-8.131753;triangle[32][27]=21.758804;triangle[32][28]=-8.045939;triangle[32][29]=22.567222;triangle[33][0]=-6.168842;triangle[33][1]=17.677875;triangle[33][2]=0.362515;triangle[33][3]=15.779509;triangle[33][4]=6.292453;triangle[33][5]=5.718600;triangle[33][6]=-19.953692;triangle[33][7]=30.169784;triangle[33][8]=-48.249014;triangle[33][9]=27.664719;triangle[33][10]=-18.349187;triangle[33][11]=24.613647;triangle[33][12]=11.633966;triangle[33][13]=35.722454;triangle[33][14]=10.262205;triangle[33][15]=36.646664;triangle[33][16]=-8.704972;triangle[33][17]=13.611795;triangle[33][18]=-10.782056;triangle[33][19]=14.625609;triangle[33][20]=-9.008785;triangle[33][21]=18.922346;triangle[33][22]=-8.351742;triangle[33][23]=20.926888;triangle[33][24]=-8.122645;triangle[33][25]=21.625829;triangle[33][26]=-8.042764;triangle[33][27]=21.969535;triangle[33][28]=-8.014911;triangle[33][29]=22.640699;triangle[34][0]=-5.513216;triangle[34][1]=18.197262;triangle[34][2]=-7.973623;triangle[34][3]=14.699949;triangle[34][4]=2.915599;triangle[34][5]=2.342350;triangle[34][6]=-19.664814;triangle[34][7]=25.906923;triangle[34][8]=-41.795746;triangle[34][9]=36.703126;triangle[34][10]=-32.088002;triangle[34][11]=33.309582;triangle[34][12]=11.608300;triangle[34][13]=35.120962;triangle[34][14]=21.630309;triangle[34][15]=45.340070;triangle[34][16]=-6.977222;triangle[34][17]=12.519579;triangle[34][18]=-14.359330;triangle[34][19]=9.714493;triangle[34][20]=-11.692347;triangle[34][21]=16.265605;triangle[34][22]=-9.938763;triangle[34][23]=20.000540;triangle[34][24]=-8.676005;triangle[34][25]=21.302831;triangle[34][26]=-8.235708;triangle[34][27]=21.756912;triangle[34][28]=-8.082186;triangle[34][29]=22.259141;triangle[35][0]=-5.103706;triangle[35][1]=16.881226;triangle[35][2]=-4.234620;triangle[35][3]=19.325278;triangle[35][4]=1.828061;triangle[35][5]=11.098527;triangle[35][6]=-21.567750;triangle[35][7]=43.600338;triangle[35][8]=-44.993898;triangle[35][9]=48.229359;triangle[35][10]=-25.396158;triangle[35][11]=33.591013;triangle[35][12]=9.036038;triangle[35][13]=31.655585;triangle[35][14]=21.901438;triangle[35][15]=45.260959;triangle[35][16]=2.836230;triangle[35][17]=20.032328;triangle[35][18]=-8.779456;triangle[35][19]=11.700131;triangle[35][20]=-9.036395;triangle[35][21]=16.431291;triangle[35][22]=-8.171368;triangle[35][23]=20.589102;triangle[35][24]=-7.408431;triangle[35][25]=22.159372;triangle[35][26]=-7.142411;triangle[35][27]=22.706891;triangle[35][28]=-7.049656;triangle[35][29]=22.897799;triangle[36][0]=-6.125795;triangle[36][1]=15.314488;triangle[36][2]=-7.580285;triangle[36][3]=9.347812;triangle[36][4]=6.316851;triangle[36][5]=-5.120009;triangle[36][6]=-16.899042;triangle[36][7]=26.047158;triangle[36][8]=-38.899216;triangle[36][9]=33.589728;triangle[36][10]=-29.394502;triangle[36][11]=12.986768;triangle[36][12]=8.611930;triangle[36][13]=17.139560;triangle[36][14]=17.148095;triangle[36][15]=35.461440;triangle[36][16]=-1.083499;triangle[36][17]=12.637950;triangle[36][18]=-8.714797;triangle[36][19]=8.033509;triangle[36][20]=-8.982344;triangle[36][21]=14.996888;triangle[36][22]=-8.425285;triangle[36][23]=19.658166;triangle[36][24]=-6.453560;triangle[36][25]=22.878181;triangle[36][26]=-5.194245;triangle[36][27]=24.772747;triangle[36][28]=-5.949877;triangle[36][29]=24.920762;triangle[37][0]=-4.900636;triangle[37][1]=15.314488;triangle[37][2]=-0.874709;triangle[37][3]=14.353215;triangle[37][4]=5.545542;triangle[37][5]=-1.705166;triangle[37][6]=-17.002365;triangle[37][7]=26.459912;triangle[37][8]=-39.388356;triangle[37][9]=33.127189;triangle[37][10]=-32.442079;triangle[37][11]=26.385665;triangle[37][12]=8.680918;triangle[37][13]=30.233664;triangle[37][14]=20.262845;triangle[37][15]=48.028580;triangle[37][16]=0.606484;triangle[37][17]=22.259526;triangle[37][18]=-8.336067;triangle[37][19]=13.480964;triangle[37][20]=-8.898119;triangle[37][21]=17.298027;triangle[37][22]=-8.964476;triangle[37][23]=20.177761;triangle[37][24]=-8.987614;triangle[37][25]=21.364624;triangle[37][26]=-8.995681;triangle[37][27]=21.778458;triangle[37][28]=-8.998494;triangle[37][29]=21.922753;triangle[38][0]=-6.025795;triangle[38][1]=17.243103;triangle[38][2]=-7.772959;triangle[38][3]=24.339302;triangle[38][4]=6.620269;triangle[38][5]=10.280003;triangle[38][6]=2.210333;triangle[38][7]=17.400081;triangle[38][8]=-14.779512;triangle[38][9]=41.057854;triangle[38][10]=-44.068505;triangle[38][11]=37.139906;triangle[38][12]=-17.539489;triangle[38][13]=35.395661;triangle[38][14]=10.809800;triangle[38][15]=48.248311;triangle[38][16]=7.581216;triangle[38][17]=43.701396;triangle[38][18]=-8.027125;triangle[38][19]=14.712689;triangle[38][20]=-9.152740;triangle[38][21]=15.713708;triangle[38][22]=-8.863257;triangle[38][23]=18.988423;triangle[38][24]=-8.300999;triangle[38][25]=20.911186;triangle[38][26]=-8.104952;triangle[38][27]=22.232934;triangle[38][28]=-7.536594;triangle[38][29]=22.732540;triangle[39][0]=-5.513216;triangle[39][1]=15.927067;triangle[39][2]=-3.661632;triangle[39][3]=19.156377;triangle[39][4]=8.286520;triangle[39][5]=5.238058;triangle[39][6]=-12.213496;triangle[39][7]=29.758420;triangle[39][8]=-37.589150;triangle[39][9]=37.940978;triangle[39][10]=-39.465568;triangle[39][11]=31.365022;triangle[39][12]=0.363238;triangle[39][13]=26.365948;triangle[39][14]=16.839965;triangle[39][15]=42.163198;triangle[39][16]=2.226142;triangle[39][17]=27.769386;triangle[39][18]=-10.262237;triangle[39][19]=16.194970;triangle[39][20]=-10.214606;triangle[39][21]=18.410926;triangle[39][22]=-9.079607;triangle[39][23]=20.097246;triangle[39][24]=-8.376436;triangle[39][25]=21.396932;triangle[39][26]=-8.131255;triangle[39][27]=22.441045;triangle[39][28]=-8.045766;triangle[39][29]=22.805104;
  
}
void fill_matrix_S(){
  
  S[0][0]=-6.168842;S[0][1]=14.701908;S[0][2]=-8.064158;S[0][3]=21.357956;S[0][4]=-0.257741;S[0][5]=24.322259;S[0][6]=3.148116;S[0][7]=11.218286;S[0][8]=-43.495832;S[0][9]=5.337127;S[0][10]=-64.811608;S[0][11]=11.870177;S[0][12]=-16.373139;S[0][13]=16.877263;S[0][14]=21.037481;S[0][15]=37.645031;S[0][16]=-14.281195;S[0][17]=37.384306;S[0][18]=-17.596132;S[0][19]=34.184813;S[0][20]=-12.447619;S[0][21]=32.761797;S[0][22]=-8.899467;S[0][23]=32.265622;S[0][24]=-7.662303;S[0][25]=32.092617;S[0][26]=-7.230931;S[0][27]=32.032293;S[0][28]=-7.080521;S[0][29]=32.011260;S[1][0]=-6.394475;S[1][1]=15.043488;S[1][2]=-8.081831;S[1][3]=19.993478;S[1][4]=6.707369;S[1][5]=18.312180;S[1][6]=-2.452676;S[1][7]=4.057403;S[1][8]=-44.025148;S[1][9]=7.894515;S[1][10]=-21.957460;S[1][11]=14.128371;S[1][12]=9.793456;S[1][13]=21.593542;S[1][14]=-6.391489;S[1][15]=36.471412;S[1][16]=-17.389740;S[1][17]=37.275580;S[1][18]=-13.631688;S[1][19]=32.757799;S[1][20]=-11.076291;S[1][21]=30.210263;S[1][22]=-9.723958;S[1][23]=28.770671;S[1][24]=-9.252429;S[1][25]=28.078716;S[1][26]=-9.088016;S[1][27]=26.854422;S[1][28]=-9.030689;S[1][29]=26.297919;S[2][0]=-6.581542;S[2][1]=15.549385;S[2][2]=-7.625945;S[2][3]=20.821195;S[2][4]=11.378887;S[2][5]=22.274970;S[2][6]=5.237573;S[2][7]=14.756511;S[2][8]=-46.636590;S[2][9]=10.138426;S[2][10]=-34.851399;S[2][11]=19.457817;S[2][12]=8.167642;S[2][13]=29.426732;S[2][14]=16.999262;S[2][15]=41.230923;S[2][16]=-13.471598;S[2][17]=37.805141;S[2][18]=-14.426903;S[2][19]=32.190694;S[2][20]=-11.113749;S[2][21]=29.542993;S[2][22]=-9.737019;S[2][23]=27.925429;S[2][24]=-8.788423;S[2][25]=27.322677;S[2][26]=-8.274906;S[2][27]=27.112511;S[2][28]=-8.095854;S[2][29]=27.039230;S[3][0]=-4.900636;S[3][1]=14.701908;S[3][2]=-6.919319;S[3][3]=20.096956;S[3][4]=7.470432;S[3][5]=20.938991;S[3][6]=-6.152394;S[3][7]=12.150068;S[3][8]=-45.339775;S[3][9]=7.476408;S[3][10]=-49.473183;S[3][11]=7.175373;S[3][12]=-1.025976;S[3][13]=8.893289;S[3][14]=4.273299;S[3][15]=19.312006;S[3][16]=-22.968943;S[3][17]=27.670596;S[3][18]=-19.154358;S[3][19]=28.346466;S[3][20]=-13.070288;S[3][21]=27.930805;S[3][22]=-9.767900;S[3][23]=27.224552;S[3][24]=-8.616429;S[3][25]=25.965653;S[3][26]=-8.214935;S[3][27]=25.988024;S[3][28]=-8.074943;S[3][29]=25.995824;S[4][0]=-5.784216;S[4][1]=15.143488;S[4][2]=-2.717663;S[4][3]=22.691969;S[4][4]=10.685139;S[4][5]=15.084989;S[4][6]=-16.713988;S[4][7]=7.313026;S[4][8]=-50.528016;S[4][9]=7.549546;S[4][10]=-14.094658;S[4][11]=7.994666;S[4][12]=18.982063;S[4][13]=10.482112;S[4][14]=-8.132900;S[4][15]=24.562839;S[4][16]=-15.785042;S[4][17]=29.216496;S[4][18]=-12.384690;S[4][19]=29.726809;S[4][20]=-9.528847;S[4][21]=29.904744;S[4][22]=-8.533076;S[4][23]=29.966786;S[4][24]=-8.185872;S[4][25]=29.988419;S[4][26]=-8.064810;S[4][27]=29.995962;S[4][28]=-8.022598;S[4][29]=29.727592;S[5][0]=-6.448375;S[5][1]=15.612047;S[5][2]=-1.207141;S[5][3]=18.685945;S[5][4]=8.543267;S[5][5]=10.650023;S[5][6]=-30.871032;S[5][7]=2.826727;S[5][8]=-31.572845;S[5][9]=9.603037;S[5][10]=1.148558;S[5][11]=22.830545;S[5][12]=6.998812;S[5][13]=35.496124;S[5][14]=-14.402540;S[5][15]=32.916960;S[5][16]=-13.548859;S[5][17]=29.882798;S[5][18]=-10.909689;S[5][19]=28.656491;S[5][20]=-9.665868;S[5][21]=28.228904;S[5][22]=-8.710471;S[5][23]=28.079814;S[5][24]=-7.779167;S[5][25]=28.027829;S[5][26]=-7.271679;S[5][27]=28.009704;S[5][28]=-7.094728;S[5][29]=28.003383;S[6][0]=-5.657236;S[6][1]=18.528965;S[6][2]=-7.834450;S[6][3]=25.348976;S[6][4]=-7.715529;S[6][5]=25.895383;S[6][6]=11.258469;S[6][7]=28.663756;S[6][8]=7.458394;S[6][9]=22.056245;S[6][10]=-27.568431;S[6][11]=18.463500;S[6][12]=-36.502264;S[6][13]=11.200306;S[6][14]=3.877572;S[6][15]=18.649073;S[6][16]=5.570414;S[6][17]=36.514511;S[6][18]=-11.626085;S[6][19]=36.359753;S[6][20]=-13.042091;S[6][21]=31.496392;S[6][22]=-8.125886;S[6][23]=27.295002;S[6][24]=-7.392572;S[6][25]=25.800218;S[6][26]=-7.136881;S[6][27]=25.279019;S[6][28]=-7.047728;S[6][29]=25.097288;S[7][0]=-7.079954;S[7][1]=14.358008;S[7][2]=-8.781843;S[7][3]=19.927917;S[7][4]=12.679995;S[7][5]=22.593944;S[7][6]=-0.804401;S[7][7]=13.829366;S[7][8]=-42.352355;S[7][9]=8.561852;S[7][10]=-32.016220;S[7][11]=8.118603;S[7][12]=15.003138;S[7][13]=16.230126;S[7][14]=9.174422;S[7][15]=27.099791;S[7][16]=-4.715501;S[7][17]=28.697460;S[7][18]=-7.542509;S[7][19]=28.243189;S[7][20]=-7.840483;S[7][21]=28.084795;S[7][22]=-7.944380;S[7][23]=28.029566;S[7][24]=-7.980606;S[7][25]=27.358988;S[7][26]=-7.993238;S[7][27]=27.125171;S[7][28]=-7.997642;S[7][29]=27.043645;S[8][0]=-5.754795;S[8][1]=19.033012;S[8][2]=-6.556145;S[8][3]=26.095048;S[8][4]=4.751014;S[8][5]=25.518063;S[8][6]=-7.613534;S[8][7]=11.648798;S[8][8]=-56.632331;S[8][9]=17.133995;S[8][10]=-26.784454;S[8][11]=20.387221;S[8][12]=16.342691;S[8][13]=20.380992;S[8][14]=-0.334864;S[8][15]=21.658583;S[8][16]=-16.174613;S[8][17]=22.461476;S[8][18]=-13.940321;S[8][19]=24.114871;S[8][20]=-11.068747;S[8][21]=24.691375;S[8][22]=-9.531327;S[8][23]=24.892389;S[8][24]=-8.533941;S[8][25]=24.962478;S[8][26]=-6.621064;S[8][27]=25.508620;S[8][28]=-5.565230;S[8][29]=25.828666;S[9][0]=-6.841444;S[9][1]=14.192398;S[9][2]=-8.898680;S[9][3]=19.006658;S[9][4]=-6.124168;S[9][5]=19.410451;S[9][6]=8.502895;S[9][7]=9.777021;S[9][8]=-32.995417;S[9][9]=9.281031;S[9][10]=-34.110881;S[9][11]=9.291921;S[9][12]=4.798964;S[9][13]=14.914387;S[9][14]=-1.315242;S[9][15]=20.644173;S[9][16]=-15.750492;S[9][17]=23.647457;S[9][18]=-13.099178;S[9][19]=24.528397;S[9][20]=-10.329295;S[9][21]=24.835562;S[9][22]=-8.812175;S[9][23]=24.942664;S[9][24]=-8.283188;S[9][25]=24.980008;S[9][26]=-8.098742;S[9][27]=24.993029;S[9][28]=-8.034429;S[9][29]=24.997569;S[10][0]=-6.394475;S[10][1]=15.167534;S[10][2]=-7.385498;S[10][3]=21.264210;S[10][4]=-0.533143;S[10][5]=22.101395;S[10][6]=0.534006;S[10][7]=9.184789;S[10][8]=-49.872153;S[10][9]=7.593643;S[10][10]=-38.317859;S[10][11]=2.205548;S[10][12]=8.976506;S[10][13]=16.128936;S[10][14]=3.400439;S[10][15]=36.008790;S[10][16]=-13.021067;S[10][17]=35.000286;S[10][18]=-12.541806;S[10][19]=31.130912;S[10][20]=-9.622372;S[10][21]=28.479102;S[10][22]=-8.565686;S[10][23]=27.515731;S[10][24]=-8.197243;S[10][25]=26.658121;S[10][26]=-8.068774;S[10][27]=26.229473;S[10][28]=-8.023980;S[10][29]=26.080012;S[11][0]=-6.738375;S[11][1]=14.744955;S[11][2]=-8.844530;S[11][3]=19.599941;S[11][4]=9.838362;S[11][5]=22.640374;S[11][6]=7.900943;S[11][7]=4.786289;S[11][8]=-35.533005;S[11][9]=3.598784;S[11][10]=-22.995261;S[11][11]=19.184995;S[11][12]=26.329981;S[11][13]=28.141431;S[11][14]=-1.498131;S[11][15]=30.746576;S[11][16]=-12.304869;S[11][17]=28.306350;S[11][18]=-12.220025;S[11][19]=24.850210;S[11][20]=-9.851753;S[11][21]=23.645128;S[11][22]=-8.455666;S[11][23]=23.224942;S[11][24]=-7.851459;S[11][25]=23.422333;S[11][26]=-7.948207;S[11][27]=23.798580;S[11][28]=-7.981941;S[11][29]=24.339279;S[12][0]=-4.943683;S[12][1]=15.970114;S[12][2]=-6.934328;S[12][3]=22.502795;S[12][4]=-6.750354;S[12][5]=25.149159;S[12][6]=12.176315;S[12][7]=26.727969;S[12][8]=-1.370203;S[12][9]=19.534734;S[12][10]=-49.044247;S[12][11]=17.508740;S[12][12]=-25.308919;S[12][13]=26.441868;S[12][14]=19.642537;S[12][15]=38.310763;S[12][16]=-10.505057;S[12][17]=37.109875;S[12][18]=-21.722212;S[12][19]=31.725650;S[12][20]=-16.652392;S[12][21]=29.299054;S[12][22]=-11.836585;S[12][23]=27.984393;S[12][24]=-9.989056;S[12][25]=27.072237;S[12][26]=-9.344862;S[12][27]=26.373866;S[12][28]=-9.120246;S[12][29]=26.130359;S[13][0]=-7.635580;S[13][1]=13.842375;S[13][2]=-9.826899;S[13][3]=18.694291;S[13][4]=-2.438864;S[13][5]=24.240822;S[13][6]=6.151631;S[13][7]=16.114494;S[13][8]=-32.593930;S[13][9]=11.592699;S[13][10]=-42.377662;S[13][11]=13.944602;S[13][12]=0.203174;S[13][13]=13.259326;S[13][14]=22.575090;S[13][15]=27.310767;S[13][16]=-14.611133;S[13][17]=24.236334;S[13][18]=-16.057408;S[13][19]=23.922382;S[13][20]=-11.764059;S[13][21]=23.972936;S[13][22]=-9.619868;S[13][23]=23.990563;S[13][24]=-8.564813;S[13][25]=23.996710;S[13][26]=-7.675235;S[13][27]=24.520556;S[13][28]=-7.235440;S[13][29]=24.832828;S[14][0]=-5.657236;S[14][1]=16.539647;S[14][2]=-7.183129;S[14][3]=22.701379;S[14][4]=-7.715175;S[14][5]=24.578842;S[14][6]=-7.341155;S[14][7]=24.378784;S[14][8]=1.885210;S[14][9]=19.795983;S[14][10]=-17.085820;S[14][11]=10.179673;S[14][12]=-48.162975;S[14][13]=16.852674;S[14][14]=-23.393558;S[14][15]=14.196137;S[14][16]=12.863390;S[14][17]=27.748945;S[14][18]=-13.754838;S[14][19]=29.408985;S[14][20]=-17.412602;S[14][21]=27.188640;S[14][22]=-13.268230;S[14][23]=26.414453;S[14][24]=-10.949561;S[14][25]=25.531931;S[14][26]=-9.158067;S[14][27]=25.185473;S[14][28]=-8.403793;S[14][29]=25.474180;S[15][0]=-7.350954;S[15][1]=14.223252;S[15][2]=-8.905519;S[15][3]=19.639737;S[15][4]=6.965286;S[15][5]=18.537566;S[15][6]=-6.929651;S[15][7]=13.458271;S[15][8]=-41.681385;S[15][9]=18.695493;S[15][10]=0.852816;S[15][11]=20.946876;S[15][12]=17.070395;S[15][13]=24.166151;S[15][14]=-3.671511;S[15][15]=24.562140;S[15][16]=-8.073946;S[15][17]=25.498649;S[15][18]=-9.228426;S[15][19]=25.825190;S[15][20]=-9.079647;S[15][21]=25.939047;S[15][22]=-8.683871;S[15][23]=25.978747;S[15][24]=-8.138451;S[15][25]=25.992590;S[15][26]=-7.396953;S[15][27]=25.427883;S[15][28]=-7.138409;S[15][29]=25.149194;S[16][0]=-6.169816;S[16][1]=17.197123;S[16][2]=-8.013176;S[16][3]=21.974147;S[16][4]=0.796520;S[16][5]=23.666764;S[16][6]=12.717217;S[16][7]=13.002665;S[16][8]=-32.101313;S[16][9]=10.777720;S[16][10]=-52.398515;S[16][11]=7.209379;S[16][12]=-5.826432;S[16][13]=12.608470;S[16][14]=2.064998;S[16][15]=24.824120;S[16][16]=-18.633202;S[16][17]=25.043080;S[16][18]=-16.036643;S[16][19]=24.709863;S[16][20]=-12.287610;S[16][21]=24.247514;S[16][22]=-10.329081;S[16][23]=24.086303;S[16][24]=-9.463422;S[16][25]=24.030092;S[16][26]=-9.161585;S[16][27]=24.010492;S[16][28]=-8.587782;S[16][29]=24.003658;S[17][0]=-7.350954;S[17][1]=12.294637;S[17][2]=-8.496656;S[17][3]=16.700727;S[17][4]=6.750710;S[17][5]=12.004181;S[17][6]=-18.011671;S[17][7]=1.564422;S[17][8]=-46.387994;S[17][9]=7.440053;S[17][10]=-17.874257;S[17][11]=11.079415;S[17][12]=1.711893;S[17][13]=20.751457;S[17][14]=-17.153462;S[17][15]=22.218302;S[17][16]=-14.489046;S[17][17]=23.378760;S[17][18]=-10.913912;S[17][19]=23.783387;S[17][20]=-9.323440;S[17][21]=23.924472;S[17][22]=-8.461455;S[17][23]=23.973665;S[17][24]=-8.160899;S[17][25]=23.990818;S[17][26]=-8.056102;S[17][27]=23.996798;S[17][28]=-8.019562;S[17][29]=23.998884;S[18][0]=-6.168842;S[18][1]=16.673570;S[18][2]=-8.094625;S[18][3]=22.217284;S[18][4]=-8.069415;S[18][5]=24.029727;S[18][6]=-1.153018;S[18][7]=13.091711;S[18][8]=-2.116338;S[18][9]=4.832065;S[18][10]=-55.605715;S[18][11]=10.667157;S[18][12]=-28.810869;S[18][13]=13.014047;S[18][14]=13.697517;S[18][15]=27.457582;S[18][16]=-17.991825;S[18][17]=29.429795;S[18][18]=-23.110490;S[18][19]=27.765428;S[18][20]=-16.395133;S[18][21]=27.076888;S[18][22]=-11.958845;S[18][23]=26.375488;S[18][24]=-9.419106;S[18][25]=25.518345;S[18][26]=-8.494812;S[18][27]=25.180736;S[18][28]=-8.172530;S[18][29]=25.063019;S[19][0]=-6.738375;S[19][1]=14.324226;S[19][2]=-8.711360;S[19][3]=18.368971;S[19][4]=5.799864;S[19][5]=13.732829;S[19][6]=-33.905465;S[19][7]=-1.494516;S[19][8]=-59.411196;S[19][9]=1.039216;S[19][10]=-9.900385;S[19][11]=10.043066;S[19][12]=-0.790949;S[19][13]=21.136366;S[19][14]=-20.183745;S[19][15]=22.401576;S[19][16]=-16.577284;S[19][17]=22.791342;S[19][18]=-12.191269;S[19][19]=22.927246;S[19][20]=-10.112727;S[19][21]=22.974632;S[19][22]=-9.387984;S[19][23]=22.991155;S[19][24]=-8.483960;S[19][25]=22.996916;S[19][26]=-8.168746;S[19][27]=22.998925;S[19][28]=-8.058838;S[19][29]=22.999625;S[20][0]=-5.513216;S[20][1]=16.530523;S[20][2]=-7.024701;S[20][3]=22.598198;S[20][4]=-4.268423;S[20][5]=23.396816;S[20][6]=1.710162;S[20][7]=16.745246;S[20][8]=-28.377465;S[20][9]=16.869450;S[20][10]=-32.792848;S[20][11]=20.720794;S[20][12]=4.444507;S[20][13]=21.804527;S[20][14]=30.351622;S[20][15]=26.286690;S[20][16]=6.241662;S[20][17]=28.965535;S[20][18]=-3.034240;S[20][19]=27.405448;S[20][20]=-6.268546;S[20][21]=26.490049;S[20][22]=-7.396279;S[20][23]=26.170870;S[20][24]=-7.789496;S[20][25]=25.490046;S[20][26]=-7.926602;S[20][27]=25.170868;S[20][28]=-7.974408;S[20][29]=25.059578;S[21][0]=-6.548375;S[21][1]=12.907216;S[21][2]=-5.697948;S[21][3]=17.783298;S[21][4]=12.174907;S[21][5]=10.825996;S[21][6]=-7.757584;S[21][7]=1.952846;S[21][8]=-40.519589;S[21][9]=2.716143;S[21][10]=-7.345090;S[21][11]=6.408501;S[21][12]=9.165359;S[21][13]=18.989624;S[21][14]=-17.958335;S[21][15]=24.723662;S[21][16]=-17.103974;S[21][17]=25.413587;S[21][18]=-12.597533;S[21][19]=25.144209;S[21][20]=-10.496194;S[21][21]=25.050283;S[21][22]=-9.521691;S[21][23]=25.017532;S[21][24]=-8.569323;S[21][25]=25.006113;S[21][26]=-8.198511;S[21][27]=25.002132;S[21][28]=-8.069216;S[21][29]=25.000743;S[22][0]=-5.925916;S[22][1]=16.785763;S[22][2]=-7.467133;S[22][3]=23.194616;S[22][4]=3.091917;S[22][5]=20.685292;S[22][6]=-6.847353;S[22][7]=5.561283;S[22][8]=-54.399744;S[22][9]=0.869478;S[22][10]=-25.387759;S[22][11]=5.156023;S[22][12]=8.946003;S[22][13]=20.235392;S[22][14]=-18.274655;S[22][15]=23.434854;S[22][16]=-19.537798;S[22][17]=23.031093;S[22][18]=-13.414636;S[22][19]=23.010842;S[22][20]=-10.539289;S[22][21]=23.003780;S[22][22]=-9.536717;S[22][23]=23.001318;S[22][24]=-9.187142;S[22][25]=23.000460;S[22][26]=-9.065252;S[22][27]=23.000160;S[22][28]=-9.022752;S[22][29]=23.000056;S[23][0]=-6.781421;S[23][1]=14.592905;S[23][2]=-8.143851;S[23][3]=17.997773;S[23][4]=16.528760;S[23][5]=16.505677;S[23][6]=-16.978092;S[23][7]=10.626549;S[23][8]=-48.820841;S[23][9]=15.259149;S[23][10]=-9.893382;S[23][11]=21.557014;S[23][12]=1.315598;S[23][13]=28.116111;S[23][14]=-16.047346;S[23][15]=28.251570;S[23][16]=-13.452046;S[23][17]=27.336395;S[23][18]=-10.552332;S[23][19]=26.465972;S[23][20]=-9.541265;S[23][21]=26.162474;S[23][22]=-9.188727;S[23][23]=26.436184;S[23][24]=-9.065805;S[23][25]=26.052088;S[23][26]=-8.679045;S[23][27]=25.366840;S[23][28]=-8.236768;S[23][29]=25.127909;S[24][0]=-6.467375;S[24][1]=17.777875;S[24][2]=-7.737717;S[24][3]=23.594911;S[24][4]=5.242625;S[24][5]=24.029726;S[24][6]=4.667043;S[24][7]=5.021547;S[24][8]=-54.456491;S[24][9]=-1.419619;S[24][10]=-22.289531;S[24][11]=1.653161;S[24][12]=23.307259;S[24][13]=22.274873;S[24][14]=2.628621;S[24][15]=30.023184;S[24][16]=-12.192004;S[24][17]=28.174650;S[24][18]=-11.131875;S[24][19]=27.000065;S[24][20]=-9.743339;S[24][21]=26.004801;S[24][22]=-9.259186;S[24][23]=25.350352;S[24][24]=-8.819373;S[24][25]=25.393160;S[24][26]=-8.285698;S[24][27]=25.788408;S[24][28]=-8.099617;S[24][29]=25.926222;S[25][0]=-6.312862;S[25][1]=14.748145;S[25][2]=-8.614375;S[25][3]=19.778856;S[25][4]=0.073111;S[25][5]=18.023887;S[25][6]=-10.825173;S[25][7]=4.903065;S[25][8]=-49.140758;S[25][9]=8.071065;S[25][10]=-18.008790;S[25][11]=13.855048;S[25][12]=8.265521;S[25][13]=18.789634;S[25][14]=-13.871449;S[25][15]=25.660591;S[25][16]=-15.717707;S[25][17]=27.184299;S[25][18]=-12.115146;S[25][19]=27.193879;S[25][20]=-9.986184;S[25][21]=26.398069;S[25][22]=-8.692540;S[25][23]=25.487476;S[25][24]=-8.241474;S[25][25]=25.169973;S[25][26]=-8.084197;S[25][27]=25.439587;S[25][28]=-8.029358;S[25][29]=25.153275;S[26][0]=-5.513216;S[26][1]=15.927067;S[26][2]=-7.684233;S[26][3]=21.834376;S[26][4]=0.140371;S[26][5]=17.852372;S[26][6]=-8.357909;S[26][7]=7.305087;S[26][8]=-48.680313;S[26][9]=16.413736;S[26][10]=-2.535522;S[26][11]=30.157272;S[26][12]=14.531539;S[26][13]=39.221462;S[26][14]=-18.249603;S[26][15]=38.526049;S[26][16]=-18.198890;S[26][17]=33.465784;S[26][18]=-12.652560;S[26][19]=29.293222;S[26][20]=-9.622247;S[26][21]=27.508623;S[26][22]=-8.565643;S[26][23]=26.987024;S[26][24]=-8.197227;S[26][25]=25.942982;S[26][26]=-8.068769;S[26][27]=25.328797;S[26][28]=-8.023978;S[26][29]=25.114645;S[27][0]=-6.125795;S[27][1]=14.089329;S[27][2]=-6.194205;S[27][3]=19.150621;S[27][4]=9.570552;S[27][5]=16.347066;S[27][6]=-22.459052;S[27][7]=7.043369;S[27][8]=-50.346671;S[27][9]=7.755423;S[27][10]=-4.603330;S[27][11]=15.182116;S[27][12]=9.346622;S[27][13]=28.101745;S[27][14]=-11.072642;S[27][15]=28.079178;S[27][16]=-11.114340;S[27][17]=27.458075;S[27][18]=-10.388546;S[27][19]=27.159721;S[27][20]=-9.512801;S[27][21]=26.587132;S[27][22]=-9.178803;S[27][23]=26.014720;S[27][24]=-9.062345;S[27][25]=25.353811;S[27][26]=-9.021738;S[27][27]=25.123366;S[27][28]=-9.007580;S[27][29]=25.043015;S[28][0]=-5.044657;S[28][1]=16.630523;S[28][2]=-6.969535;S[28][3]=23.194387;S[28][4]=2.934750;S[28][5]=31.321578;S[28][6]=7.999336;S[28][7]=24.872870;S[28][8]=-37.868132;S[28][9]=22.054521;S[28][10]=-40.389429;S[28][11]=28.179119;S[28][12]=1.138134;S[28][13]=29.810131;S[28][14]=7.018705;S[28][15]=39.399585;S[28][16]=-21.665473;S[28][17]=34.628918;S[28][18]=-19.168426;S[28][19]=29.553823;S[28][20]=-14.016514;S[28][21]=27.239141;S[28][22]=-10.830939;S[28][23]=26.432062;S[28][24]=-9.448409;S[28][25]=26.150651;S[28][26]=-8.095519;S[28][27]=28.423709;S[28][28]=-7.381984;S[28][29]=27.345947;S[29][0]=-6.872298;S[29][1]=13.899329;S[29][2]=-7.369059;S[29][3]=19.077371;S[29][4]=7.828066;S[29][5]=12.090275;S[29][6]=-27.456858;S[29][7]=4.379763;S[29][8]=-52.183384;S[29][9]=8.398527;S[29][10]=-7.581638;S[29][11]=7.198495;S[29][12]=21.573785;S[29][13]=15.615637;S[29][14]=-8.279698;S[29][15]=18.480714;S[29][16]=-11.262698;S[29][17]=21.424222;S[29][18]=-9.918572;S[29][19]=22.450560;S[29][20]=-8.910776;S[29][21]=22.498912;S[29][22]=-8.317568;S[29][23]=22.825281;S[29][24]=-8.110729;S[29][25]=23.210079;S[29][26]=-8.038609;S[29][27]=23.724572;S[29][28]=-8.013462;S[29][29]=23.903964;S[30][0]=-6.125795;S[30][1]=16.274157;S[30][2]=-8.359148;S[30][3]=22.979808;S[30][4]=-2.348071;S[30][5]=27.947001;S[30][6]=0.005246;S[30][7]=20.126738;S[30][8]=-14.162087;S[30][9]=13.932343;S[30][10]=-45.690994;S[30][11]=13.429605;S[30][12]=-7.681582;S[30][13]=17.298740;S[30][14]=18.850094;S[30][15]=26.922531;S[30][16]=-10.347230;S[30][17]=33.632397;S[30][18]=-16.041680;S[30][19]=30.518088;S[30][20]=-12.130005;S[30][21]=27.657149;S[30][22]=-10.091365;S[30][23]=26.577812;S[30][24]=-8.811003;S[30][25]=26.201471;S[30][26]=-8.282779;S[30][27]=25.970248;S[30][28]=-8.098599;S[30][29]=25.338305;S[31][0]=-7.394001;S[31][1]=13.711646;S[31][2]=-9.824455;S[31][3]=18.666130;S[31][4]=-1.041883;S[31][5]=21.835218;S[31][6]=4.871371;S[31][7]=18.848053;S[31][8]=-39.084985;S[31][9]=15.567270;S[31][10]=-41.806225;S[31][11]=14.183397;S[31][12]=5.063007;S[31][13]=20.764928;S[31][14]=11.683158;S[31][15]=22.236010;S[31][16]=-14.930596;S[31][17]=20.448825;S[31][18]=-14.194302;S[31][19]=21.217327;S[31][20]=-10.940760;S[31][21]=21.727099;S[31][22]=-9.676701;S[31][23]=21.904845;S[31][24]=-9.235951;S[31][25]=21.966822;S[31][26]=-9.082271;S[31][27]=22.557964;S[31][28]=-9.028686;S[31][29]=22.845872;S[32][0]=-6.068842;S[32][1]=16.238794;S[32][2]=-4.391314;S[32][3]=18.597973;S[32][4]=7.615827;S[32][5]=9.316941;S[32][6]=-30.874291;S[32][7]=5.545856;S[32][8]=-37.287451;S[32][9]=8.358338;S[32][10]=1.443154;S[32][11]=20.906613;S[32][12]=1.573467;S[32][13]=34.619401;S[32][14]=-18.706515;S[32][15]=29.722898;S[32][16]=-15.510884;S[32][17]=26.927094;S[32][18]=-11.960268;S[32][19]=25.020615;S[32][20]=-10.683503;S[32][21]=24.355866;S[32][22]=-9.616620;S[32][23]=24.124083;S[32][24]=-8.563680;S[32][25]=24.043265;S[32][26]=-8.196543;S[32][27]=24.015086;S[32][28]=-8.068530;S[32][29]=24.005260;S[33][0]=-6.681421;S[33][1]=14.042375;S[33][2]=6.647645;S[33][3]=20.335342;S[33][4]=-2.395453;S[33][5]=7.355125;S[33][6]=-55.133742;S[33][7]=3.484297;S[33][8]=-29.509322;S[33][9]=10.138961;S[33][10]=14.734435;S[33][11]=24.583373;S[33][12]=-11.306005;S[33][13]=25.359698;S[33][14]=-16.520005;S[33][15]=24.474097;S[33][16]=-12.362397;S[33][17]=23.755798;S[33][18]=-10.172395;S[33][19]=23.263530;S[33][20]=-9.408789;S[33][21]=23.091887;S[33][22]=-9.142536;S[33][23]=23.032039;S[33][24]=-9.049699;S[33][25]=23.011171;S[33][26]=-8.607819;S[33][27]=23.003895;S[33][28]=-8.211933;S[33][29]=23.001358;S[34][0]=-6.428865;S[34][1]=15.514488;S[34][2]=-5.960691;S[34][3]=21.618990;S[34][4]=7.744320;S[34][5]=13.274975;S[34][6]=-27.182142;S[34][7]=5.908820;S[34][8]=-37.523518;S[34][9]=12.590081;S[34][10]=0.188353;S[34][11]=13.894073;S[34][12]=6.172209;S[34][13]=28.042982;S[34][14]=-16.622647;S[34][15]=30.661566;S[34][16]=-15.335606;S[34][17]=28.714538;S[34][18]=-11.247831;S[34][19]=27.076119;S[34][20]=-9.783770;S[34][21]=25.805687;S[34][22]=-9.273284;S[34][23]=24.668346;S[34][24]=-9.095288;S[34][25]=24.802571;S[34][26]=-9.033225;S[34][27]=24.931161;S[34][28]=-9.011585;S[34][29]=24.975997;S[35][0]=-7.223975;S[35][1]=12.723339;S[35][2]=-9.049481;S[35][3]=17.806685;S[35][4]=10.260618;S[35][5]=14.357525;S[35][6]=-5.147932;S[35][7]=-3.756327;S[35][8]=-42.349284;S[35][9]=4.167724;S[35][10]=-12.631291;S[35][11]=17.955513;S[35][12]=13.243621;S[35][13]=34.702413;S[35][14]=-10.529622;S[35][15]=31.576383;S[35][16]=-12.508198;S[35][17]=27.331785;S[35][18]=-10.535614;S[35][19]=25.813043;S[35][20]=-9.191535;S[35][21]=24.670911;S[35][22]=-9.032825;S[35][23]=24.233932;S[35][24]=-9.011445;S[35][25]=24.081567;S[35][26]=-8.352669;S[35][27]=24.028441;S[35][28]=-8.122968;S[35][29]=24.009917;S[36][0]=-5.854795;S[36][1]=15.843135;S[36][2]=1.705507;S[36][3]=27.991179;S[36][4]=17.698505;S[36][5]=21.037402;S[36][6]=-22.545145;S[36][7]=11.959979;S[36][8]=-31.868283;S[36][9]=18.396797;S[36][10]=9.903748;S[36][11]=28.498483;S[36][12]=24.170432;S[36][13]=42.491324;S[36][14]=-11.314869;S[36][15]=35.039548;S[36][16]=-14.386368;S[36][17]=29.151896;S[36][18]=-11.581244;S[36][19]=26.486419;S[36][20]=-9.900024;S[36][21]=25.518282;S[36][22]=-9.313819;S[36][23]=25.180714;S[36][24]=-9.109422;S[36][25]=25.063011;S[36][26]=-9.038153;S[36][27]=25.021971;S[36][28]=-9.013303;S[36][29]=25.007661;S[37][0]=-6.738375;S[37][1]=12.674170;S[37][2]=-9.014063;S[37][3]=16.402235;S[37][4]=11.424533;S[37][5]=15.638927;S[37][6]=-5.103730;S[37][7]=8.327872;S[37][8]=-40.947547;S[37][9]=8.390718;S[37][10]=-6.973317;S[37][11]=11.783859;S[37][12]=27.403578;S[37][13]=22.968207;S[37][14]=-3.170693;S[37][15]=24.043529;S[37][16]=-10.183877;S[37][17]=25.317821;S[37][18]=-9.784223;S[37][19]=25.762139;S[37][20]=-9.273441;S[37][21]=25.917063;S[37][22]=-9.095343;S[37][23]=25.971082;S[37][24]=-9.033244;S[37][25]=25.468214;S[37][26]=-9.011592;S[37][27]=25.163256;S[37][28]=-9.004042;S[37][29]=24.405602;S[38][0]=-6.125795;S[38][1]=17.243103;S[38][2]=-7.937445;S[38][3]=23.347599;S[38][4]=-3.853444;S[38][5]=24.264015;S[38][6]=9.866614;S[38][7]=21.726788;S[38][8]=-26.318005;S[38][9]=9.519783;S[38][10]=-53.654753;S[38][11]=0.974591;S[38][12]=-14.030530;S[38][13]=11.875467;S[38][14]=21.444079;S[38][15]=26.103953;S[38][16]=-16.138609;S[38][17]=26.368989;S[38][18]=-20.095648;S[38][19]=26.089916;S[38][20]=-14.575180;S[38][21]=26.682673;S[38][22]=-11.073563;S[38][23]=26.276776;S[38][24]=-9.723007;S[38][25]=25.906506;S[38][26]=-9.252097;S[38][27]=24.664757;S[38][28]=-9.087901;S[38][29]=24.231787;S[39][0]=-6.125795;S[39][1]=14.005396;S[39][2]=-6.848148;S[39][3]=19.720310;S[39][4]=13.015195;S[39][5]=18.843653;S[39][6]=-13.496770;S[39][7]=5.865882;S[39][8]=-50.069878;S[39][9]=10.507504;S[39][10]=-26.709995;S[39][11]=8.361919;S[39][12]=8.917553;S[39][13]=21.726277;S[39][14]=-20.556654;S[39][15]=29.058482;S[39][16]=-21.086892;S[39][17]=28.068871;S[39][18]=-14.847905;S[39][19]=27.372692;S[39][20]=-11.500360;S[39][21]=26.560417;S[39][22]=-9.871822;S[39][23]=26.195405;S[39][24]=-9.303985;S[39][25]=26.068134;S[39][26]=-9.105993;S[39][27]=26.023757;S[39][28]=-9.036958;S[39][29]=26.008283;
  
}



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
