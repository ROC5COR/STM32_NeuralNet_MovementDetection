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
#define SAMPLE_SIZE 102
#define INPUT_SIZE 30
#define OUTPUT_SIZE 4
#define INPUT_SIZE_FILTER 150
#define HIDDEN_NEURON 30
#define CIRCLE_SAMPLES 40
#define SQUARE_SAMPLES 40

#define TRAINING
#define TESTING
#define ACCELERO_
#define LOUKA_
#define FILTER_
#define FPGA_COM_
#define ACCELERO_
#define COMMUNICATION_

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
//MAT *outputs;

#ifdef TRAINING
float inputs[2*20][INPUT_SIZE];
float outputs[2*20][OUTPUT_SIZE];
float circle[CIRCLE_SAMPLES][INPUT_SIZE];
float square[SQUARE_SAMPLES][INPUT_SIZE];
#endif




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
    //printMatrix_float(l1,(char*)"L1Before");
    limitMatrix(l1,l1);
    //printMatrix_float(l1, (char*)"L1After");
    sigmoid_matrix(l1,l1);
    matrixProduct_float(l1,syn2,l2);
    limitMatrix(l2,l2);
    sigmoid_matrix(l2,l2);
}

void train_solo(){
    /*if(inputs->matM != outputs->matM){
        printf("Error : Number of inputs and outputs mismatch");
        return;
    }*/
#ifdef TRAINING
    int i;
    for(i = 0; i < 2*20;i++){
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
	float INPUT[30] = {{0}};
	init_INPUT(INPUT);
	float OUTPUT[4]={{0}};

	printf("initialisations effectues!\n");

	send_STM32_Input_request();

	int i,verif;
	float temp_fpga_element = 0;

	for(i = 0; i < 20;0){
			send_input_element(INPUT[i]);
			wait_for_ack_FPGA();
			printf("le FPGA a bien recut l'element %d\n",i);
			wait_for_req_FPGA();
			printf("le FPGA a renvoye son element %d\n",i);
			temp_fpga_element = read_fpga_input_element();
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
	wait_for_ack_FPGA();
	for(int j = 0 ; j<4 ;0){
		wait_for_req_FPGA();
		temp_fpga_element = read_fpga_input_element();
		send_input_element(temp_fpga_element);
		send_ack_STM32();
		HAL_Delay(10);
		verif = FPGA_verification_result();
		if (verif==1){
			OUTPUT[j]= temp_fpga_element;
			j+=1;
		}else {
			printf("le STM32 a renvoye au FPGA un element faux \n");
		}
	}

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
  //Clock Output
  /*GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);*/

  	  //button
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);


  //HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_16);
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_16);

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


	 // uint32_t endTime, startTime = HAL_GetTick();

#ifdef LOUKA
	 
#endif

#ifdef TRAINING
	fillRandomValuesFloat(syn1,-1,1);
	fillRandomValuesFloat(syn2,-1,1);
	initTrainingSoloMatrix();
	//fill_static_matrix_inputs();
	//fill_static_matrix_outputs();

    fill_matrix_circle();
    fill_matrix_square();

    /*for(int i = 0; i < CIRCLE_SAMPLES; i++){
      for(int j = 0; j < INPUT_SIZE; j++){
        printf("%f ",circle[i][j]);
      }
      printf("\n");
    }*/
    printf("Setting data in inputs matrix...\n");
    //First matrix filling
    for(int i = 0; i < 20; i++){
      for(int j = 0; j < INPUT_SIZE; j++){
        inputs[i][j] = (float)(circle[i][j]+250.0)/(float)500.0;//Formatting data
    	//inputs[i][j] = (float)(circle[i][j])/(float)500.0;
      }
      outputs[i][0] = 0;
      outputs[i][1] = 0;
      outputs[i][2] = 0;
      outputs[i][3] = 1;
    }
    printf("First matrix : set\n");
    //Second matrix filling
   for(int i = 0; i < 20; i++){
      for(int j = 0; j < INPUT_SIZE; j++){
        inputs[i+20][j] = (float)(square[i][j]+250.0)/(float)500.0;//Formatting data
        //inputs[i+20][j] = (float)(square[i][j])/(float)500.0;
      }
      outputs[i+20][0] = 0;
      outputs[i+20][1] = 0;
      outputs[i+20][2] = 1;
      outputs[i+20][3] = 0;
    }
    printf("Second matrix : set\n");

    /*for(int i = 0; i < CIRCLE_SAMPLES; i++){
          for(int j = 0; j < INPUT_SIZE; j++){
            printf("%f ",inputs[i][j]);
          }
          printf("\n");
    }*/
    //printMatrix(syn1,(char*)"syn1 before");

	  uint32_t startTime = HAL_GetTick();
	  for(int epochs = 0; epochs < 100; epochs++){
		  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
		  train_solo();
		  //if(epochs % 100 == 0){
			  printf("Epochs : %d\n",epochs);
		  //}
		  //printMatrix(syn1,(char*)"syn1");
		  //printMatrix_float(syn2,(char*)"syn2");
		  //getchar();

	  }
	  uint32_t endTime = HAL_GetTick();

	  printf("StartTime : \t%ld\n",startTime);
	  printf("EndTime : \t%ld\n",endTime);
	  freeTrainingSoloMatrix();
#endif


#ifdef ACCELERO

	  initTestingMatrix();
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
	  printf("testing\n");
	  initTestingMatrix();

	  //Testing with new data
	  for(int i = 0; i < INPUT_SIZE; i++){
		  ((float**)input->mat)[0][i] = circle[20 + 1][i];
	  }
	  uint32_t startTimeTesting = HAL_GetTick();
	  performComputation();
	  uint32_t endTimeTesting = HAL_GetTick();

	  printMatrix(l2,(char*)"1: Circle ? : ");
	  printf("DeltaTick : %ld\n",endTimeTesting-startTimeTesting);

	  for(int sample = 20; sample < 40; sample++){
		  for(int i = 0; i < INPUT_SIZE; i++){
			  ((float**)input->mat)[0][i] = circle[sample][i];
		  }
		  printf("Sample(%d)\n",sample);
		  performComputation();
		  printMatrix(l2,(char*)"1: Circle ? : ");
	  }

	  for(int sample = 20; sample < 40; sample++){
		  for(int i = 0; i < INPUT_SIZE; i++){
			  ((float**)input->mat)[0][i] = square[sample][i];
		  }
		  printf("Sample(%d)\n",sample);
		  performComputation();
		  printMatrix(l2,(char*)"Square ? : ");
	  }

	  freeTestingMatrix();

#endif
#ifdef TRAINING
    printMatData(syn1,(char*)"syn1");
    printMatData(syn2,(char*)"syn2");
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
#endif

	}
  }
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

	  int16_t i2c_data[3] = {0, 0,0};
	  int16_t i2c_data_temp[3] ={0,0,0};

	  int16_t i2c_data_filter[1][INPUT_SIZE_FILTER*2];
	  int16_t i2c_data_pre_filter[1][INPUT_SIZE_FILTER*2];
	  int16_t i2c_data_output[1][INPUT_SIZE_FILTER*2];

	  while(i<INPUT_SIZE_FILTER){
		  while (j<30){
			  getOutput(&ACCELERO_I2C,i2c_data);
			  i2c_data_temp[0]+=i2c_data[0];
			  i2c_data_temp[1]+=i2c_data[1];
			  j++;

		  }

		  i2c_data_filter[0][2*i] = i2c_data_temp[0]/30.0;
		  i2c_data_filter[0][1+2*i] = i2c_data_temp[1]/30.0;

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
		i2c_data_pre_filter[0][0]=i2c_data_filter[0][0];
		i2c_data_pre_filter[0][1]=i2c_data_filter[0][1];
		i2c_data_output[0][0]=i2c_data_filter[0][0];
		i2c_data_output[0][1]=i2c_data_filter[0][1];

		//printf("suppression offset\n");
		//((float**)input->mat)[0][0]=0;
		//((float**)input->mat)[0][1]=0;
		((float**)input_filter->mat)[0][0]=0;
		((float**)input_filter->mat)[0][1]=0;

		//printf("Data\n");
		printf("%f\n",((float**)input_filter->mat)[0][0]);
		printf("%f\n",((float**)input_filter->mat)[0][1]);

		for(i=1;i<INPUT_SIZE_FILTER;i++){

//		  i2c_data_pre_filter[0][2*i]=i2c_data_filter[0][2*i]*0.1+i2c_data_pre_filter[0][2*(i-1)]*0.9;
//		  i2c_data_pre_filter[0][1+2*i]=i2c_data_filter[0][1+2*i]*0.1+i2c_data_pre_filter[0][1+2*(i-1)]*0.9;
//
//		  i2c_data_output[0][2*i]=i2c_data_filter[0][2*i]-i2c_data_pre_filter[0][2*i];
//		  i2c_data_output[0][1+2*i]=i2c_data_filter[0][1+2*i]-i2c_data_pre_filter[0][1+2*i];

			i2c_data_output[0][2*i]=i2c_data_filter[0][2*i]*0.1+i2c_data_output[0][2*(i-1)]*0.9;
			i2c_data_output[0][1+2*i]=i2c_data_filter[0][1+2*i]*0.1+i2c_data_output[0][1+2*(i-1)]*0.9;


		  ((float**)input_filter->mat)[0][2*i]=i2c_data_output[0][2*i];
		  ((float**)input_filter->mat)[0][2*i+1]=i2c_data_output[0][1+2*i];

		  //printf("Data\n");
		  printf("%f\n",((float**)input_filter->mat)[0][2*i]);
		  printf("%f\n",((float**)input_filter->mat)[0][1+2*i]);


	  }

}


void fill_matrix_syn1(){
((float**)syn1->mat)[0][0]=-0.053583;((float**)syn1->mat)[0][1]=0.597654;((float**)syn1->mat)[0][2]=-4.098351;((float**)syn1->mat)[0][3]=-0.705224;((float**)syn1->mat)[0][4]=2.489956;((float**)syn1->mat)[0][5]=-1.187021;((float**)syn1->mat)[0][6]=-5.766134;((float**)syn1->mat)[0][7]=-5.902740;((float**)syn1->mat)[0][8]=0.221328;((float**)syn1->mat)[0][9]=1.007022;((float**)syn1->mat)[0][10]=2.946260;((float**)syn1->mat)[0][11]=0.287398;((float**)syn1->mat)[0][12]=-1.348502;((float**)syn1->mat)[0][13]=1.090468;((float**)syn1->mat)[0][14]=-3.594762;((float**)syn1->mat)[0][15]=0.847378;((float**)syn1->mat)[0][16]=1.231352;((float**)syn1->mat)[0][17]=3.277472;((float**)syn1->mat)[0][18]=-2.083746;((float**)syn1->mat)[0][19]=-0.283242;((float**)syn1->mat)[0][20]=3.544383;((float**)syn1->mat)[0][21]=1.969509;((float**)syn1->mat)[0][22]=-3.117229;((float**)syn1->mat)[0][23]=-2.627328;((float**)syn1->mat)[0][24]=2.480145;((float**)syn1->mat)[0][25]=-1.732370;((float**)syn1->mat)[0][26]=0.640900;((float**)syn1->mat)[0][27]=1.047184;((float**)syn1->mat)[0][28]=6.082130;((float**)syn1->mat)[0][29]=1.092062;((float**)syn1->mat)[1][0]=-1.312017;((float**)syn1->mat)[1][1]=-1.069656;((float**)syn1->mat)[1][2]=-2.506498;((float**)syn1->mat)[1][3]=-2.078658;((float**)syn1->mat)[1][4]=-1.657352;((float**)syn1->mat)[1][5]=0.795636;((float**)syn1->mat)[1][6]=-1.020044;((float**)syn1->mat)[1][7]=-3.172391;((float**)syn1->mat)[1][8]=-1.700308;((float**)syn1->mat)[1][9]=-0.003799;((float**)syn1->mat)[1][10]=1.902096;((float**)syn1->mat)[1][11]=2.880908;((float**)syn1->mat)[1][12]=-2.666473;((float**)syn1->mat)[1][13]=1.317662;((float**)syn1->mat)[1][14]=0.704108;((float**)syn1->mat)[1][15]=-0.952949;((float**)syn1->mat)[1][16]=-1.008252;((float**)syn1->mat)[1][17]=2.538396;((float**)syn1->mat)[1][18]=-1.114626;((float**)syn1->mat)[1][19]=-0.916801;((float**)syn1->mat)[1][20]=3.950674;((float**)syn1->mat)[1][21]=1.905381;((float**)syn1->mat)[1][22]=-1.499751;((float**)syn1->mat)[1][23]=-4.292854;((float**)syn1->mat)[1][24]=2.420998;((float**)syn1->mat)[1][25]=-0.500839;((float**)syn1->mat)[1][26]=-1.044478;((float**)syn1->mat)[1][27]=0.522187;((float**)syn1->mat)[1][28]=4.489587;((float**)syn1->mat)[1][29]=0.119387;((float**)syn1->mat)[2][0]=-1.413311;((float**)syn1->mat)[2][1]=-0.515794;((float**)syn1->mat)[2][2]=-1.994178;((float**)syn1->mat)[2][3]=-0.138162;((float**)syn1->mat)[2][4]=5.881309;((float**)syn1->mat)[2][5]=-2.679809;((float**)syn1->mat)[2][6]=-5.906639;((float**)syn1->mat)[2][7]=-3.096250;((float**)syn1->mat)[2][8]=0.685993;((float**)syn1->mat)[2][9]=-0.319173;((float**)syn1->mat)[2][10]=1.990464;((float**)syn1->mat)[2][11]=1.380124;((float**)syn1->mat)[2][12]=-2.227687;((float**)syn1->mat)[2][13]=0.345473;((float**)syn1->mat)[2][14]=-3.889977;((float**)syn1->mat)[2][15]=0.048344;((float**)syn1->mat)[2][16]=1.544167;((float**)syn1->mat)[2][17]=4.111238;((float**)syn1->mat)[2][18]=-3.492348;((float**)syn1->mat)[2][19]=-2.729578;((float**)syn1->mat)[2][20]=3.672515;((float**)syn1->mat)[2][21]=-1.067796;((float**)syn1->mat)[2][22]=-4.254371;((float**)syn1->mat)[2][23]=-0.700071;((float**)syn1->mat)[2][24]=4.914087;((float**)syn1->mat)[2][25]=-0.476105;((float**)syn1->mat)[2][26]=-0.838250;((float**)syn1->mat)[2][27]=-0.583595;((float**)syn1->mat)[2][28]=6.195177;((float**)syn1->mat)[2][29]=-0.374393;((float**)syn1->mat)[3][0]=1.109959;((float**)syn1->mat)[3][1]=0.191988;((float**)syn1->mat)[3][2]=2.146165;((float**)syn1->mat)[3][3]=-2.180822;((float**)syn1->mat)[3][4]=-5.886238;((float**)syn1->mat)[3][5]=4.696073;((float**)syn1->mat)[3][6]=4.787238;((float**)syn1->mat)[3][7]=3.201991;((float**)syn1->mat)[3][8]=-4.713163;((float**)syn1->mat)[3][9]=3.106673;((float**)syn1->mat)[3][10]=-9.305729;((float**)syn1->mat)[3][11]=8.015347;((float**)syn1->mat)[3][12]=-0.432744;((float**)syn1->mat)[3][13]=0.350902;((float**)syn1->mat)[3][14]=7.314897;((float**)syn1->mat)[3][15]=-1.796243;((float**)syn1->mat)[3][16]=-0.413932;((float**)syn1->mat)[3][17]=3.160640;((float**)syn1->mat)[3][18]=-0.579722;((float**)syn1->mat)[3][19]=-0.942482;((float**)syn1->mat)[3][20]=0.343413;((float**)syn1->mat)[3][21]=-7.482828;((float**)syn1->mat)[3][22]=-0.063808;((float**)syn1->mat)[3][23]=1.460689;((float**)syn1->mat)[3][24]=0.815758;((float**)syn1->mat)[3][25]=1.722594;((float**)syn1->mat)[3][26]=-10.189059;((float**)syn1->mat)[3][27]=-3.185374;((float**)syn1->mat)[3][28]=1.259280;((float**)syn1->mat)[3][29]=-3.091129;((float**)syn1->mat)[4][0]=0.961116;((float**)syn1->mat)[4][1]=-2.032515;((float**)syn1->mat)[4][2]=2.626921;((float**)syn1->mat)[4][3]=1.765605;((float**)syn1->mat)[4][4]=2.748535;((float**)syn1->mat)[4][5]=-1.309143;((float**)syn1->mat)[4][6]=0.633505;((float**)syn1->mat)[4][7]=4.139172;((float**)syn1->mat)[4][8]=1.878681;((float**)syn1->mat)[4][9]=-1.892024;((float**)syn1->mat)[4][10]=-0.495952;((float**)syn1->mat)[4][11]=-4.067173;((float**)syn1->mat)[4][12]=0.405190;((float**)syn1->mat)[4][13]=-2.052018;((float**)syn1->mat)[4][14]=-2.537711;((float**)syn1->mat)[4][15]=0.223880;((float**)syn1->mat)[4][16]=0.915516;((float**)syn1->mat)[4][17]=-4.581836;((float**)syn1->mat)[4][18]=0.048087;((float**)syn1->mat)[4][19]=-0.375505;((float**)syn1->mat)[4][20]=-4.015101;((float**)syn1->mat)[4][21]=-2.446388;((float**)syn1->mat)[4][22]=0.823002;((float**)syn1->mat)[4][23]=2.874919;((float**)syn1->mat)[4][24]=-3.439570;((float**)syn1->mat)[4][25]=-0.959338;((float**)syn1->mat)[4][26]=2.021383;((float**)syn1->mat)[4][27]=-0.645583;((float**)syn1->mat)[4][28]=-6.172066;((float**)syn1->mat)[4][29]=-1.226192;((float**)syn1->mat)[5][0]=0.846006;((float**)syn1->mat)[5][1]=-0.304600;((float**)syn1->mat)[5][2]=1.565866;((float**)syn1->mat)[5][3]=-0.364610;((float**)syn1->mat)[5][4]=-1.666837;((float**)syn1->mat)[5][5]=0.070512;((float**)syn1->mat)[5][6]=2.626698;((float**)syn1->mat)[5][7]=2.231751;((float**)syn1->mat)[5][8]=-0.917112;((float**)syn1->mat)[5][9]=-0.898715;((float**)syn1->mat)[5][10]=-2.785989;((float**)syn1->mat)[5][11]=-2.067355;((float**)syn1->mat)[5][12]=1.174137;((float**)syn1->mat)[5][13]=-0.779162;((float**)syn1->mat)[5][14]=-0.605167;((float**)syn1->mat)[5][15]=-0.205781;((float**)syn1->mat)[5][16]=-0.823486;((float**)syn1->mat)[5][17]=-3.898803;((float**)syn1->mat)[5][18]=-0.057752;((float**)syn1->mat)[5][19]=-0.311124;((float**)syn1->mat)[5][20]=-4.738879;((float**)syn1->mat)[5][21]=-3.697937;((float**)syn1->mat)[5][22]=2.228278;((float**)syn1->mat)[5][23]=2.639877;((float**)syn1->mat)[5][24]=-4.048104;((float**)syn1->mat)[5][25]=0.848236;((float**)syn1->mat)[5][26]=-0.463334;((float**)syn1->mat)[5][27]=-0.742443;((float**)syn1->mat)[5][28]=-4.535523;((float**)syn1->mat)[5][29]=-0.816607;((float**)syn1->mat)[6][0]=0.118880;((float**)syn1->mat)[6][1]=-1.612307;((float**)syn1->mat)[6][2]=3.587981;((float**)syn1->mat)[6][3]=1.860618;((float**)syn1->mat)[6][4]=-1.889561;((float**)syn1->mat)[6][5]=0.436015;((float**)syn1->mat)[6][6]=6.603049;((float**)syn1->mat)[6][7]=5.980086;((float**)syn1->mat)[6][8]=-0.582270;((float**)syn1->mat)[6][9]=-1.630766;((float**)syn1->mat)[6][10]=-2.936054;((float**)syn1->mat)[6][11]=-2.926777;((float**)syn1->mat)[6][12]=2.890810;((float**)syn1->mat)[6][13]=-1.683028;((float**)syn1->mat)[6][14]=1.395796;((float**)syn1->mat)[6][15]=-1.090983;((float**)syn1->mat)[6][16]=-1.864521;((float**)syn1->mat)[6][17]=-5.072907;((float**)syn1->mat)[6][18]=1.137310;((float**)syn1->mat)[6][19]=0.590558;((float**)syn1->mat)[6][20]=-5.471736;((float**)syn1->mat)[6][21]=-1.076477;((float**)syn1->mat)[6][22]=3.249291;((float**)syn1->mat)[6][23]=4.003665;((float**)syn1->mat)[6][24]=-5.076550;((float**)syn1->mat)[6][25]=1.128864;((float**)syn1->mat)[6][26]=1.552872;((float**)syn1->mat)[6][27]=-0.971574;((float**)syn1->mat)[6][28]=-8.292823;((float**)syn1->mat)[6][29]=-1.739702;((float**)syn1->mat)[7][0]=0.174077;((float**)syn1->mat)[7][1]=-2.842941;((float**)syn1->mat)[7][2]=1.495489;((float**)syn1->mat)[7][3]=3.822083;((float**)syn1->mat)[7][4]=11.991820;((float**)syn1->mat)[7][5]=-7.730891;((float**)syn1->mat)[7][6]=-5.849920;((float**)syn1->mat)[7][7]=2.517161;((float**)syn1->mat)[7][8]=6.503394;((float**)syn1->mat)[7][9]=-4.044504;((float**)syn1->mat)[7][10]=1.674260;((float**)syn1->mat)[7][11]=-11.615273;((float**)syn1->mat)[7][12]=2.382777;((float**)syn1->mat)[7][13]=-2.759719;((float**)syn1->mat)[7][14]=-11.658626;((float**)syn1->mat)[7][15]=-0.231713;((float**)syn1->mat)[7][16]=2.021286;((float**)syn1->mat)[7][17]=-6.946409;((float**)syn1->mat)[7][18]=-0.278123;((float**)syn1->mat)[7][19]=-1.504960;((float**)syn1->mat)[7][20]=-6.459098;((float**)syn1->mat)[7][21]=-4.240177;((float**)syn1->mat)[7][22]=0.335849;((float**)syn1->mat)[7][23]=6.710317;((float**)syn1->mat)[7][24]=-4.703204;((float**)syn1->mat)[7][25]=-2.037041;((float**)syn1->mat)[7][26]=5.362828;((float**)syn1->mat)[7][27]=-0.134859;((float**)syn1->mat)[7][28]=-8.840103;((float**)syn1->mat)[7][29]=0.910966;((float**)syn1->mat)[8][0]=0.676653;((float**)syn1->mat)[8][1]=-0.173838;((float**)syn1->mat)[8][2]=3.677427;((float**)syn1->mat)[8][3]=0.504978;((float**)syn1->mat)[8][4]=-4.175120;((float**)syn1->mat)[8][5]=0.910940;((float**)syn1->mat)[8][6]=4.803748;((float**)syn1->mat)[8][7]=5.114991;((float**)syn1->mat)[8][8]=-1.607770;((float**)syn1->mat)[8][9]=-1.137258;((float**)syn1->mat)[8][10]=-4.547597;((float**)syn1->mat)[8][11]=-0.481734;((float**)syn1->mat)[8][12]=2.440280;((float**)syn1->mat)[8][13]=-2.115320;((float**)syn1->mat)[8][14]=4.263554;((float**)syn1->mat)[8][15]=-1.072985;((float**)syn1->mat)[8][16]=-1.162142;((float**)syn1->mat)[8][17]=-3.046068;((float**)syn1->mat)[8][18]=-0.153120;((float**)syn1->mat)[8][19]=-1.020138;((float**)syn1->mat)[8][20]=-3.640876;((float**)syn1->mat)[8][21]=-4.101482;((float**)syn1->mat)[8][22]=2.215025;((float**)syn1->mat)[8][23]=2.488670;((float**)syn1->mat)[8][24]=-2.536694;((float**)syn1->mat)[8][25]=1.632358;((float**)syn1->mat)[8][26]=-4.188701;((float**)syn1->mat)[8][27]=-2.122204;((float**)syn1->mat)[8][28]=-5.172194;((float**)syn1->mat)[8][29]=-2.409991;((float**)syn1->mat)[9][0]=0.504113;((float**)syn1->mat)[9][1]=-1.643699;((float**)syn1->mat)[9][2]=0.162590;((float**)syn1->mat)[9][3]=2.875843;((float**)syn1->mat)[9][4]=7.141473;((float**)syn1->mat)[9][5]=-5.190816;((float**)syn1->mat)[9][6]=-3.559948;((float**)syn1->mat)[9][7]=1.415747;((float**)syn1->mat)[9][8]=3.184219;((float**)syn1->mat)[9][9]=-3.922419;((float**)syn1->mat)[9][10]=2.668644;((float**)syn1->mat)[9][11]=-7.455475;((float**)syn1->mat)[9][12]=0.822227;((float**)syn1->mat)[9][13]=-2.293324;((float**)syn1->mat)[9][14]=-7.495311;((float**)syn1->mat)[9][15]=0.462452;((float**)syn1->mat)[9][16]=2.037493;((float**)syn1->mat)[9][17]=-2.661158;((float**)syn1->mat)[9][18]=-0.675064;((float**)syn1->mat)[9][19]=-1.095889;((float**)syn1->mat)[9][20]=-2.101721;((float**)syn1->mat)[9][21]=-0.780990;((float**)syn1->mat)[9][22]=-0.173412;((float**)syn1->mat)[9][23]=3.759717;((float**)syn1->mat)[9][24]=-2.169615;((float**)syn1->mat)[9][25]=-0.473325;((float**)syn1->mat)[9][26]=5.681112;((float**)syn1->mat)[9][27]=0.885054;((float**)syn1->mat)[9][28]=-3.707556;((float**)syn1->mat)[9][29]=1.194122;((float**)syn1->mat)[10][0]=0.718036;((float**)syn1->mat)[10][1]=-0.652579;((float**)syn1->mat)[10][2]=3.917149;((float**)syn1->mat)[10][3]=-1.025241;((float**)syn1->mat)[10][4]=-5.916859;((float**)syn1->mat)[10][5]=4.260535;((float**)syn1->mat)[10][6]=6.988031;((float**)syn1->mat)[10][7]=5.055713;((float**)syn1->mat)[10][8]=-4.250556;((float**)syn1->mat)[10][9]=2.395569;((float**)syn1->mat)[10][10]=-9.083617;((float**)syn1->mat)[10][11]=5.996635;((float**)syn1->mat)[10][12]=2.018665;((float**)syn1->mat)[10][13]=-0.679127;((float**)syn1->mat)[10][14]=8.281112;((float**)syn1->mat)[10][15]=-1.820894;((float**)syn1->mat)[10][16]=-1.553990;((float**)syn1->mat)[10][17]=-0.115402;((float**)syn1->mat)[10][18]=-0.355731;((float**)syn1->mat)[10][19]=-0.144803;((float**)syn1->mat)[10][20]=-1.947263;((float**)syn1->mat)[10][21]=-6.582301;((float**)syn1->mat)[10][22]=0.820859;((float**)syn1->mat)[10][23]=1.868673;((float**)syn1->mat)[10][24]=-0.723460;((float**)syn1->mat)[10][25]=3.459791;((float**)syn1->mat)[10][26]=-7.273926;((float**)syn1->mat)[10][27]=-4.141278;((float**)syn1->mat)[10][28]=-1.617763;((float**)syn1->mat)[10][29]=-2.614064;((float**)syn1->mat)[11][0]=0.307781;((float**)syn1->mat)[11][1]=-2.847705;((float**)syn1->mat)[11][2]=0.944062;((float**)syn1->mat)[11][3]=0.998625;((float**)syn1->mat)[11][4]=6.137892;((float**)syn1->mat)[11][5]=-2.884806;((float**)syn1->mat)[11][6]=-3.978332;((float**)syn1->mat)[11][7]=0.382905;((float**)syn1->mat)[11][8]=2.809139;((float**)syn1->mat)[11][9]=-2.302915;((float**)syn1->mat)[11][10]=-0.673100;((float**)syn1->mat)[11][11]=-1.936066;((float**)syn1->mat)[11][12]=2.513796;((float**)syn1->mat)[11][13]=-1.734937;((float**)syn1->mat)[11][14]=-6.052541;((float**)syn1->mat)[11][15]=0.498922;((float**)syn1->mat)[11][16]=3.133592;((float**)syn1->mat)[11][17]=-0.935126;((float**)syn1->mat)[11][18]=-3.196336;((float**)syn1->mat)[11][19]=-1.465289;((float**)syn1->mat)[11][20]=-1.591982;((float**)syn1->mat)[11][21]=-4.080135;((float**)syn1->mat)[11][22]=-2.544140;((float**)syn1->mat)[11][23]=2.557560;((float**)syn1->mat)[11][24]=0.314369;((float**)syn1->mat)[11][25]=1.269748;((float**)syn1->mat)[11][26]=-0.533930;((float**)syn1->mat)[11][27]=-1.655235;((float**)syn1->mat)[11][28]=-2.937499;((float**)syn1->mat)[11][29]=-1.852991;((float**)syn1->mat)[12][0]=-1.350454;((float**)syn1->mat)[12][1]=0.420997;((float**)syn1->mat)[12][2]=-0.949470;((float**)syn1->mat)[12][3]=-2.481737;((float**)syn1->mat)[12][4]=-6.000406;((float**)syn1->mat)[12][5]=2.063386;((float**)syn1->mat)[12][6]=2.782075;((float**)syn1->mat)[12][7]=-1.175950;((float**)syn1->mat)[12][8]=-0.464205;((float**)syn1->mat)[12][9]=1.837551;((float**)syn1->mat)[12][10]=-1.206054;((float**)syn1->mat)[12][11]=1.462227;((float**)syn1->mat)[12][12]=-2.333613;((float**)syn1->mat)[12][13]=0.712485;((float**)syn1->mat)[12][14]=3.991717;((float**)syn1->mat)[12][15]=-1.873411;((float**)syn1->mat)[12][16]=-2.240726;((float**)syn1->mat)[12][17]=-0.273554;((float**)syn1->mat)[12][18]=1.884867;((float**)syn1->mat)[12][19]=1.673204;((float**)syn1->mat)[12][20]=-0.125634;((float**)syn1->mat)[12][21]=3.149173;((float**)syn1->mat)[12][22]=1.302605;((float**)syn1->mat)[12][23]=-2.090202;((float**)syn1->mat)[12][24]=-0.833926;((float**)syn1->mat)[12][25]=-2.306410;((float**)syn1->mat)[12][26]=0.665625;((float**)syn1->mat)[12][27]=-0.323324;((float**)syn1->mat)[12][28]=0.442218;((float**)syn1->mat)[12][29]=-1.104386;((float**)syn1->mat)[13][0]=-0.893680;((float**)syn1->mat)[13][1]=-0.660632;((float**)syn1->mat)[13][2]=-0.644444;((float**)syn1->mat)[13][3]=-3.158346;((float**)syn1->mat)[13][4]=-4.045657;((float**)syn1->mat)[13][5]=2.687685;((float**)syn1->mat)[13][6]=0.042764;((float**)syn1->mat)[13][7]=-1.181130;((float**)syn1->mat)[13][8]=-1.002332;((float**)syn1->mat)[13][9]=0.694757;((float**)syn1->mat)[13][10]=-2.177421;((float**)syn1->mat)[13][11]=4.221119;((float**)syn1->mat)[13][12]=-1.826200;((float**)syn1->mat)[13][13]=-0.244573;((float**)syn1->mat)[13][14]=3.025695;((float**)syn1->mat)[13][15]=-0.607557;((float**)syn1->mat)[13][16]=-1.050212;((float**)syn1->mat)[13][17]=1.703688;((float**)syn1->mat)[13][18]=-1.489733;((float**)syn1->mat)[13][19]=-0.058857;((float**)syn1->mat)[13][20]=2.306285;((float**)syn1->mat)[13][21]=-1.139607;((float**)syn1->mat)[13][22]=-1.959545;((float**)syn1->mat)[13][23]=-1.009452;((float**)syn1->mat)[13][24]=1.702415;((float**)syn1->mat)[13][25]=0.943456;((float**)syn1->mat)[13][26]=-4.062872;((float**)syn1->mat)[13][27]=-0.313141;((float**)syn1->mat)[13][28]=3.948841;((float**)syn1->mat)[13][29]=-1.171460;((float**)syn1->mat)[14][0]=-1.238867;((float**)syn1->mat)[14][1]=-0.054689;((float**)syn1->mat)[14][2]=-5.175598;((float**)syn1->mat)[14][3]=-1.208157;((float**)syn1->mat)[14][4]=-1.684824;((float**)syn1->mat)[14][5]=-0.750282;((float**)syn1->mat)[14][6]=-4.057828;((float**)syn1->mat)[14][7]=-5.662192;((float**)syn1->mat)[14][8]=0.350853;((float**)syn1->mat)[14][9]=-0.002341;((float**)syn1->mat)[14][10]=4.797182;((float**)syn1->mat)[14][11]=-0.185429;((float**)syn1->mat)[14][12]=-4.620930;((float**)syn1->mat)[14][13]=0.823304;((float**)syn1->mat)[14][14]=-1.324299;((float**)syn1->mat)[14][15]=-1.047984;((float**)syn1->mat)[14][16]=-0.884937;((float**)syn1->mat)[14][17]=2.209319;((float**)syn1->mat)[14][18]=0.816429;((float**)syn1->mat)[14][19]=0.496609;((float**)syn1->mat)[14][20]=3.505179;((float**)syn1->mat)[14][21]=8.084231;((float**)syn1->mat)[14][22]=-1.360444;((float**)syn1->mat)[14][23]=-5.191607;((float**)syn1->mat)[14][24]=1.854187;((float**)syn1->mat)[14][25]=-4.470481;((float**)syn1->mat)[14][26]=2.449089;((float**)syn1->mat)[14][27]=2.974238;((float**)syn1->mat)[14][28]=2.814302;((float**)syn1->mat)[14][29]=1.460334;((float**)syn1->mat)[15][0]=-2.311942;((float**)syn1->mat)[15][1]=2.086069;((float**)syn1->mat)[15][2]=-2.952416;((float**)syn1->mat)[15][3]=-3.603230;((float**)syn1->mat)[15][4]=-10.590318;((float**)syn1->mat)[15][5]=2.396792;((float**)syn1->mat)[15][6]=1.564996;((float**)syn1->mat)[15][7]=-5.836871;((float**)syn1->mat)[15][8]=-2.818942;((float**)syn1->mat)[15][9]=2.047905;((float**)syn1->mat)[15][10]=3.104062;((float**)syn1->mat)[15][11]=0.959187;((float**)syn1->mat)[15][12]=-4.637760;((float**)syn1->mat)[15][13]=1.942760;((float**)syn1->mat)[15][14]=5.588029;((float**)syn1->mat)[15][15]=-1.252733;((float**)syn1->mat)[15][16]=-3.295873;((float**)syn1->mat)[15][17]=0.707941;((float**)syn1->mat)[15][18]=2.125034;((float**)syn1->mat)[15][19]=2.589040;((float**)syn1->mat)[15][20]=2.473874;((float**)syn1->mat)[15][21]=11.210567;((float**)syn1->mat)[15][22]=2.403301;((float**)syn1->mat)[15][23]=-9.678681;((float**)syn1->mat)[15][24]=-0.487370;((float**)syn1->mat)[15][25]=-5.006008;((float**)syn1->mat)[15][26]=2.230551;((float**)syn1->mat)[15][27]=1.693283;((float**)syn1->mat)[15][28]=1.972391;((float**)syn1->mat)[15][29]=0.356348;((float**)syn1->mat)[16][0]=-0.473566;((float**)syn1->mat)[16][1]=-0.454774;((float**)syn1->mat)[16][2]=-3.769122;((float**)syn1->mat)[16][3]=0.241253;((float**)syn1->mat)[16][4]=3.140484;((float**)syn1->mat)[16][5]=-1.713102;((float**)syn1->mat)[16][6]=-6.208061;((float**)syn1->mat)[16][7]=-6.984072;((float**)syn1->mat)[16][8]=0.997648;((float**)syn1->mat)[16][9]=-1.149798;((float**)syn1->mat)[16][10]=5.534097;((float**)syn1->mat)[16][11]=-2.480928;((float**)syn1->mat)[16][12]=-2.682177;((float**)syn1->mat)[16][13]=0.276380;((float**)syn1->mat)[16][14]=-4.889588;((float**)syn1->mat)[16][15]=-0.551055;((float**)syn1->mat)[16][16]=0.738521;((float**)syn1->mat)[16][17]=2.433455;((float**)syn1->mat)[16][18]=-1.704965;((float**)syn1->mat)[16][19]=-1.604614;((float**)syn1->mat)[16][20]=2.598282;((float**)syn1->mat)[16][21]=3.093914;((float**)syn1->mat)[16][22]=-1.800853;((float**)syn1->mat)[16][23]=-3.426843;((float**)syn1->mat)[16][24]=3.592186;((float**)syn1->mat)[16][25]=-1.013992;((float**)syn1->mat)[16][26]=2.636743;((float**)syn1->mat)[16][27]=1.691532;((float**)syn1->mat)[16][28]=4.209240;((float**)syn1->mat)[16][29]=0.175728;((float**)syn1->mat)[17][0]=-0.959452;((float**)syn1->mat)[17][1]=0.450694;((float**)syn1->mat)[17][2]=-2.134427;((float**)syn1->mat)[17][3]=-1.256036;((float**)syn1->mat)[17][4]=-2.173069;((float**)syn1->mat)[17][5]=-0.840728;((float**)syn1->mat)[17][6]=0.269830;((float**)syn1->mat)[17][7]=-2.611911;((float**)syn1->mat)[17][8]=-0.299409;((float**)syn1->mat)[17][9]=-1.227267;((float**)syn1->mat)[17][10]=3.428720;((float**)syn1->mat)[17][11]=-1.595580;((float**)syn1->mat)[17][12]=-3.700741;((float**)syn1->mat)[17][13]=-0.672935;((float**)syn1->mat)[17][14]=-0.357548;((float**)syn1->mat)[17][15]=-0.794486;((float**)syn1->mat)[17][16]=-2.502862;((float**)syn1->mat)[17][17]=-2.128783;((float**)syn1->mat)[17][18]=1.755259;((float**)syn1->mat)[17][19]=-0.297487;((float**)syn1->mat)[17][20]=-0.815630;((float**)syn1->mat)[17][21]=6.864399;((float**)syn1->mat)[17][22]=1.740871;((float**)syn1->mat)[17][23]=-4.264221;((float**)syn1->mat)[17][24]=-1.196697;((float**)syn1->mat)[17][25]=-3.585348;((float**)syn1->mat)[17][26]=5.020421;((float**)syn1->mat)[17][27]=0.701057;((float**)syn1->mat)[17][28]=0.523387;((float**)syn1->mat)[17][29]=0.994256;((float**)syn1->mat)[18][0]=0.568753;((float**)syn1->mat)[18][1]=-1.699447;((float**)syn1->mat)[18][2]=-0.225028;((float**)syn1->mat)[18][3]=0.572505;((float**)syn1->mat)[18][4]=2.480478;((float**)syn1->mat)[18][5]=-0.491309;((float**)syn1->mat)[18][6]=-3.423013;((float**)syn1->mat)[18][7]=-0.312181;((float**)syn1->mat)[18][8]=-0.519990;((float**)syn1->mat)[18][9]=-1.035054;((float**)syn1->mat)[18][10]=-2.115210;((float**)syn1->mat)[18][11]=2.120624;((float**)syn1->mat)[18][12]=0.602564;((float**)syn1->mat)[18][13]=-0.518610;((float**)syn1->mat)[18][14]=-1.508489;((float**)syn1->mat)[18][15]=0.367845;((float**)syn1->mat)[18][16]=1.020725;((float**)syn1->mat)[18][17]=2.418897;((float**)syn1->mat)[18][18]=-2.302004;((float**)syn1->mat)[18][19]=-0.669516;((float**)syn1->mat)[18][20]=1.194874;((float**)syn1->mat)[18][21]=-4.998054;((float**)syn1->mat)[18][22]=-3.335124;((float**)syn1->mat)[18][23]=1.232736;((float**)syn1->mat)[18][24]=2.003984;((float**)syn1->mat)[18][25]=2.064871;((float**)syn1->mat)[18][26]=-4.431415;((float**)syn1->mat)[18][27]=-0.868775;((float**)syn1->mat)[18][28]=1.374240;((float**)syn1->mat)[18][29]=-0.146374;((float**)syn1->mat)[19][0]=-0.867944;((float**)syn1->mat)[19][1]=0.404744;((float**)syn1->mat)[19][2]=-0.421135;((float**)syn1->mat)[19][3]=-2.048415;((float**)syn1->mat)[19][4]=-2.266989;((float**)syn1->mat)[19][5]=1.108705;((float**)syn1->mat)[19][6]=0.790705;((float**)syn1->mat)[19][7]=-0.238297;((float**)syn1->mat)[19][8]=-1.727581;((float**)syn1->mat)[19][9]=0.349406;((float**)syn1->mat)[19][10]=0.178892;((float**)syn1->mat)[19][11]=1.794061;((float**)syn1->mat)[19][12]=-0.866598;((float**)syn1->mat)[19][13]=0.930543;((float**)syn1->mat)[19][14]=2.951372;((float**)syn1->mat)[19][15]=-0.361163;((float**)syn1->mat)[19][16]=-1.674644;((float**)syn1->mat)[19][17]=1.809051;((float**)syn1->mat)[19][18]=0.487646;((float**)syn1->mat)[19][19]=-1.271207;((float**)syn1->mat)[19][20]=1.812047;((float**)syn1->mat)[19][21]=0.722970;((float**)syn1->mat)[19][22]=-1.161245;((float**)syn1->mat)[19][23]=-1.464017;((float**)syn1->mat)[19][24]=0.603790;((float**)syn1->mat)[19][25]=-0.544690;((float**)syn1->mat)[19][26]=-1.447628;((float**)syn1->mat)[19][27]=0.524692;((float**)syn1->mat)[19][28]=1.403492;((float**)syn1->mat)[19][29]=-1.588721;
}
void fill_matrix_syn2(){
((float**)syn2->mat)[0][0]=-0.860748;((float**)syn2->mat)[0][1]=-0.946414;((float**)syn2->mat)[0][2]=0.510868;((float**)syn2->mat)[0][3]=0.093052;((float**)syn2->mat)[1][0]=0.578272;((float**)syn2->mat)[1][1]=0.437867;((float**)syn2->mat)[1][2]=-0.425710;((float**)syn2->mat)[1][3]=0.235929;((float**)syn2->mat)[2][0]=-4.628509;((float**)syn2->mat)[2][1]=-4.325458;((float**)syn2->mat)[2][2]=-1.009585;((float**)syn2->mat)[2][3]=2.172581;((float**)syn2->mat)[3][0]=-1.045445;((float**)syn2->mat)[3][1]=-0.004892;((float**)syn2->mat)[3][2]=1.504184;((float**)syn2->mat)[3][3]=-0.233437;((float**)syn2->mat)[4][0]=-2.280049;((float**)syn2->mat)[4][1]=-1.165263;((float**)syn2->mat)[4][2]=4.147265;((float**)syn2->mat)[4][3]=-6.422021;((float**)syn2->mat)[5][0]=1.158191;((float**)syn2->mat)[5][1]=-3.289140;((float**)syn2->mat)[5][2]=-2.928745;((float**)syn2->mat)[5][3]=1.563332;((float**)syn2->mat)[6][0]=-3.799554;((float**)syn2->mat)[6][1]=-3.612845;((float**)syn2->mat)[6][2]=-5.051707;((float**)syn2->mat)[6][3]=5.623632;((float**)syn2->mat)[7][0]=-5.835114;((float**)syn2->mat)[7][1]=-6.388254;((float**)syn2->mat)[7][2]=-1.212695;((float**)syn2->mat)[7][3]=3.244730;((float**)syn2->mat)[8][0]=-3.260185;((float**)syn2->mat)[8][1]=0.461693;((float**)syn2->mat)[8][2]=1.798117;((float**)syn2->mat)[8][3]=-4.065910;((float**)syn2->mat)[9][0]=1.319178;((float**)syn2->mat)[9][1]=-0.730189;((float**)syn2->mat)[9][2]=-1.106432;((float**)syn2->mat)[9][3]=0.274569;((float**)syn2->mat)[10][0]=-1.534551;((float**)syn2->mat)[10][1]=3.680777;((float**)syn2->mat)[10][2]=-4.084251;((float**)syn2->mat)[10][3]=-5.985116;((float**)syn2->mat)[11][0]=4.184460;((float**)syn2->mat)[11][1]=-5.857653;((float**)syn2->mat)[11][2]=-3.294002;((float**)syn2->mat)[11][3]=-1.027014;((float**)syn2->mat)[12][0]=-0.461145;((float**)syn2->mat)[12][1]=-0.678242;((float**)syn2->mat)[12][2]=1.022365;((float**)syn2->mat)[12][3]=0.798284;((float**)syn2->mat)[13][0]=1.035317;((float**)syn2->mat)[13][1]=-0.093845;((float**)syn2->mat)[13][2]=-0.765355;((float**)syn2->mat)[13][3]=-0.602632;((float**)syn2->mat)[14][0]=0.213142;((float**)syn2->mat)[14][1]=-6.255798;((float**)syn2->mat)[14][2]=-6.255063;((float**)syn2->mat)[14][3]=3.756855;((float**)syn2->mat)[15][0]=0.088324;((float**)syn2->mat)[15][1]=0.060564;((float**)syn2->mat)[15][2]=0.489949;((float**)syn2->mat)[15][3]=-0.152535;((float**)syn2->mat)[16][0]=-0.025030;((float**)syn2->mat)[16][1]=-0.261552;((float**)syn2->mat)[16][2]=1.468414;((float**)syn2->mat)[16][3]=-1.477012;((float**)syn2->mat)[17][0]=4.213745;((float**)syn2->mat)[17][1]=-2.474695;((float**)syn2->mat)[17][2]=-1.859632;((float**)syn2->mat)[17][3]=-3.105944;((float**)syn2->mat)[18][0]=-0.931709;((float**)syn2->mat)[18][1]=0.891448;((float**)syn2->mat)[18][2]=-0.557546;((float**)syn2->mat)[18][3]=0.769618;((float**)syn2->mat)[19][0]=-0.071873;((float**)syn2->mat)[19][1]=0.460865;((float**)syn2->mat)[19][2]=-0.486254;((float**)syn2->mat)[19][3]=0.522590;((float**)syn2->mat)[20][0]=4.513178;((float**)syn2->mat)[20][1]=-2.415314;((float**)syn2->mat)[20][2]=-2.969686;((float**)syn2->mat)[20][3]=-5.186864;((float**)syn2->mat)[21][0]=-0.692704;((float**)syn2->mat)[21][1]=3.054346;((float**)syn2->mat)[21][2]=-6.758296;((float**)syn2->mat)[21][3]=-4.078369;((float**)syn2->mat)[22][0]=-3.089108;((float**)syn2->mat)[22][1]=0.035387;((float**)syn2->mat)[22][2]=-1.068687;((float**)syn2->mat)[22][3]=2.186001;((float**)syn2->mat)[23][0]=-4.137681;((float**)syn2->mat)[23][1]=-5.440299;((float**)syn2->mat)[23][2]=2.573318;((float**)syn2->mat)[23][3]=-0.432908;((float**)syn2->mat)[24][0]=3.940788;((float**)syn2->mat)[24][1]=-1.592905;((float**)syn2->mat)[24][2]=-1.644789;((float**)syn2->mat)[24][3]=-4.190807;((float**)syn2->mat)[25][0]=0.318664;((float**)syn2->mat)[25][1]=-1.258730;((float**)syn2->mat)[25][2]=0.587427;((float**)syn2->mat)[25][3]=0.785923;((float**)syn2->mat)[26][0]=-5.919216;((float**)syn2->mat)[26][1]=3.507565;((float**)syn2->mat)[26][2]=-1.892189;((float**)syn2->mat)[26][3]=-5.239147;((float**)syn2->mat)[27][0]=-0.380133;((float**)syn2->mat)[27][1]=1.446767;((float**)syn2->mat)[27][2]=-1.293356;((float**)syn2->mat)[27][3]=-2.129871;((float**)syn2->mat)[28][0]=6.050577;((float**)syn2->mat)[28][1]=-3.905009;((float**)syn2->mat)[28][2]=-3.313271;((float**)syn2->mat)[28][3]=-5.578617;((float**)syn2->mat)[29][0]=-0.007118;((float**)syn2->mat)[29][1]=0.937712;((float**)syn2->mat)[29][2]=0.217924;((float**)syn2->mat)[29][3]=-0.538679;
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
	((float**)input->mat)[0][0] = 0.450667;	((float**)input->mat)[0][1] = 0.626667;	((float**)input->mat)[0][2] = 0.533333;	((float**)input->mat)[0][3] = 0.572;	((float**)input->mat)[0][4] = 0.558667;	((float**)input->mat)[0][5] = 0.885333;	((float**)input->mat)[0][6] = 0.429333;	((float**)input->mat)[0][7] = 0.8;	((float**)input->mat)[0][8] = 0.370667;	((float**)input->mat)[0][9] = 0.584;	((float**)input->mat)[0][10] = 0.502667;	((float**)input->mat)[0][11] = 0.544;	((float**)input->mat)[0][12] = 0.614667;	((float**)input->mat)[0][13] = 0.769333;	((float**)input->mat)[0][14] = 0.505333;	((float**)input->mat)[0][15] = 0.864;	((float**)input->mat)[0][16] = 0.425333;	((float**)input->mat)[0][17] = 0.649333;	((float**)input->mat)[0][18] = 0.444;	((float**)input->mat)[0][19] = 0.574667;
}
void fill_matrix_input_4(){
//(1, 20)
  ((float**)input->mat)[0][0] = 0.5;  ((float**)input->mat)[0][1] = 0.716;  ((float**)input->mat)[0][2] = 0.582667; ((float**)input->mat)[0][3] = 0.708;  ((float**)input->mat)[0][4] = 0.5;  ((float**)input->mat)[0][5] = 0.641333; ((float**)input->mat)[0][6] = 0.414667; ((float**)input->mat)[0][7] = 0.616;  ((float**)input->mat)[0][8] = 0.414667; ((float**)input->mat)[0][9] = 0.585333; ((float**)input->mat)[0][10] = 0.44;  ((float**)input->mat)[0][11] = 0.686667;  ((float**)input->mat)[0][12] = 0.518667;  ((float**)input->mat)[0][13] = 0.710667;  ((float**)input->mat)[0][14] = 0.541333;  ((float**)input->mat)[0][15] = 0.693333;  ((float**)input->mat)[0][16] = 0.576; ((float**)input->mat)[0][17] = 0.658667;  ((float**)input->mat)[0][18] = 0.402667;  ((float**)input->mat)[0][19] = 0.734667;
}
void fill_static_matrix_inputs(){
//(102, 20)
  #ifdef TRAINING
	inputs[0][0] = 0.433333;	inputs[0][1] = 0.596;	inputs[0][2] = 0.558667;	inputs[0][3] = 0.589333;	inputs[0][4] = 0.576;	inputs[0][5] = 0.738667;	inputs[0][6] = 0.561333;	inputs[0][7] = 0.802667;	inputs[0][8] = 0.461333;	inputs[0][9] = 0.805333;	inputs[0][10] = 0.425333;	inputs[0][11] = 0.622667;	inputs[0][12] = 0.432;	inputs[0][13] = 0.578667;	inputs[0][14] = 0.46;	inputs[0][15] = 0.573333;	inputs[0][16] = 0.461333;	inputs[0][17] = 0.661333;	inputs[0][18] = 0.493333;	inputs[0][19] = 0.821333;	inputs[1][0] = 0.392;	inputs[1][1] = 0.653333;	inputs[1][2] = 0.432;	inputs[1][3] = 0.676;	inputs[1][4] = 0.544;	inputs[1][5] = 0.702667;	inputs[1][6] = 0.564;	inputs[1][7] = 0.678667;	inputs[1][8] = 0.542667;	inputs[1][9] = 0.689333;	inputs[1][10] = 0.548;	inputs[1][11] = 0.676;	inputs[1][12] = 0.518667;	inputs[1][13] = 0.682667;	inputs[1][14] = 0.42;	inputs[1][15] = 0.66;	inputs[1][16] = 0.4;	inputs[1][17] = 0.633333;	inputs[1][18] = 0.473333;	inputs[1][19] = 0.654667;	inputs[2][0] = 0.485333;	inputs[2][1] = 0.685333;	inputs[2][2] = 0.56;	inputs[2][3] = 0.749333;	inputs[2][4] = 0.496;	inputs[2][5] = 0.702667;	inputs[2][6] = 0.434667;	inputs[2][7] = 0.566667;	inputs[2][8] = 0.396;	inputs[2][9] = 0.546667;	inputs[2][10] = 0.457333;	inputs[2][11] = 0.710667;	inputs[2][12] = 0.553333;	inputs[2][13] = 0.742667;	inputs[2][14] = 0.616;	inputs[2][15] = 0.677333;	inputs[2][16] = 0.286667;	inputs[2][17] = 0.685333;	inputs[2][18] = 0.454667;	inputs[2][19] = 0.677333;	inputs[3][0] = 0.470667;	inputs[3][1] = 0.666667;	inputs[3][2] = 0.457333;	inputs[3][3] = 0.582667;	inputs[3][4] = 0.508;	inputs[3][5] = 0.648;	inputs[3][6] = 0.585333;	inputs[3][7] = 0.705333;	inputs[3][8] = 0.525333;	inputs[3][9] = 0.834667;	inputs[3][10] = 0.406667;	inputs[3][11] = 0.712;	inputs[3][12] = 0.406667;	inputs[3][13] = 0.556;	inputs[3][14] = 0.564;	inputs[3][15] = 0.613333;	inputs[3][16] = 0.562667;	inputs[3][17] = 0.725333;	inputs[3][18] = 0.464;	inputs[3][19] = 0.789333;	inputs[4][0] = 0.46;	inputs[4][1] = 0.641333;	inputs[4][2] = 0.488;	inputs[4][3] = 0.573333;	inputs[4][4] = 0.565333;	inputs[4][5] = 0.704;	inputs[4][6] = 0.586667;	inputs[4][7] = 0.828;	inputs[4][8] = 0.402667;	inputs[4][9] = 0.769333;	inputs[4][10] = 0.376;	inputs[4][11] = 0.54;	inputs[4][12] = 0.485333;	inputs[4][13] = 0.604;	inputs[4][14] = 0.606667;	inputs[4][15] = 0.678667;	inputs[4][16] = 0.502667;	inputs[4][17] = 0.914667;	inputs[4][18] = 0.44;	inputs[4][19] = 0.734667;	inputs[5][0] = 0.558667;	inputs[5][1] = 0.736;	inputs[5][2] = 0.530667;	inputs[5][3] = 0.685333;	inputs[5][4] = 0.432;	inputs[5][5] = 0.592;	inputs[5][6] = 0.442667;	inputs[5][7] = 0.570667;	inputs[5][8] = 0.42;	inputs[5][9] = 0.694667;	inputs[5][10] = 0.504;	inputs[5][11] = 0.704;	inputs[5][12] = 0.541333;	inputs[5][13] = 0.696;	inputs[5][14] = 0.582667;	inputs[5][15] = 0.686667;	inputs[5][16] = 0.404;	inputs[5][17] = 0.708;	inputs[5][18] = 0.485333;	inputs[5][19] = 0.684;	inputs[6][0] = 0.392;	inputs[6][1] = 0.645333;	inputs[6][2] = 0.373333;	inputs[6][3] = 0.652;	inputs[6][4] = 0.533333;	inputs[6][5] = 0.704;	inputs[6][6] = 0.565333;	inputs[6][7] = 0.657333;	inputs[6][8] = 0.530667;	inputs[6][9] = 0.674667;	inputs[6][10] = 0.584;	inputs[6][11] = 0.676;	inputs[6][12] = 0.541333;	inputs[6][13] = 0.648;	inputs[6][14] = 0.410667;	inputs[6][15] = 0.681333;	inputs[6][16] = 0.488;	inputs[6][17] = 0.693333;	inputs[6][18] = 0.517333;	inputs[6][19] = 0.685333;	inputs[7][0] = 0.490667;	inputs[7][1] = 0.656;	inputs[7][2] = 0.605333;	inputs[7][3] = 0.633333;	inputs[7][4] = 0.493333;	inputs[7][5] = 0.718667;	inputs[7][6] = 0.369333;	inputs[7][7] = 0.774667;	inputs[7][8] = 0.48;	inputs[7][9] = 0.618667;	inputs[7][10] = 0.517333;	inputs[7][11] = 0.558667;	inputs[7][12] = 0.453333;	inputs[7][13] = 0.616;	inputs[7][14] = 0.478667;	inputs[7][15] = 0.676;	inputs[7][16] = 0.532;	inputs[7][17] = 0.688;	inputs[7][18] = 0.684;	inputs[7][19] = 0.677333;	inputs[8][0] = 0.457333;	inputs[8][1] = 0.62;	inputs[8][2] = 0.530667;	inputs[8][3] = 0.589333;	inputs[8][4] = 0.549333;	inputs[8][5] = 0.725333;	inputs[8][6] = 0.481333;	inputs[8][7] = 0.821333;	inputs[8][8] = 0.414667;	inputs[8][9] = 0.685333;	inputs[8][10] = 0.428;	inputs[8][11] = 0.533333;	inputs[8][12] = 0.6;	inputs[8][13] = 0.626667;	inputs[8][14] = 0.529333;	inputs[8][15] = 0.808;	inputs[8][16] = 0.469333;	inputs[8][17] = 0.909333;	inputs[8][18] = 0.437333;	inputs[8][19] = 0.576;	inputs[9][0] = 0.477333;	inputs[9][1] = 0.638667;	inputs[9][2] = 0.530667;	inputs[9][3] = 0.589333;	inputs[9][4] = 0.601333;	inputs[9][5] = 0.745333;	inputs[9][6] = 0.493333;	inputs[9][7] = 0.884;	inputs[9][8] = 0.386667;	inputs[9][9] = 0.633333;	inputs[9][10] = 0.477333;	inputs[9][11] = 0.526667;	inputs[9][12] = 0.625333;	inputs[9][13] = 0.616;	inputs[9][14] = 0.492;	inputs[9][15] = 0.765333;	inputs[9][16] = 0.424;	inputs[9][17] = 0.742667;	inputs[9][18] = 0.445333;	inputs[9][19] = 0.678667;	inputs[10][0] = 0.54;	inputs[10][1] = 0.718667;	inputs[10][2] = 0.556;	inputs[10][3] = 0.66;	inputs[10][4] = 0.418667;	inputs[10][5] = 0.62;	inputs[10][6] = 0.428;	inputs[10][7] = 0.604;	inputs[10][8] = 0.417333;	inputs[10][9] = 0.650667;	inputs[10][10] = 0.492;	inputs[10][11] = 0.697333;	inputs[10][12] = 0.536;	inputs[10][13] = 0.717333;	inputs[10][14] = 0.581333;	inputs[10][15] = 0.701333;	inputs[10][16] = 0.464;	inputs[10][17] = 0.722667;	inputs[10][18] = 0.437333;	inputs[10][19] = 0.690667;	inputs[11][0] = 0.517333;	inputs[11][1] = 0.701333;	inputs[11][2] = 0.578667;	inputs[11][3] = 0.693333;	inputs[11][4] = 0.428;	inputs[11][5] = 0.658667;	inputs[11][6] = 0.448;	inputs[11][7] = 0.552;	inputs[11][8] = 0.428;	inputs[11][9] = 0.614667;	inputs[11][10] = 0.498667;	inputs[11][11] = 0.712;	inputs[11][12] = 0.573333;	inputs[11][13] = 0.701333;	inputs[11][14] = 0.502667;	inputs[11][15] = 0.7;	inputs[11][16] = 0.473333;	inputs[11][17] = 0.688;	inputs[11][18] = 0.498667;	inputs[11][19] = 0.697333;	inputs[12][0] = 0.462667;	inputs[12][1] = 0.650667;	inputs[12][2] = 0.468;	inputs[12][3] = 0.552;	inputs[12][4] = 0.588;	inputs[12][5] = 0.681333;	inputs[12][6] = 0.573333;	inputs[12][7] = 0.796;	inputs[12][8] = 0.408;	inputs[12][9] = 0.746667;	inputs[12][10] = 0.376;	inputs[12][11] = 0.552;	inputs[12][12] = 0.493333;	inputs[12][13] = 0.597333;	inputs[12][14] = 0.562667;	inputs[12][15] = 0.688;	inputs[12][16] = 0.568;	inputs[12][17] = 0.761333;	inputs[12][18] = 0.489333;	inputs[12][19] = 0.846667;	inputs[13][0] = 0.382667;	inputs[13][1] = 0.648;	inputs[13][2] = 0.321333;	inputs[13][3] = 0.749333;	inputs[13][4] = 0.616;	inputs[13][5] = 0.772;	inputs[13][6] = 0.641333;	inputs[13][7] = 0.713333;	inputs[13][8] = 0.570667;	inputs[13][9] = 0.674667;	inputs[13][10] = 0.445333;	inputs[13][11] = 0.712;	inputs[13][12] = 0.329333;	inputs[13][13] = 0.746667;	inputs[13][14] = 0.449333;	inputs[13][15] = 0.697333;	inputs[13][16] = 0.477333;	inputs[13][17] = 0.685333;	inputs[13][18] = 0.494667;	inputs[13][19] = 0.690667;	inputs[14][0] = 0.521333;	inputs[14][1] = 0.698667;	inputs[14][2] = 0.589333;	inputs[14][3] = 0.741333;	inputs[14][4] = 0.490667;	inputs[14][5] = 0.672;	inputs[14][6] = 0.425333;	inputs[14][7] = 0.542667;	inputs[14][8] = 0.418667;	inputs[14][9] = 0.594667;	inputs[14][10] = 0.468;	inputs[14][11] = 0.757333;	inputs[14][12] = 0.536;	inputs[14][13] = 0.726667;	inputs[14][14] = 0.601333;	inputs[14][15] = 0.674667;	inputs[14][16] = 0.474667;	inputs[14][17] = 0.732;	inputs[14][18] = 0.477333;	inputs[14][19] = 0.697333;	inputs[15][0] = 0.569333;	inputs[15][1] = 0.789333;	inputs[15][2] = 0.465333;	inputs[15][3] = 0.726667;	inputs[15][4] = 0.538667;	inputs[15][5] = 0.562667;	inputs[15][6] = 0.445333;	inputs[15][7] = 0.592;	inputs[15][8] = 0.376;	inputs[15][9] = 0.629333;	inputs[15][10] = 0.478667;	inputs[15][11] = 0.722667;	inputs[15][12] = 0.581333;	inputs[15][13] = 0.724;	inputs[15][14] = 0.596;	inputs[15][15] = 0.684;	inputs[15][16] = 0.416;	inputs[15][17] = 0.708;	inputs[15][18] = 0.409333;	inputs[15][19] = 0.678667;	inputs[16][0] = 0.428;	inputs[16][1] = 0.674667;	inputs[16][2] = 0.474667;	inputs[16][3] = 0.684;	inputs[16][4] = 0.597333;	inputs[16][5] = 0.676;	inputs[16][6] = 0.562667;	inputs[16][7] = 0.684;	inputs[16][8] = 0.552;	inputs[16][9] = 0.665333;	inputs[16][10] = 0.606667;	inputs[16][11] = 0.696;	inputs[16][12] = 0.494667;	inputs[16][13] = 0.674667;	inputs[16][14] = 0.384;	inputs[16][15] = 0.681333;	inputs[16][16] = 0.481333;	inputs[16][17] = 0.674667;	inputs[16][18] = 0.493333;	inputs[16][19] = 0.668;	inputs[17][0] = 0.476;	inputs[17][1] = 0.672;	inputs[17][2] = 0.457333;	inputs[17][3] = 0.565333;	inputs[17][4] = 0.541333;	inputs[17][5] = 0.722667;	inputs[17][6] = 0.534667;	inputs[17][7] = 0.825333;	inputs[17][8] = 0.434667;	inputs[17][9] = 0.801333;	inputs[17][10] = 0.384;	inputs[17][11] = 0.602667;	inputs[17][12] = 0.48;	inputs[17][13] = 0.589333;	inputs[17][14] = 0.593333;	inputs[17][15] = 0.64;	inputs[17][16] = 0.546667;	inputs[17][17] = 0.8;	inputs[17][18] = 0.438667;	inputs[17][19] = 0.829333;	inputs[18][0] = 0.390667;	inputs[18][1] = 0.674667;	inputs[18][2] = 0.461333;	inputs[18][3] = 0.714667;	inputs[18][4] = 0.641333;	inputs[18][5] = 0.705333;	inputs[18][6] = 0.605333;	inputs[18][7] = 0.696;	inputs[18][8] = 0.54;	inputs[18][9] = 0.689333;	inputs[18][10] = 0.570667;	inputs[18][11] = 0.653333;	inputs[18][12] = 0.54;	inputs[18][13] = 0.693333;	inputs[18][14] = 0.413333;	inputs[18][15] = 0.694667;	inputs[18][16] = 0.478667;	inputs[18][17] = 0.665333;	inputs[18][18] = 0.485333;	inputs[18][19] = 0.664;	inputs[19][0] = 0.450667;	inputs[19][1] = 0.550667;	inputs[19][2] = 0.546667;	inputs[19][3] = 0.566667;	inputs[19][4] = 0.533333;	inputs[19][5] = 0.816;	inputs[19][6] = 0.517333;	inputs[19][7] = 0.857333;	inputs[19][8] = 0.46;	inputs[19][9] = 0.704;	inputs[19][10] = 0.424;	inputs[19][11] = 0.598667;	inputs[19][12] = 0.464;	inputs[19][13] = 0.573333;	inputs[19][14] = 0.484;	inputs[19][15] = 0.556;	inputs[19][16] = 0.534667;	inputs[19][17] = 0.596;	inputs[19][18] = 0.476;	inputs[19][19] = 0.753333;	inputs[20][0] = 0.474667;	inputs[20][1] = 0.646667;	inputs[20][2] = 0.488;	inputs[20][3] = 0.582667;	inputs[20][4] = 0.581333;	inputs[20][5] = 0.684;	inputs[20][6] = 0.526667;	inputs[20][7] = 0.753333;	inputs[20][8] = 0.445333;	inputs[20][9] = 0.733333;	inputs[20][10] = 0.36;	inputs[20][11] = 0.610667;	inputs[20][12] = 0.450667;	inputs[20][13] = 0.572;	inputs[20][14] = 0.586667;	inputs[20][15] = 0.629333;	inputs[20][16] = 0.512;	inputs[20][17] = 0.862667;	inputs[20][18] = 0.449333;	inputs[20][19] = 0.730667;	inputs[21][0] = 0.532;	inputs[21][1] = 0.682667;	inputs[21][2] = 0.456;	inputs[21][3] = 0.612;	inputs[21][4] = 0.424;	inputs[21][5] = 0.572;	inputs[21][6] = 0.418667;	inputs[21][7] = 0.625333;	inputs[21][8] = 0.477333;	inputs[21][9] = 0.732;	inputs[21][10] = 0.578667;	inputs[21][11] = 0.698667;	inputs[21][12] = 0.610667;	inputs[21][13] = 0.669333;	inputs[21][14] = 0.388;	inputs[21][15] = 0.721333;	inputs[21][16] = 0.476;	inputs[21][17] = 0.669333;	inputs[21][18] = 0.506667;	inputs[21][19] = 0.692;	inputs[22][0] = 0.448;	inputs[22][1] = 0.568;	inputs[22][2] = 0.538667;	inputs[22][3] = 0.697333;	inputs[22][4] = 0.557333;	inputs[22][5] = 0.786667;	inputs[22][6] = 0.464;	inputs[22][7] = 0.778667;	inputs[22][8] = 0.441333;	inputs[22][9] = 0.610667;	inputs[22][10] = 0.478667;	inputs[22][11] = 0.585333;	inputs[22][12] = 0.488;	inputs[22][13] = 0.569333;	inputs[22][14] = 0.501333;	inputs[22][15] = 0.677333;	inputs[22][16] = 0.485333;	inputs[22][17] = 0.722667;	inputs[22][18] = 0.501333;	inputs[22][19] = 0.670667;	inputs[23][0] = 0.554667;	inputs[23][1] = 0.724;	inputs[23][2] = 0.512;	inputs[23][3] = 0.690667;	inputs[23][4] = 0.393333;	inputs[23][5] = 0.617333;	inputs[23][6] = 0.410667;	inputs[23][7] = 0.582667;	inputs[23][8] = 0.424;	inputs[23][9] = 0.666667;	inputs[23][10] = 0.549333;	inputs[23][11] = 0.726667;	inputs[23][12] = 0.577333;	inputs[23][13] = 0.689333;	inputs[23][14] = 0.56;	inputs[23][15] = 0.678667;	inputs[23][16] = 0.416;	inputs[23][17] = 0.690667;	inputs[23][18] = 0.489333;	inputs[23][19] = 0.68;	inputs[24][0] = 0.478667;	inputs[24][1] = 0.641333;	inputs[24][2] = 0.497333;	inputs[24][3] = 0.597333;	inputs[24][4] = 0.570667;	inputs[24][5] = 0.690667;	inputs[24][6] = 0.481333;	inputs[24][7] = 0.804;	inputs[24][8] = 0.410667;	inputs[24][9] = 0.728;	inputs[24][10] = 0.4;	inputs[24][11] = 0.554667;	inputs[24][12] = 0.498667;	inputs[24][13] = 0.581333;	inputs[24][14] = 0.586667;	inputs[24][15] = 0.682667;	inputs[24][16] = 0.504;	inputs[24][17] = 0.802667;	inputs[24][18] = 0.432;	inputs[24][19] = 0.802667;	inputs[25][0] = 0.46;	inputs[25][1] = 0.629333;	inputs[25][2] = 0.529333;	inputs[25][3] = 0.598667;	inputs[25][4] = 0.590667;	inputs[25][5] = 0.732;	inputs[25][6] = 0.493333;	inputs[25][7] = 0.838667;	inputs[25][8] = 0.394667;	inputs[25][9] = 0.634667;	inputs[25][10] = 0.436;	inputs[25][11] = 0.572;	inputs[25][12] = 0.534667;	inputs[25][13] = 0.650667;	inputs[25][14] = 0.562667;	inputs[25][15] = 0.72;	inputs[25][16] = 0.508;	inputs[25][17] = 0.786667;	inputs[25][18] = 0.445333;	inputs[25][19] = 0.762667;	inputs[26][0] = 0.397333;	inputs[26][1] = 0.617333;	inputs[26][2] = 0.445333;	inputs[26][3] = 0.52;	inputs[26][4] = 0.529333;	inputs[26][5] = 0.710667;	inputs[26][6] = 0.581333;	inputs[26][7] = 0.748;	inputs[26][8] = 0.457333;	inputs[26][9] = 0.786667;	inputs[26][10] = 0.425333;	inputs[26][11] = 0.692;	inputs[26][12] = 0.473333;	inputs[26][13] = 0.589333;	inputs[26][14] = 0.472;	inputs[26][15] = 0.546667;	inputs[26][16] = 0.577333;	inputs[26][17] = 0.765333;	inputs[26][18] = 0.492;	inputs[26][19] = 0.76;	inputs[27][0] = 0.476;	inputs[27][1] = 0.702667;	inputs[27][2] = 0.409333;	inputs[27][3] = 0.574667;	inputs[27][4] = 0.518667;	inputs[27][5] = 0.554667;	inputs[27][6] = 0.606667;	inputs[27][7] = 0.677333;	inputs[27][8] = 0.58;	inputs[27][9] = 0.748;	inputs[27][10] = 0.498667;	inputs[27][11] = 0.781333;	inputs[27][12] = 0.436;	inputs[27][13] = 0.722667;	inputs[27][14] = 0.438667;	inputs[27][15] = 0.577333;	inputs[27][16] = 0.445333;	inputs[27][17] = 0.589333;	inputs[27][18] = 0.462667;	inputs[27][19] = 0.566667;	inputs[28][0] = 0.506667;	inputs[28][1] = 0.704;	inputs[28][2] = 0.557333;	inputs[28][3] = 0.745333;	inputs[28][4] = 0.474667;	inputs[28][5] = 0.669333;	inputs[28][6] = 0.454667;	inputs[28][7] = 0.628;	inputs[28][8] = 0.393333;	inputs[28][9] = 0.581333;	inputs[28][10] = 0.441333;	inputs[28][11] = 0.681333;	inputs[28][12] = 0.542667;	inputs[28][13] = 0.726667;	inputs[28][14] = 0.690667;	inputs[28][15] = 0.688;	inputs[28][16] = 0.322667;	inputs[28][17] = 0.776;	inputs[28][18] = 0.444;	inputs[28][19] = 0.714667;	inputs[29][0] = 0.429333;	inputs[29][1] = 0.666667;	inputs[29][2] = 0.462667;	inputs[29][3] = 0.549333;	inputs[29][4] = 0.530667;	inputs[29][5] = 0.572;	inputs[29][6] = 0.572;	inputs[29][7] = 0.76;	inputs[29][8] = 0.518667;	inputs[29][9] = 0.802667;	inputs[29][10] = 0.473333;	inputs[29][11] = 0.724;	inputs[29][12] = 0.412;	inputs[29][13] = 0.570667;	inputs[29][14] = 0.454667;	inputs[29][15] = 0.582667;	inputs[29][16] = 0.430667;	inputs[29][17] = 0.561333;	inputs[29][18] = 0.481333;	inputs[29][19] = 0.769333;	inputs[30][0] = 0.550667;	inputs[30][1] = 0.717333;	inputs[30][2] = 0.512;	inputs[30][3] = 0.696;	inputs[30][4] = 0.452;	inputs[30][5] = 0.609333;	inputs[30][6] = 0.434667;	inputs[30][7] = 0.588;	inputs[30][8] = 0.42;	inputs[30][9] = 0.7;	inputs[30][10] = 0.532;	inputs[30][11] = 0.72;	inputs[30][12] = 0.593333;	inputs[30][13] = 0.701333;	inputs[30][14] = 0.422667;	inputs[30][15] = 0.730667;	inputs[30][16] = 0.46;	inputs[30][17] = 0.7;	inputs[30][18] = 0.506667;	inputs[30][19] = 0.7;	inputs[31][0] = 0.553333;	inputs[31][1] = 0.725333;	inputs[31][2] = 0.513333;	inputs[31][3] = 0.721333;	inputs[31][4] = 0.445333;	inputs[31][5] = 0.594667;	inputs[31][6] = 0.432;	inputs[31][7] = 0.529333;	inputs[31][8] = 0.442667;	inputs[31][9] = 0.674667;	inputs[31][10] = 0.525333;	inputs[31][11] = 0.733333;	inputs[31][12] = 0.562667;	inputs[31][13] = 0.704;	inputs[31][14] = 0.468;	inputs[31][15] = 0.676;	inputs[31][16] = 0.458667;	inputs[31][17] = 0.674667;	inputs[31][18] = 0.502667;	inputs[31][19] = 0.692;	inputs[32][0] = 0.437333;	inputs[32][1] = 0.753333;	inputs[32][2] = 0.409333;	inputs[32][3] = 0.518667;	inputs[32][4] = 0.473333;	inputs[32][5] = 0.586667;	inputs[32][6] = 0.584;	inputs[32][7] = 0.741333;	inputs[32][8] = 0.514667;	inputs[32][9] = 0.886667;	inputs[32][10] = 0.412;	inputs[32][11] = 0.784;	inputs[32][12] = 0.444;	inputs[32][13] = 0.618667;	inputs[32][14] = 0.492;	inputs[32][15] = 0.557333;	inputs[32][16] = 0.425333;	inputs[32][17] = 0.565333;	inputs[32][18] = 0.518667;	inputs[32][19] = 0.621333;	inputs[33][0] = 0.446667;	inputs[33][1] = 0.653333;	inputs[33][2] = 0.448;	inputs[33][3] = 0.706667;	inputs[33][4] = 0.544;	inputs[33][5] = 0.717333;	inputs[33][6] = 0.534667;	inputs[33][7] = 0.698667;	inputs[33][8] = 0.537333;	inputs[33][9] = 0.644;	inputs[33][10] = 0.598667;	inputs[33][11] = 0.677333;	inputs[33][12] = 0.628;	inputs[33][13] = 0.684;	inputs[33][14] = 0.3;	inputs[33][15] = 0.764;	inputs[33][16] = 0.46;	inputs[33][17] = 0.704;	inputs[33][18] = 0.485333;	inputs[33][19] = 0.676;	inputs[34][0] = 0.412;	inputs[34][1] = 0.662667;	inputs[34][2] = 0.488;	inputs[34][3] = 0.556;	inputs[34][4] = 0.54;	inputs[34][5] = 0.606667;	inputs[34][6] = 0.58;	inputs[34][7] = 0.722667;	inputs[34][8] = 0.533333;	inputs[34][9] = 0.825333;	inputs[34][10] = 0.449333;	inputs[34][11] = 0.758667;	inputs[34][12] = 0.393333;	inputs[34][13] = 0.642667;	inputs[34][14] = 0.458667;	inputs[34][15] = 0.588;	inputs[34][16] = 0.477333;	inputs[34][17] = 0.594667;	inputs[34][18] = 0.5;	inputs[34][19] = 0.621333;	inputs[35][0] = 0.432;	inputs[35][1] = 0.608;	inputs[35][2] = 0.333333;	inputs[35][3] = 0.692;	inputs[35][4] = 0.458667;	inputs[35][5] = 0.713333;	inputs[35][6] = 0.626667;	inputs[35][7] = 0.662667;	inputs[35][8] = 0.565333;	inputs[35][9] = 0.718667;	inputs[35][10] = 0.554667;	inputs[35][11] = 0.688;	inputs[35][12] = 0.616;	inputs[35][13] = 0.662667;	inputs[35][14] = 0.589333;	inputs[35][15] = 0.658667;	inputs[35][16] = 0.381333;	inputs[35][17] = 0.702667;	inputs[35][18] = 0.461333;	inputs[35][19] = 0.684;	inputs[36][0] = 0.477333;	inputs[36][1] = 0.632;	inputs[36][2] = 0.481333;	inputs[36][3] = 0.594667;	inputs[36][4] = 0.581333;	inputs[36][5] = 0.717333;	inputs[36][6] = 0.506667;	inputs[36][7] = 0.828;	inputs[36][8] = 0.396;	inputs[36][9] = 0.706667;	inputs[36][10] = 0.416;	inputs[36][11] = 0.537333;	inputs[36][12] = 0.562667;	inputs[36][13] = 0.637333;	inputs[36][14] = 0.598667;	inputs[36][15] = 0.705333;	inputs[36][16] = 0.458667;	inputs[36][17] = 0.838667;	inputs[36][18] = 0.433333;	inputs[36][19] = 0.724;	inputs[37][0] = 0.477333;	inputs[37][1] = 0.649333;	inputs[37][2] = 0.522667;	inputs[37][3] = 0.589333;	inputs[37][4] = 0.530667;	inputs[37][5] = 0.742667;	inputs[37][6] = 0.504;	inputs[37][7] = 0.848;	inputs[37][8] = 0.424;	inputs[37][9] = 0.685333;	inputs[37][10] = 0.430667;	inputs[37][11] = 0.554667;	inputs[37][12] = 0.570667;	inputs[37][13] = 0.628;	inputs[37][14] = 0.56;	inputs[37][15] = 0.732;	inputs[37][16] = 0.488;	inputs[37][17] = 0.845333;	inputs[37][18] = 0.452;	inputs[37][19] = 0.758667;	inputs[38][0] = 0.397333;	inputs[38][1] = 0.538667;	inputs[38][2] = 0.561333;	inputs[38][3] = 0.664;	inputs[38][4] = 0.598667;	inputs[38][5] = 0.817333;	inputs[38][6] = 0.474667;	inputs[38][7] = 0.822667;	inputs[38][8] = 0.401333;	inputs[38][9] = 0.608;	inputs[38][10] = 0.438667;	inputs[38][11] = 0.554667;	inputs[38][12] = 0.469333;	inputs[38][13] = 0.609333;	inputs[38][14] = 0.522667;	inputs[38][15] = 0.677333;	inputs[38][16] = 0.501333;	inputs[38][17] = 0.745333;	inputs[38][18] = 0.504;	inputs[38][19] = 0.690667;	inputs[39][0] = 0.476;	inputs[39][1] = 0.632;	inputs[39][2] = 0.497333;	inputs[39][3] = 0.562667;	inputs[39][4] = 0.545333;	inputs[39][5] = 0.808;	inputs[39][6] = 0.508;	inputs[39][7] = 0.808;	inputs[39][8] = 0.418667;	inputs[39][9] = 0.64;	inputs[39][10] = 0.44;	inputs[39][11] = 0.530667;	inputs[39][12] = 0.558667;	inputs[39][13] = 0.624;	inputs[39][14] = 0.552;	inputs[39][15] = 0.784;	inputs[39][16] = 0.498667;	inputs[39][17] = 0.814667;	inputs[39][18] = 0.453333;	inputs[39][19] = 0.641333;	inputs[40][0] = 0.554667;	inputs[40][1] = 0.628;	inputs[40][2] = 0.624;	inputs[40][3] = 0.698667;	inputs[40][4] = 0.556;	inputs[40][5] = 0.772;	inputs[40][6] = 0.478667;	inputs[40][7] = 0.822667;	inputs[40][8] = 0.409333;	inputs[40][9] = 0.669333;	inputs[40][10] = 0.448;	inputs[40][11] = 0.586667;	inputs[40][12] = 0.493333;	inputs[40][13] = 0.593333;	inputs[40][14] = 0.433333;	inputs[40][15] = 0.465333;	inputs[40][16] = 0.449333;	inputs[40][17] = 0.853333;	inputs[40][18] = 0.492;	inputs[40][19] = 0.718667;	inputs[41][0] = 0.392;	inputs[41][1] = 0.632;	inputs[41][2] = 0.394667;	inputs[41][3] = 0.694667;	inputs[41][4] = 0.490667;	inputs[41][5] = 0.725333;	inputs[41][6] = 0.616;	inputs[41][7] = 0.726667;	inputs[41][8] = 0.602667;	inputs[41][9] = 0.694667;	inputs[41][10] = 0.573333;	inputs[41][11] = 0.669333;	inputs[41][12] = 0.582667;	inputs[41][13] = 0.662667;	inputs[41][14] = 0.417333;	inputs[41][15] = 0.657333;	inputs[41][16] = 0.421333;	inputs[41][17] = 0.645333;	inputs[41][18] = 0.485333;	inputs[41][19] = 0.664;	inputs[42][0] = 0.4;	inputs[42][1] = 0.705333;	inputs[42][2] = 0.630667;	inputs[42][3] = 0.722667;	inputs[42][4] = 0.669333;	inputs[42][5] = 0.697333;	inputs[42][6] = 0.576;	inputs[42][7] = 0.68;	inputs[42][8] = 0.569333;	inputs[42][9] = 0.642667;	inputs[42][10] = 0.658667;	inputs[42][11] = 0.662667;	inputs[42][12] = 0.317333;	inputs[42][13] = 0.688;	inputs[42][14] = 0.445333;	inputs[42][15] = 0.656;	inputs[42][16] = 0.484;	inputs[42][17] = 0.668;	inputs[42][18] = 0.490667;	inputs[42][19] = 0.674667;	inputs[43][0] = 0.466667;	inputs[43][1] = 0.652;	inputs[43][2] = 0.504;	inputs[43][3] = 0.568;	inputs[43][4] = 0.58;	inputs[43][5] = 0.632;	inputs[43][6] = 0.512;	inputs[43][7] = 0.838667;	inputs[43][8] = 0.416;	inputs[43][9] = 0.736;	inputs[43][10] = 0.388;	inputs[43][11] = 0.592;	inputs[43][12] = 0.545333;	inputs[43][13] = 0.609333;	inputs[43][14] = 0.625333;	inputs[43][15] = 0.754667;	inputs[43][16] = 0.52;	inputs[43][17] = 0.829333;	inputs[43][18] = 0.44;	inputs[43][19] = 0.7;	inputs[44][0] = 0.470667;	inputs[44][1] = 0.657333;	inputs[44][2] = 0.478667;	inputs[44][3] = 0.605333;	inputs[44][4] = 0.501333;	inputs[44][5] = 0.592;	inputs[44][6] = 0.570667;	inputs[44][7] = 0.754667;	inputs[44][8] = 0.472;	inputs[44][9] = 0.849333;	inputs[44][10] = 0.368;	inputs[44][11] = 0.658667;	inputs[44][12] = 0.437333;	inputs[44][13] = 0.493333;	inputs[44][14] = 0.596;	inputs[44][15] = 0.692;	inputs[44][16] = 0.530667;	inputs[44][17] = 0.772;	inputs[44][18] = 0.352;	inputs[44][19] = 0.88;	inputs[45][0] = 0.474667;	inputs[45][1] = 0.664;	inputs[45][2] = 0.517333;	inputs[45][3] = 0.597333;	inputs[45][4] = 0.524;	inputs[45][5] = 0.701333;	inputs[45][6] = 0.56;	inputs[45][7] = 0.717333;	inputs[45][8] = 0.496;	inputs[45][9] = 0.869333;	inputs[45][10] = 0.358667;	inputs[45][11] = 0.662667;	inputs[45][12] = 0.438667;	inputs[45][13] = 0.578667;	inputs[45][14] = 0.562667;	inputs[45][15] = 0.604;	inputs[45][16] = 0.556;	inputs[45][17] = 0.741333;	inputs[45][18] = 0.497333;	inputs[45][19] = 0.853333;	inputs[46][0] = 0.445333;	inputs[46][1] = 0.629333;	inputs[46][2] = 0.48;	inputs[46][3] = 0.616;	inputs[46][4] = 0.577333;	inputs[46][5] = 0.625333;	inputs[46][6] = 0.517333;	inputs[46][7] = 0.838667;	inputs[46][8] = 0.428;	inputs[46][9] = 0.76;	inputs[46][10] = 0.366667;	inputs[46][11] = 0.568;	inputs[46][12] = 0.474667;	inputs[46][13] = 0.553333;	inputs[46][14] = 0.586667;	inputs[46][15] = 0.741333;	inputs[46][16] = 0.504;	inputs[46][17] = 0.845333;	inputs[46][18] = 0.438667;	inputs[46][19] = 0.636;	inputs[47][0] = 0.496;	inputs[47][1] = 0.717333;	inputs[47][2] = 0.36;	inputs[47][3] = 0.666667;	inputs[47][4] = 0.476;	inputs[47][5] = 0.533333;	inputs[47][6] = 0.54;	inputs[47][7] = 0.638667;	inputs[47][8] = 0.604;	inputs[47][9] = 0.762667;	inputs[47][10] = 0.494667;	inputs[47][11] = 0.849333;	inputs[47][12] = 0.450667;	inputs[47][13] = 0.72;	inputs[47][14] = 0.453333;	inputs[47][15] = 0.558667;	inputs[47][16] = 0.481333;	inputs[47][17] = 0.582667;	inputs[47][18] = 0.470667;	inputs[47][19] = 0.589333;	inputs[48][0] = 0.445333;	inputs[48][1] = 0.648;	inputs[48][2] = 0.424;	inputs[48][3] = 0.7;	inputs[48][4] = 0.513333;	inputs[48][5] = 0.688;	inputs[48][6] = 0.552;	inputs[48][7] = 0.681333;	inputs[48][8] = 0.544;	inputs[48][9] = 0.674667;	inputs[48][10] = 0.516;	inputs[48][11] = 0.674667;	inputs[48][12] = 0.517333;	inputs[48][13] = 0.661333;	inputs[48][14] = 0.508;	inputs[48][15] = 0.666667;	inputs[48][16] = 0.433333;	inputs[48][17] = 0.657333;	inputs[48][18] = 0.437333;	inputs[48][19] = 0.674667;	inputs[49][0] = 0.454667;	inputs[49][1] = 0.554667;	inputs[49][2] = 0.526667;	inputs[49][3] = 0.632;	inputs[49][4] = 0.577333;	inputs[49][5] = 0.754667;	inputs[49][6] = 0.488;	inputs[49][7] = 0.82;	inputs[49][8] = 0.444;	inputs[49][9] = 0.792;	inputs[49][10] = 0.46;	inputs[49][11] = 0.654667;	inputs[49][12] = 0.496;	inputs[49][13] = 0.597333;	inputs[49][14] = 0.48;	inputs[49][15] = 0.586667;	inputs[49][16] = 0.472;	inputs[49][17] = 0.605333;	inputs[49][18] = 0.524;	inputs[49][19] = 0.764;	inputs[50][0] = 0.54;	inputs[50][1] = 0.716;	inputs[50][2] = 0.549333;	inputs[50][3] = 0.746667;	inputs[50][4] = 0.485333;	inputs[50][5] = 0.6;	inputs[50][6] = 0.416;	inputs[50][7] = 0.561333;	inputs[50][8] = 0.422667;	inputs[50][9] = 0.624;	inputs[50][10] = 0.513333;	inputs[50][11] = 0.733333;	inputs[50][12] = 0.598667;	inputs[50][13] = 0.714667;	inputs[50][14] = 0.476;	inputs[50][15] = 0.682667;	inputs[50][16] = 0.44;	inputs[50][17] = 0.710667;	inputs[50][18] = 0.488;	inputs[50][19] = 0.710667;	inputs[51][0] = 0.490667;	inputs[51][1] = 0.638667;	inputs[51][2] = 0.542667;	inputs[51][3] = 0.721333;	inputs[51][4] = 0.513333;	inputs[51][5] = 0.697333;	inputs[51][6] = 0.481333;	inputs[51][7] = 0.645333;	inputs[51][8] = 0.421333;	inputs[51][9] = 0.608;	inputs[51][10] = 0.444;	inputs[51][11] = 0.578667;	inputs[51][12] = 0.429333;	inputs[51][13] = 0.712;	inputs[51][14] = 0.556;	inputs[51][15] = 0.728;	inputs[51][16] = 0.617333;	inputs[51][17] = 0.682667;	inputs[51][18] = 0.426667;	inputs[51][19] = 0.690667;	inputs[52][0] = 0.478667;	inputs[52][1] = 0.642667;	inputs[52][2] = 0.484;	inputs[52][3] = 0.632;	inputs[52][4] = 0.490667;	inputs[52][5] = 0.612;	inputs[52][6] = 0.544;	inputs[52][7] = 0.604;	inputs[52][8] = 0.537333;	inputs[52][9] = 0.781333;	inputs[52][10] = 0.382667;	inputs[52][11] = 0.801333;	inputs[52][12] = 0.401333;	inputs[52][13] = 0.590667;	inputs[52][14] = 0.5;	inputs[52][15] = 0.569333;	inputs[52][16] = 0.622667;	inputs[52][17] = 0.681333;	inputs[52][18] = 0.521333;	inputs[52][19] = 0.848;	inputs[53][0] = 0.457333;	inputs[53][1] = 0.601333;	inputs[53][2] = 0.569333;	inputs[53][3] = 0.564;	inputs[53][4] = 0.576;	inputs[53][5] = 0.757333;	inputs[53][6] = 0.473333;	inputs[53][7] = 0.890667;	inputs[53][8] = 0.370667;	inputs[53][9] = 0.604;	inputs[53][10] = 0.445333;	inputs[53][11] = 0.525333;	inputs[53][12] = 0.493333;	inputs[53][13] = 0.604;	inputs[53][14] = 0.590667;	inputs[53][15] = 0.817333;	inputs[53][16] = 0.428;	inputs[53][17] = 0.8;	inputs[53][18] = 0.434667;	inputs[53][19] = 0.617333;	inputs[54][0] = 0.5;	inputs[54][1] = 0.685333;	inputs[54][2] = 0.576;	inputs[54][3] = 0.765333;	inputs[54][4] = 0.497333;	inputs[54][5] = 0.709333;	inputs[54][6] = 0.445333;	inputs[54][7] = 0.546667;	inputs[54][8] = 0.412;	inputs[54][9] = 0.524;	inputs[54][10] = 0.404;	inputs[54][11] = 0.752;	inputs[54][12] = 0.530667;	inputs[54][13] = 0.758667;	inputs[54][14] = 0.632;	inputs[54][15] = 0.684;	inputs[54][16] = 0.413333;	inputs[54][17] = 0.724;	inputs[54][18] = 0.434667;	inputs[54][19] = 0.688;	inputs[55][0] = 0.488;	inputs[55][1] = 0.685333;	inputs[55][2] = 0.54;	inputs[55][3] = 0.741333;	inputs[55][4] = 0.514667;	inputs[55][5] = 0.74;	inputs[55][6] = 0.502667;	inputs[55][7] = 0.568;	inputs[55][8] = 0.444;	inputs[55][9] = 0.542667;	inputs[55][10] = 0.378667;	inputs[55][11] = 0.625333;	inputs[55][12] = 0.492;	inputs[55][13] = 0.756;	inputs[55][14] = 0.617333;	inputs[55][15] = 0.712;	inputs[55][16] = 0.393333;	inputs[55][17] = 0.684;	inputs[55][18] = 0.408;	inputs[55][19] = 0.713333;	inputs[56][0] = 0.532;	inputs[56][1] = 0.72;	inputs[56][2] = 0.578667;	inputs[56][3] = 0.712;	inputs[56][4] = 0.44;	inputs[56][5] = 0.598667;	inputs[56][6] = 0.389333;	inputs[56][7] = 0.537333;	inputs[56][8] = 0.406667;	inputs[56][9] = 0.696;	inputs[56][10] = 0.541333;	inputs[56][11] = 0.732;	inputs[56][12] = 0.658667;	inputs[56][13] = 0.674667;	inputs[56][14] = 0.321333;	inputs[56][15] = 0.758667;	inputs[56][16] = 0.444;	inputs[56][17] = 0.701333;	inputs[56][18] = 0.390667;	inputs[56][19] = 0.725333;	inputs[57][0] = 0.548;	inputs[57][1] = 0.697333;	inputs[57][2] = 0.488;	inputs[57][3] = 0.644;	inputs[57][4] = 0.437333;	inputs[57][5] = 0.589333;	inputs[57][6] = 0.454667;	inputs[57][7] = 0.58;	inputs[57][8] = 0.46;	inputs[57][9] = 0.733333;	inputs[57][10] = 0.553333;	inputs[57][11] = 0.724;	inputs[57][12] = 0.572;	inputs[57][13] = 0.692;	inputs[57][14] = 0.432;	inputs[57][15] = 0.717333;	inputs[57][16] = 0.472;	inputs[57][17] = 0.702667;	inputs[57][18] = 0.406667;	inputs[57][19] = 0.7;	inputs[58][0] = 0.429333;	inputs[58][1] = 0.609333;	inputs[58][2] = 0.38;	inputs[58][3] = 0.682667;	inputs[58][4] = 0.537333;	inputs[58][5] = 0.7;	inputs[58][6] = 0.564;	inputs[58][7] = 0.688;	inputs[58][8] = 0.528;	inputs[58][9] = 0.68;	inputs[58][10] = 0.532;	inputs[58][11] = 0.670667;	inputs[58][12] = 0.530667;	inputs[58][13] = 0.66;	inputs[58][14] = 0.414667;	inputs[58][15] = 0.666667;	inputs[58][16] = 0.373333;	inputs[58][17] = 0.678667;	inputs[58][18] = 0.468;	inputs[58][19] = 0.698667;	inputs[59][0] = 0.493333;	inputs[59][1] = 0.690667;	inputs[59][2] = 0.604;	inputs[59][3] = 0.756;	inputs[59][4] = 0.490667;	inputs[59][5] = 0.662667;	inputs[59][6] = 0.401333;	inputs[59][7] = 0.594667;	inputs[59][8] = 0.428;	inputs[59][9] = 0.592;	inputs[59][10] = 0.445333;	inputs[59][11] = 0.738667;	inputs[59][12] = 0.522667;	inputs[59][13] = 0.717333;	inputs[59][14] = 0.588;	inputs[59][15] = 0.694667;	inputs[59][16] = 0.469333;	inputs[59][17] = 0.694667;	inputs[59][18] = 0.456;	inputs[59][19] = 0.678667;	inputs[60][0] = 0.5;	inputs[60][1] = 0.682667;	inputs[60][2] = 0.592;	inputs[60][3] = 0.738667;	inputs[60][4] = 0.514667;	inputs[60][5] = 0.645333;	inputs[60][6] = 0.386667;	inputs[60][7] = 0.618667;	inputs[60][8] = 0.412;	inputs[60][9] = 0.536;	inputs[60][10] = 0.473333;	inputs[60][11] = 0.753333;	inputs[60][12] = 0.537333;	inputs[60][13] = 0.74;	inputs[60][14] = 0.62;	inputs[60][15] = 0.652;	inputs[60][16] = 0.384;	inputs[60][17] = 0.717333;	inputs[60][18] = 0.470667;	inputs[60][19] = 0.696;	inputs[61][0] = 0.437333;	inputs[61][1] = 0.761333;	inputs[61][2] = 0.428;	inputs[61][3] = 0.512;	inputs[61][4] = 0.542667;	inputs[61][5] = 0.566667;	inputs[61][6] = 0.634667;	inputs[61][7] = 0.753333;	inputs[61][8] = 0.532;	inputs[61][9] = 0.869333;	inputs[61][10] = 0.44;	inputs[61][11] = 0.782667;	inputs[61][12] = 0.425333;	inputs[61][13] = 0.594667;	inputs[61][14] = 0.434667;	inputs[61][15] = 0.554667;	inputs[61][16] = 0.424;	inputs[61][17] = 0.613333;	inputs[61][18] = 0.48;	inputs[61][19] = 0.793333;	inputs[62][0] = 0.574667;	inputs[62][1] = 0.733333;	inputs[62][2] = 0.548;	inputs[62][3] = 0.681333;	inputs[62][4] = 0.437333;	inputs[62][5] = 0.612;	inputs[62][6] = 0.426667;	inputs[62][7] = 0.590667;	inputs[62][8] = 0.428;	inputs[62][9] = 0.674667;	inputs[62][10] = 0.508;	inputs[62][11] = 0.728;	inputs[62][12] = 0.564;	inputs[62][13] = 0.714667;	inputs[62][14] = 0.585333;	inputs[62][15] = 0.693333;	inputs[62][16] = 0.416;	inputs[62][17] = 0.717333;	inputs[62][18] = 0.485333;	inputs[62][19] = 0.689333;	inputs[63][0] = 0.434667;	inputs[63][1] = 0.658667;	inputs[63][2] = 0.372;	inputs[63][3] = 0.68;	inputs[63][4] = 0.505333;	inputs[63][5] = 0.726667;	inputs[63][6] = 0.642667;	inputs[63][7] = 0.689333;	inputs[63][8] = 0.56;	inputs[63][9] = 0.694667;	inputs[63][10] = 0.533333;	inputs[63][11] = 0.708;	inputs[63][12] = 0.557333;	inputs[63][13] = 0.674667;	inputs[63][14] = 0.502667;	inputs[63][15] = 0.641333;	inputs[63][16] = 0.413333;	inputs[63][17] = 0.701333;	inputs[63][18] = 0.452;	inputs[63][19] = 0.645333;	inputs[64][0] = 0.425333;	inputs[64][1] = 0.604;	inputs[64][2] = 0.508;	inputs[64][3] = 0.608;	inputs[64][4] = 0.577333;	inputs[64][5] = 0.68;	inputs[64][6] = 0.558667;	inputs[64][7] = 0.84;	inputs[64][8] = 0.442667;	inputs[64][9] = 0.768;	inputs[64][10] = 0.392;	inputs[64][11] = 0.588;	inputs[64][12] = 0.476;	inputs[64][13] = 0.588;	inputs[64][14] = 0.572;	inputs[64][15] = 0.661333;	inputs[64][16] = 0.561333;	inputs[64][17] = 0.833333;	inputs[64][18] = 0.465333;	inputs[64][19] = 0.862667;	inputs[65][0] = 0.405333;	inputs[65][1] = 0.64;	inputs[65][2] = 0.528;	inputs[65][3] = 0.533333;	inputs[65][4] = 0.62;	inputs[65][5] = 0.678667;	inputs[65][6] = 0.604;	inputs[65][7] = 0.845333;	inputs[65][8] = 0.493333;	inputs[65][9] = 0.888;	inputs[65][10] = 0.416;	inputs[65][11] = 0.692;	inputs[65][12] = 0.434667;	inputs[65][13] = 0.548;	inputs[65][14] = 0.497333;	inputs[65][15] = 0.533333;	inputs[65][16] = 0.486667;	inputs[65][17] = 0.824;	inputs[65][18] = 0.52;	inputs[65][19] = 0.824;	inputs[66][0] = 0.369333;	inputs[66][1] = 0.669333;	inputs[66][2] = 0.317333;	inputs[66][3] = 0.676;	inputs[66][4] = 0.477333;	inputs[66][5] = 0.718667;	inputs[66][6] = 0.549333;	inputs[66][7] = 0.730667;	inputs[66][8] = 0.574667;	inputs[66][9] = 0.690667;	inputs[66][10] = 0.566667;	inputs[66][11] = 0.677333;	inputs[66][12] = 0.552;	inputs[66][13] = 0.688;	inputs[66][14] = 0.425333;	inputs[66][15] = 0.685333;	inputs[66][16] = 0.377333;	inputs[66][17] = 0.694667;	inputs[66][18] = 0.492;	inputs[66][19] = 0.688;	inputs[67][0] = 0.454667;	inputs[67][1] = 0.656;	inputs[67][2] = 0.461333;	inputs[67][3] = 0.553333;	inputs[67][4] = 0.544;	inputs[67][5] = 0.685333;	inputs[67][6] = 0.529333;	inputs[67][7] = 0.792;	inputs[67][8] = 0.397333;	inputs[67][9] = 0.777333;	inputs[67][10] = 0.376;	inputs[67][11] = 0.512;	inputs[67][12] = 0.524;	inputs[67][13] = 0.585333;	inputs[67][14] = 0.62;	inputs[67][15] = 0.709333;	inputs[67][16] = 0.528;	inputs[67][17] = 0.812;	inputs[67][18] = 0.410667;	inputs[67][19] = 0.78;	inputs[68][0] = 0.513333;	inputs[68][1] = 0.682667;	inputs[68][2] = 0.402667;	inputs[68][3] = 0.588;	inputs[68][4] = 0.418667;	inputs[68][5] = 0.692;	inputs[68][6] = 0.468;	inputs[68][7] = 0.696;	inputs[68][8] = 0.646667;	inputs[68][9] = 0.702667;	inputs[68][10] = 0.584;	inputs[68][11] = 0.688;	inputs[68][12] = 0.521333;	inputs[68][13] = 0.650667;	inputs[68][14] = 0.509333;	inputs[68][15] = 0.648;	inputs[68][16] = 0.344;	inputs[68][17] = 0.68;	inputs[68][18] = 0.418667;	inputs[68][19] = 0.657333;	inputs[69][0] = 0.350667;	inputs[69][1] = 0.669333;	inputs[69][2] = 0.268;	inputs[69][3] = 0.657333;	inputs[69][4] = 0.513333;	inputs[69][5] = 0.701333;	inputs[69][6] = 0.614667;	inputs[69][7] = 0.716;	inputs[69][8] = 0.568;	inputs[69][9] = 0.697333;	inputs[69][10] = 0.546667;	inputs[69][11] = 0.669333;	inputs[69][12] = 0.526667;	inputs[69][13] = 0.696;	inputs[69][14] = 0.344;	inputs[69][15] = 0.684;	inputs[69][16] = 0.468;	inputs[69][17] = 0.674667;	inputs[69][18] = 0.492;	inputs[69][19] = 0.661333;	inputs[70][0] = 0.461333;	inputs[70][1] = 0.630667;	inputs[70][2] = 0.494667;	inputs[70][3] = 0.56;	inputs[70][4] = 0.530667;	inputs[70][5] = 0.714667;	inputs[70][6] = 0.509333;	inputs[70][7] = 0.786667;	inputs[70][8] = 0.425333;	inputs[70][9] = 0.774667;	inputs[70][10] = 0.392;	inputs[70][11] = 0.542667;	inputs[70][12] = 0.488;	inputs[70][13] = 0.592;	inputs[70][14] = 0.577333;	inputs[70][15] = 0.690667;	inputs[70][16] = 0.513333;	inputs[70][17] = 0.810667;	inputs[70][18] = 0.432;	inputs[70][19] = 0.792;	inputs[71][0] = 0.573333;	inputs[71][1] = 0.618667;	inputs[71][2] = 0.488;	inputs[71][3] = 0.786667;	inputs[71][4] = 0.368;	inputs[71][5] = 0.765333;	inputs[71][6] = 0.498667;	inputs[71][7] = 0.581333;	inputs[71][8] = 0.536;	inputs[71][9] = 0.502667;	inputs[71][10] = 0.406667;	inputs[71][11] = 0.652;	inputs[71][12] = 0.465333;	inputs[71][13] = 0.697333;	inputs[71][14] = 0.565333;	inputs[71][15] = 0.694667;	inputs[71][16] = 0.630667;	inputs[71][17] = 0.733333;	inputs[71][18] = 0.370667;	inputs[71][19] = 0.717333;	inputs[72][0] = 0.470667;	inputs[72][1] = 0.618667;	inputs[72][2] = 0.481333;	inputs[72][3] = 0.592;	inputs[72][4] = 0.538667;	inputs[72][5] = 0.766667;	inputs[72][6] = 0.498667;	inputs[72][7] = 0.84;	inputs[72][8] = 0.402667;	inputs[72][9] = 0.682667;	inputs[72][10] = 0.434667;	inputs[72][11] = 0.526667;	inputs[72][12] = 0.524;	inputs[72][13] = 0.618667;	inputs[72][14] = 0.598667;	inputs[72][15] = 0.777333;	inputs[72][16] = 0.484;	inputs[72][17] = 0.876;	inputs[72][18] = 0.464;	inputs[72][19] = 0.592;	inputs[73][0] = 0.46;	inputs[73][1] = 0.670667;	inputs[73][2] = 0.474667;	inputs[73][3] = 0.524;	inputs[73][4] = 0.610667;	inputs[73][5] = 0.76;	inputs[73][6] = 0.496;	inputs[73][7] = 0.853333;	inputs[73][8] = 0.369333;	inputs[73][9] = 0.610667;	inputs[73][10] = 0.442667;	inputs[73][11] = 0.537333;	inputs[73][12] = 0.62;	inputs[73][13] = 0.688;	inputs[73][14] = 0.565333;	inputs[73][15] = 0.830667;	inputs[73][16] = 0.444;	inputs[73][17] = 0.801333;	inputs[73][18] = 0.466667;	inputs[73][19] = 0.510667;	inputs[74][0] = 0.488;	inputs[74][1] = 0.628;	inputs[74][2] = 0.513333;	inputs[74][3] = 0.656;	inputs[74][4] = 0.586667;	inputs[74][5] = 0.793333;	inputs[74][6] = 0.545333;	inputs[74][7] = 0.848;	inputs[74][8] = 0.436;	inputs[74][9] = 0.750667;	inputs[74][10] = 0.453333;	inputs[74][11] = 0.569333;	inputs[74][12] = 0.461333;	inputs[74][13] = 0.554667;	inputs[74][14] = 0.466667;	inputs[74][15] = 0.565333;	inputs[74][16] = 0.478667;	inputs[74][17] = 0.828;	inputs[74][18] = 0.498667;	inputs[74][19] = 0.729333;	inputs[75][0] = 0.458667;	inputs[75][1] = 0.632;	inputs[75][2] = 0.485333;	inputs[75][3] = 0.590667;	inputs[75][4] = 0.568;	inputs[75][5] = 0.746667;	inputs[75][6] = 0.481333;	inputs[75][7] = 0.809333;	inputs[75][8] = 0.406667;	inputs[75][9] = 0.688;	inputs[75][10] = 0.421333;	inputs[75][11] = 0.561333;	inputs[75][12] = 0.505333;	inputs[75][13] = 0.552;	inputs[75][14] = 0.569333;	inputs[75][15] = 0.674667;	inputs[75][16] = 0.534667;	inputs[75][17] = 0.764;	inputs[75][18] = 0.394667;	inputs[75][19] = 0.824;	inputs[76][0] = 0.441333;	inputs[76][1] = 0.696;	inputs[76][2] = 0.414667;	inputs[76][3] = 0.536;	inputs[76][4] = 0.52;	inputs[76][5] = 0.62;	inputs[76][6] = 0.573333;	inputs[76][7] = 0.76;	inputs[76][8] = 0.517333;	inputs[76][9] = 0.830667;	inputs[76][10] = 0.445333;	inputs[76][11] = 0.724;	inputs[76][12] = 0.44;	inputs[76][13] = 0.586667;	inputs[76][14] = 0.473333;	inputs[76][15] = 0.554667;	inputs[76][16] = 0.488;	inputs[76][17] = 0.525333;	inputs[76][18] = 0.497333;	inputs[76][19] = 0.770667;	inputs[77][0] = 0.493333;	inputs[77][1] = 0.641333;	inputs[77][2] = 0.545333;	inputs[77][3] = 0.690667;	inputs[77][4] = 0.538667;	inputs[77][5] = 0.709333;	inputs[77][6] = 0.469333;	inputs[77][7] = 0.668;	inputs[77][8] = 0.445333;	inputs[77][9] = 0.541333;	inputs[77][10] = 0.457333;	inputs[77][11] = 0.585333;	inputs[77][12] = 0.449333;	inputs[77][13] = 0.766667;	inputs[77][14] = 0.557333;	inputs[77][15] = 0.706667;	inputs[77][16] = 0.604;	inputs[77][17] = 0.669333;	inputs[77][18] = 0.392;	inputs[77][19] = 0.744;	inputs[78][0] = 0.376;	inputs[78][1] = 0.476;	inputs[78][2] = 0.530667;	inputs[78][3] = 0.588;	inputs[78][4] = 0.568;	inputs[78][5] = 0.76;	inputs[78][6] = 0.494667;	inputs[78][7] = 0.832;	inputs[78][8] = 0.445333;	inputs[78][9] = 0.632;	inputs[78][10] = 0.452;	inputs[78][11] = 0.550667;	inputs[78][12] = 0.458667;	inputs[78][13] = 0.570667;	inputs[78][14] = 0.489333;	inputs[78][15] = 0.676;	inputs[78][16] = 0.497333;	inputs[78][17] = 0.762667;	inputs[78][18] = 0.493333;	inputs[78][19] = 0.686667;	inputs[79][0] = 0.445333;	inputs[79][1] = 0.692;	inputs[79][2] = 0.461333;	inputs[79][3] = 0.548;	inputs[79][4] = 0.537333;	inputs[79][5] = 0.594667;	inputs[79][6] = 0.582667;	inputs[79][7] = 0.738667;	inputs[79][8] = 0.546667;	inputs[79][9] = 0.78;	inputs[79][10] = 0.477333;	inputs[79][11] = 0.769333;	inputs[79][12] = 0.42;	inputs[79][13] = 0.616;	inputs[79][14] = 0.453333;	inputs[79][15] = 0.577333;	inputs[79][16] = 0.477333;	inputs[79][17] = 0.590667;	inputs[79][18] = 0.442667;	inputs[79][19] = 0.504;	inputs[80][0] = 0.424;	inputs[80][1] = 0.586667;	inputs[80][2] = 0.352;	inputs[80][3] = 0.721333;	inputs[80][4] = 0.48;	inputs[80][5] = 0.704;	inputs[80][6] = 0.556;	inputs[80][7] = 0.685333;	inputs[80][8] = 0.548;	inputs[80][9] = 0.666667;	inputs[80][10] = 0.521333;	inputs[80][11] = 0.657333;	inputs[80][12] = 0.536;	inputs[80][13] = 0.633333;	inputs[80][14] = 0.568;	inputs[80][15] = 0.648;	inputs[80][16] = 0.385333;	inputs[80][17] = 0.684;	inputs[80][18] = 0.457333;	inputs[80][19] = 0.688;	inputs[81][0] = 0.546667;	inputs[81][1] = 0.716;	inputs[81][2] = 0.58;	inputs[81][3] = 0.701333;	inputs[81][4] = 0.388;	inputs[81][5] = 0.661333;	inputs[81][6] = 0.402667;	inputs[81][7] = 0.577333;	inputs[81][8] = 0.406667;	inputs[81][9] = 0.650667;	inputs[81][10] = 0.482667;	inputs[81][11] = 0.72;	inputs[81][12] = 0.572;	inputs[81][13] = 0.709333;	inputs[81][14] = 0.584;	inputs[81][15] = 0.688;	inputs[81][16] = 0.42;	inputs[81][17] = 0.744;	inputs[81][18] = 0.446667;	inputs[81][19] = 0.689333;	inputs[82][0] = 0.522667;	inputs[82][1] = 0.728;	inputs[82][2] = 0.54;	inputs[82][3] = 0.694667;	inputs[82][4] = 0.466667;	inputs[82][5] = 0.616;	inputs[82][6] = 0.428;	inputs[82][7] = 0.589333;	inputs[82][8] = 0.441333;	inputs[82][9] = 0.586667;	inputs[82][10] = 0.481333;	inputs[82][11] = 0.736;	inputs[82][12] = 0.554667;	inputs[82][13] = 0.737333;	inputs[82][14] = 0.570667;	inputs[82][15] = 0.674667;	inputs[82][16] = 0.466667;	inputs[82][17] = 0.677333;	inputs[82][18] = 0.494667;	inputs[82][19] = 0.709333;	inputs[83][0] = 0.44;	inputs[83][1] = 0.734667;	inputs[83][2] = 0.454667;	inputs[83][3] = 0.529333;	inputs[83][4] = 0.598667;	inputs[83][5] = 0.661333;	inputs[83][6] = 0.602667;	inputs[83][7] = 0.725333;	inputs[83][8] = 0.550667;	inputs[83][9] = 0.826667;	inputs[83][10] = 0.454667;	inputs[83][11] = 0.757333;	inputs[83][12] = 0.418667;	inputs[83][13] = 0.597333;	inputs[83][14] = 0.457333;	inputs[83][15] = 0.566667;	inputs[83][16] = 0.488;	inputs[83][17] = 0.545333;	inputs[83][18] = 0.481333;	inputs[83][19] = 0.773333;	inputs[84][0] = 0.472;	inputs[84][1] = 0.618667;	inputs[84][2] = 0.502667;	inputs[84][3] = 0.561333;	inputs[84][4] = 0.573333;	inputs[84][5] = 0.796;	inputs[84][6] = 0.46;	inputs[84][7] = 0.868;	inputs[84][8] = 0.410667;	inputs[84][9] = 0.645333;	inputs[84][10] = 0.453333;	inputs[84][11] = 0.509333;	inputs[84][12] = 0.548;	inputs[84][13] = 0.626667;	inputs[84][14] = 0.497333;	inputs[84][15] = 0.857333;	inputs[84][16] = 0.432;	inputs[84][17] = 0.674667;	inputs[84][18] = 0.449333;	inputs[84][19] = 0.578667;	inputs[85][0] = 0.472;	inputs[85][1] = 0.598667;	inputs[85][2] = 0.554667;	inputs[85][3] = 0.612;	inputs[85][4] = 0.616;	inputs[85][5] = 0.866667;	inputs[85][6] = 0.418667;	inputs[85][7] = 0.814667;	inputs[85][8] = 0.36;	inputs[85][9] = 0.597333;	inputs[85][10] = 0.464;	inputs[85][11] = 0.54;	inputs[85][12] = 0.538667;	inputs[85][13] = 0.724;	inputs[85][14] = 0.610667;	inputs[85][15] = 0.777333;	inputs[85][16] = 0.473333;	inputs[85][17] = 0.776;	inputs[85][18] = 0.441333;	inputs[85][19] = 0.694667;	inputs[86][0] = 0.462667;	inputs[86][1] = 0.618667;	inputs[86][2] = 0.365333;	inputs[86][3] = 0.690667;	inputs[86][4] = 0.686667;	inputs[86][5] = 0.746667;	inputs[86][6] = 0.593333;	inputs[86][7] = 0.697333;	inputs[86][8] = 0.537333;	inputs[86][9] = 0.669333;	inputs[86][10] = 0.561333;	inputs[86][11] = 0.649333;	inputs[86][12] = 0.488;	inputs[86][13] = 0.685333;	inputs[86][14] = 0.417333;	inputs[86][15] = 0.682667;	inputs[86][16] = 0.450667;	inputs[86][17] = 0.68;	inputs[86][18] = 0.481333;	inputs[86][19] = 0.684;	inputs[87][0] = 0.534667;	inputs[87][1] = 0.769333;	inputs[87][2] = 0.546667;	inputs[87][3] = 0.725333;	inputs[87][4] = 0.453333;	inputs[87][5] = 0.573333;	inputs[87][6] = 0.389333;	inputs[87][7] = 0.541333;	inputs[87][8] = 0.424;	inputs[87][9] = 0.632;	inputs[87][10] = 0.513333;	inputs[87][11] = 0.744;	inputs[87][12] = 0.562667;	inputs[87][13] = 0.724;	inputs[87][14] = 0.586667;	inputs[87][15] = 0.654667;	inputs[87][16] = 0.44;	inputs[87][17] = 0.689333;	inputs[87][18] = 0.478667;	inputs[87][19] = 0.716;	inputs[88][0] = 0.462667;	inputs[88][1] = 0.630667;	inputs[88][2] = 0.478667;	inputs[88][3] = 0.541333;	inputs[88][4] = 0.586667;	inputs[88][5] = 0.762667;	inputs[88][6] = 0.510667;	inputs[88][7] = 0.876;	inputs[88][8] = 0.389333;	inputs[88][9] = 0.725333;	inputs[88][10] = 0.404;	inputs[88][11] = 0.526667;	inputs[88][12] = 0.578667;	inputs[88][13] = 0.594667;	inputs[88][14] = 0.624;	inputs[88][15] = 0.733333;	inputs[88][16] = 0.489333;	inputs[88][17] = 0.801333;	inputs[88][18] = 0.4;	inputs[88][19] = 0.802667;	inputs[89][0] = 0.493333;	inputs[89][1] = 0.688;	inputs[89][2] = 0.596;	inputs[89][3] = 0.754667;	inputs[89][4] = 0.526667;	inputs[89][5] = 0.674667;	inputs[89][6] = 0.422667;	inputs[89][7] = 0.598667;	inputs[89][8] = 0.416;	inputs[89][9] = 0.554667;	inputs[89][10] = 0.470667;	inputs[89][11] = 0.748;	inputs[89][12] = 0.573333;	inputs[89][13] = 0.730667;	inputs[89][14] = 0.576;	inputs[89][15] = 0.696;	inputs[89][16] = 0.413333;	inputs[89][17] = 0.68;	inputs[89][18] = 0.402667;	inputs[89][19] = 0.713333;	inputs[90][0] = 0.377333;	inputs[90][1] = 0.698667;	inputs[90][2] = 0.552;	inputs[90][3] = 0.753333;	inputs[90][4] = 0.594667;	inputs[90][5] = 0.712;	inputs[90][6] = 0.608;	inputs[90][7] = 0.704;	inputs[90][8] = 0.530667;	inputs[90][9] = 0.68;	inputs[90][10] = 0.574667;	inputs[90][11] = 0.684;	inputs[90][12] = 0.496;	inputs[90][13] = 0.752;	inputs[90][14] = 0.393333;	inputs[90][15] = 0.709333;	inputs[90][16] = 0.494667;	inputs[90][17] = 0.688;	inputs[90][18] = 0.501333;	inputs[90][19] = 0.682667;	inputs[91][0] = 0.441333;	inputs[91][1] = 0.590667;	inputs[91][2] = 0.530667;	inputs[91][3] = 0.632;	inputs[91][4] = 0.588;	inputs[91][5] = 0.717333;	inputs[91][6] = 0.538667;	inputs[91][7] = 0.816;	inputs[91][8] = 0.497333;	inputs[91][9] = 0.798667;	inputs[91][10] = 0.445333;	inputs[91][11] = 0.618667;	inputs[91][12] = 0.422667;	inputs[91][13] = 0.601333;	inputs[91][14] = 0.445333;	inputs[91][15] = 0.589333;	inputs[91][16] = 0.498667;	inputs[91][17] = 0.772;	inputs[91][18] = 0.468;	inputs[91][19] = 0.784;	inputs[92][0] = 0.442667;	inputs[92][1] = 0.638667;	inputs[92][2] = 0.384;	inputs[92][3] = 0.657333;	inputs[92][4] = 0.548;	inputs[92][5] = 0.712;	inputs[92][6] = 0.578667;	inputs[92][7] = 0.688;	inputs[92][8] = 0.578667;	inputs[92][9] = 0.665333;	inputs[92][10] = 0.550667;	inputs[92][11] = 0.682667;	inputs[92][12] = 0.582667;	inputs[92][13] = 0.650667;	inputs[92][14] = 0.5;	inputs[92][15] = 0.697333;	inputs[92][16] = 0.394667;	inputs[92][17] = 0.74;	inputs[92][18] = 0.454667;	inputs[92][19] = 0.66;	inputs[93][0] = 0.501333;	inputs[93][1] = 0.689333;	inputs[93][2] = 0.570667;	inputs[93][3] = 0.716;	inputs[93][4] = 0.498667;	inputs[93][5] = 0.674667;	inputs[93][6] = 0.424;	inputs[93][7] = 0.6;	inputs[93][8] = 0.424;	inputs[93][9] = 0.574667;	inputs[93][10] = 0.445333;	inputs[93][11] = 0.706667;	inputs[93][12] = 0.533333;	inputs[93][13] = 0.742667;	inputs[93][14] = 0.556;	inputs[93][15] = 0.684;	inputs[93][16] = 0.593333;	inputs[93][17] = 0.689333;	inputs[93][18] = 0.44;	inputs[93][19] = 0.674667;	inputs[94][0] = 0.434667;	inputs[94][1] = 0.569333;	inputs[94][2] = 0.521333;	inputs[94][3] = 0.648;	inputs[94][4] = 0.556;	inputs[94][5] = 0.690667;	inputs[94][6] = 0.508;	inputs[94][7] = 0.816;	inputs[94][8] = 0.445333;	inputs[94][9] = 0.790667;	inputs[94][10] = 0.446667;	inputs[94][11] = 0.638667;	inputs[94][12] = 0.449333;	inputs[94][13] = 0.561333;	inputs[94][14] = 0.453333;	inputs[94][15] = 0.557333;	inputs[94][16] = 0.482667;	inputs[94][17] = 0.813333;	inputs[94][18] = 0.496;	inputs[94][19] = 0.741333;	inputs[95][0] = 0.450667;	inputs[95][1] = 0.749333;	inputs[95][2] = 0.384;	inputs[95][3] = 0.526667;	inputs[95][4] = 0.522667;	inputs[95][5] = 0.590667;	inputs[95][6] = 0.566667;	inputs[95][7] = 0.724;	inputs[95][8] = 0.568;	inputs[95][9] = 0.752;	inputs[95][10] = 0.488;	inputs[95][11] = 0.830667;	inputs[95][12] = 0.410667;	inputs[95][13] = 0.716;	inputs[95][14] = 0.449333;	inputs[95][15] = 0.590667;	inputs[95][16] = 0.485333;	inputs[95][17] = 0.573333;	inputs[95][18] = 0.457333;	inputs[95][19] = 0.557333;	inputs[96][0] = 0.416;	inputs[96][1] = 0.674667;	inputs[96][2] = 0.372;	inputs[96][3] = 0.701333;	inputs[96][4] = 0.464;	inputs[96][5] = 0.682667;	inputs[96][6] = 0.58;	inputs[96][7] = 0.722667;	inputs[96][8] = 0.534667;	inputs[96][9] = 0.684;	inputs[96][10] = 0.564;	inputs[96][11] = 0.662667;	inputs[96][12] = 0.549333;	inputs[96][13] = 0.674667;	inputs[96][14] = 0.290667;	inputs[96][15] = 0.701333;	inputs[96][16] = 0.44;	inputs[96][17] = 0.678667;	inputs[96][18] = 0.513333;	inputs[96][19] = 0.676;	inputs[97][0] = 0.576;	inputs[97][1] = 0.76;	inputs[97][2] = 0.477333;	inputs[97][3] = 0.717333;	inputs[97][4] = 0.526667;	inputs[97][5] = 0.502667;	inputs[97][6] = 0.36;	inputs[97][7] = 0.573333;	inputs[97][8] = 0.397333;	inputs[97][9] = 0.741333;	inputs[97][10] = 0.577333;	inputs[97][11] = 0.706667;	inputs[97][12] = 0.629333;	inputs[97][13] = 0.685333;	inputs[97][14] = 0.408;	inputs[97][15] = 0.681333;	inputs[97][16] = 0.392;	inputs[97][17] = 0.676;	inputs[97][18] = 0.481333;	inputs[97][19] = 0.710667;	inputs[98][0] = 0.481333;	inputs[98][1] = 0.573333;	inputs[98][2] = 0.52;	inputs[98][3] = 0.66;	inputs[98][4] = 0.544;	inputs[98][5] = 0.714667;	inputs[98][6] = 0.498667;	inputs[98][7] = 0.789333;	inputs[98][8] = 0.457333;	inputs[98][9] = 0.757333;	inputs[98][10] = 0.396;	inputs[98][11] = 0.621333;	inputs[98][12] = 0.456;	inputs[98][13] = 0.564;	inputs[98][14] = 0.521333;	inputs[98][15] = 0.646667;	inputs[98][16] = 0.48;	inputs[98][17] = 0.606667;	inputs[98][18] = 0.417333;	inputs[98][19] = 0.824;	inputs[99][0] = 0.453333;	inputs[99][1] = 0.605333;	inputs[99][2] = 0.589333;	inputs[99][3] = 0.577333;	inputs[99][4] = 0.537333;	inputs[99][5] = 0.785333;	inputs[99][6] = 0.426667;	inputs[99][7] = 0.822667;	inputs[99][8] = 0.34;	inputs[99][9] = 0.533333;	inputs[99][10] = 0.508;	inputs[99][11] = 0.553333;	inputs[99][12] = 0.578667;	inputs[99][13] = 0.726667;	inputs[99][14] = 0.486667;	inputs[99][15] = 0.781333;	inputs[99][16] = 0.416;	inputs[99][17] = 0.778667;	inputs[99][18] = 0.470667;	inputs[99][19] = 0.513333;	inputs[100][0] = 0.462667;	inputs[100][1] = 0.648;	inputs[100][2] = 0.474667;	inputs[100][3] = 0.573333;	inputs[100][4] = 0.577333;	inputs[100][5] = 0.686667;	inputs[100][6] = 0.525333;	inputs[100][7] = 0.784;	inputs[100][8] = 0.416;	inputs[100][9] = 0.778667;	inputs[100][10] = 0.386667;	inputs[100][11] = 0.569333;	inputs[100][12] = 0.52;	inputs[100][13] = 0.564;	inputs[100][14] = 0.561333;	inputs[100][15] = 0.705333;	inputs[100][16] = 0.474667;	inputs[100][17] = 0.749333;	inputs[100][18] = 0.402667;	inputs[100][19] = 0.748;	inputs[101][0] = 0.378667;	inputs[101][1] = 0.633333;	inputs[101][2] = 0.353333;	inputs[101][3] = 0.733333;	inputs[101][4] = 0.449333;	inputs[101][5] = 0.718667;	inputs[101][6] = 0.562667;	inputs[101][7] = 0.710667;	inputs[101][8] = 0.585333;	inputs[101][9] = 0.665333;	inputs[101][10] = 0.562667;	inputs[101][11] = 0.670667;	inputs[101][12] = 0.529333;	inputs[101][13] = 0.646667;	inputs[101][14] = 0.24;	inputs[101][15] = 0.632;	inputs[101][16] = 0.369333;	inputs[101][17] = 0.636;	inputs[101][18] = 0.461333;	inputs[101][19] = 0.677333;
  #endif
}
void fill_static_matrix_outputs(){
//(102, 4)
	#ifdef TRAINING
  outputs[0][0] = 0;	outputs[0][1] = 0;	outputs[0][2] = 1;	outputs[0][3] = 0;	outputs[1][0] = 0;	outputs[1][1] = 0;	outputs[1][2] = 0;	outputs[1][3] = 1;	outputs[2][0] = 1;	outputs[2][1] = 0;	outputs[2][2] = 0;	outputs[2][3] = 0;	outputs[3][0] = 0;	outputs[3][1] = 1;	outputs[3][2] = 0;	outputs[3][3] = 0;	outputs[4][0] = 0;	outputs[4][1] = 1;	outputs[4][2] = 0;	outputs[4][3] = 0;	outputs[5][0] = 1;	outputs[5][1] = 0;	outputs[5][2] = 0;	outputs[5][3] = 0;	outputs[6][0] = 0;	outputs[6][1] = 0;	outputs[6][2] = 0;	outputs[6][3] = 1;	outputs[7][0] = 1;	outputs[7][1] = 0;	outputs[7][2] = 0;	outputs[7][3] = 0;	outputs[8][0] = 0;	outputs[8][1] = 1;	outputs[8][2] = 0;	outputs[8][3] = 0;	outputs[9][0] = 0;	outputs[9][1] = 1;	outputs[9][2] = 0;	outputs[9][3] = 0;	outputs[10][0] = 1;	outputs[10][1] = 0;	outputs[10][2] = 0;	outputs[10][3] = 0;	outputs[11][0] = 1;	outputs[11][1] = 0;	outputs[11][2] = 0;	outputs[11][3] = 0;	outputs[12][0] = 0;	outputs[12][1] = 1;	outputs[12][2] = 0;	outputs[12][3] = 0;	outputs[13][0] = 0;	outputs[13][1] = 0;	outputs[13][2] = 0;	outputs[13][3] = 1;	outputs[14][0] = 1;	outputs[14][1] = 0;	outputs[14][2] = 0;	outputs[14][3] = 0;	outputs[15][0] = 1;	outputs[15][1] = 0;	outputs[15][2] = 0;	outputs[15][3] = 0;	outputs[16][0] = 0;	outputs[16][1] = 0;	outputs[16][2] = 0;	outputs[16][3] = 1;	outputs[17][0] = 0;	outputs[17][1] = 1;	outputs[17][2] = 0;	outputs[17][3] = 0;	outputs[18][0] = 0;	outputs[18][1] = 0;	outputs[18][2] = 0;	outputs[18][3] = 1;	outputs[19][0] = 0;	outputs[19][1] = 0;	outputs[19][2] = 1;	outputs[19][3] = 0;	outputs[20][0] = 0;	outputs[20][1] = 1;	outputs[20][2] = 0;	outputs[20][3] = 0;	outputs[21][0] = 1;	outputs[21][1] = 0;	outputs[21][2] = 0;	outputs[21][3] = 0;	outputs[22][0] = 0;	outputs[22][1] = 0;	outputs[22][2] = 1;	outputs[22][3] = 0;	outputs[23][0] = 1;	outputs[23][1] = 0;	outputs[23][2] = 0;	outputs[23][3] = 0;	outputs[24][0] = 0;	outputs[24][1] = 1;	outputs[24][2] = 0;	outputs[24][3] = 0;	outputs[25][0] = 0;	outputs[25][1] = 1;	outputs[25][2] = 0;	outputs[25][3] = 0;	outputs[26][0] = 0;	outputs[26][1] = 0;	outputs[26][2] = 1;	outputs[26][3] = 0;	outputs[27][0] = 0;	outputs[27][1] = 0;	outputs[27][2] = 1;	outputs[27][3] = 0;	outputs[28][0] = 1;	outputs[28][1] = 0;	outputs[28][2] = 0;	outputs[28][3] = 0;	outputs[29][0] = 0;	outputs[29][1] = 0;	outputs[29][2] = 1;	outputs[29][3] = 0;	outputs[30][0] = 1;	outputs[30][1] = 0;	outputs[30][2] = 0;	outputs[30][3] = 0;	outputs[31][0] = 1;	outputs[31][1] = 0;	outputs[31][2] = 0;	outputs[31][3] = 0;	outputs[32][0] = 0;	outputs[32][1] = 0;	outputs[32][2] = 1;	outputs[32][3] = 0;	outputs[33][0] = 0;	outputs[33][1] = 0;	outputs[33][2] = 0;	outputs[33][3] = 1;	outputs[34][0] = 0;	outputs[34][1] = 0;	outputs[34][2] = 1;	outputs[34][3] = 0;	outputs[35][0] = 0;	outputs[35][1] = 0;	outputs[35][2] = 0;	outputs[35][3] = 1;	outputs[36][0] = 0;	outputs[36][1] = 1;	outputs[36][2] = 0;	outputs[36][3] = 0;	outputs[37][0] = 0;	outputs[37][1] = 1;	outputs[37][2] = 0;	outputs[37][3] = 0;	outputs[38][0] = 0;	outputs[38][1] = 0;	outputs[38][2] = 1;	outputs[38][3] = 0;	outputs[39][0] = 0;	outputs[39][1] = 1;	outputs[39][2] = 0;	outputs[39][3] = 0;	outputs[40][0] = 0;	outputs[40][1] = 0;	outputs[40][2] = 1;	outputs[40][3] = 0;	outputs[41][0] = 0;	outputs[41][1] = 0;	outputs[41][2] = 0;	outputs[41][3] = 1;	outputs[42][0] = 0;	outputs[42][1] = 0;	outputs[42][2] = 0;	outputs[42][3] = 1;	outputs[43][0] = 0;	outputs[43][1] = 1;	outputs[43][2] = 0;	outputs[43][3] = 0;	outputs[44][0] = 0;	outputs[44][1] = 1;	outputs[44][2] = 0;	outputs[44][3] = 0;	outputs[45][0] = 0;	outputs[45][1] = 1;	outputs[45][2] = 0;	outputs[45][3] = 0;	outputs[46][0] = 0;	outputs[46][1] = 1;	outputs[46][2] = 0;	outputs[46][3] = 0;	outputs[47][0] = 0;	outputs[47][1] = 0;	outputs[47][2] = 1;	outputs[47][3] = 0;	outputs[48][0] = 0;	outputs[48][1] = 0;	outputs[48][2] = 0;	outputs[48][3] = 1;	outputs[49][0] = 0;	outputs[49][1] = 0;	outputs[49][2] = 1;	outputs[49][3] = 0;	outputs[50][0] = 1;	outputs[50][1] = 0;	outputs[50][2] = 0;	outputs[50][3] = 0;	outputs[51][0] = 1;	outputs[51][1] = 0;	outputs[51][2] = 0;	outputs[51][3] = 0;	outputs[52][0] = 0;	outputs[52][1] = 1;	outputs[52][2] = 0;	outputs[52][3] = 0;	outputs[53][0] = 0;	outputs[53][1] = 1;	outputs[53][2] = 0;	outputs[53][3] = 0;	outputs[54][0] = 1;	outputs[54][1] = 0;	outputs[54][2] = 0;	outputs[54][3] = 0;	outputs[55][0] = 1;	outputs[55][1] = 0;	outputs[55][2] = 0;	outputs[55][3] = 0;	outputs[56][0] = 1;	outputs[56][1] = 0;	outputs[56][2] = 0;	outputs[56][3] = 0;	outputs[57][0] = 1;	outputs[57][1] = 0;	outputs[57][2] = 0;	outputs[57][3] = 0;	outputs[58][0] = 0;	outputs[58][1] = 0;	outputs[58][2] = 0;	outputs[58][3] = 1;	outputs[59][0] = 1;	outputs[59][1] = 0;	outputs[59][2] = 0;	outputs[59][3] = 0;	outputs[60][0] = 1;	outputs[60][1] = 0;	outputs[60][2] = 0;	outputs[60][3] = 0;	outputs[61][0] = 0;	outputs[61][1] = 0;	outputs[61][2] = 1;	outputs[61][3] = 0;	outputs[62][0] = 1;	outputs[62][1] = 0;	outputs[62][2] = 0;	outputs[62][3] = 0;	outputs[63][0] = 0;	outputs[63][1] = 0;	outputs[63][2] = 0;	outputs[63][3] = 1;	outputs[64][0] = 0;	outputs[64][1] = 1;	outputs[64][2] = 0;	outputs[64][3] = 0;	outputs[65][0] = 0;	outputs[65][1] = 0;	outputs[65][2] = 1;	outputs[65][3] = 0;	outputs[66][0] = 0;	outputs[66][1] = 0;	outputs[66][2] = 0;	outputs[66][3] = 1;	outputs[67][0] = 0;	outputs[67][1] = 1;	outputs[67][2] = 0;	outputs[67][3] = 0;	outputs[68][0] = 0;	outputs[68][1] = 0;	outputs[68][2] = 0;	outputs[68][3] = 1;	outputs[69][0] = 0;	outputs[69][1] = 0;	outputs[69][2] = 0;	outputs[69][3] = 1;	outputs[70][0] = 0;	outputs[70][1] = 1;	outputs[70][2] = 0;	outputs[70][3] = 0;	outputs[71][0] = 1;	outputs[71][1] = 0;	outputs[71][2] = 0;	outputs[71][3] = 0;	outputs[72][0] = 0;	outputs[72][1] = 1;	outputs[72][2] = 0;	outputs[72][3] = 0;	outputs[73][0] = 0;	outputs[73][1] = 1;	outputs[73][2] = 0;	outputs[73][3] = 0;	outputs[74][0] = 0;	outputs[74][1] = 0;	outputs[74][2] = 1;	outputs[74][3] = 0;	outputs[75][0] = 0;	outputs[75][1] = 1;	outputs[75][2] = 0;	outputs[75][3] = 0;	outputs[76][0] = 0;	outputs[76][1] = 0;	outputs[76][2] = 1;	outputs[76][3] = 0;	outputs[77][0] = 1;	outputs[77][1] = 0;	outputs[77][2] = 0;	outputs[77][3] = 0;	outputs[78][0] = 0;	outputs[78][1] = 0;	outputs[78][2] = 1;	outputs[78][3] = 0;	outputs[79][0] = 0;	outputs[79][1] = 0;	outputs[79][2] = 1;	outputs[79][3] = 0;	outputs[80][0] = 0;	outputs[80][1] = 0;	outputs[80][2] = 0;	outputs[80][3] = 1;	outputs[81][0] = 1;	outputs[81][1] = 0;	outputs[81][2] = 0;	outputs[81][3] = 0;	outputs[82][0] = 1;	outputs[82][1] = 0;	outputs[82][2] = 0;	outputs[82][3] = 0;	outputs[83][0] = 0;	outputs[83][1] = 0;	outputs[83][2] = 1;	outputs[83][3] = 0;	outputs[84][0] = 0;	outputs[84][1] = 1;	outputs[84][2] = 0;	outputs[84][3] = 0;	outputs[85][0] = 0;	outputs[85][1] = 1;	outputs[85][2] = 0;	outputs[85][3] = 0;	outputs[86][0] = 0;	outputs[86][1] = 0;	outputs[86][2] = 0;	outputs[86][3] = 1;	outputs[87][0] = 1;	outputs[87][1] = 0;	outputs[87][2] = 0;	outputs[87][3] = 0;	outputs[88][0] = 0;	outputs[88][1] = 1;	outputs[88][2] = 0;	outputs[88][3] = 0;	outputs[89][0] = 1;	outputs[89][1] = 0;	outputs[89][2] = 0;	outputs[89][3] = 0;	outputs[90][0] = 0;	outputs[90][1] = 0;	outputs[90][2] = 0;	outputs[90][3] = 1;	outputs[91][0] = 0;	outputs[91][1] = 0;	outputs[91][2] = 1;	outputs[91][3] = 0;	outputs[92][0] = 0;	outputs[92][1] = 0;	outputs[92][2] = 0;	outputs[92][3] = 1;	outputs[93][0] = 1;	outputs[93][1] = 0;	outputs[93][2] = 0;	outputs[93][3] = 0;	outputs[94][0] = 0;	outputs[94][1] = 0;	outputs[94][2] = 1;	outputs[94][3] = 0;	outputs[95][0] = 0;	outputs[95][1] = 0;	outputs[95][2] = 1;	outputs[95][3] = 0;	outputs[96][0] = 0;	outputs[96][1] = 0;	outputs[96][2] = 0;	outputs[96][3] = 1;	outputs[97][0] = 1;	outputs[97][1] = 0;	outputs[97][2] = 0;	outputs[97][3] = 0;	outputs[98][0] = 0;	outputs[98][1] = 0;	outputs[98][2] = 1;	outputs[98][3] = 0;	outputs[99][0] = 0;	outputs[99][1] = 1;	outputs[99][2] = 0;	outputs[99][3] = 0;	outputs[100][0] = 0;	outputs[100][1] = 1;	outputs[100][2] = 0;	outputs[100][3] = 0;	outputs[101][0] = 0;	outputs[101][1] = 0;	outputs[101][2] = 0;	outputs[101][3] = 1;
  #endif
}
void fill_matrix_circle(){
  #ifdef TRAINING
  circle[0][0]=0.000000;circle[0][1]=0.000000;circle[0][2]=-1.800000;circle[0][3]=3.600000;circle[0][4]=47.880000;circle[0][5]=22.140000;circle[0][6]=-34.308000;circle[0][7]=28.026000;circle[0][8]=-23.677200;circle[0][9]=5.423400;circle[0][10]=-22.209480;circle[0][11]=-43.718940;circle[0][12]=91.611468;circle[0][13]=-42.047046;circle[0][14]=-143.449679;circle[0][15]=139.457659;circle[0][16]=26.595289;circle[0][17]=-21.188107;circle[0][18]=6.835760;circle[0][19]=-6.469297;circle[0][20]=-0.147816;circle[0][21]=-7.622367;circle[0][22]=0.766966;circle[0][23]=-5.960130;circle[0][24]=-0.209731;circle[0][25]=-6.264117;circle[0][26]=1.611242;circle[0][27]=-5.637705;circle[0][28]=-2.149882;circle[0][29]=-3.273935;circle[1][0]=0.000000;circle[1][1]=0.000000;circle[1][2]=-2.700000;circle[1][3]=1.800000;circle[1][4]=-1.530000;circle[1][5]=2.520000;circle[1][6]=0.423000;circle[1][7]=-16.632000;circle[1][8]=6.680700;circle[1][9]=93.031200;circle[1][10]=-97.487370;circle[1][11]=-8.071920;circle[1][12]=-23.838633;circle[1][13]=-83.764728;circle[1][14]=54.145230;circle[1][15]=92.011745;circle[1][16]=30.730707;circle[1][17]=96.310570;circle[1][18]=-12.842363;circle[1][19]=-50.120487;circle[1][20]=2.841873;circle[1][21]=-10.008438;circle[1][22]=2.557686;circle[1][23]=-16.207594;circle[1][24]=-0.398083;circle[1][25]=-9.186835;circle[1][26]=3.241725;circle[1][27]=-5.568151;circle[1][28]=2.017553;circle[1][29]=-3.211336;circle[2][0]=0.000000;circle[2][1]=0.000000;circle[2][2]=-3.600000;circle[2][3]=0.000000;circle[2][4]=-2.340000;circle[2][5]=0.000000;circle[2][6]=75.294000;circle[2][7]=-66.600000;circle[2][8]=-105.035400;circle[2][9]=81.360000;circle[2][10]=-33.331860;circle[2][11]=-57.276000;circle[2][12]=185.101326;circle[2][13]=-173.948400;circle[2][14]=57.691193;circle[2][15]=113.446440;circle[2][16]=-25.477926;circle[2][17]=-13.098204;circle[2][18]=-22.030133;circle[2][19]=5.311616;circle[2][20]=-18.027120;circle[2][21]=12.880455;circle[2][22]=-16.224408;circle[2][23]=15.192409;circle[2][24]=-13.701967;circle[2][25]=9.173168;circle[2][26]=-12.331770;circle[2][27]=9.155852;circle[2][28]=-13.798593;circle[2][29]=10.940266;circle[3][0]=0.000000;circle[3][1]=0.000000;circle[3][2]=-1.800000;circle[3][3]=1.800000;circle[3][4]=12.780000;circle[3][5]=5.220000;circle[3][6]=55.602000;circle[3][7]=48.798000;circle[3][8]=2.341800;circle[3][9]=50.218200;circle[3][10]=-43.792380;circle[3][11]=-174.403620;circle[3][12]=62.286858;circle[3][13]=-218.163258;circle[3][14]=-24.941828;circle[3][15]=106.953068;circle[3][16]=-65.647645;circle[3][17]=153.857761;circle[3][18]=-4.182881;circle[3][19]=-12.728015;circle[3][20]=-1.964592;circle[3][21]=2.044786;circle[3][22]=0.031867;circle[3][23]=18.040308;circle[3][24]=-1.771320;circle[3][25]=7.236277;circle[3][26]=-2.494188;circle[3][27]=3.812649;circle[3][28]=1.355231;circle[3][29]=7.031384;circle[4][0]=0.000000;circle[4][1]=0.000000;circle[4][2]=-2.700000;circle[4][3]=-0.900000;circle[4][4]=0.270000;circle[4][5]=4.590000;circle[4][6]=37.143000;circle[4][7]=78.831000;circle[4][8]=22.628700;circle[4][9]=-71.252100;circle[4][10]=-37.234170;circle[4][11]=-41.626890;circle[4][12]=139.289247;circle[4][13]=-125.664201;circle[4][14]=8.360322;circle[4][15]=116.402219;circle[4][16]=-30.275710;circle[4][17]=-9.538003;circle[4][18]=-19.148139;circle[4][19]=5.815797;circle[4][20]=-19.033325;circle[4][21]=5.234218;circle[4][22]=-14.429993;circle[4][23]=4.710796;circle[4][24]=-12.986993;circle[4][25]=4.239716;circle[4][26]=-13.488294;circle[4][27]=2.915745;circle[4][28]=-8.539465;circle[4][29]=4.424170;circle[5][0]=0.000000;circle[5][1]=0.000000;circle[5][2]=-4.500000;circle[5][3]=4.500000;circle[5][4]=4.050000;circle[5][5]=5.850000;circle[5][6]=81.045000;circle[5][7]=-82.935000;circle[5][8]=-62.059500;circle[5][9]=102.658500;circle[5][10]=-41.453550;circle[5][11]=-67.807350;circle[5][12]=-78.708195;circle[5][13]=-74.526615;circle[5][14]=101.062625;circle[5][15]=19.326047;circle[5][16]=32.456362;circle[5][17]=83.093442;circle[5][18]=-11.289274;circle[5][19]=-14.315902;circle[5][20]=-10.160347;circle[5][21]=2.415688;circle[5][22]=-7.344312;circle[5][23]=4.874119;circle[5][24]=-4.809881;circle[5][25]=7.986707;circle[5][26]=-7.928893;circle[5][27]=6.288036;circle[5][28]=-5.336003;circle[5][29]=5.659233;circle[6][0]=0.000000;circle[6][1]=0.000000;circle[6][2]=-2.700000;circle[6][3]=3.600000;circle[6][4]=7.470000;circle[6][5]=-4.860000;circle[6][6]=7.623000;circle[6][7]=44.226000;circle[6][8]=-17.439300;circle[6][9]=62.303400;circle[6][10]=4.104630;circle[6][11]=-172.526940;circle[6][12]=-2.605833;circle[6][13]=-9.474246;circle[6][14]=-29.345250;circle[6][15]=80.573179;circle[6][16]=-8.410725;circle[6][17]=-20.184139;circle[6][18]=-2.169652;circle[6][19]=-0.165725;circle[6][20]=-1.052687;circle[6][21]=3.450847;circle[6][22]=-0.947418;circle[6][23]=5.805762;circle[6][24]=-3.552676;circle[6][25]=2.525186;circle[6][26]=0.402591;circle[6][27]=5.872668;circle[6][28]=4.862332;circle[6][29]=4.385401;circle[7][0]=0.000000;circle[7][1]=0.000000;circle[7][2]=-1.800000;circle[7][3]=3.600000;circle[7][4]=1.980000;circle[7][5]=6.840000;circle[7][6]=37.782000;circle[7][7]=6.156000;circle[7][8]=-224.296200;circle[7][9]=167.540400;circle[7][10]=53.733420;circle[7][11]=-253.313640;circle[7][12]=-16.439922;circle[7][13]=224.717724;circle[7][14]=25.704070;circle[7][15]=-26.354048;circle[7][16]=6.033663;circle[7][17]=-13.818644;circle[7][18]=7.230297;circle[7][19]=-10.636779;circle[7][20]=12.807267;circle[7][21]=-5.973101;circle[7][22]=3.426540;circle[7][23]=-7.175791;circle[7][24]=3.983886;circle[7][25]=-4.658212;circle[7][26]=4.485498;circle[7][27]=-6.892391;circle[7][28]=6.736948;circle[7][29]=-7.103152;circle[8][0]=0.000000;circle[8][1]=0.000000;circle[8][2]=-1.800000;circle[8][3]=0.900000;circle[8][4]=10.080000;circle[8][5]=27.810000;circle[8][6]=51.372000;circle[8][7]=-25.371000;circle[8][8]=2.134800;circle[8][9]=8.666100;circle[8][10]=20.821320;circle[8][11]=-10.200510;circle[8][12]=-72.160812;circle[8][13]=40.319541;circle[8][14]=97.955269;circle[8][15]=-201.312413;circle[8][16]=9.859742;circle[8][17]=-38.981172;circle[8][18]=-6.426232;circle[8][19]=52.216945;circle[8][20]=-54.383609;circle[8][21]=49.695251;circle[8][22]=-69.645248;circle[8][23]=114.025726;circle[8][24]=10.219277;circle[8][25]=-19.776847;circle[8][26]=3.797349;circle[8][27]=-0.699162;circle[8][28]=-0.182386;circle[8][29]=2.970754;circle[9][0]=0.000000;circle[9][1]=0.000000;circle[9][2]=-3.600000;circle[9][3]=4.500000;circle[9][4]=-1.440000;circle[9][5]=0.450000;circle[9][6]=-3.996000;circle[9][7]=7.605000;circle[9][8]=81.003600;circle[9][9]=-1.255500;circle[9][10]=-183.596760;circle[9][11]=73.570050;circle[9][12]=169.562916;circle[9][13]=103.113045;circle[9][14]=-106.593376;circle[9][15]=-71.898259;circle[9][16]=0.365962;circle[9][17]=-16.108434;circle[9][18]=3.929366;circle[9][19]=-9.097590;circle[9][20]=0.836429;circle[9][21]=-6.387831;circle[9][22]=-0.147214;circle[9][23]=-7.549048;circle[9][24]=0.767508;circle[9][25]=-4.094143;circle[9][26]=4.290757;circle[9][27]=-4.584729;circle[9][28]=-1.538319;circle[9][29]=-1.426256;circle[10][0]=0.000000;circle[10][1]=0.000000;circle[10][2]=-7.200000;circle[10][3]=1.800000;circle[10][4]=-1.080000;circle[10][5]=1.620000;circle[10][6]=6.228000;circle[10][7]=12.258000;circle[10][8]=2.905200;circle[10][9]=-14.167800;circle[10][10]=-27.985320;circle[10][11]=59.248980;circle[10][12]=5.413212;circle[10][13]=45.224082;circle[10][14]=-14.028109;circle[10][15]=17.301674;circle[10][16]=59.374702;circle[10][17]=-71.728494;circle[10][18]=40.837232;circle[10][19]=-138.355644;circle[10][20]=-82.046492;circle[10][21]=29.379920;circle[10][22]=-10.841842;circle[10][23]=-5.958072;circle[10][24]=-6.157658;circle[10][25]=4.537735;circle[10][26]=-1.941892;circle[10][27]=8.583962;circle[10][28]=2.752297;circle[10][29]=5.925566;circle[11][0]=0.000000;circle[11][1]=0.000000;circle[11][2]=-7.200000;circle[11][3]=3.600000;circle[11][4]=-4.680000;circle[11][5]=5.040000;circle[11][6]=-3.312000;circle[11][7]=-7.164000;circle[11][8]=-10.180800;circle[11][9]=6.152400;circle[11][10]=35.837280;circle[11][11]=-7.062840;circle[11][12]=-144.146448;circle[11][13]=44.043444;circle[11][14]=-172.931803;circle[11][15]=11.739100;circle[11][16]=210.661377;circle[11][17]=-37.134810;circle[11][18]=19.495239;circle[11][19]=55.678671;circle[11][20]=-40.954285;circle[11][21]=27.610804;circle[11][22]=6.341144;circle[11][23]=-30.950277;circle[11][24]=4.807030;circle[11][25]=-6.255249;circle[11][26]=6.126327;circle[11][27]=-7.429724;circle[11][28]=8.213694;circle[11][29]=0.513248;circle[12][0]=0.000000;circle[12][1]=0.000000;circle[12][2]=-1.800000;circle[12][3]=0.000000;circle[12][4]=-0.720000;circle[12][5]=0.900000;circle[12][6]=62.352000;circle[12][7]=-96.390000;circle[12][8]=-10.483200;circle[12][9]=-12.951000;circle[12][10]=-155.234880;circle[12][11]=10.844100;circle[12][12]=50.188608;circle[12][13]=-181.940310;circle[12][14]=53.269747;circle[12][15]=-20.646279;circle[12][16]=44.342772;circle[12][17]=71.418349;circle[12][18]=0.308495;circle[12][19]=-62.623486;circle[12][20]=-12.322354;circle[12][21]=102.038863;circle[12][22]=8.709881;circle[12][23]=16.234976;circle[12][24]=-20.061107;circle[12][25]=8.311479;circle[12][26]=-0.954996;circle[12][27]=8.380331;circle[12][28]=-3.559497;circle[12][29]=19.242298;circle[13][0]=0.000000;circle[13][1]=0.000000;circle[13][2]=-3.600000;circle[13][3]=0.000000;circle[13][4]=-2.340000;circle[13][5]=0.900000;circle[13][6]=49.194000;circle[13][7]=-47.790000;circle[13][8]=29.874600;circle[13][9]=11.889000;circle[13][10]=-67.612860;circle[13][11]=80.000100;circle[13][12]=23.748426;circle[13][13]=-23.399910;circle[13][14]=-56.026417;circle[13][15]=-31.859919;circle[13][16]=5.376225;circle[13][17]=-26.873927;circle[13][18]=9.338603;circle[13][19]=37.013466;circle[13][20]=-12.295258;circle[13][21]=28.812119;circle[13][22]=8.734268;circle[13][23]=61.930907;circle[13][24]=-6.539159;circle[13][25]=-48.662184;circle[13][26]=4.014757;circle[13][27]=-12.295965;circle[13][28]=-0.886719;circle[13][29]=-3.866369;circle[14][0]=0.000000;circle[14][1]=0.000000;circle[14][2]=-0.900000;circle[14][3]=0.000000;circle[14][4]=9.090000;circle[14][5]=-54.900000;circle[14][6]=36.981000;circle[14][7]=27.990000;circle[14][8]=57.582900;circle[14][9]=55.791000;circle[14][10]=-132.675390;circle[14][11]=-19.988100;circle[14][12]=-58.207851;circle[14][13]=21.610710;circle[14][14]=17.812934;circle[14][15]=-12.950361;circle[14][16]=-26.268359;circle[14][17]=64.844675;circle[14][18]=9.658477;circle[14][19]=88.060208;circle[14][20]=2.392629;circle[14][21]=-31.445813;circle[14][22]=3.953366;circle[14][23]=-14.801232;circle[14][24]=1.758029;circle[14][25]=-7.921109;circle[14][26]=2.482227;circle[14][27]=-6.228998;circle[14][28]=3.134004;circle[14][29]=-4.706098;circle[15][0]=0.000000;circle[15][1]=0.000000;circle[15][2]=0.000000;circle[15][3]=2.700000;circle[15][4]=10.800000;circle[15][5]=-146.070000;circle[15][6]=29.520000;circle[15][7]=42.237000;circle[15][8]=-20.232000;circle[15][9]=47.913300;circle[15][10]=-6.508800;circle[15][11]=-0.978030;circle[15][12]=-28.357920;circle[15][13]=-17.980227;circle[15][14]=64.477872;circle[15][15]=61.217796;circle[15][16]=6.730085;circle[15][17]=-43.903984;circle[15][18]=-29.042924;circle[15][19]=-16.113585;circle[15][20]=-9.938631;circle[15][21]=4.397773;circle[15][22]=-3.544768;circle[15][23]=8.457996;circle[15][24]=-4.990291;circle[15][25]=5.812196;circle[15][26]=-3.591262;circle[15][27]=7.030977;circle[15][28]=-5.932136;circle[15][29]=6.327879;circle[16][0]=0.000000;circle[16][1]=0.000000;circle[16][2]=-0.900000;circle[16][3]=2.700000;circle[16][4]=-1.710000;circle[16][5]=5.130000;circle[16][6]=4.761000;circle[16][7]=-0.783000;circle[16][8]=-1.115100;circle[16][9]=102.795300;circle[16][10]=-11.803590;circle[16][11]=3.415770;circle[16][12]=-7.023231;circle[16][13]=-15.825807;circle[16][14]=-4.520908;circle[16][15]=14.556774;circle[16][16]=112.031183;circle[16][17]=-128.198904;circle[16][18]=42.328065;circle[16][19]=101.520987;circle[16][20]=-13.204742;circle[16][21]=47.268888;circle[16][22]=-39.784268;circle[16][23]=-14.158001;circle[16][24]=-13.305841;circle[16][25]=-26.242201;circle[16][26]=-11.075257;circle[16][27]=-11.017981;circle[16][28]=-9.967731;circle[16][29]=-7.216183;circle[17][0]=0.000000;circle[17][1]=0.000000;circle[17][2]=-2.700000;circle[17][3]=2.700000;circle[17][4]=-1.530000;circle[17][5]=4.230000;circle[17][6]=65.223000;circle[17][7]=35.307000;circle[17][8]=-77.199300;circle[17][9]=74.976300;circle[17][10]=-160.379370;circle[17][11]=69.278670;circle[17][12]=-34.541433;circle[17][13]=1.150803;circle[17][14]=112.012710;circle[17][15]=-88.064277;circle[17][16]=40.511439;circle[17][17]=-17.157850;circle[17][18]=75.160295;circle[17][19]=82.657935;circle[17][20]=-30.455734;circle[17][21]=45.592142;circle[17][22]=2.289839;circle[17][23]=-51.667072;circle[17][24]=-1.539145;circle[17][25]=-18.600365;circle[17][26]=1.314770;circle[17][27]=-14.040329;circle[17][28]=-0.616707;circle[17][29]=-10.836296;circle[18][0]=0.000000;circle[18][1]=0.000000;circle[18][2]=-1.800000;circle[18][3]=2.700000;circle[18][4]=-4.320000;circle[18][5]=2.430000;circle[18][6]=13.212000;circle[18][7]=-51.813000;circle[18][8]=-48.409200;circle[18][9]=16.368300;circle[18][10]=-55.268280;circle[18][11]=50.731470;circle[18][12]=-28.141452;circle[18][13]=16.858323;circle[18][14]=97.972693;circle[18][15]=20.572491;circle[18][16]=31.475424;circle[18][17]=-6.684758;circle[18][18]=20.227881;circle[18][19]=29.983717;circle[18][20]=-6.994907;circle[18][21]=-16.214654;circle[18][22]=-1.795416;circle[18][23]=-1.093189;circle[18][24]=-6.115874;circle[18][25]=-2.783870;circle[18][26]=-4.604287;circle[18][27]=-2.505483;circle[18][28]=-3.243858;circle[18][29]=-2.254935;circle[19][0]=0.000000;circle[19][1]=0.000000;circle[19][2]=-0.900000;circle[19][3]=0.900000;circle[19][4]=0.090000;circle[19][5]=-0.090000;circle[19][6]=-6.219000;circle[19][7]=32.319000;circle[19][8]=20.502900;circle[19][9]=45.287100;circle[19][10]=-3.147390;circle[19][11]=2.958390;circle[19][12]=-106.332651;circle[19][13]=70.162551;circle[19][14]=-6.599386;circle[19][15]=63.146296;circle[19][16]=-51.839447;circle[19][17]=12.731666;circle[19][18]=28.944497;circle[19][19]=-58.741500;circle[19][20]=16.150048;circle[19][21]=6.532650;circle[19][22]=57.735043;circle[19][23]=-59.820615;circle[19][24]=31.261539;circle[19][25]=73.961446;circle[19][26]=-103.264615;circle[19][27]=122.365302;circle[19][28]=21.361846;circle[19][29]=-75.271229;circle[20][0]=0.000000;circle[20][1]=0.000000;circle[20][2]=-5.400000;circle[20][3]=-16.200000;circle[20][4]=0.540000;circle[20][5]=-15.480000;circle[20][6]=30.186000;circle[20][7]=-26.532000;circle[20][8]=-106.932600;circle[20][9]=78.721200;circle[20][10]=-12.539340;circle[20][11]=25.849080;circle[20][12]=25.614594;circle[20][13]=-55.935828;circle[20][14]=7.753135;circle[20][15]=26.157755;circle[20][16]=94.277821;circle[20][17]=11.841979;circle[20][18]=-41.149961;circle[20][19]=2.557781;circle[20][20]=-1.034965;circle[20][21]=-8.497997;circle[20][22]=-0.931468;circle[20][23]=-1.348197;circle[20][24]=-4.438322;circle[20][25]=-0.313377;circle[20][26]=-3.994489;circle[20][27]=0.617960;circle[20][28]=-4.495040;circle[20][29]=-0.343836;circle[21][0]=0.000000;circle[21][1]=0.000000;circle[21][2]=-2.700000;circle[21][3]=2.700000;circle[21][4]=-3.330000;circle[21][5]=-0.270000;circle[21][6]=36.603000;circle[21][7]=-142.443000;circle[21][8]=106.742700;circle[21][9]=28.401300;circle[21][10]=-95.631570;circle[21][11]=43.561170;circle[21][12]=14.731587;circle[21][13]=-34.594947;circle[21][14]=-8.341572;circle[21][15]=-73.435452;circle[21][16]=79.792585;circle[21][17]=89.608093;circle[21][18]=-23.586673;circle[21][19]=-8.452716;circle[21][20]=-14.928006;circle[21][21]=4.092555;circle[21][22]=-14.335205;circle[21][23]=9.083300;circle[21][24]=-11.101685;circle[21][25]=10.874970;circle[21][26]=-10.891516;circle[21][27]=7.987473;circle[21][28]=-11.602365;circle[21][29]=7.188726;circle[22][0]=0.000000;circle[22][1]=0.000000;circle[22][2]=-0.900000;circle[22][3]=2.700000;circle[22][4]=28.890000;circle[22][5]=11.430000;circle[22][6]=-3.699000;circle[22][7]=-11.313000;circle[22][8]=-46.529100;circle[22][9]=60.918300;circle[22][10]=-16.676190;circle[22][11]=-76.573530;circle[22][12]=86.691429;circle[22][13]=21.083823;circle[22][14]=-14.677714;circle[22][15]=41.475441;circle[22][16]=-24.009943;circle[22][17]=-14.872103;circle[22][18]=-2.708948;circle[22][19]=-4.384893;circle[22][20]=-3.338053;circle[22][21]=-0.346404;circle[22][22]=-3.004248;circle[22][23]=3.288237;circle[22][24]=-8.103823;circle[22][25]=-3.340587;circle[22][26]=-3.693441;circle[22][27]=0.593472;circle[22][28]=-2.424097;circle[22][29]=1.434125;circle[23][0]=0.000000;circle[23][1]=0.000000;circle[23][2]=-4.500000;circle[23][3]=2.700000;circle[23][4]=-5.850000;circle[23][5]=2.430000;circle[23][6]=49.635000;circle[23][7]=31.887000;circle[23][8]=-197.428500;circle[23][9]=87.198300;circle[23][10]=38.314350;circle[23][11]=-163.621530;circle[23][12]=41.682915;circle[23][13]=68.740623;circle[23][14]=-7.485376;circle[23][15]=-13.733439;circle[23][16]=-6.736839;circle[23][17]=-3.360095;circle[23][18]=9.236845;circle[23][19]=-3.924086;circle[23][20]=2.913161;circle[23][21]=2.768323;circle[23][22]=4.421844;circle[23][23]=0.691490;circle[23][24]=2.179660;circle[23][25]=-1.177659;circle[23][26]=0.161694;circle[23][27]=2.540107;circle[23][28]=-2.554475;circle[23][29]=-0.413903;circle[24][0]=0.000000;circle[24][1]=0.000000;circle[24][2]=-1.800000;circle[24][3]=4.500000;circle[24][4]=-1.620000;circle[24][5]=7.650000;circle[24][6]=-5.958000;circle[24][7]=-10.215000;circle[24][8]=28.837800;circle[24][9]=21.406500;circle[24][10]=11.554020;circle[24][11]=-26.634150;circle[24][12]=-26.501382;circle[24][13]=53.429265;circle[24][14]=-8.551244;circle[24][15]=-93.213662;circle[24][16]=57.103881;circle[24][17]=-19.992295;circle[24][18]=-19.706507;circle[24][19]=134.106934;circle[24][20]=-16.835857;circle[24][21]=-9.803759;circle[24][22]=-10.652271;circle[24][23]=-5.223383;circle[24][24]=-6.887044;circle[24][25]=-5.601045;circle[24][26]=-4.398340;circle[24][27]=-2.340940;circle[24][28]=-3.958506;circle[24][29]=-4.806846;circle[25][0]=0.000000;circle[25][1]=0.000000;circle[25][2]=-5.400000;circle[25][3]=-4.500000;circle[25][4]=-5.760000;circle[25][5]=3.150000;circle[25][6]=-17.784000;circle[25][7]=-66.465000;circle[25][8]=9.194400;circle[25][9]=102.181500;circle[25][10]=117.174960;circle[25][11]=-16.936650;circle[25][12]=-110.542536;circle[25][13]=46.857015;circle[25][14]=38.211718;circle[25][15]=-32.528686;circle[25][16]=71.290546;circle[25][17]=-48.175818;circle[25][18]=-5.138509;circle[25][19]=1.641764;circle[25][20]=-65.824658;circle[25][21]=120.277588;circle[25][22]=-7.942192;circle[25][23]=-18.650171;circle[25][24]=-1.747973;circle[25][25]=-1.485154;circle[25][26]=-98.773176;circle[25][27]=3.163361;circle[25][28]=6.504142;circle[25][29]=-7.052975;circle[26][0]=0.000000;circle[26][1]=0.000000;circle[26][2]=-3.600000;circle[26][3]=6.300000;circle[26][4]=0.360000;circle[26][5]=6.570000;circle[26][6]=5.724000;circle[26][7]=9.513000;circle[26][8]=80.751600;circle[26][9]=43.661700;circle[26][10]=-92.923560;circle[26][11]=-22.804470;circle[26][12]=7.268796;circle[26][13]=-79.024023;circle[26][14]=-23.158084;circle[26][15]=-24.321621;circle[26][16]=-11.842275;circle[26][17]=98.710541;circle[26][18]=-6.158048;circle[26][19]=-23.660513;circle[26][20]=-1.942243;circle[26][21]=-3.294461;circle[26][22]=0.051981;circle[26][23]=1.534985;circle[26][24]=-0.853217;circle[26][25]=1.381486;circle[26][26]=0.132105;circle[26][27]=0.343338;circle[26][28]=-0.781106;circle[26][29]=1.209004;circle[27][0]=0.000000;circle[27][1]=0.000000;circle[27][2]=-3.600000;circle[27][3]=1.800000;circle[27][4]=-1.440000;circle[27][5]=2.520000;circle[27][6]=14.904000;circle[27][7]=13.968000;circle[27][8]=55.713600;circle[27][9]=42.271200;circle[27][10]=-67.757760;circle[27][11]=-54.655920;circle[27][12]=-40.281984;circle[27][13]=-53.690328;circle[27][14]=101.446214;circle[27][15]=-122.121295;circle[27][16]=-7.698407;circle[27][17]=163.690834;circle[27][18]=-27.628566;circle[27][19]=55.521751;circle[27][20]=-12.265710;circle[27][21]=-19.330424;circle[27][22]=-9.239139;circle[27][23]=-4.797382;circle[27][24]=-3.815225;circle[27][25]=-0.717644;circle[27][26]=-3.433702;circle[27][27]=-0.645879;circle[27][28]=-3.990332;circle[27][29]=1.218709;circle[28][0]=0.000000;circle[28][1]=0.000000;circle[28][2]=-4.500000;circle[28][3]=4.500000;circle[28][4]=-0.450000;circle[28][5]=1.350000;circle[28][6]=-4.905000;circle[28][7]=-4.185000;circle[28][8]=81.085500;circle[28][9]=-8.266500;circle[28][10]=-143.023050;circle[28][11]=65.460150;circle[28][12]=-54.020745;circle[28][13]=13.014135;circle[28][14]=23.381330;circle[28][15]=-55.787278;circle[28][16]=-5.056803;circle[28][17]=-85.308551;circle[28][18]=55.748877;circle[28][19]=47.422304;circle[28][20]=-36.226011;circle[28][21]=73.280074;circle[28][22]=8.796590;circle[28][23]=-17.747933;circle[28][24]=7.916931;circle[28][25]=-4.273140;circle[28][26]=3.525238;circle[28][27]=0.654174;circle[28][28]=5.872714;circle[28][29]=-1.211243;circle[29][0]=0.000000;circle[29][1]=0.000000;circle[29][2]=0.000000;circle[29][3]=0.000000;circle[29][4]=-1.800000;circle[29][5]=5.400000;circle[29][6]=28.980000;circle[29][7]=4.860000;circle[29][8]=-29.718000;circle[29][9]=50.274000;circle[29][10]=44.353800;circle[29][11]=-14.153400;circle[29][12]=-109.481580;circle[29][13]=-129.738060;circle[29][14]=68.866578;circle[29][15]=28.135746;circle[29][16]=25.979920;circle[29][17]=57.722171;circle[29][18]=-135.918072;circle[29][19]=77.149954;circle[29][20]=12.673735;circle[29][21]=-21.465041;circle[29][22]=6.006362;circle[29][23]=-5.818537;circle[29][24]=8.105726;circle[29][25]=-1.636683;circle[29][26]=4.595153;circle[29][27]=-3.273015;circle[29][28]=5.935638;circle[29][29]=0.654286;circle[30][0]=0.000000;circle[30][1]=0.000000;circle[30][2]=-3.600000;circle[30][3]=2.700000;circle[30][4]=-4.140000;circle[30][5]=1.530000;circle[30][6]=9.774000;circle[30][7]=-10.323000;circle[30][8]=-11.903400;circle[30][9]=36.609300;circle[30][10]=-9.813060;circle[30][11]=41.948370;circle[30][12]=50.568246;circle[30][13]=-144.946467;circle[30][14]=29.311421;circle[30][15]=104.448180;circle[30][16]=-23.119721;circle[30][17]=-23.896638;circle[30][18]=-13.607749;circle[30][19]=-5.306974;circle[30][20]=-5.946974;circle[30][21]=2.423723;circle[30][22]=-5.352276;circle[30][23]=0.381351;circle[30][24]=-3.917049;circle[30][25]=1.243216;circle[30][26]=-2.625344;circle[30][27]=2.018894;circle[30][28]=-5.962810;circle[30][29]=2.717005;circle[31][0]=0.000000;circle[31][1]=0.000000;circle[31][2]=-1.800000;circle[31][3]=-4.500000;circle[31][4]=-43.920000;circle[31][5]=-33.750000;circle[31][6]=35.172000;circle[31][7]=-10.575000;circle[31][8]=-5.245200;circle[31][9]=59.782500;circle[31][10]=-114.520680;circle[31][11]=28.604250;circle[31][12]=-103.968612;circle[31][13]=-117.356175;circle[31][14]=128.728249;circle[31][15]=108.579443;circle[31][16]=-51.544576;circle[31][17]=21.221498;circle[31][18]=-9.490118;circle[31][19]=-16.900652;circle[31][20]=11.258894;circle[31][21]=0.089414;circle[31][22]=2.033004;circle[31][23]=6.380472;circle[31][24]=5.429704;circle[31][25]=-0.557575;circle[31][26]=7.586733;circle[31][27]=-10.401817;circle[31][28]=6.828060;circle[31][29]=-7.561636;circle[32][0]=0.000000;circle[32][1]=0.000000;circle[32][2]=-2.700000;circle[32][3]=2.700000;circle[32][4]=-6.030000;circle[32][5]=0.630000;circle[32][6]=76.473000;circle[32][7]=-129.033000;circle[32][8]=-103.974300;circle[32][9]=166.470300;circle[32][10]=-26.976870;circle[32][11]=109.323270;circle[32][12]=126.020817;circle[32][13]=-359.709057;circle[32][14]=123.318735;circle[32][15]=290.061849;circle[32][16]=-30.313138;circle[32][17]=-41.344336;circle[32][18]=-21.881824;circle[32][19]=17.690097;circle[32][20]=-19.693642;circle[32][21]=-6.578912;circle[32][22]=8.375722;circle[32][23]=3.078979;circle[32][24]=-14.961850;circle[32][25]=-1.728919;circle[32][26]=-15.265665;circle[32][27]=-2.456027;circle[32][28]=-16.439098;circle[32][29]=-1.310424;circle[33][0]=0.000000;circle[33][1]=0.000000;circle[33][2]=0.000000;circle[33][3]=3.600000;circle[33][4]=-3.600000;circle[33][5]=-25.560000;circle[33][6]=87.660000;circle[33][7]=82.296000;circle[33][8]=-260.406000;circle[33][9]=115.466400;circle[33][10]=-14.765400;circle[33][11]=121.919760;circle[33][12]=148.711140;circle[33][13]=-126.972216;circle[33][14]=-16.459974;circle[33][15]=108.925006;circle[33][16]=-13.013977;circle[33][17]=-35.167495;circle[33][18]=5.387421;circle[33][19]=-21.750745;circle[33][20]=3.048679;circle[33][21]=-22.275671;circle[33][22]=6.343811;circle[33][23]=-18.248104;circle[33][24]=-2.390570;circle[33][25]=-20.923293;circle[33][26]=2.348487;circle[33][27]=-20.630964;circle[33][28]=1.213638;circle[33][29]=-15.867868;circle[34][0]=0.000000;circle[34][1]=0.000000;circle[34][2]=-1.800000;circle[34][3]=0.000000;circle[34][4]=-4.320000;circle[34][5]=0.000000;circle[34][6]=3.312000;circle[34][7]=-56.700000;circle[34][8]=147.880800;circle[34][9]=-1.530000;circle[34][10]=-199.007280;circle[34][11]=189.423000;circle[34][12]=-55.806552;circle[34][13]=57.080700;circle[34][14]=241.374103;circle[34][15]=-116.027370;circle[34][16]=-17.663307;circle[34][17]=195.275367;circle[34][18]=-29.396976;circle[34][19]=-45.652170;circle[34][20]=-22.857279;circle[34][21]=-20.386953;circle[34][22]=-14.271551;circle[34][23]=-20.148257;circle[34][24]=-8.344396;circle[34][25]=-20.833432;circle[34][26]=-10.209956;circle[34][27]=-18.750089;circle[34][28]=-6.488961;circle[34][29]=-15.075080;circle[35][0]=0.000000;circle[35][1]=0.000000;circle[35][2]=3.600000;circle[35][3]=8.100000;circle[35][4]=-21.060000;circle[35][5]=-7.110000;circle[35][6]=41.346000;circle[35][7]=71.901000;circle[35][8]=12.911400;circle[35][9]=13.410900;circle[35][10]=-105.379740;circle[35][11]=-33.830190;circle[35][12]=11.358234;circle[35][13]=-61.947171;circle[35][14]=46.222411;circle[35][15]=-101.652454;circle[35][16]=-50.199830;circle[35][17]=84.012791;circle[35][18]=14.220153;circle[35][19]=-19.788488;circle[35][20]=4.698137;circle[35][21]=11.890361;circle[35][22]=-13.771676;circle[35][23]=-11.798675;circle[35][24]=-15.094509;circle[35][25]=-8.818808;circle[35][26]=8.014942;circle[35][27]=-3.436927;circle[35][28]=5.413448;circle[35][29]=-3.993234;circle[36][0]=0.000000;circle[36][1]=0.000000;circle[36][2]=-1.800000;circle[36][3]=-2.700000;circle[36][4]=-40.320000;circle[36][5]=-29.430000;circle[36][6]=-12.888000;circle[36][7]=155.313000;circle[36][8]=-1.699200;circle[36][9]=0.281700;circle[36][10]=-132.929280;circle[36][11]=-153.646470;circle[36][12]=159.363648;circle[36][13]=-59.081823;circle[36][14]=-14.072717;circle[36][15]=49.426359;circle[36][16]=-8.165445;circle[36][17]=-20.316277;circle[36][18]=4.351099;circle[36][19]=4.215351;circle[36][20]=-1.484011;circle[36][21]=9.193816;circle[36][22]=0.464391;circle[36][23]=7.374434;circle[36][24]=-2.282049;circle[36][25]=7.536991;circle[36][26]=-0.253844;circle[36][27]=6.783292;circle[36][28]=1.571541;circle[36][29]=7.004963;circle[37][0]=0.000000;circle[37][1]=0.000000;circle[37][2]=-1.800000;circle[37][3]=-0.900000;circle[37][4]=-10.620000;circle[37][5]=-4.410000;circle[37][6]=50.742000;circle[37][7]=59.931000;circle[37][8]=12.367800;circle[37][9]=9.837900;circle[37][10]=-114.868980;circle[37][11]=8.854110;circle[37][12]=72.117918;circle[37][13]=-57.731301;circle[37][14]=46.906126;circle[37][15]=204.541829;circle[37][16]=-13.584486;circle[37][17]=-37.312354;circle[37][18]=-12.226038;circle[37][19]=-24.581118;circle[37][20]=-7.403434;circle[37][21]=-15.823007;circle[37][22]=-3.963091;circle[37][23]=-16.040706;circle[37][24]=-8.066782;circle[37][25]=-9.036635;circle[37][26]=-9.060103;circle[37][27]=-6.332972;circle[37][28]=-8.154093;circle[37][29]=-9.299675;circle[38][0]=0.000000;circle[38][1]=0.000000;circle[38][2]=0.000000;circle[38][3]=0.000000;circle[38][4]=27.900000;circle[38][5]=6.300000;circle[38][6]=60.210000;circle[38][7]=-20.430000;circle[38][8]=-147.411000;circle[38][9]=75.213000;circle[38][10]=-28.269900;circle[38][11]=96.491700;circle[38][12]=32.157090;circle[38][13]=-72.457470;circle[38][14]=19.041381;circle[38][15]=71.588277;circle[38][16]=-22.462757;circle[38][17]=-56.170551;circle[38][18]=8.583519;circle[38][19]=-13.653496;circle[38][20]=-5.774833;circle[38][21]=-10.488146;circle[38][22]=4.702650;circle[38][23]=-4.939331;circle[38][24]=2.432385;circle[38][25]=-6.245398;circle[38][26]=3.989147;circle[38][27]=-1.120858;circle[38][28]=2.690232;circle[38][29]=-1.908773;circle[39][0]=0.000000;circle[39][1]=0.000000;circle[39][2]=-0.900000;circle[39][3]=1.800000;circle[39][4]=0.090000;circle[39][5]=4.320000;circle[39][6]=-12.519000;circle[39][7]=-21.312000;circle[39][8]=76.932900;circle[39][9]=-14.680800;circle[39][10]=61.139610;circle[39][11]=97.487280;circle[39][12]=-142.074351;circle[39][13]=13.938552;circle[39][14]=146.633084;circle[39][15]=-154.855303;circle[39][16]=1.469776;circle[39][17]=71.230227;circle[39][18]=-27.477202;circle[39][19]=-24.092796;circle[39][20]=-15.729482;circle[39][21]=-8.183516;circle[39][22]=-3.356534;circle[39][23]=5.234836;circle[39][24]=-14.720880;circle[39][25]=6.511352;circle[39][26]=-9.648792;circle[39][27]=6.760217;circle[39][28]=-9.583913;circle[39][29]=4.284195;
  #endif
}
void fill_matrix_square(){
  #ifdef TRAINING
  square[0][0]=0.000000;square[0][1]=0.000000;square[0][2]=1.800000;square[0][3]=0.000000;square[0][4]=-1.980000;square[0][5]=4.500000;square[0][6]=-6.282000;square[0][7]=1.350000;square[0][8]=0.646200;square[0][9]=-32.985000;square[0][10]=-71.418420;square[0][11]=75.613500;square[0][12]=20.323422;square[0][13]=16.752150;square[0][14]=94.791080;square[0][15]=-182.923065;square[0][16]=90.711972;square[0][17]=-109.730759;square[0][18]=9.640775;square[0][19]=14.642317;square[0][20]=-24.623303;square[0][21]=142.778086;square[0][22]=-59.960973;square[0][23]=88.900277;square[0][24]=9.935125;square[0][25]=-29.789751;square[0][26]=-64.858388;square[0][27]=71.289224;square[0][28]=82.027451;square[0][29]=12.860302;square[1][0]=0.000000;square[1][1]=0.000000;square[1][2]=0.000000;square[1][3]=1.800000;square[1][4]=0.000000;square[1][5]=3.420000;square[1][6]=-3.600000;square[1][7]=1.278000;square[1][8]=-10.440000;square[1][9]=0.250200;square[1][10]=-10.296000;square[1][11]=-46.574820;square[1][12]=8.733600;square[1][13]=1.282662;square[1][14]=-17.339760;square[1][15]=17.354396;square[1][16]=-113.705784;square[1][17]=-105.881044;square[1][18]=44.364794;square[1][19]=80.207061;square[1][20]=18.328315;square[1][21]=-1.613645;square[1][22]=234.295483;square[1][23]=-51.852281;square[1][24]=-115.834065;square[1][25]=-144.767053;square[1][26]=1.949342;square[1][27]=45.209652;square[1][28]=-1.845593;square[1][29]=16.388687;square[2][0]=0.000000;square[2][1]=0.000000;square[2][2]=-3.600000;square[2][3]=0.900000;square[2][4]=-0.540000;square[2][5]=2.610000;square[2][6]=18.414000;square[2][7]=-17.451000;square[2][8]=-1.427400;square[2][9]=10.394100;square[2][10]=-45.384660;square[2][11]=-12.245310;square[2][12]=15.853806;square[2][13]=68.179221;square[2][14]=3.468425;square[2][15]=-21.438701;square[2][16]=-12.178417;square[2][17]=25.705169;square[2][18]=-11.860575;square[2][19]=17.734652;square[2][20]=27.125482;square[2][21]=14.161187;square[2][22]=70.312934;square[2][23]=109.045068;square[2][24]=30.881641;square[2][25]=-152.059439;square[2][26]=-1.006524;square[2][27]=-82.853495;square[2][28]=51.294129;square[2][29]=119.831855;square[3][0]=0.000000;square[3][1]=0.000000;square[3][2]=-4.500000;square[3][3]=5.400000;square[3][4]=-15.750000;square[3][5]=29.160000;square[3][6]=3.825000;square[3][7]=-5.256000;square[3][8]=-33.457500;square[3][9]=-13.730400;square[3][10]=156.188250;square[3][11]=30.842640;square[3][12]=-178.930575;square[3][13]=-75.741624;square[3][14]=-50.337518;square[3][15]=-126.667462;square[3][16]=-60.603766;square[3][17]=53.399285;square[3][18]=24.656611;square[3][19]=172.259356;square[3][20]=-40.809050;square[3][21]=-77.166580;square[3][22]=215.271855;square[3][23]=-104.549922;square[3][24]=-29.455331;square[3][25]=36.405071;square[3][26]=-60.709798;square[3][27]=-178.735436;square[3][28]=43.461182;square[3][29]=24.538107;square[4][0]=0.000000;square[4][1]=0.000000;square[4][2]=-2.700000;square[4][3]=0.000000;square[4][4]=-2.430000;square[4][5]=0.000000;square[4][6]=1.413000;square[4][7]=-2.700000;square[4][8]=46.271700;square[4][9]=-9.630000;square[4][10]=-46.555470;square[4][11]=100.233000;square[4][12]=-22.099923;square[4][13]=-42.990300;square[4][14]=89.010069;square[4][15]=26.108730;square[4][16]=9.909062;square[4][17]=30.697857;square[4][18]=-27.081844;square[4][19]=20.428071;square[4][20]=-4.573659;square[4][21]=4.885264;square[4][22]=-14.016294;square[4][23]=-128.803262;square[4][24]=-11.714664;square[4][25]=29.877064;square[4][26]=-17.743198;square[4][27]=-18.110642;square[4][28]=-72.668878;square[4][29]=31.400422;square[5][0]=0.000000;square[5][1]=0.000000;square[5][2]=-0.900000;square[5][3]=0.900000;square[5][4]=-0.810000;square[5][5]=2.610000;square[5][6]=-23.229000;square[5][7]=-15.651000;square[5][8]=-65.906100;square[5][9]=-23.085900;square[5][10]=-5.315490;square[5][11]=54.822690;square[5][12]=-7.483941;square[5][13]=-1.959579;square[5][14]=144.464453;square[5][15]=-10.763621;square[5][16]=67.018008;square[5][17]=13.712741;square[5][18]=-30.583793;square[5][19]=15.041467;square[5][20]=-95.025414;square[5][21]=21.637320;square[5][22]=32.377128;square[5][23]=-85.826412;square[5][24]=55.239415;square[5][25]=10.056229;square[5][26]=-65.484527;square[5][27]=73.850606;square[5][28]=-22.036074;square[5][29]=2.565546;square[6][0]=0.000000;square[6][1]=0.000000;square[6][2]=-1.800000;square[6][3]=3.600000;square[6][4]=-0.720000;square[6][5]=1.440000;square[6][6]=-40.248000;square[6][7]=10.296000;square[6][8]=-34.423200;square[6][9]=-32.133600;square[6][10]=64.419120;square[6][11]=1.679760;square[6][12]=-38.322792;square[6][13]=-66.888216;square[6][14]=-2.090513;square[6][15]=80.200606;square[6][16]=-2.781462;square[6][17]=-26.819455;square[6][18]=-32.203315;square[6][19]=-34.037509;square[6][20]=-8.282984;square[6][21]=-1.833759;square[6][22]=-11.054685;square[6][23]=1.049617;square[6][24]=-2.749217;square[6][25]=101.744656;square[6][26]=73.125705;square[6][27]=-247.729810;square[6][28]=-14.286866;square[6][29]=95.643171;square[7][0]=0.000000;square[7][1]=0.000000;square[7][2]=-1.800000;square[7][3]=0.000000;square[7][4]=-4.320000;square[7][5]=1.800000;square[7][6]=-15.588000;square[7][7]=-64.980000;square[7][8]=-49.129200;square[7][9]=79.218000;square[7][10]=-1.916280;square[7][11]=7.396200;square[7][12]=66.675348;square[7][13]=39.956580;square[7][14]=68.107813;square[7][15]=34.160922;square[7][16]=-24.202968;square[7][17]=28.044830;square[7][18]=-78.482671;square[7][19]=-30.559653;square[7][20]=55.365596;square[7][21]=-97.703688;square[7][22]=37.229036;square[7][23]=3.866681;square[7][24]=28.106133;square[7][25]=-112.619987;square[7][26]=29.795519;square[7][27]=8.442012;square[7][28]=-0.184033;square[7][29]=11.197810;square[8][0]=0.000000;square[8][1]=0.000000;square[8][2]=-3.600000;square[8][3]=0.900000;square[8][4]=-5.940000;square[8][5]=-0.090000;square[8][6]=-10.746000;square[8][7]=-63.981000;square[8][8]=103.728600;square[8][9]=-27.882900;square[8][10]=20.455740;square[8][11]=116.205390;square[8][12]=-10.389834;square[8][13]=-2.515149;square[8][14]=50.949149;square[8][15]=67.036366;square[8][16]=5.354234;square[8][17]=-12.567271;square[8][18]=-89.681189;square[8][19]=24.689456;square[8][20]=-5.113070;square[8][21]=-140.679489;square[8][22]=26.898237;square[8][23]=28.188460;square[8][24]=-47.791587;square[8][25]=-61.030386;square[8][26]=-17.812428;square[8][27]=46.772652;square[8][28]=9.168815;square[8][29]=6.095387;square[9][0]=0.000000;square[9][1]=0.000000;square[9][2]=-0.900000;square[9][3]=1.800000;square[9][4]=-1.710000;square[9][5]=-0.180000;square[9][6]=-3.339000;square[9][7]=3.438000;square[9][8]=14.994900;square[9][9]=7.594200;square[9][10]=-43.204590;square[9][11]=31.134780;square[9][12]=17.815869;square[9][13]=-46.678698;square[9][14]=17.834282;square[9][15]=12.889172;square[9][16]=-0.149146;square[9][17]=-45.999745;square[9][18]=-71.234231;square[9][19]=32.400229;square[9][20]=-11.010808;square[9][21]=-12.239794;square[9][22]=-21.609728;square[9][23]=-49.715814;square[9][24]=-31.148755;square[9][25]=49.755767;square[9][26]=-16.333879;square[9][27]=42.080190;square[9][28]=8.699509;square[9][29]=-17.927829;square[10][0]=0.000000;square[10][1]=0.000000;square[10][2]=0.000000;square[10][3]=3.600000;square[10][4]=-2.700000;square[10][5]=1.440000;square[10][6]=-6.930000;square[10][7]=11.196000;square[10][8]=-49.437000;square[10][9]=-55.623600;square[10][10]=-10.293300;square[10][11]=-102.261240;square[10][12]=4.236030;square[10][13]=55.564884;square[10][14]=22.712427;square[10][15]=-40.891604;square[10][16]=118.541184;square[10][17]=15.397556;square[10][18]=-22.912934;square[10][19]=6.657800;square[10][20]=7.278359;square[10][21]=27.592020;square[10][22]=-17.749477;square[10][23]=-25.567182;square[10][24]=-11.474529;square[10][25]=-26.610463;square[10][26]=-23.827076;square[10][27]=-68.049417;square[10][28]=-65.544368;square[10][29]=44.055525;square[11][0]=0.000000;square[11][1]=0.000000;square[11][2]=0.900000;square[11][3]=0.000000;square[11][4]=-1.890000;square[11][5]=1.800000;square[11][6]=-0.801000;square[11][7]=-1.080000;square[11][8]=-2.520900;square[11][9]=-57.672000;square[11][10]=24.731190;square[11][11]=74.995200;square[11][12]=-58.741929;square[11][13]=-42.304320;square[11][14]=41.632264;square[11][15]=10.526112;square[11][16]=52.769038;square[11][17]=-1.326499;square[11][18]=-7.407866;square[11][19]=39.306151;square[11][20]=-10.267080;square[11][21]=-42.924464;square[11][22]=3.359628;square[11][23]=-6.232018;square[11][24]=22.823666;square[11][25]=-6.508816;square[11][26]=-96.458701;square[11][27]=-42.757935;square[11][28]=18.487169;square[11][29]=15.517859;square[12][0]=0.000000;square[12][1]=0.000000;square[12][2]=0.900000;square[12][3]=0.900000;square[12][4]=-3.690000;square[12][5]=2.610000;square[12][6]=51.579000;square[12][7]=20.349000;square[12][8]=-3.078900;square[12][9]=-68.985900;square[12][10]=-18.971010;square[12][11]=42.312690;square[12][12]=-22.473909;square[12][13]=-60.918579;square[12][14]=98.573482;square[12][15]=73.873279;square[12][16]=-12.983866;square[12][17]=-39.714049;square[12][18]=-45.885480;square[12][19]=-35.742644;square[12][20]=17.203068;square[12][21]=33.531620;square[12][22]=-65.517239;square[12][23]=245.278458;square[12][24]=68.834485;square[12][25]=-118.549388;square[12][26]=-18.148963;square[12][27]=65.205551;square[12][28]=-10.034067;square[12][29]=1.084996;square[13][0]=0.000000;square[13][1]=0.000000;square[13][2]=-4.500000;square[13][3]=0.000000;square[13][4]=-5.850000;square[13][5]=1.800000;square[13][6]=21.735000;square[13][7]=29.520000;square[13][8]=-29.038500;square[13][9]=5.868000;square[13][10]=-25.234650;square[13][11]=16.081200;square[13][12]=-7.411185;square[13][13]=5.473080;square[13][14]=-13.870067;square[13][15]=-5.874228;square[13][16]=59.516940;square[13][17]=90.113195;square[13][18]=7.665246;square[13][19]=-16.998125;square[13][20]=13.198722;square[13][21]=-43.198312;square[13][22]=-1.621151;square[13][23]=43.921519;square[13][24]=-58.159036;square[13][25]=-27.970633;square[13][26]=-8.243132;square[13][27]=99.926430;square[13][28]=-11.918819;square[13][29]=-141.366213;square[14][0]=0.000000;square[14][1]=0.000000;square[14][2]=-2.700000;square[14][3]=4.500000;square[14][4]=-1.530000;square[14][5]=4.950000;square[14][6]=40.923000;square[14][7]=5.355000;square[14][8]=-47.769300;square[14][9]=10.219500;square[14][10]=-36.692370;square[14][11]=-0.702450;square[14][12]=2.976867;square[14][13]=-56.432205;square[14][14]=47.679180;square[14][15]=121.111015;square[14][16]=24.911262;square[14][17]=108.099914;square[14][18]=-35.179864;square[14][19]=-44.910077;square[14][20]=-62.261878;square[14][21]=-34.119070;square[14][22]=17.764310;square[14][23]=-32.507163;square[14][24]=57.387879;square[14][25]=-15.756446;square[14][26]=-5.950909;square[14][27]=-57.380802;square[14][28]=51.344182;square[14][29]=47.357278;square[15][0]=0.000000;square[15][1]=0.000000;square[15][2]=1.800000;square[15][3]=2.700000;square[15][4]=-5.580000;square[15][5]=1.530000;square[15][6]=12.978000;square[15][7]=12.177000;square[15][8]=17.980200;square[15][9]=-4.340700;square[15][10]=-36.017820;square[15][11]=-3.006630;square[15][12]=17.983962;square[15][13]=0.894033;square[15][14]=-23.414434;square[15][15]=-45.095370;square[15][16]=14.927009;square[15][17]=65.614167;square[15][18]=-76.565692;square[15][19]=-30.947250;square[15][20]=66.990877;square[15][21]=-39.552525;square[15][22]=73.791790;square[15][23]=12.102728;square[15][24]=16.912611;square[15][25]=126.092455;square[15][26]=-6.378650;square[15][27]=-137.616791;square[15][28]=-12.940785;square[15][29]=15.644888;square[16][0]=0.000000;square[16][1]=0.000000;square[16][2]=-2.700000;square[16][3]=1.800000;square[16][4]=-7.830000;square[16][5]=6.120000;square[16][6]=-45.747000;square[16][7]=-52.092000;square[16][8]=38.027700;square[16][9]=100.717200;square[16][10]=54.024930;square[16][11]=-31.754520;square[16][12]=16.222437;square[16][13]=-79.879068;square[16][14]=-51.999807;square[16][15]=34.308839;square[16][16]=60.300174;square[16][17]=83.077955;square[16][18]=-20.429843;square[16][19]=-125.029841;square[16][20]=-10.286859;square[16][21]=-9.926857;square[16][22]=-47.958173;square[16][23]=31.565829;square[16][24]=42.337644;square[16][25]=-36.390754;square[16][26]=-17.696120;square[16][27]=7.748322;square[16][28]=-9.626508;square[16][29]=11.473489;square[17][0]=0.000000;square[17][1]=0.000000;square[17][2]=-1.800000;square[17][3]=3.600000;square[17][4]=-3.420000;square[17][5]=4.140000;square[17][6]=-0.378000;square[17][7]=-0.774000;square[17][8]=-18.340200;square[17][9]=-140.196600;square[17][10]=154.493820;square[17][11]=117.723060;square[17][12]=15.744438;square[17][13]=-32.649246;square[17][14]=-17.330006;square[17][15]=18.315679;square[17][16]=-29.097005;square[17][17]=43.484111;square[17][18]=9.812695;square[17][19]=-14.864300;square[17][20]=83.531426;square[17][21]=-259.077870;square[17][22]=-5.821717;square[17][23]=11.629917;square[17][24]=-44.839545;square[17][25]=52.766925;square[17][26]=128.844409;square[17][27]=-19.109767;square[17][28]=-41.540032;square[17][29]=13.401209;square[18][0]=0.000000;square[18][1]=0.000000;square[18][2]=-2.700000;square[18][3]=4.500000;square[18][4]=-3.330000;square[18][5]=1.350000;square[18][6]=0.603000;square[18][7]=-3.285000;square[18][8]=2.342700;square[18][9]=-20.956500;square[18][10]=27.308430;square[18][11]=48.639150;square[18][12]=-15.922413;square[18][13]=-47.124765;square[18][14]=5.469828;square[18][15]=-24.412288;square[18][16]=-32.877155;square[18][17]=-29.171060;square[18][18]=7.310561;square[18][19]=14.246046;square[18][20]=-51.020495;square[18][21]=33.521442;square[18][22]=-2.718446;square[18][23]=-70.630702;square[18][24]=112.753399;square[18][25]=138.032368;square[18][26]=42.978059;square[18][27]=-16.170869;square[18][28]=-25.219747;square[18][29]=6.146218;square[19][0]=0.000000;square[19][1]=0.000000;square[19][2]=-5.400000;square[19][3]=1.800000;square[19][4]=-3.060000;square[19][5]=13.320000;square[19][6]=2.646000;square[19][7]=152.388000;square[19][8]=-49.818600;square[19][9]=-215.650800;square[19][10]=1.063260;square[19][11]=59.714280;square[19][12]=-17.943066;square[19][13]=62.742852;square[19][14]=-71.948759;square[19][15]=-14.631433;square[19][16]=26.146117;square[19][17]=-103.168290;square[19][18]=-15.168495;square[19][19]=171.748539;square[19][20]=12.448354;square[19][21]=23.173685;square[19][22]=38.203519;square[19][23]=-95.243683;square[19][24]=52.383167;square[19][25]=-9.219315;square[19][26]=-5.055150;square[19][27]=-6.497383;square[19][28]=-2.749635;square[19][29]=7.652355;square[20][0]=0.000000;square[20][1]=0.000000;square[20][2]=0.000000;square[20][3]=2.700000;square[20][4]=7.200000;square[20][5]=22.230000;square[20][6]=12.780000;square[20][7]=86.607000;square[20][8]=-23.598000;square[20][9]=-19.253700;square[20][10]=66.961800;square[20][11]=17.771670;square[20][12]=26.065620;square[20][13]=-62.305497;square[20][14]=-15.240942;square[20][15]=6.925053;square[20][16]=-28.116848;square[20][17]=-208.867453;square[20][18]=-58.605163;square[20][19]=68.519293;square[20][20]=-84.244647;square[20][21]=-1.332637;square[20][22]=-16.420182;square[20][23]=108.600627;square[20][24]=-0.378164;square[20][25]=-11.159436;square[20][26]=-10.240347;square[20][27]=12.456508;square[20][28]=-28.116313;square[20][29]=-30.189143;square[21][0]=0.000000;square[21][1]=0.000000;square[21][2]=-5.400000;square[21][3]=6.300000;square[21][4]=0.540000;square[21][5]=-0.630000;square[21][6]=15.786000;square[21][7]=-0.567000;square[21][8]=68.207400;square[21][9]=89.489700;square[21][10]=-11.513340;square[21][11]=-38.259270;square[21][12]=-4.062006;square[21][13]=-70.433343;square[21][14]=-179.155805;square[21][15]=2.309991;square[21][16]=62.859775;square[21][17]=91.178992;square[21][18]=21.473798;square[21][19]=-7.038907;square[21][20]=-22.073582;square[21][21]=-59.435016;square[21][22]=-68.466224;square[21][23]=-12.091515;square[21][24]=49.980398;square[21][25]=18.817637;square[21][26]=-3.617641;square[21][27]=16.935873;square[21][28]=23.744123;square[21][29]=-4.557714;square[22][0]=0.000000;square[22][1]=0.000000;square[22][2]=-0.900000;square[22][3]=1.800000;square[22][4]=22.590000;square[22][5]=-23.580000;square[22][6]=-45.369000;square[22][7]=93.078000;square[22][8]=12.267900;square[22][9]=-58.429800;square[22][10]=48.841110;square[22][11]=32.013180;square[22][12]=72.756999;square[22][13]=-17.088138;square[22][14]=-111.818701;square[22][15]=16.120676;square[22][16]=4.663169;square[22][17]=-1.691392;square[22][18]=-73.203148;square[22][19]=-151.822253;square[22][20]=41.217167;square[22][21]=95.559973;square[22][22]=-79.904550;square[22][23]=-11.196025;square[22][24]=80.185905;square[22][25]=-95.576422;square[22][26]=-2.532685;square[22][27]=2.181220;square[22][28]=-4.079417;square[22][29]=18.163098;square[23][0]=0.000000;square[23][1]=0.000000;square[23][2]=-5.400000;square[23][3]=1.800000;square[23][4]=-1.260000;square[23][5]=3.420000;square[23][6]=-1.134000;square[23][7]=-41.022000;square[23][8]=-37.920600;square[23][9]=-146.719800;square[23][10]=-6.228540;square[23][11]=50.652180;square[23][12]=20.494314;square[23][13]=-19.213038;square[23][14]=78.744883;square[23][15]=-57.791734;square[23][16]=-29.029606;square[23][17]=-18.712561;square[23][18]=22.473355;square[23][19]=136.158695;square[23][20]=-40.973981;square[23][21]=-44.857174;square[23][22]=-48.576583;square[23][23]=12.728543;square[23][24]=108.381076;square[23][25]=-51.544311;square[23][26]=-32.057032;square[23][27]=25.610120;square[23][28]=-15.351329;square[23][29]=18.549108;square[24][0]=0.000000;square[24][1]=0.000000;square[24][2]=-2.700000;square[24][3]=3.600000;square[24][4]=-17.730000;square[24][5]=17.640000;square[24][6]=3.843000;square[24][7]=10.476000;square[24][8]=-41.541300;square[24][9]=-139.071600;square[24][10]=31.012830;square[24][11]=44.035560;square[24][12]=-53.988453;square[24][13]=42.332004;square[24][14]=-8.089608;square[24][15]=18.298804;square[24][16]=-36.980647;square[24][17]=-2.431077;square[24][18]=33.317418;square[24][19]=198.512031;square[24][20]=-188.714324;square[24][21]=-21.139172;square[24][22]=105.557108;square[24][23]=-120.725255;square[24][24]=36.501398;square[24][25]=19.147271;square[24][26]=5.851258;square[24][27]=1.932543;square[24][28]=7.966132;square[24][29]=-3.660711;square[25][0]=0.000000;square[25][1]=0.000000;square[25][2]=0.000000;square[25][3]=-0.900000;square[25][4]=-0.900000;square[25][5]=3.690000;square[25][6]=-20.610000;square[25][7]=-90.279000;square[25][8]=-41.949000;square[25][9]=160.848900;square[25][10]=42.345900;square[25][11]=-29.835990;square[25][12]=-42.888690;square[25][13]=-10.652391;square[25][14]=-82.699821;square[25][15]=16.512848;square[25][16]=37.170161;square[25][17]=94.061563;square[25][18]=-88.046855;square[25][19]=-133.144593;square[25][20]=-23.442170;square[25][21]=14.269866;square[25][22]=59.902047;square[25][23]=19.142880;square[25][24]=-5.488157;square[25][25]=8.228592;square[25][26]=10.360658;square[25][27]=2.905733;square[25][28]=10.224593;square[25][29]=-1.884841;square[26][0]=0.000000;square[26][1]=0.000000;square[26][2]=0.900000;square[26][3]=0.000000;square[26][4]=-16.290000;square[26][5]=35.100000;square[26][6]=34.839000;square[26][7]=-52.110000;square[26][8]=0.755100;square[26][9]=-91.899000;square[26][10]=50.179590;square[26][11]=54.090900;square[26][12]=-111.438369;square[26][13]=106.281810;square[26][14]=-18.394532;square[26][15]=-6.946371;square[26][16]=16.744921;square[26][17]=-145.751734;square[26][18]=-26.329571;square[26][19]=86.623439;square[26][20]=-42.596614;square[26][21]=9.561096;square[26][22]=108.363047;square[26][23]=-65.195014;square[26][24]=7.526743;square[26][25]=-10.975513;square[26][26]=-11.225932;square[26][27]=11.722039;square[26][28]=-6.503338;square[26][29]=11.449835;square[27][0]=0.000000;square[27][1]=0.000000;square[27][2]=-2.700000;square[27][3]=0.900000;square[27][4]=-1.530000;square[27][5]=3.510000;square[27][6]=-31.977000;square[27][7]=-17.541000;square[27][8]=50.420700;square[27][9]=35.513100;square[27][10]=-27.521370;square[27][11]=-28.338210;square[27][12]=-85.969233;square[27][13]=-2.104389;square[27][14]=-26.072310;square[27][15]=49.406050;square[27][16]=45.834921;square[27][17]=36.365445;square[27][18]=-83.848571;square[27][19]=66.028900;square[27][20]=18.136286;square[27][21]=-24.273990;square[27][22]=78.422658;square[27][23]=26.753409;square[27][24]=0.380392;square[27][25]=6.078068;square[27][26]=-5.057647;square[27][27]=-11.629738;square[27][28]=-1.851883;square[27][29]=-8.666765;square[28][0]=0.000000;square[28][1]=0.000000;square[28][2]=-2.700000;square[28][3]=3.600000;square[28][4]=-0.630000;square[28][5]=33.840000;square[28][6]=28.233000;square[28][7]=-112.644000;square[28][8]=11.909700;square[28][9]=35.420400;square[28][10]=-9.081270;square[28][11]=-56.321640;square[28][12]=-82.873143;square[28][13]=-35.389476;square[28][14]=19.914171;square[28][15]=-12.050528;square[28][16]=4.422754;square[28][17]=34.154524;square[28][18]=13.880479;square[28][19]=92.839072;square[28][20]=-24.407569;square[28][21]=117.755165;square[28][22]=6.833188;square[28][23]=-34.420352;square[28][24]=-12.750131;square[28][25]=-5.778317;square[28][26]=-3.375118;square[28][27]=-4.300485;square[28][28]=-3.037606;square[28][29]=-5.670436;square[29][0]=0.000000;square[29][1]=0.000000;square[29][2]=-0.900000;square[29][3]=5.400000;square[29][4]=-1.710000;square[29][5]=4.860000;square[29][6]=-15.939000;square[29][7]=16.974000;square[29][8]=9.954900;square[29][9]=7.176600;square[29][10]=-0.940590;square[29][11]=-44.841060;square[29][12]=-71.946531;square[29][13]=65.843046;square[29][14]=26.148122;square[29][15]=-14.541259;square[29][16]=-101.566690;square[29][17]=-26.587133;square[29][18]=-32.910021;square[29][19]=-1.428419;square[29][20]=16.280981;square[29][21]=68.914422;square[29][22]=1.152883;square[29][23]=-124.277020;square[29][24]=-23.262405;square[29][25]=22.250682;square[29][26]=96.963835;square[29][27]=11.025614;square[29][28]=-0.032548;square[29][29]=3.623053;square[30][0]=0.000000;square[30][1]=0.000000;square[30][2]=-4.500000;square[30][3]=2.700000;square[30][4]=-1.350000;square[30][5]=11.430000;square[30][6]=32.985000;square[30][7]=47.187000;square[30][8]=23.386500;square[30][9]=-13.331700;square[30][10]=16.547850;square[30][11]=107.701470;square[30][12]=-34.606935;square[30][13]=-31.768677;square[30][14]=-13.146242;square[30][15]=38.008191;square[30][16]=-73.031617;square[30][17]=-93.592628;square[30][18]=-35.128456;square[30][19]=37.266634;square[30][20]=14.284390;square[30][21]=-59.160029;square[30][22]=-19.544049;square[30][23]=11.555974;square[30][24]=-13.089644;square[30][25]=39.200377;square[30][26]=67.419320;square[30][27]=18.180339;square[30][28]=-22.122612;square[30][29]=12.762305;square[31][0]=0.000000;square[31][1]=0.000000;square[31][2]=-0.900000;square[31][3]=1.800000;square[31][4]=-4.410000;square[31][5]=3.420000;square[31][6]=-12.069000;square[31][7]=72.378000;square[31][8]=35.037900;square[31][9]=-69.859800;square[31][10]=-35.965890;square[31][11]=-64.673820;square[31][12]=26.130699;square[31][13]=112.793562;square[31][14]=-12.482371;square[31][15]=-19.085794;square[31][16]=29.265866;square[31][17]=-36.077215;square[31][18]=84.839280;square[31][19]=5.330507;square[31][20]=18.755352;square[31][21]=42.597456;square[31][22]=-10.120184;square[31][23]=23.937710;square[31][24]=-31.608165;square[31][25]=-0.956061;square[31][26]=39.952651;square[31][27]=25.239545;square[31][28]=-29.742614;square[31][29]=0.215591;square[32][0]=0.000000;square[32][1]=0.000000;square[32][2]=0.000000;square[32][3]=4.500000;square[32][4]=-5.400000;square[32][5]=3.150000;square[32][6]=8.640000;square[32][7]=41.535000;square[32][8]=-6.624000;square[32][9]=-19.318500;square[32][10]=37.238400;square[32][11]=40.213350;square[32][12]=-3.385440;square[32][13]=-17.807985;square[32][14]=4.153104;square[32][15]=-73.627186;square[32][16]=19.937794;square[32][17]=21.935532;square[32][18]=-54.055986;square[32][19]=17.941979;square[32][20]=36.849613;square[32][21]=-2.752219;square[32][22]=38.564652;square[32][23]=-146.476997;square[32][24]=40.108186;square[32][25]=122.870703;square[32][26]=-84.502632;square[32][27]=-47.816368;square[32][28]=13.047631;square[32][29]=-6.134731;square[33][0]=0.000000;square[33][1]=0.000000;square[33][2]=-4.500000;square[33][3]=4.500000;square[33][4]=-11.250000;square[33][5]=4.050000;square[33][6]=-35.325000;square[33][7]=57.645000;square[33][8]=34.807500;square[33][9]=-11.119500;square[33][10]=-12.773250;square[33][11]=-67.607550;square[33][12]=-33.095925;square[33][13]=108.353205;square[33][14]=-7.286332;square[33][15]=-77.982116;square[33][16]=128.442301;square[33][17]=-6.283904;square[33][18]=-30.201929;square[33][19]=51.944486;square[33][20]=-116.281736;square[33][21]=95.350038;square[33][22]=32.146437;square[33][23]=-173.384966;square[33][24]=-38.568206;square[33][25]=23.953531;square[33][26]=143.488614;square[33][27]=68.358178;square[33][28]=-10.360247;square[33][29]=-1.477640;square[34][0]=0.000000;square[34][1]=0.000000;square[34][2]=-0.900000;square[34][3]=5.400000;square[34][4]=-2.610000;square[34][5]=4.860000;square[34][6]=48.051000;square[34][7]=91.674000;square[34][8]=-10.754100;square[34][9]=-83.993400;square[34][10]=1.121310;square[34][11]=67.505940;square[34][12]=-46.690821;square[34][13]=-64.344654;square[34][14]=-63.621739;square[34][15]=-2.110189;square[34][16]=47.140435;square[34][17]=-28.899170;square[34][18]=9.126391;square[34][19]=78.390747;square[34][20]=11.813752;square[34][21]=49.851673;square[34][22]=-114.467623;square[34][23]=-72.133495;square[34][24]=24.779139;square[34][25]=-16.320145;square[34][26]=-1.998775;square[34][27]=-4.788131;square[34][28]=0.001103;square[34][29]=1.090682;square[35][0]=0.000000;square[35][1]=0.000000;square[35][2]=-2.700000;square[35][3]=1.800000;square[35][4]=-3.330000;square[35][5]=2.520000;square[35][6]=-73.197000;square[35][7]=-44.532000;square[35][8]=55.622700;square[35][9]=3.121200;square[35][10]=-4.839570;square[35][11]=-9.790920;square[35][12]=1.044387;square[35][13]=-46.611828;square[35][14]=5.439948;square[35][15]=86.749355;square[35][16]=83.195953;square[35][17]=-0.225581;square[35][18]=-61.023642;square[35][19]=58.296977;square[35][20]=-2.721278;square[35][21]=11.967280;square[35][22]=-49.249150;square[35][23]=156.570552;square[35][24]=-56.024235;square[35][25]=-66.986503;square[35][26]=11.678189;square[35][27]=27.012147;square[35][28]=87.910370;square[35][29]=-20.689068;square[36][0]=0.000000;square[36][1]=0.000000;square[36][2]=-1.800000;square[36][3]=2.700000;square[36][4]=0.180000;square[36][5]=1.530000;square[36][6]=64.062000;square[36][7]=199.377000;square[36][8]=15.355800;square[36][9]=-83.360700;square[36][10]=1.220220;square[36][11]=-62.424630;square[36][12]=-43.901802;square[36][13]=10.417833;square[36][14]=-51.211622;square[36][15]=-17.623950;square[36][16]=18.709540;square[36][17]=-44.661555;square[36][18]=88.838586;square[36][19]=23.704600;square[36][20]=-10.045272;square[36][21]=60.034140;square[36][22]=4.459255;square[36][23]=-5.369274;square[36][24]=15.713329;square[36][25]=-17.432346;square[36][26]=-18.258004;square[36][27]=-3.989112;square[36][28]=-11.932203;square[36][29]=-1.790201;square[37][0]=0.000000;square[37][1]=0.000000;square[37][2]=-2.700000;square[37][3]=2.700000;square[37][4]=0.270000;square[37][5]=2.430000;square[37][6]=-43.857000;square[37][7]=6.687000;square[37][8]=6.428700;square[37][9]=-29.081700;square[37][10]=3.985830;square[37][11]=29.626470;square[37][12]=-28.812753;square[37][13]=15.863823;square[37][14]=-43.931478;square[37][15]=-46.022559;square[37][16]=82.861670;square[37][17]=107.979697;square[37][18]=34.975503;square[37][19]=-89.118273;square[37][20]=4.477953;square[37][21]=-60.406446;square[37][22]=42.730157;square[37][23]=21.234199;square[37][24]=-3.842858;square[37][25]=37.110779;square[37][26]=-15.158572;square[37][27]=4.599701;square[37][28]=-10.042715;square[37][29]=6.839731;square[38][0]=0.000000;square[38][1]=0.000000;square[38][2]=-1.800000;square[38][3]=2.700000;square[38][4]=-6.120000;square[38][5]=-56.970000;square[38][6]=25.092000;square[38][7]=52.227000;square[38][8]=-5.317200;square[38][9]=81.204300;square[38][10]=-38.085480;square[38][11]=-60.116130;square[38][12]=-35.176932;square[38][13]=-33.404517;square[38][14]=-47.859239;square[38][15]=12.235935;square[38][16]=51.426685;square[38][17]=45.212341;square[38][18]=7.584017;square[38][19]=-117.708893;square[38][20]=-43.574385;square[38][21]=49.761996;square[38][22]=74.183053;square[38][23]=61.885797;square[38][24]=67.664748;square[38][25]=-3.702783;square[38][26]=-13.801727;square[38][27]=-1.532505;square[38][28]=-10.621554;square[38][29]=0.420746;square[39][0]=0.000000;square[39][1]=0.000000;square[39][2]=-0.900000;square[39][3]=0.900000;square[39][4]=0.090000;square[39][5]=32.310000;square[39][6]=-22.419000;square[39][7]=-31.221000;square[39][8]=18.522900;square[39][9]=-73.098900;square[39][10]=125.570610;square[39][11]=74.610990;square[39][12]=-98.486451;square[39][13]=-5.750109;square[39][14]=15.762194;square[39][15]=-49.275098;square[39][16]=-106.414025;square[39][17]=134.752412;square[39][18]=-25.572623;square[39][19]=-49.722829;square[39][20]=21.084639;square[39][21]=20.049453;square[39][22]=-0.823824;square[39][23]=-12.555492;square[39][24]=0.158558;square[39][25]=-4.099943;square[39][26]=1.042702;square[39][27]=-1.889948;square[39][28]=2.738432;square[39][29]=4.599046;
  #endif
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
