/**
 * @file com_functions.c
 * @author Gauthier
 * @version 1.0
 * @brief Fonctions utiles � la communication FPGA/STM32
 */
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_gpio.h"
#include <math.h>
#include "com_functions.h"

//float temp_fpga_element;

/**
 * @brief Initialisation de la communication FPGA/STM32
 * @details
 */
void init_com()
{
	/*Initialisation des entr�es pour MODE*/
	GPIO_InitTypeDef GPIO_InitStructureModeIn;

	//__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitStructureModeIn.Pin = MODE_IN_0 | MODE_IN_1 | MODE_IN_2 | MODE_IN_3;
	GPIO_InitStructureModeIn.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructureModeIn.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(MODE_PORT, &GPIO_InitStructureModeIn);

	/*Initialisation des sorties pour MODE*/
	GPIO_InitTypeDef GPIO_InitStructureModeOut;
	//__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitStructureModeOut.Pin = MODE_OUT_0 | MODE_OUT_1 | MODE_OUT_2 | MODE_OUT_3 | DATA_OUT_2;
	GPIO_InitStructureModeOut.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructureModeOut.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MODE_PORT, &GPIO_InitStructureModeOut);

	/*Initialisation des entr�es pour DATA*/
	GPIO_InitTypeDef GPIO_InitStructureDataIn;
	//__HAL_RCC_GPIOA_CLK_ENABLE();
	GPIO_InitStructureDataIn.Pin = DATA_IN_0 | DATA_IN_1 | DATA_IN_2 | DATA_IN_3 | DATA_IN_4 | DATA_IN_5 | DATA_IN_6 | DATA_IN_7 | DATA_IN_8 | DATA_IN_9 | DATA_IN_10 | DATA_IN_11| DATA_IN_12 | DATA_IN_13 | DATA_IN_14 | DATA_IN_15;
	GPIO_InitStructureDataIn.Mode = GPIO_MODE_INPUT;
	GPIO_InitStructureDataIn.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(DATA_IN_PORT, &GPIO_InitStructureDataIn);

	/*Initialisation des sorties pour DATA*/
	GPIO_InitTypeDef GPIO_InitStructureDataOut;
	//__HAL_RCC_GPIOB_CLK_ENABLE();

	//GPIO_InitStructureDataOut.Pin = DATA_OUT_0 | DATA_OUT_1 | DATA_OUT_2 | DATA_OUT_3 | DATA_OUT_4 | DATA_OUT_5 | DATA_OUT_6 | DATA_OUT_7 | DATA_OUT_8 | DATA_OUT_9 | DATA_OUT_10 | DATA_OUT_11 | DATA_OUT_12 | DATA_OUT_13 | DATA_OUT_14 | DATA_OUT_15;
	GPIO_InitStructureDataOut.Pin = DATA_OUT_0 | DATA_OUT_1 | DATA_OUT_3 | DATA_OUT_4 | DATA_OUT_5 | DATA_OUT_6  | DATA_OUT_7 | DATA_OUT_8 | DATA_OUT_9 | DATA_OUT_10 | DATA_OUT_11 | DATA_OUT_12 | DATA_OUT_13 | DATA_OUT_14 | DATA_OUT_15;
	GPIO_InitStructureDataOut.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructureDataOut.Pull = GPIO_NOPULL;
	printf("1er\n");
	HAL_GPIO_Init(DATA_OUT_PORT, &GPIO_InitStructureDataOut);
	printf("2eme\n");

}

/**
 * @brief Envoi d'un element de INPUT au FPGA
 * @details
 */
void send_input_element(float layer_element)
{
	int Tab_bits[16];

	read_bits_layer(layer_element, Tab_bits);
	for(int i = 0; i < 16;i++){
		printf("%d,",Tab_bits[i]);
	}
	printf("\n");

	if(Tab_bits[0]==1){
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_0, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_0, GPIO_PIN_RESET);
	}
	if(Tab_bits[1]==1){
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_1, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_1, GPIO_PIN_RESET);
	}
	if(Tab_bits[2]==1){
		HAL_GPIO_WritePin(GPIOC, DATA_OUT_2, GPIO_PIN_SET);//Bug fix
	}else{
		HAL_GPIO_WritePin(GPIOC, DATA_OUT_2, GPIO_PIN_RESET);
	}
	if(Tab_bits[3]==1){
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_3, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_3, GPIO_PIN_RESET);
	}
	if(Tab_bits[4]==1){
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_4, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_4, GPIO_PIN_RESET);
	}
	if(Tab_bits[5]==1){
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_5, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_5, GPIO_PIN_RESET);
	}
	if(Tab_bits[6]==1){
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_6, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_6, GPIO_PIN_RESET);
	}
	if(Tab_bits[7]==1){
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_7, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_7, GPIO_PIN_RESET);
	}
	if(Tab_bits[8]==1){
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_8, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_8, GPIO_PIN_RESET);
	}
	if(Tab_bits[9]==1){
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_9, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_9, GPIO_PIN_RESET);
	}
	if(Tab_bits[10]==1){
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_10, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_10, GPIO_PIN_RESET);
	}
	if(Tab_bits[11]==1){
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_11, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_11, GPIO_PIN_RESET);
	}
	if(Tab_bits[12]==1){
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_12, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_12, GPIO_PIN_RESET);
	}
	if(Tab_bits[13]==1){
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_13, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_13, GPIO_PIN_RESET);
	}
	if(Tab_bits[14]==1){
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_14, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_14, GPIO_PIN_RESET);
	}
	if(Tab_bits[15]==1){
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_15, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_15, GPIO_PIN_RESET);
	}
}



float read_fpga_input_element()
{
	int Tab_bits[16];
	Tab_bits[0]=HAL_GPIO_ReadPin(DATA_IN_PORT, DATA_IN_0);
	Tab_bits[1]=HAL_GPIO_ReadPin(DATA_IN_PORT, DATA_IN_1);
	Tab_bits[2]=HAL_GPIO_ReadPin(DATA_IN_PORT, DATA_IN_2);
	Tab_bits[3]=HAL_GPIO_ReadPin(DATA_IN_PORT, DATA_IN_3);
	Tab_bits[4]=HAL_GPIO_ReadPin(DATA_IN_PORT, DATA_IN_4);
	Tab_bits[5]=HAL_GPIO_ReadPin(DATA_IN_PORT, DATA_IN_5);
	Tab_bits[6]=HAL_GPIO_ReadPin(DATA_IN_PORT, DATA_IN_6);
	Tab_bits[7]=HAL_GPIO_ReadPin(DATA_IN_PORT, DATA_IN_7);
	Tab_bits[8]=HAL_GPIO_ReadPin(DATA_IN_PORT, DATA_IN_8);
	Tab_bits[9]=HAL_GPIO_ReadPin(DATA_IN_PORT, DATA_IN_9);
	Tab_bits[10]=HAL_GPIO_ReadPin(DATA_IN_PORT, DATA_IN_10);
	Tab_bits[11]=HAL_GPIO_ReadPin(DATA_IN_PORT, DATA_IN_11);
	Tab_bits[12]=HAL_GPIO_ReadPin(DATA_IN_PORT, DATA_IN_12);
	Tab_bits[13]=HAL_GPIO_ReadPin(DATA_IN_PORT, DATA_IN_13);
	Tab_bits[14]=HAL_GPIO_ReadPin(DATA_IN_PORT, DATA_IN_14);
	Tab_bits[15]=HAL_GPIO_ReadPin(DATA_IN_PORT, DATA_IN_15);

	for(int i = 0; i < 16; i++){
		printf("%d,",Tab_bits[i]);
	}
	printf("\n");
	return load_bits_to_element(Tab_bits);


}


/**
 * @brief V�rification de l'�l�ment de LAYER que l'on vient d'envoyer
 * @details Comparaison entre l'�l�ment LAYER renvoy� par le FPGA
 * et l'�l�ment pr�c�demment envoy�
 * @param LAYER � v�rifier
 */
int verif_input_element(float input_element, float temp_fpga_element)
{
	int verif=0;
	if (input_element!=temp_fpga_element)
	{
		verif=0;
	}else {
		verif=1;
	}
	return verif;
}


void read_bits_layer(float input, int Tab_bits[])
{
	if(input > 15){input = 15;}
	if(input < -15){input = -15;}

	int sign = 0x0;
	if(input < 0){
		sign = 0x1;
		input = - input;
	}
	uint16_t int_part = floor(input);
	uint16_t frac_part = (input - (float)int_part)*2048;
	//printf("Conversion ent(hexa):%02x ; frac(hexa):%04x\n",int_part,frac_part);
	//printf("Conversion ent(int):%u ; frac(int):%u\n",int_part,frac_part);

	uint16_t total = 0x0000;
	total |= sign<<15;
	total |= int_part<<11;
	total |= frac_part;
	//printf("Variable sent on pins (hexa): %05x ; Total(int):%d\n",total,total);

	int i=0;
	for (i=0;i<16;i++){
		//Tab_bits[i]=(layer_element & (1<<i)) >> i;
		Tab_bits[i] = (total & (1<<i))>>i;
	}
}


float load_bits_to_element(int Tab_bits[16])
{

	uint16_t integer_part = Tab_bits[14]<<3 | Tab_bits[13]<<2 | Tab_bits[12]<<1 | Tab_bits[11]<<0;
	uint16_t fracionnal_part = Tab_bits[10]<<10 | Tab_bits[9]<<9 | Tab_bits[8]<<8 | Tab_bits[7]<<7 | Tab_bits[6]<<6 | Tab_bits[5]<<5 | Tab_bits[4]<<4 | Tab_bits[3]<<3 | Tab_bits[2]<<2 | Tab_bits[1]<<1 | Tab_bits[0]<<0;
	//printf("integer_part : %d ; fractionnal_part : %d\n",integer_part,fracionnal_part);
	float float_total = (float)integer_part+((float)fracionnal_part/2048);
	if(Tab_bits[15] == 1){
		float_total = - float_total;
	}
	printf("Variable read from pins : %f\n",float_total);
	return float_total;
}

void send_STM32_Input_request()
{
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_3, GPIO_PIN_RESET);
}

void send_ack_STM32()
{
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_3, GPIO_PIN_SET);
}

/*void send_STM32_L2_request()
{
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_3, GPIO_PIN_RESET);
}*/

void send_verif_OK()
{
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_0, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_3, GPIO_PIN_SET);
}

void send_verif_false()
{
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MODE_PORT, MODE_OUT_3, GPIO_PIN_RESET);
}

void wait_for_ack_FPGA()
{
	int Mode[4]={0};
	while(Mode[0]!=1 && Mode[1]!=0 && Mode[2]!=1 && Mode[3]!=0)
	{
		HAL_Delay(10);
		Mode[0]=HAL_GPIO_ReadPin(MODE_PORT, MODE_IN_0);
		Mode[1]=HAL_GPIO_ReadPin(MODE_PORT, MODE_IN_1);
		Mode[2]=HAL_GPIO_ReadPin(MODE_PORT, MODE_IN_2);
		Mode[3]=HAL_GPIO_ReadPin(MODE_PORT, MODE_IN_3);
	}
}

void wait_for_req_FPGA()
{
	int Mode[4]={0};
	while(Mode[0]!=0 && Mode[1]!=0 && Mode[2]!=1 && Mode[3]!=0)
	{
		HAL_Delay(10);
		Mode[0]=HAL_GPIO_ReadPin(MODE_PORT, MODE_IN_0);
		Mode[1]=HAL_GPIO_ReadPin(MODE_PORT, MODE_IN_1);
		Mode[2]=HAL_GPIO_ReadPin(MODE_PORT, MODE_IN_2);
		Mode[3]=HAL_GPIO_ReadPin(MODE_PORT, MODE_IN_3);
	}
}

int FPGA_verification_result(){
	int Mode[4]={0};
	int result;
	Mode[0]=HAL_GPIO_ReadPin(MODE_PORT, MODE_IN_0);
	Mode[1]=HAL_GPIO_ReadPin(MODE_PORT, MODE_IN_1);
	Mode[2]=HAL_GPIO_ReadPin(MODE_PORT, MODE_IN_2);
	Mode[3]=HAL_GPIO_ReadPin(MODE_PORT, MODE_IN_3);
	if (Mode[0]==Mode[1]==Mode[2]==Mode[3]==1){
		result = 1;
	}else {
		result = 0;
	}
	return result;
}


void reset_all_Data_outputs()
{
	HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, DATA_OUT_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_4, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_6, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_8, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_12, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_13, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_14, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DATA_OUT_PORT, DATA_OUT_15, GPIO_PIN_RESET);

}

void init_INPUT(float input[]){
	int i=0;
	float cpt = -15.0;
	for (i=0;i<30;i++)
	{
			input[i] = cpt;
			cpt = cpt + 0.1;
	}
}
