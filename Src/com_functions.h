#include "stm32l4xx_hal_gpio.h"
#define MODE_PORT GPIOC

#define MODE_OUT_0 GPIO_PIN_8
#define MODE_OUT_1 GPIO_PIN_9
#define MODE_OUT_2 GPIO_PIN_2
#define MODE_OUT_3 GPIO_PIN_3

#define DATA_OUT_2 GPIO_PIN_10

#define MODE_IN_0 GPIO_PIN_7
#define MODE_IN_1 GPIO_PIN_6
#define MODE_IN_2 GPIO_PIN_5
#define MODE_IN_3 GPIO_PIN_4

#define DATA_OUT_PORT GPIOA

#define DATA_OUT_0 GPIO_PIN_0
#define DATA_OUT_1 GPIO_PIN_1
//#define DATA_OUT_2 GPIO_PIN_2 //Warning !
#define DATA_OUT_3 GPIO_PIN_3
#define DATA_OUT_4 GPIO_PIN_4
#define DATA_OUT_5 GPIO_PIN_5
#define DATA_OUT_6 GPIO_PIN_6
#define DATA_OUT_7 GPIO_PIN_7
#define DATA_OUT_8 GPIO_PIN_8
#define DATA_OUT_9 GPIO_PIN_9
#define DATA_OUT_10 GPIO_PIN_10
#define DATA_OUT_11 GPIO_PIN_11
#define DATA_OUT_12 GPIO_PIN_12
#define DATA_OUT_13 GPIO_PIN_13
#define DATA_OUT_14 GPIO_PIN_14
#define DATA_OUT_15 GPIO_PIN_15

#define DATA_IN_PORT GPIOB

#define DATA_IN_0 GPIO_PIN_0
#define DATA_IN_1 GPIO_PIN_1
#define DATA_IN_2 GPIO_PIN_2
#define DATA_IN_3 GPIO_PIN_3
#define DATA_IN_4 GPIO_PIN_4
#define DATA_IN_5 GPIO_PIN_5
#define DATA_IN_6 GPIO_PIN_6
#define DATA_IN_7 GPIO_PIN_7
#define DATA_IN_8 GPIO_PIN_8
#define DATA_IN_9 GPIO_PIN_9
#define DATA_IN_10 GPIO_PIN_10
#define DATA_IN_11 GPIO_PIN_11
#define DATA_IN_12 GPIO_PIN_12
#define DATA_IN_13 GPIO_PIN_13
#define DATA_IN_14 GPIO_PIN_14
#define DATA_IN_15 GPIO_PIN_15

void init_com();
void send_input_element(float layer_element);
float read_fpga_input_element();
int verif_input_element(float layer_element, float temp_fpga_element);
void read_bits_input(float layer_element, int Tab_bits[]);
float load_bits_to_element(int Tab_bits[]);

/*void send_STM32_Input_request();
void send_ack_STM32();
void send_STM32_L2_request();*/
void send_STM32_start_request();
void send_STM32_next_input_request();
void send_STM32_input_ack();
void send_STM32_output_ack();


void send_verif_OK();
void send_verif_false();


/*
void wait_for_ack_FPGA();
void wait_for_req_FPGA();

*/

void wait_for_FPGA_on_S0();
void wait_for_input_ack_FPGA();
void wait_for_verif_input_req_FPGA();
void wait_for_end_input_ack_FPGA();
void wait_for_output_req_FPGA();
void wait_for_FPGA_verif();
void wait_for_FPGA_end_cycle();


int FPGA_verification_result();
void reset_all_Data_outputs();


void init_INPUT(float input[]);
