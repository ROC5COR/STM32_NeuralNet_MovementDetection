/*
 * ADXL345, triple axis, I2C interface, accelerometer.
 */  
 
#include "adxl345.h"


void ADXL345_Init(I2C_HandleTypeDef *handle){

    //400kHz, allowing us to use the fastest data rates.
    //i2c_.frequency(400000);  Already defined in handle
    // initialize the BW data rate
    uint8_t tx[2];
    tx[0] = ADXL345_BW_RATE_REG;
    tx[1] = ADXL345_1600HZ; //value greater than or equal to 0x0A is written into the rate bits (Bit D3 through Bit D0) in the BW_RATE register 
    //i2c_.write( ADXL345_I2C_WRITE , tx, 2);
    HAL_I2C_Master_Transmit(handle,ADXL345_I2C_WRITE,tx,2,1000);

    //Data format (for +-16g) - This is done by setting Bit D3 of the DATA_FORMAT register (Address 0x31) and writing a value of 0x03 to the range bits (Bit D1 and Bit D0) of the DATA_FORMAT register (Address 0x31).
   
    uint8_t rx[2];
    rx[0] = ADXL345_DATA_FORMAT_REG;
    rx[1] = 0x0B; 
     // full res and +_16g
    //i2c_.write( ADXL345_I2C_WRITE , rx, 2);
    HAL_I2C_Master_Transmit(handle,ADXL345_I2C_WRITE,rx,2,1000);
 
    // Set Offset  - programmed into the OFSX, OFSY, and OFXZ registers, respectively, as 0xFD, 0x03 and 0xFE.
    uint8_t x[2];
    x[0] = ADXL345_OFSX_REG ;
    x[1] = 0xFD; 
    HAL_I2C_Master_Transmit(handle,ADXL345_I2C_WRITE,x,2,1000);
    //i2c_.write( ADXL345_I2C_WRITE , x, 2);
    uint8_t y[2];
    y[0] = ADXL345_OFSY_REG ;
    y[1] = 0x03; 
    HAL_I2C_Master_Transmit(handle,ADXL345_I2C_WRITE,y,2,1000);
     //i2c_.write( ADXL345_I2C_WRITE , y, 2);
    uint8_t z[2];
    z[0] = ADXL345_OFSZ_REG ;
    z[1] = 0xFE; 
    HAL_I2C_Master_Transmit(handle,ADXL345_I2C_WRITE,z,2,1000);
    //i2c_.write( ADXL345_I2C_WRITE , z, 2);
}


uint8_t SingleByteRead(I2C_HandleTypeDef *handle, uint8_t address){
   uint8_t tx = address;
   uint8_t output;
    //i2c_.write( ADXL345_I2C_WRITE , &tx, 1);  //tell it what you want to read
   HAL_I2C_Master_Transmit(handle,ADXL345_I2C_WRITE,&tx,1,1000);
   HAL_I2C_Master_Receive(handle,ADXL345_I2C_READ,&output,1,1000);

    //i2c_.read( ADXL345_I2C_READ , &output, 1);    //tell it where to store the data
    return output;
  
}


/*
***info on the i2c_.write***
address     8-bit I2C slave address [ addr | 0 ]
data        Pointer to the byte-array data to send
length        Number of bytes to send
repeated    Repeated start, true - do not send stop at end
returns     0 on success (ack), or non-0 on failure (nack)
*/

int SingleByteWrite(I2C_HandleTypeDef *handle, uint8_t address, uint8_t data){
   int ack = 0;
   uint8_t tx[2];
   tx[0] = address;
   tx[1] = data;
   int ok = 0;
   if(HAL_I2C_Master_Transmit(handle,ADXL345_I2C_WRITE,tx,2,1000) != HAL_OK){
	   ok = 1;
   }
   return ack | ok;
   //return   ack | i2c_.write( ADXL345_I2C_WRITE , tx, 2);
}



void multiByteRead(I2C_HandleTypeDef *handle, uint8_t address, uint8_t* output, int size) {
	HAL_I2C_Master_Transmit(handle,ADXL345_I2C_WRITE,&address,1,1000);
	HAL_I2C_Master_Receive(handle,ADXL345_I2C_READ,output,size,1000);
    //i2c_.write( ADXL345_I2C_WRITE, &address, 1);  //tell it where to read from
    //i2c_.read( ADXL345_I2C_READ , output, size);      //tell it where to store the data read
}


int multiByteWrite(I2C_HandleTypeDef *handle, uint8_t address, uint8_t* ptr_data, int size) {
        int ack = 0;
        if(HAL_I2C_Master_Transmit(handle, ADXL345_I2C_WRITE, &address, 1, 1000) != 0){
        	ack = 1;
        }
        	//ack = i2c_.write( ADXL345_I2C_WRITE, &address, 1);  //tell it where to write to
        //return ack | i2c_.write( ADXL345_I2C_READ, ptr_data, size);  //tell it what data to write
        int ok = 0;
        if(HAL_I2C_Master_Transmit(handle, ADXL345_I2C_READ, ptr_data, size, 1000) != HAL_OK){
        	ok = 1;
        }

        return ack | ok;
}


void getOutput(I2C_HandleTypeDef *handle, int16_t* readings){
	uint8_t buffer[6];
    multiByteRead(handle, ADXL345_DATAX0_REG, buffer, 6);
    
    //readings[0] = (int16_t)buffer[1] << 8 | (int16_t)buffer[0];
    //readings[1] = (int16_t)buffer[3] << 8 | (int16_t)buffer[2];
    //readings[2] = (int16_t)buffer[5] << 8 | (int16_t)buffer[4];
    //printf("raw data : %d\t%d\t%d\t%d\t%d\t%d\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5]);
    readings[0] = ((int16_t)buffer[1] << 8 | (int16_t)buffer[0]);
    readings[1] = ((int16_t)buffer[3] << 8 | (int16_t)buffer[2]);
    readings[2] = ((int16_t)buffer[5] << 8 | (int16_t)buffer[4]);
}



char getDeviceID(I2C_HandleTypeDef *handle) {
    return SingleByteRead(handle, ADXL345_DEVID_REG);
    }
//
int setPowerMode(I2C_HandleTypeDef *handle, char mode) {

    //Get the current register contents, so we don't clobber the rate value.
    char registerContents = (mode << 4) | SingleByteRead(handle,ADXL345_BW_RATE_REG);

   return SingleByteWrite(handle, ADXL345_BW_RATE_REG, registerContents);

}

char getPowerControl(I2C_HandleTypeDef *handle) {
    return SingleByteRead(handle, ADXL345_POWER_CTL_REG);
}

int setPowerControl(I2C_HandleTypeDef *handle, char settings) {
    return SingleByteWrite(handle, ADXL345_POWER_CTL_REG, settings);

}



char getDataFormatControl(I2C_HandleTypeDef *handle){

    return SingleByteRead(handle, ADXL345_DATA_FORMAT_REG);
}

int setDataFormatControl(I2C_HandleTypeDef *handle, char settings){

   return SingleByteWrite(handle, ADXL345_DATA_FORMAT_REG, settings);
    
}

int setDataRate(I2C_HandleTypeDef *handle, char rate) {

    //Get the current register contents, so we don't clobber the power bit.
    char registerContents = SingleByteRead(handle, ADXL345_BW_RATE_REG);

    registerContents &= 0x10;
    registerContents |= rate;

    return SingleByteWrite(handle, ADXL345_BW_RATE_REG, registerContents);

}


char getOffset(I2C_HandleTypeDef *handle, char axis) {

    char address = 0;

    if (axis == ADXL345_X) {
        address = ADXL345_OFSX_REG;
    } else if (axis == ADXL345_Y) {
        address = ADXL345_OFSY_REG;
    } else if (axis == ADXL345_Z) {
        address = ADXL345_OFSZ_REG;
    }

   return SingleByteRead(handle, address);
}

int setOffset(I2C_HandleTypeDef *handle, char axis, char offset) {

    char address = 0;

    if (axis == ADXL345_X) {
        address = ADXL345_OFSX_REG;
    } else if (axis == ADXL345_Y) {
        address = ADXL345_OFSY_REG;
    } else if (axis == ADXL345_Z) {
        address = ADXL345_OFSZ_REG;
    }

   return SingleByteWrite(handle, address, offset);

}


char getFifoControl(I2C_HandleTypeDef *handle){

    return SingleByteRead(handle, ADXL345_FIFO_CTL);

}

int setFifoControl(I2C_HandleTypeDef *handle, char settings){
   return SingleByteWrite(handle, ADXL345_FIFO_STATUS, settings);

}

char getFifoStatus(I2C_HandleTypeDef *handle){

    return SingleByteRead(handle, ADXL345_FIFO_STATUS);

}



char getTapThreshold(I2C_HandleTypeDef *handle) {

    return SingleByteRead(handle, ADXL345_THRESH_TAP_REG);
}

int setTapThreshold(I2C_HandleTypeDef *handle, char threshold) {

   return SingleByteWrite(handle, ADXL345_THRESH_TAP_REG, threshold);

}


float getTapDuration(I2C_HandleTypeDef *handle) {

    return (float)SingleByteRead(handle, ADXL345_DUR_REG)*625;
}

int setTapDuration(I2C_HandleTypeDef *handle, short int duration_us) {

    short int tapDuration = duration_us / 625;
    uint8_t tapChar[2];
     tapChar[0] = (tapDuration & 0x00FF);
     tapChar[1] = (tapDuration >> 8) & 0x00FF;
    return multiByteWrite(handle, ADXL345_DUR_REG, tapChar, 2);

}

float getTapLatency(I2C_HandleTypeDef *handle) {

    return (float)SingleByteRead(handle, ADXL345_LATENT_REG)*1.25;
}

int setTapLatency(I2C_HandleTypeDef *handle, short int latency_ms) {

    latency_ms = latency_ms / 1.25;
    uint8_t latChar[2];
     latChar[0] = (latency_ms & 0x00FF);
     latChar[1] = (latency_ms << 8) & 0xFF00;
    return multiByteWrite(handle, ADXL345_LATENT_REG, latChar, 2);

}

float getWindowTime(I2C_HandleTypeDef *handle) {

    return (float)SingleByteRead(handle, ADXL345_WINDOW_REG)*1.25;
}

int setWindowTime(I2C_HandleTypeDef *handle, short int window_ms) {

    window_ms = window_ms / 1.25;
    uint8_t windowChar[2];
    windowChar[0] = (window_ms & 0x00FF);
    windowChar[1] = ((window_ms << 8) & 0xFF00);
   return multiByteWrite(handle, ADXL345_WINDOW_REG, windowChar, 2);

}

char getActivityThreshold(I2C_HandleTypeDef *handle) {

    return SingleByteRead(handle, ADXL345_THRESH_ACT_REG);
}

int setActivityThreshold(I2C_HandleTypeDef *handle, char threshold) {
    return SingleByteWrite(handle, ADXL345_THRESH_ACT_REG, threshold);

}

char getInactivityThreshold(I2C_HandleTypeDef *handle) {
    return SingleByteRead(handle, ADXL345_THRESH_INACT_REG);
       
}

//int FUNCTION(short int * ptr_Output)
//short int FUNCTION ()

int setInactivityThreshold(I2C_HandleTypeDef *handle, char threshold) {
    return SingleByteWrite(handle, ADXL345_THRESH_INACT_REG, threshold);

}

char getTimeInactivity(I2C_HandleTypeDef *handle) {

    return SingleByteRead(handle, ADXL345_TIME_INACT_REG);

}

int setTimeInactivity(I2C_HandleTypeDef *handle, char timeInactivity) {
    return SingleByteWrite(handle, ADXL345_TIME_INACT_REG, timeInactivity);

}

char getActivityInactivityControl(I2C_HandleTypeDef *handle) {

    return SingleByteRead(handle, ADXL345_ACT_INACT_CTL_REG);

}

int setActivityInactivityControl(I2C_HandleTypeDef *handle, char settings) {
    return SingleByteWrite(handle, ADXL345_ACT_INACT_CTL_REG, settings);
    
}

char getFreefallThreshold(I2C_HandleTypeDef *handle) {

    return SingleByteRead(handle, ADXL345_THRESH_FF_REG);

}

int setFreefallThreshold(I2C_HandleTypeDef *handle, char threshold) {
   return SingleByteWrite(handle, ADXL345_THRESH_FF_REG, threshold);

}

char getFreefallTime(I2C_HandleTypeDef *handle) {

    return SingleByteRead(handle, ADXL345_TIME_FF_REG)*5;

}

int setFreefallTime(I2C_HandleTypeDef *handle, short int freefallTime_ms) {
     freefallTime_ms = freefallTime_ms / 5;
     uint8_t fallChar[2];
     fallChar[0] = (freefallTime_ms & 0x00FF);
     fallChar[1] = (freefallTime_ms << 8) & 0xFF00;
    
    return multiByteWrite(handle, ADXL345_TIME_FF_REG, fallChar, 2);

}

char getTapAxisControl(I2C_HandleTypeDef *handle) {

    return SingleByteRead(handle, ADXL345_TAP_AXES_REG);

}

int setTapAxisControl(I2C_HandleTypeDef *handle, char settings) {
   return SingleByteWrite(handle, ADXL345_TAP_AXES_REG, settings);

}

char getTapSource(I2C_HandleTypeDef *handle) {

    return SingleByteRead(handle, ADXL345_ACT_TAP_STATUS_REG);

}



char getInterruptEnableControl(I2C_HandleTypeDef *handle) {

    return SingleByteRead(handle, ADXL345_INT_ENABLE_REG);

}

int setInterruptEnableControl(I2C_HandleTypeDef *handle, char settings) {
   return SingleByteWrite(handle, ADXL345_INT_ENABLE_REG, settings);

}

char getInterruptMappingControl(I2C_HandleTypeDef *handle) {

    return SingleByteRead(handle, ADXL345_INT_MAP_REG);

}

int setInterruptMappingControl(I2C_HandleTypeDef *handle, char settings) {
    return SingleByteWrite(handle, ADXL345_INT_MAP_REG, settings);

}

char getInterruptSource(I2C_HandleTypeDef *handle){

    return SingleByteRead(handle, ADXL345_INT_SOURCE_REG);

}





