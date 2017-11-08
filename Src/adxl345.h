#ifndef ADXL345_I2C_H
#define ADXL345_I2C_H

#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_i2c.h"

//Registers.
#define ADXL345_DEVID_REG          0x00
#define ADXL345_THRESH_TAP_REG     0x1D
#define ADXL345_OFSX_REG           0x1E
#define ADXL345_OFSY_REG           0x1F
#define ADXL345_OFSZ_REG           0x20
#define ADXL345_DUR_REG            0x21
#define ADXL345_LATENT_REG         0x22
#define ADXL345_WINDOW_REG         0x23
#define ADXL345_THRESH_ACT_REG     0x24
#define ADXL345_THRESH_INACT_REG   0x25
#define ADXL345_TIME_INACT_REG     0x26
#define ADXL345_ACT_INACT_CTL_REG  0x27
#define ADXL345_THRESH_FF_REG      0x28
#define ADXL345_TIME_FF_REG        0x29
#define ADXL345_TAP_AXES_REG       0x2A
#define ADXL345_ACT_TAP_STATUS_REG 0x2B
#define ADXL345_BW_RATE_REG        0x2C
#define ADXL345_POWER_CTL_REG      0x2D
#define ADXL345_INT_ENABLE_REG     0x2E
#define ADXL345_INT_MAP_REG        0x2F
#define ADXL345_INT_SOURCE_REG     0x30
#define ADXL345_DATA_FORMAT_REG    0x31
#define ADXL345_DATAX0_REG         0x32
#define ADXL345_DATAX1_REG         0x33
#define ADXL345_DATAY0_REG         0x34
#define ADXL345_DATAY1_REG         0x35
#define ADXL345_DATAZ0_REG         0x36
#define ADXL345_DATAZ1_REG         0x37
#define ADXL345_FIFO_CTL           0x38
#define ADXL345_FIFO_STATUS        0x39

//Data rate codes.
#define ADXL345_3200HZ      0x0F
#define ADXL345_1600HZ      0x0E
#define ADXL345_800HZ       0x0D
#define ADXL345_400HZ       0x0C
#define ADXL345_200HZ       0x0B
#define ADXL345_100HZ       0x0A
#define ADXL345_50HZ        0x09
#define ADXL345_25HZ        0x08
#define ADXL345_12HZ5       0x07
#define ADXL345_6HZ25       0x06

// read or write bytes
#define ADXL345_I2C_READ    0xA7  
#define ADXL345_I2C_WRITE   0xA6 
#define ADXL345_I2C_ADDRESS 0x53   //the ADXL345 7-bit address is 0x53 when ALT ADDRESS is low as it is on the sparkfun chip: when ALT ADDRESS is high the address is 0x1D

/////////////when ALT ADDRESS pin is high:
//#define ADXL345_I2C_READ    0x3B   
//#define ADXL345_I2C_WRITE   0x3A
//#define ADXL345_I2C_ADDRESS 0x1D 

#define ADXL345_X           0x00
#define ADXL345_Y           0x01
#define ADXL345_Z           0x02



// modes
#define MeasurementMode     0x08


/**
 *
 */
void ADXL345_Init(I2C_HandleTypeDef *handle);
/**
 * Get the output of all three axes.
 *
 * @param Pointer to a buffer to hold the accelerometer value for the
 *        x-axis, y-axis and z-axis [in that order].
 */
void getOutput(I2C_HandleTypeDef *handle, int16_t* readings);

/**
 * Read the device ID register on the device.
 *
 * @return The device ID code [0xE5]
 */
char getDeviceID(I2C_HandleTypeDef *handle);



 /**
 * Set the power mode.
 *
 * @param mode 0 -> Normal operation.
 *             1 -> Reduced power operation.
 */
int setPowerMode(I2C_HandleTypeDef *handle, char mode);

 /**
 * Set the power control settings.
 *
 * See datasheet for details.
 *
 * @param The control byte to write to the POWER_CTL register.
 */
int setPowerControl(I2C_HandleTypeDef *handle, char settings);
  /**
 * Get the power control settings.
 *
 * See datasheet for details.
 *
 * @return The contents of the POWER_CTL register.
 */
char getPowerControl(I2C_HandleTypeDef *handle);


/**
 * Get the data format settings.
 *
 * @return The contents of the DATA_FORMAT register.
 */

char getDataFormatControl(I2C_HandleTypeDef *handle);

/**
 * Set the data format settings.
 *
 * @param settings The control byte to write to the DATA_FORMAT register.
 */
int setDataFormatControl(I2C_HandleTypeDef *handle, char settings);

   /**
 * Set the data rate.
 *
 * @param rate The rate code (see #defines or datasheet).
 */
int setDataRate(I2C_HandleTypeDef *handle, char rate);


   /**
 * Get the current offset for a particular axis.
 *
 * @param axis 0x00 -> X-axis
 *             0x01 -> Y-axis
 *             0x02 -> Z-axis
 * @return The current offset as an 8-bit 2's complement number with scale
 *         factor 15.6mg/LSB.
 */

   char getOffset(I2C_HandleTypeDef *handle, char axis);

/**
 * Set the offset for a particular axis.
 *
 * @param axis 0x00 -> X-axis
 *             0x01 -> Y-axis
 *             0x02 -> Z-axis
 * @param offset The offset as an 8-bit 2's complement number with scale
 *               factor 15.6mg/LSB.
 */
int setOffset(I2C_HandleTypeDef *handle, char axis, char offset);



/**
 * Get the FIFO control settings.
 *
 * @return The contents of the FIFO_CTL register.
 */
char getFifoControl(I2C_HandleTypeDef *handle);

/**
 * Set the FIFO control settings.
 *
 * @param The control byte to write to the FIFO_CTL register.
 */
int setFifoControl(I2C_HandleTypeDef *handle, char settings);

/**
 * Get FIFO status.
 *
 * @return The contents of the FIFO_STATUS register.
 */
char getFifoStatus(I2C_HandleTypeDef *handle);

/**
 * Read the tap threshold on the device.
 *
 * @return The tap threshold as an 8-bit number with a scale factor of
 *         62.5mg/LSB.
 */
char getTapThreshold(I2C_HandleTypeDef *handle);

/**
 * Set the tap threshold.
 *
 * @param The tap threshold as an 8-bit number with a scale factor of
 *        62.5mg/LSB.
 */
int setTapThreshold(I2C_HandleTypeDef *handle, char threshold);

/**
 * Get the tap duration required to trigger an event.
 *
 * @return The max time that an event must be above the tap threshold to
 *         qualify as a tap event, in microseconds.
 */
float getTapDuration(I2C_HandleTypeDef *handle);

/**
 * Set the tap duration required to trigger an event.
 *
 * @param duration_us The max time that an event must be above the tap
 *                    threshold to qualify as a tap event, in microseconds.
 *                    Time will be normalized by the scale factor which is
 *                    625us/LSB. A value of 0 disables the single/double
 *                    tap functions.
 */
int setTapDuration(I2C_HandleTypeDef *handle, short int duration_us);

/**
 * Get the tap latency between the detection of a tap and the time window.
 *
 * @return The wait time from the detection of a tap event to the start of
 *         the time window during which a possible second tap event can be
 *         detected in milliseconds.
 */
float getTapLatency(I2C_HandleTypeDef *handle);

/**
 * Set the tap latency between the detection of a tap and the time window.
 *
 * @param latency_ms The wait time from the detection of a tap event to the
 *                   start of the time window during which a possible
 *                   second tap event can be detected in milliseconds.
 *                   A value of 0 disables the double tap function.
 */
int setTapLatency(I2C_HandleTypeDef *handle, short int latency_ms);

/**
 * Get the time of window between tap latency and a double tap.
 *
 * @return The amount of time after the expiration of the latency time
 *         during which a second valid tap can begin, in milliseconds.
 */
float getWindowTime(I2C_HandleTypeDef *handle);

/**
 * Set the time of the window between tap latency and a double tap.
 *
 * @param window_ms The amount of time after the expiration of the latency
 *                  time during which a second valid tap can begin,
 *                  in milliseconds.
 */
int setWindowTime(I2C_HandleTypeDef *handle, short int window_ms);

/**
 * Get the threshold value for detecting activity.
 *
 * @return The threshold value for detecting activity as an 8-bit number.
 *         Scale factor is 62.5mg/LSB.
 */
 char getActivityThreshold(I2C_HandleTypeDef *handle);

/**
 * Set the threshold value for detecting activity.
 *
 * @param threshold The threshold value for detecting activity as an 8-bit
 *                  number. Scale factor is 62.5mg/LSB. A value of 0 may
 *                  result in undesirable behavior if the activity
 *                  interrupt is enabled.
 */
int setActivityThreshold(I2C_HandleTypeDef *handle, char threshold);

/**
 * Get the threshold value for detecting inactivity.
 *
 * @return The threshold value for detecting inactivity as an 8-bit number.
 *         Scale factor is 62.5mg/LSB.
 */
 char getInactivityThreshold(I2C_HandleTypeDef *handle);

/**
 * Set the threshold value for detecting inactivity.
 *
 * @param threshold The threshold value for detecting inactivity as an
 *                  8-bit number. Scale factor is 62.5mg/LSB.
 */
int setInactivityThreshold(I2C_HandleTypeDef *handle, char threshold);

/**
 * Get the time required for inactivity to be declared.
 *
 * @return The amount of time that acceleration must be less than the
 *         inactivity threshold for inactivity to be declared, in
 *         seconds.
 */
 char getTimeInactivity(I2C_HandleTypeDef *handle);

/**
 * Set the time required for inactivity to be declared.
 *
 * @param inactivity The amount of time that acceleration must be less than
 *                   the inactivity threshold for inactivity to be
 *                   declared, in seconds. A value of 0 results in an
 *                   interrupt when the output data is less than the
 *                   threshold inactivity.
 */
int setTimeInactivity(I2C_HandleTypeDef *handle, char timeInactivity);

/**
 * Get the activity/inactivity control settings.
 *
 *      D7            D6             D5            D4
 * +-----------+--------------+--------------+--------------+
 * | ACT ac/dc | ACT_X enable | ACT_Y enable | ACT_Z enable |
 * +-----------+--------------+--------------+--------------+
 *
 *        D3             D2               D1              D0
 * +-------------+----------------+----------------+----------------+
 * | INACT ac/dc | INACT_X enable | INACT_Y enable | INACT_Z enable |
 * +-------------+----------------+----------------+----------------+
 *
 * See datasheet for details.
 *
 * @return The contents of the ACT_INACT_CTL register.
 */
 char getActivityInactivityControl(I2C_HandleTypeDef *handle);

/**
 * Set the activity/inactivity control settings.
 *
 *      D7            D6             D5            D4
 * +-----------+--------------+--------------+--------------+
 * | ACT ac/dc | ACT_X enable | ACT_Y enable | ACT_Z enable |
 * +-----------+--------------+--------------+--------------+
 *
 *        D3             D2               D1              D0
 * +-------------+----------------+----------------+----------------+
 * | INACT ac/dc | INACT_X enable | INACT_Y enable | INACT_Z enable |
 * +-------------+----------------+----------------+----------------+
 *
 * See datasheet for details.
 *
 * @param settings The control byte to write to the ACT_INACT_CTL register.
 */
int setActivityInactivityControl(I2C_HandleTypeDef *handle, char settings);

/**
 * Get the threshold for free fall detection.
 *
 * @return The threshold value for free-fall detection, as an 8-bit number,
 *         with scale factor 62.5mg/LSB.
 */
 char getFreefallThreshold(I2C_HandleTypeDef *handle);

/**
 * Set the threshold for free fall detection.
 *
 * @return The threshold value for free-fall detection, as an 8-bit number,
 *         with scale factor 62.5mg/LSB. A value of 0 may result in
 *         undesirable behavior if the free-fall interrupt is enabled.
 *         Values between 300 mg and 600 mg (0x05 to 0x09) are recommended.
 */
int setFreefallThreshold(I2C_HandleTypeDef *handle, char threshold);

/**
 * Get the time required to generate a free fall interrupt.
 *
 * @return The minimum time that the value of all axes must be less than
 *         the freefall threshold to generate a free-fall interrupt, in
 *         milliseconds.
 */
 char getFreefallTime(I2C_HandleTypeDef *handle);

/**
 * Set the time required to generate a free fall interrupt.
 *
 * @return The minimum time that the value of all axes must be less than
 *         the freefall threshold to generate a free-fall interrupt, in
 *         milliseconds. A value of 0 may result in undesirable behavior
 *         if the free-fall interrupt is enabled. Values between 100 ms
 *         and 350 ms (0x14 to 0x46) are recommended.
 */
int setFreefallTime(I2C_HandleTypeDef *handle, short int freefallTime_ms);

/**
 * Get the axis tap settings.
 *
 *      D3           D2            D1             D0
 * +----------+--------------+--------------+--------------+
 * | Suppress | TAP_X enable | TAP_Y enable | TAP_Z enable |
 * +----------+--------------+--------------+--------------+
 *
 * (D7-D4 are 0s).
 *
 * See datasheet for more details.
 *
 * @return The contents of the TAP_AXES register.
 */
 char getTapAxisControl(I2C_HandleTypeDef *handle);

/**
 * Set the axis tap settings.
 *
 *      D3           D2            D1             D0
 * +----------+--------------+--------------+--------------+
 * | Suppress | TAP_X enable | TAP_Y enable | TAP_Z enable |
 * +----------+--------------+--------------+--------------+
 *
 * (D7-D4 are 0s).
 *
 * See datasheet for more details.
 *
 * @param The control byte to write to the TAP_AXES register.
 */
int setTapAxisControl(I2C_HandleTypeDef *handle, char settings);

/**
 * Get the source of a tap.
 *
 * @return The contents of the ACT_TAP_STATUS register.
 */
 char getTapSource(I2C_HandleTypeDef *handle);

 /**
 * Get the interrupt enable settings.
 *
 * @return The contents of the INT_ENABLE register.
 */

 char getInterruptEnableControl(I2C_HandleTypeDef *handle);

/**
 * Set the interrupt enable settings.
 *
 * @param settings The control byte to write to the INT_ENABLE register.
 */
int setInterruptEnableControl(I2C_HandleTypeDef *handle, char settings);

/**
 * Get the interrupt mapping settings.
 *
 * @return The contents of the INT_MAP register.
 */
 char getInterruptMappingControl(I2C_HandleTypeDef *handle);

/**
 * Set the interrupt mapping settings.
 *
 * @param settings The control byte to write to the INT_MAP register.
 */
int setInterruptMappingControl(I2C_HandleTypeDef *handle, char settings);

/**
 * Get the interrupt source.
 *
 * @return The contents of the INT_SOURCE register.
 */
 char getInterruptSource(I2C_HandleTypeDef *handle);

/**
 * Read one byte from a register on the device.
 *
 * @param: - the address to be read from
 *
 * @return: the value of the data read
 */
 uint8_t SingleByteRead(I2C_HandleTypeDef *handle, uint8_t address);

/**
 * Write one byte to a register on the device.
 *
 * @param:
	- address of the register to write to.
	- the value of the data to store
 */


int SingleByteWrite(I2C_HandleTypeDef *handle, uint8_t address, uint8_t data);

/**
 * Read several consecutive bytes on the device and store them in a given location.
 *
 * @param startAddress: The address of the first register to read from.
 * @param ptr_output: a pointer to the location to store the data being read
 * @param size: The number of bytes to read.
 */
 void multiByteRead(I2C_HandleTypeDef *handle, uint8_t address, uint8_t* output, int size);

/**
 * Write several consecutive bytes  on the device.
 *
 * @param startAddress: The address of the first register to write to.
 * @param ptr_data: Pointer to a location which contains the data to write.
 * @param size: The number of bytes to write.
 */
 int multiByteWrite(I2C_HandleTypeDef *handle, uint8_t address, uint8_t* ptr_data, int size);

#endif /* ADXL345_I2C_H */

