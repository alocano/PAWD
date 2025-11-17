/*!
 * @file Adafruit_VCNL4020.c
 *
 * @brief Adafruit VCNL4020 Proximity/Ambient Light sensor driver
 *
 *  Created on: Oct 22, 2025
 *  Author: Jake Ngugi
 */

#include <stdbool.h>
#include <stdint.h>
#include "Adafruit_VCNL4020.h"
#include "main.h"
extern I2C_HandleTypeDef hi2c2;


/*!
 * @brief  Helper function to read from a register
 * @param  *dev  Sensor Structure
 * @param  reg   target register of VCNL4020
 * @param value data stored in target register
 * @return True if data was successfully read and stored
 */
bool VCNL4020_ReadRegister(VCNL4020_HandleTypeDef *dev, uint8_t reg, uint8_t *value) {
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(dev->hi2c, dev->address, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return false;

    status = HAL_I2C_Master_Receive(dev->hi2c, dev->address, value, 1, HAL_MAX_DELAY);
    return (status == HAL_OK);
}

/*!
 * @brief  Helper function to write to a register
 * @param  *dev  Sensor Structure
 * @param  reg   target register of VCNL4020
 * @param value data written to target register
 * @return True if data was successfully written
 */
//
bool VCNL4020_WriteRegister(VCNL4020_HandleTypeDef *dev, uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(dev->hi2c, dev->address, data, 2, HAL_MAX_DELAY);
    return (status == HAL_OK);
}



/*!
 * @brief  Helper function to read 16-bit value from a register
 * @param  *dev  Sensor Structure
 * @param  reg   target register of VCNL4020
 * @param value data stored in target register
 * @return True if data was successfully read and stored
 */
bool VCNL4020_ReadRegister16(VCNL4020_HandleTypeDef *dev, uint8_t reg, uint16_t *value) {
    uint8_t data[2];

    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(dev->hi2c, dev->address, &reg, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) return false;

    status = HAL_I2C_Master_Receive(dev->hi2c, dev->address, data, 2, HAL_MAX_DELAY);
    if (status != HAL_OK) return false;

    *value = (data[0] << 8) | data[1];
    return true;
}


/*!
 * @brief  Initializes the VCNL4020 sensor and checks for a valid Product ID Revision.
 * @param  *dev  Sensor Structure
 * @param  *hi2c2  I2C Structure
 * @return True if initialization was successfully
 */
bool VCNL4020_Init(VCNL4020_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c2){
	if (dev == NULL || hi2c2 == NULL) {
		return false;
	}

	// Initialize sensor structure
	dev->hi2c = hi2c2;
	dev->address = VCNL4020_I2C_ADDRESS << 1; // HAL requires 7-bit address shifted left

	// Verify Product ID Revision
	uint8_t who_am_i = VCNL4020_GetProdRevision(dev);

	if (who_am_i != 0x21) {
		return false;
	}

	// To set up all configuration, first disable everything
	if (!VCNL4020_Enable(dev, false, false, false)) {
		return false;
	}

	if (!VCNL4020_SetOnDemand(dev, false, false)) {
		return false;
	}

	// Set fastest rate so folks see stuff, can always config lower power later
	if (!VCNL4020_SetProxRate(dev, PROX_RATE_250_PER_S)) {
		return false;
	}

	if (!VCNL4020_SetProxLEDmA(dev, 200)) {
		return false;
	}

	// Default IRQ on data ready
	if (!VCNL4020_SetInterruptConfig(dev, true, true, false, false, INT_COUNT_1)) {
		return false;
	}

	// Default frequency
	if (!VCNL4020_SetProxFrequency(dev, PROX_FREQ_390_625_KHZ)) {
		return false;
	}

	// Enable all functions
	if (!VCNL4020_Enable(dev, true, true, true)) {
		return false;
	}

	if (!VCNL4020_SetOnDemand(dev, false, false)) {
		return false;
	}

	dev->initialized = true;
	return true;

}





/*!
 * @brief  Enables or disables the ALS, Proximity, and Self-Timed measurements on Register #0.
 * @param  *dev  			  Sensor Structure
 * @param  als_enable         True to enable the ALS, otherwise false.
 * @param  prox_enable        True to enable the Proximity, otherwise false.
 * @param  self_timed_enable  True to enable the Self-Timed measurements, otherwise false.
 * @return True if all new bits are written to register.
 */
bool VCNL4020_Enable(VCNL4020_HandleTypeDef *dev, bool als_enable, bool prox_enable, bool self_timed_enable) {
    uint8_t config = 0;

    if (!VCNL4020_ReadRegister(dev, VCNL4020_REG_COMMAND, &config)) {
        return false;
    }

    // Clear enable bits
    config &= ~0x07;

    // Set new enable bits
    if (als_enable) config |= 0x04;
    if (prox_enable) config |= 0x02;
    if (self_timed_enable) config |= 0x01;

    return VCNL4020_WriteRegister(dev, VCNL4020_REG_COMMAND, config);
}

/*!
 * @brief  Sets the on-demand bits for ALS and Proximity measurements via Register #0.
 * @param  *dev  			  Sensor Structure
 * @param  als_on_demand  True to set the ALS on-demand bit, otherwise false.
 * @param  prox_on_demand True to set the Proximity on-demand bit, otherwise false.
 * @return True if all new bits are written to register.
 */
bool VCNL4020_SetOnDemand(VCNL4020_HandleTypeDef *dev, bool als_on_demand, bool prox_on_demand) {
    uint8_t config = 0;

    if (!VCNL4020_ReadRegister(dev, VCNL4020_REG_COMMAND, &config)) {
        return false;
    }

    // Clear on-demand bits
    config &= ~0x18;

    // Set new on-demand bits
    if (als_on_demand) config |= 0x10;
    if (prox_on_demand) config |= 0x08;

    return VCNL4020_WriteRegister(dev, VCNL4020_REG_COMMAND, config);
}


/*!
 * @brief  Gets the Product ID Revision from Register #1.
 * @param  *dev  Sensor Structure
 * @return 8-bit value representing the Product ID Revision.
 */
uint8_t VCNL4020_GetProdRevision(VCNL4020_HandleTypeDef *dev) {
    uint8_t prod_rev = 0;
    VCNL4020_ReadRegister(dev, VCNL4020_REG_PRODUCT_ID, &prod_rev);
    return prod_rev;
}


/*!
 * @brief  Sets the Proximity Rate via Register #2.
 * @param  *dev  Sensor Structure.
 * @param  rate  The rate to set, as defined in the vcnl4020_proxrate enum.
 * @return True if all new bits are written to register.
 */
bool VCNL4020_SetProxRate(VCNL4020_HandleTypeDef *dev, vcnl4020_proxrate rate) {
    uint8_t config = 0;

    if (!VCNL4020_ReadRegister(dev, VCNL4020_REG_PROX_RATE, &config)) {
        return false;
    }

    // Clear rate bits (bits 4-6)
    config &= ~0x07;

    // Set new rate
    config |= rate;

    return VCNL4020_WriteRegister(dev, VCNL4020_REG_PROX_RATE, config);
}


/*!
 * @brief  Gets the current Proximity Rate from Register #2 .
 * @param  *dev  Sensor Structure.
 * @return The current rate, as defined in the vcnl4020_proxrate enum.
 */
vcnl4020_proxrate VCNL4020_GetProxRate(VCNL4020_HandleTypeDef *dev){
	uint8_t rate_value=0;

	HAL_I2C_Mem_Read(dev->hi2c, dev->address, VCNL4020_REG_PROX_RATE, 1, &rate_value, 1, HAL_MAX_DELAY);

	// Extract the rate bits (typically 3 bits in the register)
	uint8_t rate_bits = rate_value & 0x07;

	// Return the corresponding enum value
	return (vcnl4020_proxrate)rate_bits;
}

/*!
 * @brief  Sets the Proximity Frequency in Register #15 Proximity Modulator Timing Adjustment.
 * @param  *dev  Sensor Structure.
 * @param  freq  The proximity frequency setting, as defined in the vcnl4020_proxfreq enum.
 * @return True if all new bits are written to register.
 */
bool VCNL4020_SetProxFrequency(VCNL4020_HandleTypeDef *dev, vcnl4020_proxfreq freq){
    uint8_t config = 0;

    if (!VCNL4020_ReadRegister(dev, VCNL4020_REG_PROX_ADJUST, &config)) {
        return false;
    }

    // Clear rate bits (bits 3-4)
    config &= ~0x18;

    // Set new rate
    config |= (freq << 3);

    return VCNL4020_WriteRegister(dev, VCNL4020_REG_PROX_ADJUST, config);
}


/*!
 * @brief  Gets the proximity frequency setting from Register #15.
 * @param  *dev  Sensor Structure.
 * @return  The current proximity frequency setting, as defined in the vcnl4020_proxfreq enum.
 */
vcnl4020_proxfreq VCNL4020_GetProxFrequency(VCNL4020_HandleTypeDef *dev){
	uint8_t freq_value=0;

	HAL_I2C_Mem_Read(dev->hi2c, dev->address, VCNL4020_REG_PROX_ADJUST, 1, &freq_value, 1, HAL_MAX_DELAY);

	// Extract the rate bits (typically 3 bits in the register)
	uint8_t freq_bits = freq_value & 0x18;

	// Return the corresponding enum value
	return (vcnl4020_proxfreq)freq_bits;
}



/*!
 * @brief  Sets the LED current for Proximity Mode in mA via Register #3.
 * @param  *dev  Sensor Structure.
 * @param  current_ma  The LED current in mA.
 * @return True if all new bits are written to register.
 *
 */
bool VCNL4020_SetProxLEDmA(VCNL4020_HandleTypeDef *dev, uint8_t current_ma) {
    // Convert mA to register value (0-20 = 10mA to 200mA in 10mA steps)
    uint8_t led_config = (current_ma / 10);

    if (led_config > 0x14) { // Max 200mA
        led_config = 0x14;
    }

    return VCNL4020_WriteRegister(dev, VCNL4020_REG_IR_LED_CURRENT, led_config);
}


/*!
 * @brief  Gets the LED current for Proximity Mode in mA.
 * @param  *dev  Sensor Structure.
 * @return The LED current in mA.
 */
uint8_t VCNL4020_GetProxLEDmA(VCNL4020_HandleTypeDef *dev){
	uint8_t led_value=0;

	HAL_I2C_Mem_Read(dev->hi2c, dev->address, VCNL4020_REG_IR_LED_CURRENT, 1, &led_value, 1, HAL_MAX_DELAY);

	// Extract the rate bits (typically 3 bits in the register)
	uint8_t led_current_bits = (led_value & 0x3F)*10;

	// Return the corresponding enum value
	return led_current_bits;

}


/*!
 * @brief  Reads the Proximity Measurement Result.
 * @param  *dev  Sensor Structure.
 * @return The 16-bit Proximity Measurement Result.
 */
uint16_t VCNL4020_ReadProximity(VCNL4020_HandleTypeDef *dev){
	uint8_t reg_addr = VCNL4020_REG_PROX_RESULT_HIGH;
	uint8_t rx_data[2];
	uint16_t proximity_result;

	// Read 2 bytes from the proximity result register (high byte first)
	HAL_I2C_Mem_Read(dev->hi2c, dev->address, reg_addr, I2C_MEMADD_SIZE_8BIT, rx_data, 2, HAL_MAX_DELAY);

	// Combine the two bytes into a 16-bit value (MSB first)
	proximity_result = (rx_data[0] << 8) | rx_data[1];

	return proximity_result;


}


/*!
 * @brief  Checks if the Proximity data is ready.
 * @param  *dev  Sensor Structure.
 * @return True if Proximity data is ready, otherwise false.
 */
bool VCNL4020_IsProxReady(VCNL4020_HandleTypeDef *dev){
    uint8_t prox_data_rdy = 0;

    // Read the command register
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(dev->hi2c, dev->address, VCNL4020_REG_COMMAND, 1, &prox_data_rdy, 1, HAL_MAX_DELAY);

    // Check if I2C read was successful
    if (status != HAL_OK) {
        return false; // Or handle error appropriately
    }

    // Extract the proximity data ready bit (bit 5) and return as boolean
    return (prox_data_rdy & 0x20) != 0;

}


/*!
 * @brief  Sets the Low Threshold for Proximity Measurement.
 * @param  *dev  Sensor Structure.
 * @param  threshold  The 16-bit Low Threshold value.
 */
void VCNL4020_setLowThreshold(VCNL4020_HandleTypeDef *dev, uint16_t threshold) {
  uint8_t data[2];

  // Split 16-bit threshold into two 8-bit values (MSB first)
  data[0] = (threshold >> 8) & 0xFF;  // High byte
  data[1] = threshold & 0xFF;         // Low byte

  // Write both bytes to the sensor
  HAL_I2C_Mem_Write(dev->hi2c, dev->address, VCNL4020_REG_LOW_THRES_HIGH, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
}

/*!
 * @brief  Gets the Low Threshold for Proximity Measurement.
 * @param  *dev  Sensor Structure.
 * @return The 16-bit Low Threshold value.
 */
uint16_t VCNL4020_getLowThreshold(VCNL4020_HandleTypeDef *dev) {
  uint8_t data[2];
  uint16_t threshold;

  // Read two bytes from the sensor
  HAL_I2C_Mem_Read(dev->hi2c, dev->address, VCNL4020_REG_LOW_THRES_HIGH, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

  // Combine two 8-bit values into 16-bit threshold (MSB first)
  threshold = (data[0] << 8) | data[1];

  return threshold;
}

/*!
 * @brief  Sets the High Threshold for Proximity Measurement.
 * @param  *dev  Sensor Structure.
 * @param  threshold  The 16-bit High Threshold value.
 */
void VCNL4020_setHighThreshold(VCNL4020_HandleTypeDef *dev, uint16_t threshold) {
  uint8_t data[2];

  // Split 16-bit threshold into two 8-bit values (MSB first)
  data[0] = (threshold >> 8) & 0xFF;  // High byte
  data[1] = threshold & 0xFF;         // Low byte

  // Write both bytes to the sensor
  HAL_I2C_Mem_Write(dev->hi2c, dev->address, VCNL4020_REG_HIGH_THRES_HIGH, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
}

/*!
 * @brief  Gets the High Threshold for Proximity Measurement.
 * @param  *dev  Sensor Structure.
 * @return The 16-bit High Threshold value.
 */
uint16_t VCNL4020_getHighThreshold(VCNL4020_HandleTypeDef *dev) {
  uint8_t data[2];
  uint16_t threshold;

  // Read two bytes from the sensor
  HAL_I2C_Mem_Read(dev->hi2c, dev->address, VCNL4020_REG_HIGH_THRES_HIGH, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);

  // Combine two 8-bit values into 16-bit threshold (MSB first)
  threshold = (data[0] << 8) | data[1];

  return threshold;
}



bool VCNL4020_SetInterruptConfig(VCNL4020_HandleTypeDef *dev, bool proxReady, bool alsReady, bool thresh, bool threshALS, vcnl4020_int_count intCount){
	uint8_t reg_value = 0;

	// Construct byte according to bit positions in Register #9
	reg_value |= (intCount & 0x07) << 5; // bits 7-5
	reg_value |= (proxReady ? 1 : 0) << 3;
	reg_value |= (alsReady ? 1 : 0) << 2;
	reg_value |= (thresh ? 1 : 0) << 1;
	reg_value |= (threshALS ? 1 : 0);


	// Write the modified value back to the register
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(dev->hi2c, dev->address, VCNL4020_REG_INT_CTRL,I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
	if (status != HAL_OK){
		return false;
	}

	else{
		return (status == HAL_OK);
	}
}


/*!
 * @brief  Gets the status of the interrupts from INTERRUPT STATUS Register #14.
 * @return uint8_t containing the lower 4 bits of the INTERRUPT STATUS REGISTER.
 */
uint8_t VCNL4020_getInterruptStatus(VCNL4020_HandleTypeDef *dev) {
  uint8_t int_status;

  // Read the interrupt status register
  HAL_I2C_Mem_Read(dev->hi2c, dev->address, VCNL4020_REG_INT_STATUS, I2C_MEMADD_SIZE_8BIT, &int_status, 1, HAL_MAX_DELAY);

  // Mask the lower 4 bits to get the interrupt status
  return (int_status & 0x0F);
}


/*!
 * @brief  Clears the specified interrupt flags in INTERRUPT STATUS REGISTER #14.
 *
 * @param  proxready  True to clear the Proximity Ready interrupt flag, False to leave it.
 * @param  alsready   True to clear the ALS Ready interrupt flag, False to leave it.
 * @param  th_low     True to clear the Low Threshold interrupt flag, False to leave it.
 * @param  th_high    True to clear the High Threshold interrupt flag, False to leave it.
 */
void VCNL4020_clearInterrupts(VCNL4020_HandleTypeDef *dev, bool proxready, bool alsready,bool th_low, bool th_high) {
  uint8_t int_status;

  // Read the current value of the register
  HAL_I2C_Mem_Read(dev->hi2c, dev->address, VCNL4020_REG_INT_STATUS, I2C_MEMADD_SIZE_8BIT, &int_status, 1, HAL_MAX_DELAY);

  // Prepare the bits to be cleared (set bits to 1 to clear them)
  uint8_t clear_bits = 0;
  if (proxready)
    clear_bits |= VCNL4020_INT_PROX_READY;
  if (alsready)
    clear_bits |= VCNL4020_INT_ALS_READY;
  if (th_low)
    clear_bits |= VCNL4020_INT_TH_LOW;
  if (th_high)
    clear_bits |= VCNL4020_INT_TH_HI;

  // Clear the specified bits by writing '1' to them
  // Note: Writing 1 clears the interrupt flags in this register
  HAL_I2C_Mem_Write(dev->hi2c, dev->address, VCNL4020_REG_INT_STATUS, I2C_MEMADD_SIZE_8BIT, &clear_bits, 1, HAL_MAX_DELAY);

}

