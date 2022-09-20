#include "ds2782.h"

static void writeRegister(uint8_t addr, uint8_t value);
static uint8_t readRegister(uint8_t addr);

extern I2C_HandleTypeDef hi2c1;

/**
 * @brief Write DS2782 register
 * @param: addr - register address
 * @param: value - register data
 */
static void writeRegister(uint8_t addr, uint8_t value)
{
	uint8_t data[2] = {0};
	data[0] = addr;
	data[1] = value;

	HAL_I2C_Master_Transmit(&hi2c1, DS2782_SLAVE_ADDR, data, 2, 1000);
}

/**
 * @brief Read DS2782 register
 * @param: addr - register address
 * @return: register data
 */
static uint8_t readRegister(uint8_t addr)
{
	uint8_t txData = 0;
	uint8_t rxData = 0;

	txData = addr;

	HAL_I2C_Master_Transmit(&hi2c1, DS2782_SLAVE_ADDR, &txData, 1, 1000); // send register address
	HAL_I2C_Master_Receive(&hi2c1, DS2782_SLAVE_ADDR, &rxData, 1, 1000); // receive register value
	return rxData;
}
