#include "MCP23X17.h"
typedef enum {false, true} bool;

bool protocolspi = 1;	//set SPI
bool protocoli2c = 1;	//set I2C

  void setup_MCP23X17()
  {
    MCP23X17_write(IO_DEVICE_1, MCP_IOCONA, 0x38);      //Device Configutation
    MCP23X17_write(IO_DEVICE_1, MCP_IOCONB, 0x38);      //Device Configutation
    MCP23X17_write(IO_DEVICE_1, MCP_IODIRA, 0xFF);      //Set pins as inputs or outputs on side A
    MCP23X17_write(IO_DEVICE_1, MCP_IODIRB, 0xFF);      //Set pins as inputs or outputs on side B
    MCP23X17_write(IO_DEVICE_1, MCP_GPPUA, 0xFF);       //I/O pullup pin state on side A
    MCP23X17_write(IO_DEVICE_1, MCP_GPPUB, 0xFF);       //I/O pullup pin state on side B
    MCP23X17_write(IO_DEVICE_1, MCP_IPOLA, 0xFF);       //Signal polarity on side A
    MCP23X17_write(IO_DEVICE_1, MCP_IPOLB, 0xFF);       //Signal polarity on side B

    MCP23X17_write(IO_DEVICE_2, MCP_IOCONA, 0x38);      //Device Configutation
    MCP23X17_write(IO_DEVICE_2, MCP_IOCONB, 0x38);      //Device Configutation
    MCP23X17_write(IO_DEVICE_2, MCP_IODIRA, 0xFF);      //Set pins as inputs or outputs on side A
    MCP23X17_write(IO_DEVICE_2, MCP_IODIRB, 0xFF);      //Set pins as inputs or outputs on side B
    MCP23X17_write(IO_DEVICE_2, MCP_GPPUA, 0xFF);       //I/O pullup pin state on side A
    MCP23X17_write(IO_DEVICE_2, MCP_GPPUB, 0xFF);       //I/O pullup pin state on side B
    MCP23X17_write(IO_DEVICE_2, MCP_IPOLA, 0xFF);       //Signal polarity on side A
    MCP23X17_write(IO_DEVICE_2, MCP_IPOLB, 0xFF);       //Signal polarity on side B

    //Tesr protocol
	HAL_Delay(50);
    uint8_t select;
    select = MCP23S17_read(IO_DEVICE_1, MCP_GPIOA);
    if (select == 0xFF)
    {					//select only I2C
      protocolspi = 0;
      protocoli2c = 1;
    }
	else 
	{					//select only SPI
      protocolspi = 1;
      protocoli2c = 0;
	}
  }


  void MCP23X17_write(uint8_t device, uint8_t address, uint8_t value)
  {
    if (protocolspi) MCP23S17_write(device, address, value);
    if (protocoli2c) MCP23017_write(device, address, value);
  }

  uint8_t MCP23X17_read(uint8_t device, uint8_t address)
  {
    uint8_t received_data = 0;
    if (protocolspi) received_data = MCP23S17_read(device, address);
    if (protocoli2c) received_data = MCP23017_read(device, address);
	return received_data;
  }


  void MCP23S17_write(uint8_t device, uint8_t address, uint8_t value)
  {
    uint8_t SPI_TX[3] = {device, address, value};
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, SPI_TX, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  }

  uint8_t MCP23S17_read(uint8_t device, uint8_t address)
  {
    device |= 0x1;                                       //change device to read mode 
    uint8_t received_spi = 0;
    uint8_t SPI_TX[2] = {device, address};
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, SPI_TX, 2, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi3, &received_spi, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    return received_spi;
  }


  void MCP23017_write(uint8_t device, uint8_t address, uint8_t value)
  {
	HAL_I2C_Mem_Write(&hi2c3, device, address, 1, &value, 1, HAL_MAX_DELAY);
  }
  uint8_t MCP23017_read(uint8_t device, uint8_t address)
  {
    device |= 0x1;                                       //change device to read mode 
    uint8_t received_i2c = 0;
	HAL_I2C_Mem_Read(&hi2c3, device, address, 1, &received_i2c, 1, HAL_MAX_DELAY);
	return received_i2c;
  }