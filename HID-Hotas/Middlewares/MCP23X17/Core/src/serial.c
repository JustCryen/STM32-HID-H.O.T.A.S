#include "MCP23X17.h"

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

/*  MCP23X17_write(IO_DEVICE_2, MCP_IOCONA, 0x38);      //Device Configutation
    MCP23X17_write(IO_DEVICE_2, MCP_IOCONB, 0x38);      //Device Configutation
    MCP23X17_write(IO_DEVICE_2, MCP_IODIRA, 0xFF);      //Set pins as inputs or outputs on side A
    MCP23X17_write(IO_DEVICE_2, MCP_IODIRB, 0xFF);      //Set pins as inputs or outputs on side B
    MCP23X17_write(IO_DEVICE_2, MCP_GPPUA, 0xFF);       //I/O pullup pin state on side A
    MCP23X17_write(IO_DEVICE_2, MCP_GPPUB, 0xFF);       //I/O pullup pin state on side B
    MCP23X17_write(IO_DEVICE_2, MCP_IPOLA, 0xFF);       //Signal polarity on side A
    MCP23X17_write(IO_DEVICE_2, MCP_IPOLB, 0xFF);       //Signal polarity on side B
*/}

#ifdef serial_MCP23S17
  void MCP23X17_write(uint8_t device, uint8_t address, uint8_t value)
  {
    uint8_t SPI_TX[3] = {device, address, value};
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, SPI_TX, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  }

  uint8_t MCP23X17_read(uint8_t device, uint8_t address)
  {
    device |= 0x1;                                       //change device to read mode 
    uint8_t received_data = 0;
    uint8_t SPI_TX[2] = {device, address};
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi3, SPI_TX, 2, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi3, &received_data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
    return received_data;
  }
#endif

#ifdef serial_MCP23017
  void MCP23X17_write(uint8_t device, uint8_t address, uint8_t value)
  {
    uint8_t SPI_TX[3] = {device, address, value};
    
  }
  uint8_t MCP23X17_read(uint8_t device, uint8_t address)
  {
    device |= 0x1;                                       //change device to read mode 
    uint8_t received_data = 0;
    uint8_t SPI_TX[2] = {device, address};
    
	return received_data;
  }
#endif