#include "MCP23X17.h"
// #include <stdbool.h>

  bool protocol = 0;	//default SPI

  void setup_MCP23X17()
  {
    // MCP23X17_write(IO_DEVICE_1, MCP_IOCONA, 0x38);      //Device Configutation
    // MCP23X17_write(IO_DEVICE_1, MCP_IOCONB, 0x38);      //Device Configutation
    // MCP23X17_write(IO_DEVICE_1, MCP_IODIRA, 0xFF);      //Set pins as inputs or outputs on side A
    // MCP23X17_write(IO_DEVICE_1, MCP_IODIRB, 0xFF);      //Set pins as inputs or outputs on side B
    // MCP23X17_write(IO_DEVICE_1, MCP_GPPUA, 0xFF);       //I/O pullup pin state on side A
    // MCP23X17_write(IO_DEVICE_1, MCP_GPPUB, 0xFF);       //I/O pullup pin state on side B
    // MCP23X17_write(IO_DEVICE_1, MCP_IPOLA, 0xFF);       //Signal polarity on side A
    // MCP23X17_write(IO_DEVICE_1, MCP_IPOLB, 0xFF);       //Signal polarity on side B

    // MCP23X17_write(IO_DEVICE_2, MCP_IOCONA, 0x38);      //Device Configutation
    // MCP23X17_write(IO_DEVICE_2, MCP_IOCONB, 0x38);      //Device Configutation
    // MCP23X17_write(IO_DEVICE_2, MCP_IODIRA, 0xFF);      //Set pins as inputs or outputs on side A
    // MCP23X17_write(IO_DEVICE_2, MCP_IODIRB, 0xFF);      //Set pins as inputs or outputs on side B
    // MCP23X17_write(IO_DEVICE_2, MCP_GPPUA, 0xFF);       //I/O pullup pin state on side A
    // MCP23X17_write(IO_DEVICE_2, MCP_GPPUB, 0xFF);       //I/O pullup pin state on side B
    // MCP23X17_write(IO_DEVICE_2, MCP_IPOLA, 0xFF);       //Signal polarity on side A
    // MCP23X17_write(IO_DEVICE_2, MCP_IPOLB, 0xFF);       //Signal polarity on side B

    setup_MCP23S17();
    
	//Tesr protocol
    uint8_t select = 0;
	HAL_Delay(1000);
    select = MCP23S17_read(IO_DEVICE_1, MCP_GPIOA);
    if (select == 0xFF)
    {					//set I2C
	  protocol = 1;
	  setup_MCP23017();
    }
	else 
	{					//keep SPI
      protocol = 0;
	}
  }


  void setup_MCP23S17()
  {
    MCP23S17_write(IO_DEVICE_1, MCP_IOCONA, 0x38);      //Device Configutation
    MCP23S17_write(IO_DEVICE_1, MCP_IOCONB, 0x38);      //Device Configutation
    MCP23S17_write(IO_DEVICE_1, MCP_IODIRA, 0xFF);      //Set pins as inputs or outputs on side A
    MCP23S17_write(IO_DEVICE_1, MCP_IODIRB, 0xFF);      //Set pins as inputs or outputs on side B
    MCP23S17_write(IO_DEVICE_1, MCP_GPPUA, 0xFF);       //I/O pullup pin state on side A
    MCP23S17_write(IO_DEVICE_1, MCP_GPPUB, 0xFF);       //I/O pullup pin state on side B
    MCP23S17_write(IO_DEVICE_1, MCP_IPOLA, 0xFF);       //Signal polarity on side A
    MCP23S17_write(IO_DEVICE_1, MCP_IPOLB, 0xFF);       //Signal polarity on side B

    MCP23S17_write(IO_DEVICE_2, MCP_IOCONA, 0x38);      //Device Configutation
    MCP23S17_write(IO_DEVICE_2, MCP_IOCONB, 0x38);      //Device Configutation
    MCP23S17_write(IO_DEVICE_2, MCP_IODIRA, 0xFF);      //Set pins as inputs or outputs on side A
    MCP23S17_write(IO_DEVICE_2, MCP_IODIRB, 0xFF);      //Set pins as inputs or outputs on side B
    MCP23S17_write(IO_DEVICE_2, MCP_GPPUA, 0xFF);       //I/O pullup pin state on side A
    MCP23S17_write(IO_DEVICE_2, MCP_GPPUB, 0xFF);       //I/O pullup pin state on side B
    MCP23S17_write(IO_DEVICE_2, MCP_IPOLA, 0xFF);       //Signal polarity on side A
    MCP23S17_write(IO_DEVICE_2, MCP_IPOLB, 0xFF);       //Signal polarity on side B
  }


  void setup_MCP23017()
  {
    MCP23017_write(IO_DEVICE_1, MCP_IOCONA, 0x38);      //Device Configutation
    MCP23017_write(IO_DEVICE_1, MCP_IOCONB, 0x38);      //Device Configutation
    MCP23017_write(IO_DEVICE_1, MCP_IODIRA, 0xFF);      //Set pins as inputs or outputs on side A
    MCP23017_write(IO_DEVICE_1, MCP_IODIRB, 0xBF);      //Set pins as inputs or outputs on side B - added output led
    MCP23017_write(IO_DEVICE_1, MCP_GPPUA, 0xFF);       //I/O pullup pin state on side A
    MCP23017_write(IO_DEVICE_1, MCP_GPPUB, 0xFF);       //I/O pullup pin state on side B
    MCP23017_write(IO_DEVICE_1, MCP_IPOLA, 0xFF);       //Signal polarity on side A
    MCP23017_write(IO_DEVICE_1, MCP_IPOLB, 0xFF);       //Signal polarity on side B

    MCP23017_write(IO_DEVICE_2, MCP_IOCONA, 0x38);      //Device Configutation
    MCP23017_write(IO_DEVICE_2, MCP_IOCONB, 0x38);      //Device Configutation
    MCP23017_write(IO_DEVICE_2, MCP_IODIRA, 0xFF);      //Set pins as inputs or outputs on side A
    MCP23017_write(IO_DEVICE_2, MCP_IODIRB, 0xFF);      //Set pins as inputs or outputs on side B
    MCP23017_write(IO_DEVICE_2, MCP_GPPUA, 0xFF);       //I/O pullup pin state on side A
    MCP23017_write(IO_DEVICE_2, MCP_GPPUB, 0xFF);       //I/O pullup pin state on side B
    MCP23017_write(IO_DEVICE_2, MCP_IPOLA, 0xFF);       //Signal polarity on side A
    MCP23017_write(IO_DEVICE_2, MCP_IPOLB, 0xFF);       //Signal polarity on side B
  }

  void MCP23X17_write(uint8_t device, uint8_t address, uint8_t value)
  {
    if (protocol == 0) MCP23S17_write(device, address, value);
    if (protocol == 1) MCP23017_write(device, address, value);
  }

  uint8_t MCP23X17_read(uint8_t device, uint8_t address)
  {
    uint8_t received_data = 0;
    if (protocol == 0) received_data = MCP23S17_read(device, address);
    if (protocol == 1) received_data = MCP23017_read(device, address);
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