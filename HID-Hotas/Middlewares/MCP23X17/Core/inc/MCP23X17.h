#define IO_DEVICE_1       0x40      //OPCODE address device 1 (Write) / for read 0x41
#define IO_DEVICE_2       0x42      //OPCODE address device 2 (Write) / for read 0x43

#define MCP_IODIRA        0x00      //Data Direction Register for PORTA
#define MCP_IODIRB        0x01      //Data Direction Register for PORTB
#define MCP_IPOLA         0x02      //Input Polarity Register for PORTA
#define MCP_IPOLB         0x03      //Input Polarity Register for PORTB
#define MCP_GPINTENA      0x04      //Interrupt-on-change enable Register for PORTA
#define MCP_GPINTENB      0x05      //Interrupt-on-change enable Register for PORTB
#define MCP_DEFVALA       0x06      //Default Value Register for PORTA
#define MCP_DEFVALB       0x07      //Default Value Register for PORTB
#define MCP_INTCONA       0x08      //Interrupt-on-change control Register for PORTA
#define MCP_INTCONB       0x09      //Interrupt-on-change control Register for PORTB
#define MCP_IOCONA        0x0A      //Configuration register for device
#define MCP_IOCONB        0x0B      //Configuration register for device
#define MCP_GPPUA         0x0C      //100kOhm pullup resistor register for PORTA (sets pin to input when set)
#define MCP_GPPUB         0x0D      //100kOhm pullup resistor register for PORTB (sets pin to input when set)
#define MCP_INTFA         0x0E      //Interrupt flag Register for PORTA
#define MCP_INTFB         0x0F      //Interrupt flag Register for PORTB
#define MCP_INTCAPA       0x10      //Interrupt captured value Register for PORTA
#define MCP_INTCAPB       0x11      //Interrupt captured value Register for PORTB
#define MCP_GPIOA         0x12      //General purpose I/O Register for PORTA
#define MCP_GPIOB         0x13      //General purpose I/O Register for PORTB
#define MCP_OLATA         0x14      //Output latch Register for PORTA
#define MCP_OLATB         0x15      //Output latch Register for PORTB

  void setup_MCP23S17();
  void setup_MCP23017();
  void MCP23X17_write(uint8_t device, uint8_t address, uint8_t value);
  uint8_t MCP23X17_read(uint8_t device, uint8_t address);
  void MCP23S17_write(uint8_t device, uint8_t address, uint8_t value);
  uint8_t MCP23S17_read(uint8_t device, uint8_t address);
  void MCP23017_write(uint8_t device, uint8_t address, uint8_t value);
  uint8_t MCP23017_read(uint8_t device, uint8_t address);

  typedef enum {false, true} bool;
  
  typedef struct InputIndexing
  {
    uint8_t byte_index;
    uint8_t bit_index;
  } InputIndexing;
  
  InputIndexing input_map_spi[] = {
    {0, 0}, {0, 3}, {0, 4}, {0, 2}, {0, 1}, {0, 5}, {0, 6}, {0, 7}, // Paddle / Countermeasures (4pos) / Trigger (2stage) / Weapon Release
    {1, 0}, {1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}, // Trigger management (4pos) / Display management (4pos)
    {2, 0}, {2, 7}, {3, 0}, {3, 1}, {3, 2}, {3, 3}, {3, 4}, {2, 1}  // Expand / Missle step / Trim (5pos) / -
  };
  
  InputIndexing input_map[] = {
    {0, 0}, {2, 1}, {2, 4}, {2, 2}, {2, 5}, {2, 3}, {3, 3}, {3, 4}, // Paddle / Countermeasures (5pos) / Trigger (2stage)
    {1, 5}, {1, 3}, {1, 2}, {1, 4}, {1, 0}, {1, 1}, {0, 7}, {0, 6}, // Weapon Release / Trigger management (5pos) / Display management (2/5pos)
    {0, 3}, {0, 5}, {0, 4}, {2, 0}, {0, 2}, {2, 7}, {3, 2}, {3, 1}, // Display management (3/5pos) / Expand / Missle step / Trim (3/5pos)
    {3, 0}, {2, 6}, {3, 5}, {3, 6}, {3, 7}, {1, 7}, {0, 1}, {1, 6}  // Trim (2/5pos) / - (zero padding 6bit)
                  //                                       /  LED  / 
  };
