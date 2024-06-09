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

void setup_MCP23X17();
void MCP23X17_write(uint8_t device, uint8_t address, uint8_t value);
uint8_t MCP23X17_read(uint8_t device, uint8_t address);