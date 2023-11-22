#ifndef _obd_h
#define _obd_h

#define CAN_VMCU_ID 0x7DF
#define CAN_RPM_PID 0x0C
#define CAN_SPEED_PID 0x0D

#define CAN_VIN_ID 0x7DF

#define can_tx 2 // tx of serial can module connect to D2
#define can_rx 3 // rx of serial can module connect to D3

#define CAN_DATA_BUF_LEN 64
#define OBD_DATA_BUF_LEN 512

#define OBD_FRAME_BITMASK (0xF0)
#define OBD_LEN_BITMASK (0x0F)

void uart_init(unsigned long);
void uart_write(unsigned char);
unsigned char uart_read();
int uart_available();

int obdRequestBlocking(unsigned long, unsigned char*, unsigned char*, unsigned char*, unsigned long);

#endif