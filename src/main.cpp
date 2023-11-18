#include <Arduino.h>
#include <Arduino_GFX_Library.h>
#include <HardwareSerial.h>
#include <Serial_CAN_Module.h>

// see https://stackoverflow.com/questions/1644868/define-macro-for-debug-printing-in-c
// TL;DR compliler should always see variables for refactoring, do {} while 0
// causes macro to create a new frame like a function call
#define DEBUG 1
#define debug_println(...)               \
    do                                   \
    {                                    \
        if (DEBUG)                       \
            Serial.println(__VA_ARGS__); \
    } while (0);
#define debug_print(...)               \
    do                                 \
    {                                  \
        if (DEBUG)                     \
            Serial.print(__VA_ARGS__); \
    } while (0);

/*******************************************************************************
 * Arduino_GFX setting
 ******************************************************************************/

#define TFT_CS 5    // GPIO5
#define TFT_RESET 3 // GPIO3
#define TFT_DC 4    // GPIO4
#define TFT_MOSI 10 // GPIO10/MOSI
#define TFT_SCK 8   // GPIO8/SCK
#define TFT_LED 2   // GPIO2
#define TFT_MISO -1 // not used for TFT

#define GFX_BL TFT_LED // backlight pin

/* More data bus class: https://github.com/moononournation/Arduino_GFX/wiki/Data-Bus-Class */
Arduino_DataBus *bus = new Arduino_HWSPI(TFT_DC, TFT_CS);

/* More display class: https://github.com/moononournation/Arduino_GFX/wiki/Display-Class */
Arduino_GFX *gfx = new Arduino_ILI9488_18bit(bus, TFT_RESET, 3 /* rotation */, false /* IPS */);

/*******************************************************************************
 * Serial CAN Bus
 ******************************************************************************/

#define CAN_SERIAL 0
#define CAN_VMCU_ID 0x7DF
#define CAN_VMCU_TACH 0x0C

#define CAN_VIN_ID 0x7DF

#define CAN_DATA_BUF_LEN 8
#define OBD_DATA_BUF_LEN 512

#define OBD_FRAME_BITMASK (0xF0)
#define OBD_LEN_BITMASK (0x0F)

#define CAN_FRAME_LEN 8

Serial_CAN can;

unsigned long __id = 0;                // can id
unsigned char __dta[CAN_DATA_BUF_LEN]; // data buffer (CAN)

unsigned char __obd_len = 0;               // message length (OBD)
unsigned char __obd_dta[OBD_DATA_BUF_LEN]; // data buffer (OBD)

int can_receive_frame_blocking(
    unsigned long *__id,
    unsigned char *__dta,
    unsigned long timeout_ms = -1)
{
    int res = 0;
    while (millis() < timeout_ms)
    {
        // busy wait until timeout or read_can returns successful read
        res = can.recv(__id, __dta);

        if (res)
        {
            // Arduino serial doesn't support sprintf :(
            debug_print(millis());
            debug_print(" 0x");
            debug_print(*__id, HEX);
            for (unsigned char i = 0; i < CAN_FRAME_LEN; i++)
            {
                // should be optimized out by compiler if #define DEBUG 0
                debug_print("0x");
                debug_print(__dta[i], HEX);
                debug_print('\t');
            }
            debug_println();

            return res;
        }
    }
    debug_print(millis());
    debug_println(" timeout waiting for frame");
    return 0;
}

int obd_receive_blocking(
    unsigned long id,
    unsigned char *len,
    unsigned char *buf,
    unsigned long timeout_ms = -1)
{
    // does blocking reads until
    // * Frames with target id (id + 0x08) are received (including consecutive);
    //      returning length stored to buf
    // * millis() >= timeout_ms; returning -1
    // * expected frame type mismatch; returning -2
    // * frame missed; returning -3

    int res = 0;
    unsigned short obd_len = 0;
    unsigned char frame_type = 0;

    *len = 0;

    // get first frame
    debug_print(millis());
    debug_println(" receiving first frame... ");
    res = can_receive_frame_blocking(
        &__id,
        __dta,
        timeout_ms);

    // if we get a timeout then return
    if (res == 0)
        return -1;

    // OBD-II https://en.wikipedia.org/wiki/ISO_15765-2

    // analyze first frame
    frame_type = (__dta[0] & OBD_FRAME_BITMASK) >> 4;

    if (frame_type == 0x00)
    { // Single
        obd_len = (__dta[0] & OBD_LEN_BITMASK);
        memcpy(buf, &__dta[1], obd_len);
    }
    else if (frame_type == 0x01)
    {
        // First
        unsigned short rem_len = 0;
        obd_len = ((__dta[0] & OBD_LEN_BITMASK) << 8) | __dta[1];
        rem_len = obd_len - 6;

        // load first frame contents into obd buffer and advance
        memcpy(buf, &__dta[2], CAN_FRAME_LEN - 2);

        // let the transmitter know that it is OK to send remaining frames
        // OBD flow frame 0x30 0x00 0x0A (0x0A ms delay between consecutive)
        debug_print("Sending flow control for remaining ");
        debug_print(rem_len);
        debug_println(" bytes...");
        unsigned char tmp[8] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        unsigned short frame_num = 1;
        can.send(0x7E0, 0, 0, 8, tmp);

        // begin receiving until a timeout or all frames received
        do
        {
            res = can_receive_frame_blocking(
                &__id,
                __dta,
                timeout_ms);

            if (res)
            {
                frame_type = (__dta[0] & OBD_FRAME_BITMASK) >> 4;
                unsigned char frame_counter = __dta[0] & OBD_LEN_BITMASK;

                // TODO frame_type should be 0x02, if it is 0x01, 0x03-0x04 then
                // there is a problem!
                if (frame_type != 2)
                    return -2;

                // track if frame counter is correct (if not, frames may be missed)
                // TODO send abort flow control if frame missed
                if (frame_num % 0x10 != frame_counter)
                    return -3;

                // frame size should also be 8 per OBD spec

                // copy bytes (frame 1 -> buf + 6, 1 -> buf + 13, ...)
                if (rem_len > 7)
                {
                    memcpy(buf + (6 + (7 * (frame_num - 1))), &__dta[1], CAN_FRAME_LEN - 1);
                    rem_len -= (CAN_FRAME_LEN - 1);
                }
                else
                {
                    memcpy(buf + (6 + (7 * (frame_num - 1))), &__dta[1], rem_len);
                    rem_len = 0;
                }

                // update frame number
                frame_num++;
            }
            else
            {
                // timeout
                return -1;
            }
        } while (rem_len > 0);
    }
    else
    { // Consecutive or Flow
        return -2;
    }

    *len = obd_len;
    return obd_len;
}

int obd_request_blocking(
    unsigned long id,
    unsigned char *tmp,     // assumes that the caller allocates 8 bytes
    unsigned char *obd_len, // len of data from OBD II
    unsigned char *obd_dta, // data from OBD II
    unsigned long timeout_ms_rel = 1000)
{
    int res = 0;
    unsigned long timeout;

    // id, extended/standard frame, remote/data frame, can fd/2.0, data len, buf
    can.send(id, 0, 0, 8, tmp);

    // blocking receive
    timeout = millis() + timeout_ms_rel;
    res = obd_receive_blocking(CAN_VIN_ID, obd_len, obd_dta, timeout);
    if (res > 0)
    {
        debug_println("Received: ");
        for (int i = 0; i < *obd_len; i++)
        {
            // compiler should optimize out this entire loop if #define DEBUG 0
            debug_print("0x");
            debug_print(__obd_dta[i], HEX);
            debug_print('\t');
        }
        debug_println();
    }
    else
    {
        debug_print("Error receiving, code ");
        debug_println(res);
    }

    return res;
}

int obd_get_vin_blocking(unsigned long timeout_ms_rel = 1000)
{
    debug_println("Asking for VIN...");
    unsigned char tmp[8] = {0x02, 0x09, 0x02, 0x55, 0x55, 0x55, 0x55, 0x55};
    // OBD request 0x7DF 0x02 0x09 0x02
    return obd_request_blocking(CAN_VIN_ID, tmp, &__obd_len, __obd_dta, timeout_ms_rel);
}

int obd_pid_request_blocking(unsigned long id, unsigned char pid, unsigned long timeout_ms_rel = 1000)
{
    debug_print("Requesting with ID: ");
    debug_print(id, HEX);
    debug_print(", PID: ");
    debug_println(pid, HEX);

    unsigned char tmp[8] = {0x02, 0x01, pid, 0x55, 0x55, 0x55, 0x55, 0x55};
    return obd_request_blocking(id, tmp, &__obd_len, __obd_dta, timeout_ms_rel);
}

void setup(void)
{
    // Debug setup
    if (DEBUG) {
        Serial.begin(115200);
        delay(100);
    }

    // CAN setup
    debug_print("Begin can...");
    can.begin(CAN_SERIAL, 9600);
    can.baudRate(SERIAL_RATE_115200);
    can.canRate(CAN_RATE_500);

    delay(100);
    obd_get_vin_blocking(200);        
}

void loop()
{
}