/*******************************************************************************
 * Modified from HelloWorld example of Arduino_GFX
 * run on Xiao ESP32C3 + 480x320 ILI9488 SPI TFT
 ******************************************************************************/
#include <Arduino_GFX_Library.h>
#include <HardwareSerial.h>
#include <stdio.h>
#include <Serial_CAN_FD.h>

/*******************************************************************************
 * Arduino_GFX setting
 ******************************************************************************/

#define TFT_CS    5   //GPIO5
#define TFT_RESET 3   //GPIO3
#define TFT_DC    4   //GPIO4
#define TFT_MOSI  10  //GPIO10/MOSI
#define TFT_SCK   8   //GPIO8/SCK
#define TFT_LED   2   //GPIO2
#define TFT_MISO  -1  // not used for TFT

#define GFX_BL TFT_LED // backlight pin

/* More data bus class: https://github.com/moononournation/Arduino_GFX/wiki/Data-Bus-Class */
Arduino_DataBus *bus = new Arduino_HWSPI(TFT_DC, TFT_CS);

/* More display class: https://github.com/moononournation/Arduino_GFX/wiki/Display-Class */
Arduino_GFX *gfx = new Arduino_ILI9488_18bit(bus, TFT_RESET, 3 /* rotation */, false /* IPS */);


/*******************************************************************************
 * End of Arduino_GFX setting
 ******************************************************************************/

/*******************************************************************************
 * Serial CAN Bus settting
 ******************************************************************************/

#define CAN_SERIAL 0
#define CAN_VMCU_ID 0x7E3
#define CAN_VMCU_TACH 0x02

HardwareSerial can_serial(CAN_SERIAL);
unsigned long __id = 0;
unsigned char __ext = 0;
unsigned char __rtr = 0;
unsigned char __fdf = 0;
unsigned char __len = 0;
unsigned char __dta_sz = 64;
unsigned char __dta[64];

void sendPid(unsigned char __pid) {
    unsigned char tmp[8] = {0x02, 0x01, __pid, 0, 0, 0, 0, 0};
    can_send(CAN_VMCU_ID, 0, 0, 0, 8, tmp);
}

void uart_init(unsigned long baudrate)
{
    can_serial.begin(baudrate);
}

void uart_write(unsigned char c)
{
    can_serial.write(c);
}

unsigned char uart_read()
{
    return can_serial.read();
}

int uart_available()
{
    return can_serial.available();
}


/*******************************************************************************
 * End Serial CAN Bus settting
 ******************************************************************************/

void setup(void)
{
    // Debug setup
    int timeout = 0;
    Serial.begin(115200);
    while(!Serial && timeout < 15) {
        timeout++;
        delay(1000);
    }

    // GFX setup
    gfx->begin();
    gfx->fillScreen(BLACK);

#ifdef GFX_BL
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
#endif

    gfx->setCursor(10, 10);
    gfx->setTextColor(RED);
    gfx->println("Hello World!");

    delay(5000); // 5 seconds

    // CAN setup
    uart_init(115200);
    can_speed_fd(500000, 500000); 
}

void loop()
{
    // gfx->setCursor(random(gfx->width()), random(gfx->height()));
    // gfx->setTextColor(random(0xffff), random(0xffff));
    // gfx->setTextSize(random(6) /* x scale */, random(6) /* y scale */, random(2) /* pixel_margin */);
    // gfx->println("Hello World!");

    // delay(1000); // 1 second

    Serial.println("New request...");
    sendPid(CAN_VMCU_TACH);
    unsigned long __timeout = millis();
    while (millis() - __timeout < 1000) // 1s timeout
    {
        __id = 0;
        for (int i = 0; i < __dta_sz; i++) {
            __dta[i] = 0;
        }

        if (read_can(&__id, &__ext, &__rtr, &__fdf, &__len, __dta)) {                // check if get data
            for (int i = 0; i < __len; i++)
            {
                Serial.print(millis());
                Serial.print(" ");
                Serial.print("0x");
                Serial.print(__dta[i], HEX);
                Serial.print('\t');
            }
            Serial.println();
            break;
        }
    }

    delay(1000);
}