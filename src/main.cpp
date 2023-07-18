/*******************************************************************************
 * Modified from HelloWorld example of Arduino_GFX
 * run on Xiao ESP32C3 + 480x320 ILI9488 SPI TFT
 ******************************************************************************/
#include <Arduino_GFX_Library.h>
#include <HardwareSerial.h>
#include <Serial_CAN_Module.h>

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
Serial_CAN can;
unsigned long id = 0;
unsigned char dta[8];

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
    can.begin(CAN_SERIAL, 9600);
    if (can.baudRate(SERIAL_RATE_115200)) {
        Serial.println("Baud rate set!");
    } else {
        Serial.println("Baud rate not set!");
    }

    if (can.canRate(CAN_RATE_500)) {
        Serial.println("Can rate set!");
    } else {
        Serial.println("Can rate not set!");
    }
}

void loop()
{
    // gfx->setCursor(random(gfx->width()), random(gfx->height()));
    // gfx->setTextColor(random(0xffff), random(0xffff));
    // gfx->setTextSize(random(6) /* x scale */, random(6) /* y scale */, random(2) /* pixel_margin */);
    // gfx->println("Hello World!");

    // delay(1000); // 1 second

    if(can.recv(&id, dta))
    {
        Serial.print("GET DATA FROM ID: ");
        Serial.println(id);
        for(int i=0; i<8; i++)
        {
            Serial.print("0x");
            Serial.print(dta[i], HEX);
            Serial.print('\t');
        }
        Serial.println();
    }
}