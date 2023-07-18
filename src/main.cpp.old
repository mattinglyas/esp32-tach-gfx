#include <Arduino.h>
#include <HardwareSerial.h>
#include <Serial_CAN_Module.h>

Serial_CAN can;

void setup(void)
{
    Serial.begin(9600);
    can.begin(0, 9600);
}

unsigned long id = 0;
unsigned char dta[8];

void loop()
{
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