#include <Serial_CAN_FD.h>
#include <Arduino_GFX_Library.h>
#include <stdbool.h>
#include <stdint.h>
#include <DebugLog.h>
#include <3fx_fixed.hpp>
#include "obd.h"

/*******************************************************************************
 * Synchronization & FreeRTOS
 ******************************************************************************/

SemaphoreHandle_t obd_info_mutex;
volatile unsigned short obd_rpm;
volatile unsigned char obd_speed;

xTaskHandle obd_task_handle;
xTaskHandle displayTask_handle;

/*******************************************************************************
 * Serial CAN Bus
 ******************************************************************************/

HardwareSerial can_serial(0);
#define OBD_REFRESH_MS 2000

// required by can library
// mixing method name conventions :(
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

int obdGetVinBlocking(unsigned char *len, unsigned char *dta, unsigned long timeout_ms_rel = 1000)
{
    LOG_DEBUG("Asking for VIN...");
    unsigned char tmp[8] = {0x02, 0x09, 0x02, 0x55, 0x55, 0x55, 0x55, 0x55};
    // OBD request 0x7DF 0x02 0x09 0x02
    return obdRequestBlocking(CAN_VIN_ID, tmp, len, dta, timeout_ms_rel);
}

int obdPidRequestBlocking(unsigned long id, unsigned char pid, unsigned char *len, unsigned char *dta, unsigned long timeout_ms_rel = 1000)
{
    LOG_DEBUG("Requesting with ID:", id, "PID", pid);

    unsigned char tmp[8] = {0x02, 0x01, pid, 0x55, 0x55, 0x55, 0x55, 0x55};
    return obdRequestBlocking(id, tmp, len, dta, timeout_ms_rel);
}



/*******************************************************************************
 * Arduino_GFX
 ******************************************************************************/

#define TFT_CS 5    // GPIO5
#define TFT_RESET 3 // GPIO3
#define TFT_DC 4    // GPIO4
#define TFT_MOSI 10 // GPIO10/MOSI
#define TFT_SCK 8   // GPIO8/SCK
#define TFT_LED 2   // GPIO2
#define TFT_MISO -1 // not used for TFT

#define GFX_BL TFT_LED // backlight pin
#define DRAW_REFRESH_MS 1000

#define RPM_X 86
#define RPM_Y 40

void updateRPM(Arduino_GFX *gfx, int16_t x, int16_t y, int32_t rpm, int32_t rpm_old, int16_t color, int16_t background)
{
    int32_t thousands = rpm / 1000;
    int32_t hundreds = rpm % 1000;
    int32_t thousands_old = rpm_old / 1000;
    int32_t hundreds_old = rpm_old % 1000;

    char buf0[8];
    char buf1[8];
    memset(buf0, 0, 8);
    memset(buf1, 0, 8);

    //thousands
    snprintf(buf0, 8, "%02d", thousands);
    snprintf(buf1, 8, "%02d", thousands_old);

    // only need to draw the difference between the two old and new values
    for (int i = 0; buf0[i] != 0; i++)
    {
        if (buf0[i] == buf1[i]) 
        {
            buf0[i] = ' ';
            buf1[i] = ' ';
        }
    }
    // erase old 
    gfx->setCursor(x, y);
    gfx->setTextColor(background);
    gfx->setTextSize(15, 15);
    gfx->print(buf1);

    // draw new
    gfx->setCursor(x, y);
    gfx->setTextColor(color);
    gfx->setTextSize(15, 15);
    gfx->print(buf0);

    // hundreds
    snprintf(buf0, 8, "%03d", hundreds);
    snprintf(buf1, 8, "%03d", hundreds_old);

    // only need to draw the difference between the two old and new values
    for (int i = 0; buf0[i] != 0; i++)
    {
        if (buf0[i] == buf1[i]) 
        {
            buf0[i] = ' ';
            buf1[i] = ' ';
        }
    }
    // erase old 
    gfx->setCursor(x + 180, y);
    gfx->setTextColor(background);
    gfx->setTextSize(7, 7);
    gfx->print(buf1);

    // draw new
    gfx->setCursor(x + 180, y);
    gfx->setTextColor(color);
    gfx->setTextSize(7, 7);
    gfx->print(buf0);
}

void drawRPMHeader(Arduino_GFX *gfx, int16_t x, int16_t y, int16_t color)
{
    gfx->setCursor(x + 180, y + 56);
    gfx->setTextColor(color);
    gfx->setTextSize(7,7);
    gfx->print("RPM");
}

/*******************************************************************************
 * Tasks
 ******************************************************************************/

// debug task to feed draw task dummy data
void fakeObdTask(void *pv_parameters)
{
    // task runs every OBD_REFRESH_MS to grab updates from vehicle
    TickType_t x_last_wake_time;
    const TickType_t x_period = pdMS_TO_TICKS(OBD_REFRESH_MS);

    // init with current time for updates
    x_last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        // wait for next cycle
        vTaskDelayUntil(&x_last_wake_time, x_period);

        // WARNING not portable check endianness of system!!
        // equivalent to (obd_dta[3] * 256) + obd_data[4]
        xSemaphoreTake(obd_info_mutex, portMAX_DELAY);
        obd_rpm = (((unsigned char) random(255)) << 8) | (unsigned char) random(255);
        obd_speed = (unsigned char) random(255);
        xSemaphoreGive(obd_info_mutex);
    }
}

void obdTask(void *pv_parameters)
{
    unsigned char obd_len = 0;               // message length (OBD layer)
    unsigned char obd_dta[OBD_DATA_BUF_LEN]; // data buffer (OBD layer)
    int res;

    LOG_INFO("Begin can...");
    uart_init(9600);
    can_speed_fd(500000, 500000); // set can bus baudrate to 500k
    LOG_INFO("done");

    // task runs every OBD_REFRESH_MS to grab updates from vehicle
    TickType_t x_last_wake_time;
    const TickType_t x_period = pdMS_TO_TICKS(OBD_REFRESH_MS);

    // start by getting VIN
    obdGetVinBlocking(&obd_len, obd_dta, 200);

    // init with current time for updates
    x_last_wake_time = xTaskGetTickCount();

    for (;;)
    {
        // wait for next cycle
        vTaskDelayUntil(&x_last_wake_time, x_period);

        // get rpm
        res = obdPidRequestBlocking(CAN_VMCU_ID, CAN_RPM_PID, &obd_len, obd_dta, 200);
        if (res > 0 && obd_len == 4)
        {
            // WARNING not portable check endianness of system!!
            // equivalent to (obd_dta[3] * 256) + obd_data[4]
            xSemaphoreTake(obd_info_mutex, portMAX_DELAY);
            obd_rpm = (obd_dta[2] << 8) | obd_dta[3];
            xSemaphoreGive(obd_info_mutex);
        }
    }
}

void displayTask(void *pv_parameters)
{
    // set up tft display
    /* More data bus class: https://github.com/moononournation/Arduino_GFX/wiki/Data-Bus-Class */
    Arduino_DataBus *bus = new Arduino_HWSPI(TFT_DC, TFT_CS);
    Arduino_GFX *gfx = new Arduino_ILI9488_18bit(bus, TFT_RESET, 1 /* rotation */, false /* IPS */);
    ThreeFX_Fixed *tfx = new ThreeFX_Fixed(gfx);
    gfx->begin();

    // light up display and wipe screen
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
    gfx->fillScreen(BLACK);

    int32_t rpm = 0;
    int32_t oldRpm = 99999;
    int32_t speed;
    int32_t oldSpeed;

    // task runs every DRAW_REFRESH_MS to grab updates from vehicle
    TickType_t x_last_wake_time;
    const TickType_t x_period = pdMS_TO_TICKS(DRAW_REFRESH_MS);

    // init with current time for updates
    x_last_wake_time = xTaskGetTickCount();

    // draw first frame
    drawRPMHeader(gfx, RPM_X, RPM_Y, CYAN);     
    updateRPM(gfx, RPM_X, RPM_Y, rpm, oldRpm, CYAN, BLACK);   

    for (;;)
    {
        // wait for next cycle
        vTaskDelayUntil(&x_last_wake_time, x_period);

        // grab obd info 
        xSemaphoreTake(obd_info_mutex, portMAX_DELAY);
        rpm = obd_rpm;
        speed = obd_speed;
        xSemaphoreGive(obd_info_mutex);

        rpm /= 4;

        // update tach display
        if (oldRpm != rpm)
        {
            // drawRPM(gfx, 40, 40, oldRpm, BLACK);

            // // draw next frame
            // drawRPM(gfx, 40, 40, rpm, CYAN);
            updateRPM(gfx, RPM_X, RPM_Y, rpm, oldRpm, CYAN, BLACK);
        }
        oldRpm = rpm;
    }
}

/*******************************************************************************
 * Sketch
 ******************************************************************************/

void setup()
{
    Serial.begin(115200);
    LOG_ATTACH_SERIAL(Serial);
    vTaskDelay(5000);

    obd_info_mutex = xSemaphoreCreateMutex();

    LOG_INFO("Starting tasks...");
    xTaskCreatePinnedToCore(
        obdTask,
        "OBD Task",
        OBD_DATA_BUF_LEN + 4096,
        nullptr,
        5,
        &obd_task_handle,
        1);

    xTaskCreatePinnedToCore(
        displayTask,
        "Display Task",
        65536,
        nullptr,
        5,
        &displayTask_handle,
        1);
}

void loop()
{
    // nothing here
}

// END FILE