#include "Serial_CAN_FD.h"
#include <Arduino_GFX_Library.h>
#include <stdbool.h>
#include <obd.h>
#include <debug.h>
#include <stdint.h>
#include <3d.h>

/*******************************************************************************
 * Synchronization & FreeRTOS
 ******************************************************************************/

SemaphoreHandle_t __obd_info_mutex;
volatile unsigned short __obd_rpm;
volatile unsigned char __obd_speed;

xTaskHandle __obd_task_handle;
xTaskHandle __displayTask_handle;

/*******************************************************************************
 * Serial CAN Bus
 ******************************************************************************/

HardwareSerial __can_serial(0);
#define OBD_REFRESH_MS 2000

// required by can library
// mixing method name conventions :(
void uart_init(unsigned long baudrate)
{
    __can_serial.begin(baudrate);
}

void uart_write(unsigned char c)
{
    __can_serial.write(c);
}

unsigned char uart_read()
{
    return __can_serial.read();
}

int uart_available()
{
    return __can_serial.available();
}


int obdGetVinBlocking(unsigned char *len, unsigned char *dta, unsigned long timeout_ms_rel = 1000)
{
    debug_println("Asking for VIN...");
    unsigned char tmp[8] = {0x02, 0x09, 0x02, 0x55, 0x55, 0x55, 0x55, 0x55};
    // OBD request 0x7DF 0x02 0x09 0x02
    return obdRequestBlocking(CAN_VIN_ID, tmp, len, dta, timeout_ms_rel);
}

int obdPidRequestBlocking(unsigned long id, unsigned char pid, unsigned char *len, unsigned char *dta, unsigned long timeout_ms_rel = 1000)
{
    debug_print("Requesting with ID: ");
    debug_print(id, HEX);
    debug_print(", PID: ");
    debug_println(pid, HEX);

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

// parameters for generating perspective grid
#define Z_PLANE_COLOR CYAN
#define NUM_Z_PLANE_VERT 17
#define NUM_Z_PLANE_HORIZ 9
#define XS_MIN (-14 * WORLD_SCALE_FACTOR)
#define XS_MAX (14 * WORLD_SCALE_FACTOR)
#define ZS_MIN (4 * WORLD_SCALE_FACTOR)
#define ZS_MAX (12 * WORLD_SCALE_FACTOR)
#define Y_HEIGHT (-4 * WORLD_SCALE_FACTOR)

// linearly scale speed and rpm
#define SPEED_SCALE 10
#define SPEED_B 10

#define RPM_SCALE 5000
#define RPM_B 10

line3d __z_grid_horiz[NUM_Z_PLANE_HORIZ];
line3d __z_grid_vert[NUM_Z_PLANE_VERT];

// init grid by linspace
void initGridWorldCoordinates()
{
    int32_t delta;
    // horizontals
    delta = (ZS_MAX - ZS_MIN) / (NUM_Z_PLANE_HORIZ - 1);
    for (int i = 0; i < NUM_Z_PLANE_HORIZ; i++)
    {
        __z_grid_horiz[i].p1.x = XS_MIN;
        __z_grid_horiz[i].p1.y = Y_HEIGHT;
        __z_grid_horiz[i].p1.z = (ZS_MIN + delta * i);

        __z_grid_horiz[i].p2.x = XS_MAX;
        __z_grid_horiz[i].p2.y = Y_HEIGHT;
        __z_grid_horiz[i].p2.z = (ZS_MIN + delta * i);
    }

    // vertical
    delta = (XS_MAX - XS_MIN) / (NUM_Z_PLANE_VERT - 1);
    for (int i = 0; i < NUM_Z_PLANE_VERT; i++)
    {
        __z_grid_vert[i].p1.x = (XS_MIN + delta * i);
        __z_grid_vert[i].p1.y = Y_HEIGHT;
        __z_grid_vert[i].p1.z = ZS_MIN;

        __z_grid_vert[i].p2.x = (XS_MIN + delta * i);
        __z_grid_vert[i].p2.y = Y_HEIGHT;
        __z_grid_vert[i].p2.z = ZS_MAX;
    }
}

// moves the grid by dx and dz and scrolls the grid lines if they underflow or 
// overflow
void updateGridWorldCoordinates(int32_t dx, int32_t dz)
{
    int32_t temp;
    for (int i = 0; i < NUM_Z_PLANE_HORIZ; i++)
    {
        temp = __z_grid_horiz[i].p1.z + dz;

        if (temp > ZS_MAX) {
            temp -= (ZS_MAX - ZS_MIN);
        } 
        else if (temp < ZS_MIN)
        {
            temp += (ZS_MAX - ZS_MIN);
        }

        __z_grid_horiz[i].p1.z = temp;
        __z_grid_horiz[i].p2.z = temp;
    }

    // vertical
    for (int i = 0; i < NUM_Z_PLANE_VERT; i++)
    {
        temp = __z_grid_vert[i].p1.x + dx;

        if (temp > XS_MAX) {
            temp -= (XS_MAX - XS_MIN);
        } 
        else if (temp < XS_MIN)
        {
            temp += (XS_MAX - XS_MIN);
        }

        __z_grid_vert[i].p1.x = temp;
        __z_grid_vert[i].p2.x = temp;
    }
}

void drawZPlaneVert(Arduino_GFX *gfx, int16_t color)
{
    for (int i = 0; i < NUM_Z_PLANE_VERT; i++) 
    {
        drawLine3d(gfx, __z_grid_vert[i], color);
    }
}

void drawZPlaneHoriz(Arduino_GFX *gfx, int16_t color)
{
    for (int i = 0; i < NUM_Z_PLANE_HORIZ; i++)
    {
        drawLine3d(gfx, __z_grid_horiz[i], color);
    }
}

void drawRPMHeader(Arduino_GFX *gfx, int16_t x, int16_t y, int16_t color)
{
    gfx->setCursor(x + 180, y + 56);
    gfx->setTextColor(color);
    gfx->setTextSize(7,7);
    gfx->print("RPM");
}

void drawRPM(Arduino_GFX *gfx, int16_t x, int16_t y, int32_t rpm, int16_t color)
{
    int32_t thousands = rpm / 1000;
    int32_t hundreds = rpm % 1000;

    // small buffer for sprintf
    char buf[8];
    memset(buf, 0, 8);

    // thousands
    snprintf(buf, 8, "%02d", thousands);
    gfx->setCursor(x, y);
    gfx->setTextColor(color);
    gfx->setTextSize(15, 15);
    gfx->print(buf);

    // hundreds
    snprintf(buf, 8, "%03d", hundreds);
    gfx->setCursor(x + 180, y);
    gfx->setTextSize(7, 7);
    gfx->print(buf);
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
        xSemaphoreTake(__obd_info_mutex, portMAX_DELAY);
        __obd_rpm = (((unsigned char) random(255)) << 8) | (unsigned char) random(255);
        __obd_speed = (unsigned char) random(255);
        xSemaphoreGive(__obd_info_mutex);
    }
}

void obdTask(void *pv_parameters)
{
    unsigned char obd_len = 0;               // message length (OBD layer)
    unsigned char obd_dta[OBD_DATA_BUF_LEN]; // data buffer (OBD layer)
    int res;

    debug_print("Begin can...");
    uart_init(9600);
    can_speed_fd(500000, 500000); // set can bus baudrate to 500k
    debug_println("done");

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
            xSemaphoreTake(__obd_info_mutex, portMAX_DELAY);
            __obd_rpm = (obd_dta[3] << 8) | obd_dta[4];
            xSemaphoreGive(__obd_info_mutex);
        }

        // // get speed
        // res = obdPidRequestBlocking(CAN_VMCU_ID, CAN_SPEED_PID, &obd_len, obd_dta, 200);
        // if (res > 0 && obd_len == 3)
        // {
        //     xSemaphoreTake(__obd_info_mutex, portMAX_DELAY);
        //     __obd_speed = obd_dta[3];
        //     xSemaphoreGive(__obd_info_mutex);
        // }
    }
}

void displayTask(void *pv_parameters)
{
    // set up tft display
    /* More data bus class: https://github.com/moononournation/Arduino_GFX/wiki/Data-Bus-Class */
    Arduino_DataBus *bus = new Arduino_HWSPI(TFT_DC, TFT_CS);
    Arduino_GFX *gfx = new Arduino_ILI9488_18bit(bus, TFT_RESET, 1 /* rotation */, false /* IPS */);
    gfx->begin();

    // light up display and wipe screen
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
    gfx->fillScreen(BLACK);

    int32_t rpm;
    int32_t oldRpm;
    int32_t speed;
    int32_t oldSpeed;

    // task runs every DRAW_REFRESH_MS to grab updates from vehicle
    TickType_t x_last_wake_time;
    const TickType_t x_period = pdMS_TO_TICKS(DRAW_REFRESH_MS);

    // init with current time for updates
    x_last_wake_time = xTaskGetTickCount();

    // ready zPlaneLines background
    initGridWorldCoordinates();

    // draw first frame
    drawRPMHeader(gfx, 40, 40, CYAN);     
    drawRPM(gfx, 40, 40, rpm, CYAN);   
    drawZPlaneVert(gfx, Z_PLANE_COLOR);
    drawZPlaneHoriz(gfx, Z_PLANE_COLOR);

    for (;;)
    {
        // wait for next cycle
        vTaskDelayUntil(&x_last_wake_time, x_period);

        // grab obd info 
        xSemaphoreTake(__obd_info_mutex, portMAX_DELAY);
        rpm = __obd_rpm;
        speed = __obd_speed;
        xSemaphoreGive(__obd_info_mutex);

        rpm /= 4;

        // update tach display
        if (oldRpm != rpm)
        {
            drawRPM(gfx, 40, 40, oldRpm, BLACK);

            // draw next frame
            drawRPM(gfx, 40, 40, rpm, CYAN);
        }
        oldRpm = rpm;

        //update grid display (causes flickering)
        // drawZPlaneVert(gfx, BLACK);
        // drawZPlaneHoriz(gfx, BLACK);
        // updateGridWorldCoordinates(
        //     (rpm / RPM_SCALE) + RPM_B, 
        //     (speed / SPEED_SCALE) + SPEED_B);
        // drawZPlaneVert(gfx, Z_PLANE_COLOR);
        // drawZPlaneHoriz(gfx, Z_PLANE_COLOR);
    }
}

/*******************************************************************************
 * Sketch
 ******************************************************************************/

void setup()
{
    if (DEBUG)
    {
        Serial.begin(115200);
        vTaskDelay(2500);
    }

    __obd_info_mutex = xSemaphoreCreateMutex();

    debug_println("Starting tasks...");
    xTaskCreatePinnedToCore(
        obdTask,
        "OBD Task",
        OBD_DATA_BUF_LEN + 4096,
        nullptr,
        5,
        &__obd_task_handle,
        1);

    xTaskCreatePinnedToCore(
        displayTask,
        "Display Task",
        65536,
        nullptr,
        5,
        &__displayTask_handle,
        1);
}

void loop()
{
    // nothing here
}

// END FILE