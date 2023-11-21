#include "Serial_CAN_FD.h"
#include <Arduino_GFX_Library.h>
#include <stdbool.h>
#include <can.h>
#include <debug.h>

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

// storage for clipping line segments (see drawLineOnScreen)
#define CL_INSIDE (0)
#define CL_LEFT (1 << 0)
#define CL_RIGHT (1 << 1)
#define CL_BOTTOM (1 << 2)
#define CL_TOP (1 << 3)

#define DRAW_REFRESH_MS 20

typedef struct {
    int32_t x;
    int32_t y;
    int32_t z;
} point3d;

typedef struct {
    int32_t x;
    int32_t y;
} point2d;

typedef struct {
    point3d p1;
    point3d p2;
} line3d;

typedef struct {
    double x;
    double y;
    double z;
} screenPoint3d;

/* More data bus class: https://github.com/moononournation/Arduino_GFX/wiki/Data-Bus-Class */
Arduino_DataBus *__bus;
Arduino_GFX *__gfx;

// camera parameters
#define ZNEAR (3.)
#define ZFAR (50.)
#define FOV (1 / tan(45)) // 1 / tan(THETA / 2)
#define LAMBDA (ZFAR / ZFAR - ZNEAR)
#define WORLD_SCALE_FACTOR 1000

// parameters for generating perspective grid
#define Z_PLANE_COLOR CYAN
#define NUM_Z_PLANE_VERT 17
#define NUM_Z_PLANE_HORIZ 9
#define XS_MIN (-14 * WORLD_SCALE_FACTOR)
#define XS_MAX (14 * WORLD_SCALE_FACTOR)
#define ZS_MIN (4 * WORLD_SCALE_FACTOR)
#define ZS_MAX (12 * WORLD_SCALE_FACTOR)
#define Y_HEIGHT (-4 * WORLD_SCALE_FACTOR)
// TODO check if compiler optimizes this out
#define GFX_H (__gfx->height())
#define GFX_W (__gfx->width())
#define GFX_A ((double) GFX_H / (double) GFX_W) 

line3d __z_grid_horiz[NUM_Z_PLANE_HORIZ];
line3d __z_grid_vert[NUM_Z_PLANE_VERT];

void debug_printPoint3d(point3d wp)
{
    debug_print("(");
    debug_print(wp.x);
    debug_print(", ");
    debug_print(wp.y);
    debug_print(", ");    
    debug_print(wp.z);
    debug_print(")");
}

void debug_printPoint2d(point2d p)
{
    debug_print("(");
    debug_print(p.x);
    debug_print(", ");
    debug_print(p.y);
    debug_print(")");
}

// converts a world point to a normalized screen point with ordered depth
// there is some loss here due to floating point math, it is best to not 
// convert back and forth many times
inline screenPoint3d worldToScreen(point3d wp)
{
    screenPoint3d res;
    // double casts for world coords
    double dx = (double) wp.x / (double) WORLD_SCALE_FACTOR;
    double dy = (double) wp.y / (double) WORLD_SCALE_FACTOR;
    double dz = (double) wp.z / (double) WORLD_SCALE_FACTOR;

    res.x = ((GFX_A * FOV * dx) / dz);
    res.y = ((FOV * dy) / dz);
    // kept as value so that points can be ordered by depth later
    res.z = ((dz - ZNEAR) * LAMBDA / dz);

    return res;
}

// converts a screen point (normalized to [-1, 1] that are edges of screen)
// to actual pixel values
inline point2d screenToPixel(screenPoint3d sp)
{
    point2d res;
    res.x = (int32_t) ((sp.x + 1./2.) * GFX_W);
    res.y = (int32_t) ((1./2. - sp.y) * GFX_H);

    return res;
}

// converts a world y and z coordinate into an on-screen y coordinate
inline int32_t yHeightFromZ(double y, double z) {
    double height = (double) __gfx->height();
    double proj = FOV * y;
    double persp = proj / z;
    double pxels = persp * height + height / 2;
    return height - pxels;
}

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
void updateGridWorldCoordinates(int16_t dx, int16_t dz)
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

inline bool inRangeInc(int32_t val, int32_t lower, int32_t upper)
{
    return (val >= lower && upper >= val);
}

// helper for cohen-sutherland clipping
inline unsigned char cohenSutherlandCode(int32_t x, int32_t y, int32_t width, int32_t height)
{
    unsigned char code = CL_INSIDE;

    if (x < 0)
        code |= CL_LEFT;
    else if (x > width)
        code |= CL_RIGHT;

    if (y < 0)
        code |= CL_TOP;
    else if (y > height)
        code |= CL_BOTTOM;

    return code;
}

// clips line so coordinates fall within (0, width), (0, height)
// returns true if line segment can be drawn or false if no part of the segment
// is on screen
// https://www.geeksforgeeks.org/line-clipping-set-1-cohen-sutherland-algorithm/
bool cohenSutherlandClip(int32_t *x0, int32_t *y0, int32_t *x1, int32_t *y1, int32_t width, int32_t height)
{
    // calculate codes for both coordinates
    unsigned char code0 = cohenSutherlandCode(*x0, *y0, width, height);
    unsigned char code1 = cohenSutherlandCode(*x1, *y1, width, height);

    // track if line is entirely outside of window
    bool accept = false;

    // iterate through codes and adjust coordinates as needed
    // loop runs at most four times
    while (true)
    {
        unsigned char code_out;
        int32_t x, y;

        if ((code0 == 0) && (code1 == 0)) break;
        if (code0 & code1) break;

        if (code0)
            code_out = code0;
        else
            code_out = code1;

        // find intersection point
        // TODO is there a better way to get coords than integer division?
        if (code_out & CL_TOP)
        {
            x = *x0 + (*x1 - *x0) * (0 - *y0) / (*y1 - *y0);
            y = 0;
        }
        else if (code_out & CL_BOTTOM)
        {
            x = *x0 + (*x1 - *x0) * (height - *y0) / (*y1 - *y0);
            y = height;
        }
        else if (code_out & CL_RIGHT)
        {
            x = width;
            y = *y0 + (*y1 - *y0) * (width - *x0) / (*x1 - *x0);
        }
        else if (code_out & CL_LEFT)
        {
            x = 0;
            y = *y0 + (*y1 - *y0) * (0 - *x0) / (*x1 - *x0);
        }
        else
        {
            // something must have gone wrong to get here...
        }

        if (code_out == code0)
        {
            *x0 = x;
            *y0 = y;
            code0 = cohenSutherlandCode(*x0, *y0, width, height);
        }
        else
        {
            *x1 = x;
            *y1 = y;
            code1 = cohenSutherlandCode(*x1, *y1, width, height);
        }
    }

    // line can now be drawn on screen if both segment codes are 0 (centered)
    // otherwise, code0 and code1 match, indiciating that both endpoints are
    // offscreen
    return ((code0 | code1) == CL_INSIDE);
}

// overload that defaults to screen height and width
bool cohenSutherlandClip(int32_t *x0, int32_t *y0, int32_t *x1, int32_t *y1)
{
    return cohenSutherlandClip(x0, y0, x1, y1, GFX_W, GFX_H);
}

// draws a world coordinate line
void drawLine3d(line3d wl, uint16_t color)
{
    point2d pp1 = screenToPixel(worldToScreen(wl.p1));
    point2d pp2 = screenToPixel(worldToScreen(wl.p2));

    if (cohenSutherlandClip(&pp1.x, &pp1.y, &pp2.x, &pp2.y))
    {
        __gfx->drawLine(pp1.x, pp1.y, pp2.x, pp2.y, color);
    }
}

void drawZPlaneVert(int16_t color)
{
    for (int i = 0; i < NUM_Z_PLANE_VERT; i++) 
    {
        drawLine3d(__z_grid_vert[i], color);
    }
}

void drawZPlaneHoriz(int16_t color)
{
    for (int i = 0; i < NUM_Z_PLANE_HORIZ; i++)
    {
        drawLine3d(__z_grid_horiz[i], color);
    }
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

        res = obdPidRequestBlocking(CAN_VMCU_ID, CAN_RPM_PID, &obd_len, obd_dta, 200);
        if (res > 0 && obd_len == 4)
        {
            // WARNING not portable check endianness of system!!
            // equivalent to (obd_dta[3] * 256) + obd_data[4]
            xSemaphoreTake(__obd_info_mutex, portMAX_DELAY);
            __obd_rpm = (obd_dta[3] << 8) | obd_dta[4];
            xSemaphoreGive(__obd_info_mutex);
        }
    }
}

void displayTask(void *pv_parameters)
{
    // set up tft display
    __gfx->begin();
    pinMode(GFX_BL, OUTPUT);
    digitalWrite(GFX_BL, HIGH);
    __gfx->fillScreen(BLACK);

    // task runs every DRAW_REFRESH_MS to grab updates from vehicle
    TickType_t x_last_wake_time;
    const TickType_t x_period = pdMS_TO_TICKS(DRAW_REFRESH_MS);

    // init with current time for updates
    x_last_wake_time = xTaskGetTickCount();

    // ready zPlaneLines background
    initGridWorldCoordinates();

    // draw first frame
    drawZPlaneVert(Z_PLANE_COLOR);
    debug_println();
    drawZPlaneHoriz(Z_PLANE_COLOR);
    __gfx->drawFastHLine(0, yHeightFromZ(Y_HEIGHT / WORLD_SCALE_FACTOR, ZS_MAX / WORLD_SCALE_FACTOR), __gfx->width(), Z_PLANE_COLOR);

    // storage for previous frame to speed up erasing
    line3d last_frame_vert[NUM_Z_PLANE_VERT];
    line3d last_frame_horiz[NUM_Z_PLANE_HORIZ];

    for (;;)
    {
        // wait for next cycle
        vTaskDelayUntil(&x_last_wake_time, x_period);

        // erase current frame
        drawZPlaneVert(BLACK);
        drawZPlaneHoriz(BLACK);

        // advance
        updateGridWorldCoordinates(-90, -45);

        // draw next frame
        drawZPlaneVert(Z_PLANE_COLOR);
        drawZPlaneHoriz(Z_PLANE_COLOR);
        __gfx->drawFastHLine(0, yHeightFromZ(Y_HEIGHT / WORLD_SCALE_FACTOR, ZS_MAX / WORLD_SCALE_FACTOR), __gfx->width(), Z_PLANE_COLOR);
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

    __bus = new Arduino_HWSPI(TFT_DC, TFT_CS);
    __gfx = new Arduino_ILI9488_18bit(__bus, TFT_RESET, 1 /* rotation */, false /* IPS */);

    __obd_info_mutex = xSemaphoreCreateMutex();

    debug_println("Starting tasks...");
    xTaskCreatePinnedToCore(
        fakeObdTask,
        "OBD Task",
        OBD_DATA_BUF_LEN + 4096,
        nullptr,
        10,
        &__obd_task_handle,
        1);

    xTaskCreatePinnedToCore(
        displayTask,
        "Display Task",
        65536,
        nullptr,
        10,
        &__displayTask_handle,
        1);
}

void loop()
{
    // nothing here
}

// END FILE