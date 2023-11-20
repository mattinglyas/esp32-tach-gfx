#include "Serial_CAN_FD.h"
#include <SoftwareSerial.h>
#include <Arduino_GFX_Library.h>
#include <stdbool.h>

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

#define CAN_VMCU_ID 0x7DF
#define CAN_RPM_PID 0x0C

#define CAN_VIN_ID 0x7DF

#define can_tx 2 // tx of serial can module connect to D2
#define can_rx 3 // rx of serial can module connect to D3

#define CAN_DATA_BUF_LEN 64
#define OBD_DATA_BUF_LEN 512

#define OBD_FRAME_BITMASK (0xF0)
#define OBD_LEN_BITMASK (0x0F)

#define OBD_REFRESH_MS 1000

HardwareSerial __can_serial(0);

#define uart_can __can_serial

unsigned long __id = 0;                // can id
unsigned char __ext = 0;               // extended frame or standard frame
unsigned char __rtr = 0;               // remote frame or data frame
unsigned char __fdf = 0;               // can fd or can 2.0
unsigned char __len = 0;               // data length
unsigned char __dta[CAN_DATA_BUF_LEN]; // data buffer (CAN layer)

// required by can library
// mixing method name conventions :(
void uart_init(unsigned long baudrate)
{
    uart_can.begin(baudrate);
}

void uart_write(unsigned char c)
{
    uart_can.write(c);
}

unsigned char uart_read()
{
    return uart_can.read();
}

int uart_available()
{
    return uart_can.available();
}

int canReceiveFrameBlocking(
    unsigned long *id,
    unsigned char *ext,
    unsigned char *rtr,
    unsigned char *fdf,
    unsigned char *len,
    unsigned char *dta,
    unsigned long timeout_ms = -1)
{
    int res = 0;
    while (millis() < timeout_ms)
    {
        // busy wait until timeout or read_can returns successful read
        res = read_can(id, ext, rtr, fdf, len, dta);

        if (res)
        {
            // Arduino serial doesn't support sprintf :(
            debug_print(millis());
            debug_print(" 0x");
            debug_print(*id, HEX);
            debug_print(" ");
            debug_print(*ext);
            debug_print(" ");
            debug_print(*rtr);
            debug_print(" ");
            debug_print(*fdf);
            debug_print(" ");
            debug_print(*len);
            debug_print(" ");
            for (unsigned char i = 0; i < *len; i++)
            {
                // should be optimized out by compiler if #define DEBUG 0
                debug_print("0x");
                debug_print(dta[i], HEX);
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

int obdReceiveBlocking(
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
    res = canReceiveFrameBlocking(
        &__id,
        &__ext,
        &__rtr,
        &__fdf,
        &__len,
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
        // WARNING not portable check endianness of system!!
        obd_len = ((__dta[0] & OBD_LEN_BITMASK) << 8) | __dta[1];
        rem_len = obd_len - 6;

        // load first frame contents into obd buffer and advance
        memcpy(buf, &__dta[2], __len - 2);

        // let the transmitter know that it is OK to send remaining frames
        // OBD flow frame 0x30 0x00 0x0A (0x0A ms delay between consecutive)
        debug_print("Sending flow control for remaining ");
        debug_print(rem_len);
        debug_println(" bytes...");
        unsigned char tmp[8] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        unsigned short frame_num = 1;
        can_send(0x7E0, 0, 0, 0, 8, tmp);

        // begin receiving until a timeout or all frames received
        do
        {
            res = canReceiveFrameBlocking(
                &__id,
                &__ext,
                &__rtr,
                &__fdf,
                &__len,
                __dta,
                timeout_ms);

            if (res)
            {
                // WARNING not portable check endianness of system!!
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
                    memcpy(buf + (6 + (7 * (frame_num - 1))), &__dta[1], __len - 1);
                    rem_len -= (__len - 1);
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

int obdRequestBlocking(
    unsigned long id,
    unsigned char *tmp,     // assumes that the caller allocates 8 bytes
    unsigned char *obd_len, // len of data from OBD II
    unsigned char *obd_dta, // data from OBD II
    unsigned long timeout_ms_rel = 1000)
{
    int res = 0;
    unsigned long timeout;

    // id, extended/standard frame, remote/data frame, can fd/2.0, data len, buf
    can_send(id, 0, 0, 0, 8, tmp);

    // blocking receive
    timeout = millis() + timeout_ms_rel;
    res = obdReceiveBlocking(CAN_VIN_ID, obd_len, obd_dta, timeout);
    if (res > 0)
    {
        debug_println("Received: ");
        for (int i = 0; i < *obd_len; i++)
        {
            // compiler should optimize out this entire loop if #define DEBUG 0
            debug_print("0x");
            debug_print(obd_dta[i], HEX);
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

/* More data bus class: https://github.com/moononournation/Arduino_GFX/wiki/Data-Bus-Class */
Arduino_DataBus *__bus;
Arduino_GFX *__gfx;

/* coordinates for 'z' plane lines derived from MATLAB script in matrix_test in x, y pixels */
#define NUM_Z_PLANE_VERT 13
#define NUM_Z_PLANE_HORIZ 9
#define Z_PLANE_COLOR CYAN
// original parameters for z plane derived from MATLAB script
#define Y_HEIGHT -4.
// camera parameters
#define ZNEAR 3
#define ZFAR 50
#define FOV (1 / tan(45)) // 1 / tan(THETA / 2)

int32_t __z_grid_x0[NUM_Z_PLANE_VERT] = {-353, -254, -155, -56, 42, 141, 240, 339, 438, 536, 635, 734, 833};
int32_t __z_grid_y0[NUM_Z_PLANE_VERT] = {358, 358, 358, 358, 358, 358, 358, 358, 358, 358, 358, 358, 358};
int32_t __z_grid_x1[NUM_Z_PLANE_VERT] = {42, 75, 108, 141, 174, 207, 240, 273, 306, 339, 372, 405, 438};
int32_t __z_grid_y1[NUM_Z_PLANE_VERT] = {226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226};

// using fixed point math to store the z coordinate of the horizontal lines
#define ZS_SCALE_FACTOR 1000
#define ZS_MIN (4 * ZS_SCALE_FACTOR)
#define ZS_MAX (12 * ZS_SCALE_FACTOR)
int32_t __z_grid_zs[NUM_Z_PLANE_HORIZ];

// converts a world y and z coordinate into an on-screen y coordinate
inline int32_t y_height_from_z(double y, double z) {
    double height = (double) __gfx->height();
    double proj = FOV * y;
    double persp = proj / z;
    double pxels = persp * height + height / 2;
    return height - pxels;
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
    while ((code0 != code1) && (code0 | code1))
    {
        unsigned char code_out;
        int32_t x, y;

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

// convert coordinates to ready-to-draw coordinates with cohenSutherland
void initZPlaneLinesVert()
{
    for (int i = 0; i < NUM_Z_PLANE_VERT; i++)
    {
        cohenSutherlandClip(
            &__z_grid_x0[i], 
            &__z_grid_y0[i], 
            &__z_grid_x1[i], 
            &__z_grid_y1[i], 
            __gfx->width(), 
            __gfx->height()
            );
    }
}

// ready storage for horizontal lines world coordinates by linspace
void initZPlaneLinesHoriz() 
{
    int32_t delta = (ZS_MAX - ZS_MIN) / (NUM_Z_PLANE_HORIZ - 1);
    for (int i = 0; i < NUM_Z_PLANE_HORIZ; i++) 
    {
        __z_grid_zs[i] = ZS_MIN + delta * i;
    }
}

void drawZPlaneVert(int16_t color)
{
    for (int i = 0; i < NUM_Z_PLANE_VERT; i++) 
    {
        __gfx->drawLine(__z_grid_x0[i], __z_grid_y0[i], __z_grid_x1[i], __z_grid_y1[i], color);
    }
}

void drawZPlaneHoriz(int32_t *zs, int16_t color)
{
    double z;
    int32_t y;

    for (int i = 0; i < NUM_Z_PLANE_HORIZ; i++)
    {
        // convert world coordinates into pixel coordinates
        z = ((double) zs[i]) / ZS_SCALE_FACTOR;
        y = y_height_from_z(Y_HEIGHT, z);

        __gfx->drawFastHLine(0, y, __gfx->width(), color);
    }
}

void updateZPlaneHoriz(int32_t *old_zs, int32_t *new_zs, int16_t color)
{
    double z0, z1;
    int32_t y0, y1;

    for (int i = 0; i < NUM_Z_PLANE_HORIZ; i++)
    {
        // convert world coordinates into pixel coordinates
        z0 = ((double) old_zs[i]) / ZS_SCALE_FACTOR;
        y0 = y_height_from_z(Y_HEIGHT, z0);

        z1 = ((double) new_zs[i]) / ZS_SCALE_FACTOR;
        y1 = y_height_from_z(Y_HEIGHT, z1);

        if (y0 != y1)
        {
            __gfx->drawFastHLine(0, y0, __gfx->width(), BLACK);
            __gfx->drawFastHLine(0, y1, __gfx->width(), color);
        }
    }
}

// advance all horizontal lines by some amount
// assumes amount is smaller than (ZS_MAX - ZS_MIN)
void advanceZPlaneHoriz(int32_t amount)
{
    int32_t rel;
    for (int i = 0; i < NUM_Z_PLANE_HORIZ; i++)
    {
        rel = __z_grid_zs[i] + amount;
        if (rel < ZS_MIN) rel += (ZS_MAX - ZS_MIN);
        else if (rel > ZS_MAX) rel -= (ZS_MAX - ZS_MIN);
        __z_grid_zs[i] = rel;
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
    delay(100);
    can_speed_fd(500000, 500000); // set can bus baudrate to 500k
    delay(100);
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
    initZPlaneLinesVert();
    initZPlaneLinesHoriz();

    // draw first frame
    drawZPlaneVert(Z_PLANE_COLOR);
    drawZPlaneHoriz(__z_grid_zs, Z_PLANE_COLOR);

    int32_t old_zs[NUM_Z_PLANE_HORIZ];
    uint16_t old_rpm;
    int16_t speed;
    uint16_t rpm;

    for (;;)
    {
        // wait for next cycle
        vTaskDelayUntil(&x_last_wake_time, x_period);

        // get speed and RPM from obd info
        xSemaphoreTake(__obd_info_mutex, portMAX_DELAY);
        speed = __obd_speed;
        rpm = __obd_rpm;
        xSemaphoreGive(__obd_info_mutex);

        // update RPM display
        rpm /= 4;
        if (old_rpm != rpm)
        {
            __gfx->setTextSize(5);
            __gfx->setCursor(30, 30);
            __gfx->setTextColor(BLACK);
            __gfx->print(old_rpm);
            __gfx->setTextColor(Z_PLANE_COLOR);
            __gfx->setCursor(30, 30);
            __gfx->print(rpm);
        }
        old_rpm = rpm;

        // save previous z grid frame for erasing
        memcpy(old_zs, __z_grid_zs, sizeof(int32_t) * NUM_Z_PLANE_HORIZ);

        // advance Z plane
        advanceZPlaneHoriz(-speed);
        updateZPlaneHoriz(old_zs, __z_grid_zs, Z_PLANE_COLOR);

        // draw verticals in case pixels were erased in advancing horizontals
        drawZPlaneVert(Z_PLANE_COLOR);

        // draw top line in case it was erased
        __gfx->drawFastHLine(0, y_height_from_z(Y_HEIGHT, ZS_MAX / ZS_SCALE_FACTOR), __gfx->width(), Z_PLANE_COLOR);
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
    __gfx = new Arduino_ILI9488_18bit(__bus, TFT_RESET, 3 /* rotation */, false /* IPS */);

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