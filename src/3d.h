#ifndef _3d_h
#define _3d_h
#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>
#include <Arduino_GFX_Library.h>

// storage for clipping line segments (see drawLineOnScreen)
#define CL_INSIDE (0)
#define CL_LEFT (1 << 0)
#define CL_RIGHT (1 << 1)
#define CL_BOTTOM (1 << 2)
#define CL_TOP (1 << 3)

// camera parameters
#define ZNEAR (3.)
#define ZFAR (50.)
#define FOV (1 / tan(45)) // 1 / tan(THETA / 2)
#define LAMBDA (ZFAR / ZFAR - ZNEAR)
#define WORLD_SCALE_FACTOR 1000

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

void debug_printPoint3d(point3d wp);
void debug_printPoint2d(point2d p);

inline screenPoint3d worldToScreen(Arduino_GFX *, point3d);
inline point2d screenToPixel(Arduino_GFX *, screenPoint3d);
inline point2d worldToPixel(Arduino_GFX *, point3d);

void drawPoint3d(Arduino_GFX *, line3d, uint16_t);
void drawLine3d(Arduino_GFX *, line3d, uint16_t);

bool cohenSutherlandClip(int32_t *, int32_t *, int32_t *, int32_t *, int32_t, int32_t);
bool cohenSutherlandClip(Arduino_GFX *, int32_t *, int32_t *, int32_t *, int32_t *);

#endif