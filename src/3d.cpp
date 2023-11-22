#include <debug.h>
#include <3d.h>
#include <Arduino_GFX_Library.h>

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

inline bool inRangeInc(int32_t val, int32_t lower, int32_t upper)
{
    return (val >= lower && upper >= val);
}

// converts a world point to a normalized screen point with ordered depth
// there is some loss here due to floating point math, it is best to not 
// convert back and forth many times
inline screenPoint3d worldToScreen(Arduino_GFX *gfx, point3d wp)
{
    screenPoint3d res;
    // double casts for world coords
    double dx = (double) wp.x / (double) WORLD_SCALE_FACTOR;
    double dy = (double) wp.y / (double) WORLD_SCALE_FACTOR;
    double dz = (double) wp.z / (double) WORLD_SCALE_FACTOR;
    double a = (double) gfx->height() / (double) gfx->width();

    res.x = ((a * FOV * dx) / dz);
    res.y = ((FOV * dy) / dz);
    // kept as value so that points can be ordered by depth later
    res.z = ((dz - ZNEAR) * LAMBDA / dz);

    return res;
}

// converts a screen point (normalized to [-1, 1] that are edges of screen)
// to actual pixel values
inline point2d screenToPixel(Arduino_GFX *gfx, screenPoint3d sp)
{
    point2d res;
    res.x = (int32_t) ((sp.x + 1./2.) * gfx->width());
    res.y = (int32_t) ((1./2. - sp.y) * gfx->height());

    return res;
}

inline point2d worldToPixel(Arduino_GFX *gfx, point3d wp)
{
    return screenToPixel(gfx, worldToScreen(gfx, wp));
}

void drawPoint3d(Arduino_GFX *gfx, point3d pl, uint16_t color)
{
    point2d p2 = worldToPixel(gfx, pl);
    if (inRangeInc(p2.x, 0, gfx->width()) && inRangeInc(p2.y, 0, gfx->height()))
        gfx->drawPixel(p2.x, p2.y, color);
}

// draws a world coordinate line
void drawLine3d(Arduino_GFX *gfx, line3d wl, uint16_t color)
{
    point2d pp1 = worldToPixel(gfx, wl.p1);
    point2d pp2 = worldToPixel(gfx, wl.p2);

    if (cohenSutherlandClip(gfx, &pp1.x, &pp1.y, &pp2.x, &pp2.y))
    {
        gfx->drawLine(pp1.x, pp1.y, pp2.x, pp2.y, color);
    }
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
bool cohenSutherlandClip(Arduino_GFX *gfx, int32_t *x0, int32_t *y0, int32_t *x1, int32_t *y1)
{
    return cohenSutherlandClip(x0, y0, x1, y1, gfx->width(), gfx->height());
}
