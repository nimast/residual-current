#ifndef EINK_DISPLAY_H
#define EINK_DISPLAY_H

#include "EPD_13in3e.h"
#include "GUI_Paint.h"
#include "fonts.h"
#include <algorithm> // for std::swap

// Define POINT struct since it's not in the library
typedef struct {
    int X;
    int Y;
} PointType;

// Maximum number of points in a polygon
#define MAX_POLY_POINTS 10

// Wrapper class for the 13.3" e-ink display
class EInkDisplay {
public:
    // Initialize the display
    bool begin(int bufferSize = 50000);
    
    // Power management
    void sleep();
    void wakeUp();
    
    // Clear the display
    void clear(UBYTE color = EPD_13IN3E_WHITE);
    
    // Basic drawing functions (wrappers around Paint_* functions)
    void drawPoint(int x, int y, UBYTE color, DOT_PIXEL size = DOT_PIXEL_1X1, DOT_STYLE style = DOT_FILL_AROUND);
    void drawLine(int x0, int y0, int x1, int y1, UBYTE color, DOT_PIXEL width = DOT_PIXEL_1X1, LINE_STYLE style = LINE_STYLE_SOLID);
    void drawRectangle(int x0, int y0, int x1, int y1, UBYTE color, DOT_PIXEL width = DOT_PIXEL_1X1, DRAW_FILL fill = DRAW_FILL_EMPTY);
    void drawCircle(int x, int y, int radius, UBYTE color, DOT_PIXEL width = DOT_PIXEL_1X1, DRAW_FILL fill = DRAW_FILL_EMPTY);
    
    // Advanced drawing functions
    void drawPolygon(PointType *points, int numPoints, UBYTE color, DOT_PIXEL width = DOT_PIXEL_1X1, DRAW_FILL fill = DRAW_FILL_EMPTY);
    void drawDiamond(int x, int y, int size, UBYTE color, DOT_PIXEL width = DOT_PIXEL_1X1, DRAW_FILL fill = DRAW_FILL_EMPTY);
    void drawStar(int x, int y, int size, UBYTE color, DOT_PIXEL width = DOT_PIXEL_2X2);
    
    // Text functions
    void drawString(int x, int y, const char* text, sFONT* font, UBYTE foreground, UBYTE background);
    
    // Update the entire display with the current buffer contents
    void updateDisplay();
    
    // Get screen properties
    int getWidth() { return EPD_13IN3E_WIDTH; }
    int getHeight() { return EPD_13IN3E_HEIGHT; }
    
private:
    UBYTE *_buffer = NULL;
    int _bufferSize = 0;
    int _bufferWidth = 300;  // Default working area width
    int _bufferHeight = 300; // Default working area height
    
    // Triangle filling helper for polygon implementation
    void fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, UBYTE color);
};

// Color definitions for convenience
#define COLOR_BLACK   EPD_13IN3E_BLACK
#define COLOR_WHITE   EPD_13IN3E_WHITE
#define COLOR_YELLOW  EPD_13IN3E_YELLOW
#define COLOR_RED     EPD_13IN3E_RED
#define COLOR_BLUE    EPD_13IN3E_BLUE
#define COLOR_GREEN   EPD_13IN3E_GREEN

#endif // EINK_DISPLAY_H 