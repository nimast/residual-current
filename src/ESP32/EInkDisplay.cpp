#include "EInkDisplay.h"
#include <algorithm> // for std::swap

// Initialize the display and allocate buffer memory
bool EInkDisplay::begin(int bufferSize) {
    // Initialize display module
    DEV_Module_Init();
    
    // Initialize display
    EPD_13IN3E_Init();
    
    // Allocate memory for display buffer
    _bufferSize = bufferSize;
    if ((_buffer = (UBYTE *)malloc(_bufferSize)) == NULL) {
        return false;
    }
    
    // Initialize Paint object with buffer
    memset(_buffer, 0xFF, _bufferSize);
    
    // Calculate buffer dimensions based on buffer size
    // We'll assume a square working area for simplicity
    int maxDimension = sqrt(_bufferSize);
    _bufferWidth = _bufferHeight = maxDimension;
    
    // Initial setup of Paint with buffer dimensions
    Paint_NewImage(_buffer, _bufferWidth, _bufferHeight, 0, EPD_13IN3E_WHITE);
    Paint_SelectImage(_buffer);
    Paint_Clear(EPD_13IN3E_WHITE);
    
    return true;
}

// Put display to sleep to preserve it
void EInkDisplay::sleep() {
    EPD_13IN3E_Sleep();
}

// Wake up display from sleep
void EInkDisplay::wakeUp() {
    EPD_13IN3E_Init();
}

// Clear the entire display
void EInkDisplay::clear(UBYTE color) {
    EPD_13IN3E_Clear(color);
}

// Basic drawing primitives - mostly wrappers around Paint_* functions
void EInkDisplay::drawPoint(int x, int y, UBYTE color, DOT_PIXEL size, DOT_STYLE style) {
    // Skip if point is outside buffer
    if (x < 0 || x >= _bufferWidth || y < 0 || y >= _bufferHeight) {
        return;
    }
    
    Paint_DrawPoint(x, y, color, size, style);
}

void EInkDisplay::drawLine(int x0, int y0, int x1, int y1, UBYTE color, DOT_PIXEL width, LINE_STYLE style) {
    // Simple clipping
    if ((x0 < 0 && x1 < 0) || (x0 >= _bufferWidth && x1 >= _bufferWidth) ||
        (y0 < 0 && y1 < 0) || (y0 >= _bufferHeight && y1 >= _bufferHeight)) {
        return;
    }
    
    // Clip coordinates to buffer boundaries
    x0 = std::max(0, std::min(_bufferWidth - 1, x0));
    y0 = std::max(0, std::min(_bufferHeight - 1, y0));
    x1 = std::max(0, std::min(_bufferWidth - 1, x1));
    y1 = std::max(0, std::min(_bufferHeight - 1, y1));
    
    Paint_DrawLine(x0, y0, x1, y1, color, width, style);
}

void EInkDisplay::drawRectangle(int x0, int y0, int x1, int y1, UBYTE color, DOT_PIXEL width, DRAW_FILL fill) {
    // Simple clipping
    if (x1 < 0 || x0 >= _bufferWidth || y1 < 0 || y0 >= _bufferHeight) {
        return;
    }
    
    // Clip coordinates to buffer boundaries
    x0 = std::max(0, std::min(_bufferWidth - 1, x0));
    y0 = std::max(0, std::min(_bufferHeight - 1, y0));
    x1 = std::max(0, std::min(_bufferWidth - 1, x1));
    y1 = std::max(0, std::min(_bufferHeight - 1, y1));
    
    Paint_DrawRectangle(x0, y0, x1, y1, color, width, fill);
}

void EInkDisplay::drawCircle(int x, int y, int radius, UBYTE color, DOT_PIXEL width, DRAW_FILL fill) {
    // Simple clipping
    if (x + radius < 0 || x - radius >= _bufferWidth || y + radius < 0 || y - radius >= _bufferHeight) {
        return;
    }
    
    // Clip center coordinates to make circle as visible as possible
    x = std::max(radius, std::min(_bufferWidth - radius, x));
    y = std::max(radius, std::min(_bufferHeight - radius, y));
    
    Paint_DrawCircle(x, y, radius, color, width, fill);
}

// Advanced drawing functions

// Polygon drawing implementation
void EInkDisplay::drawPolygon(PointType *points, int numPoints, UBYTE color, DOT_PIXEL width, DRAW_FILL fill) {
    if (numPoints < 3 || numPoints > MAX_POLY_POINTS) {
        return; // Need at least 3 points for a polygon
    }
    
    // Draw edges of polygon
    for (int i = 0; i < numPoints; i++) {
        int next = (i + 1) % numPoints;
        drawLine(points[i].X, points[i].Y, points[next].X, points[next].Y, color, width, LINE_STYLE_SOLID);
    }
    
    // Fill polygon if requested
    if (fill == DRAW_FILL_FULL) {
        // Simple triangle fan filling from first point
        // This is a simplified approach that works for convex polygons
        for (int i = 1; i < numPoints - 1; i++) {
            fillTriangle(points[0].X, points[0].Y,
                         points[i].X, points[i].Y,
                         points[i+1].X, points[i+1].Y,
                         color);
        }
    }
}

// Helper for filling triangles - used by polygon fill
void EInkDisplay::fillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, UBYTE color) {
    // Sort vertices by y-coordinate (y0 <= y1 <= y2)
    if (y0 > y1) {
        std::swap(y0, y1);
        std::swap(x0, x1);
    }
    if (y1 > y2) {
        std::swap(y1, y2);
        std::swap(x1, x2);
    }
    if (y0 > y1) {
        std::swap(y0, y1);
        std::swap(x0, x1);
    }

    if (y0 == y2) { // Flat triangle
        return;
    }

    // Calculate slopes
    float dx01 = (float)(x1 - x0) / std::max(1, y1 - y0);
    float dx02 = (float)(x2 - x0) / std::max(1, y2 - y0);
    float dx12 = (float)(x2 - x1) / std::max(1, y2 - y1);

    // First half of the triangle (flat bottom)
    float x_left, x_right;
    for (int y = std::max(0, y0); y <= std::min(_bufferHeight - 1, y1); y++) {
        x_left = x0 + dx01 * (y - y0);
        x_right = x0 + dx02 * (y - y0);
        
        if (x_left > x_right) {
            std::swap(x_left, x_right);
        }
        
        // Draw horizontal line
        for (int x = std::max(0, (int)x_left); x <= std::min(_bufferWidth - 1, (int)x_right); x++) {
            Paint_DrawPoint(x, y, color, DOT_PIXEL_1X1, DOT_FILL_AROUND);
        }
    }
    
    // Second half of the triangle (flat top)
    for (int y = std::max(0, y1 + 1); y <= std::min(_bufferHeight - 1, y2); y++) {
        x_left = x1 + dx12 * (y - y1);
        x_right = x0 + dx02 * (y - y0);
        
        if (x_left > x_right) {
            std::swap(x_left, x_right);
        }
        
        // Draw horizontal line
        for (int x = std::max(0, (int)x_left); x <= std::min(_bufferWidth - 1, (int)x_right); x++) {
            Paint_DrawPoint(x, y, color, DOT_PIXEL_1X1, DOT_FILL_AROUND);
        }
    }
}

// Convenience function to draw a diamond shape
void EInkDisplay::drawDiamond(int x, int y, int size, UBYTE color, DOT_PIXEL width, DRAW_FILL fill) {
    PointType points[4] = {
        {x, y - size},        // Top
        {x + size, y},        // Right
        {x, y + size},        // Bottom
        {x - size, y}         // Left
    };
    
    drawPolygon(points, 4, color, width, fill);
}

// Draw a star shape using lines
void EInkDisplay::drawStar(int x, int y, int size, UBYTE color, DOT_PIXEL width) {
    // Draw crossed lines
    drawLine(x - size, y, x + size, y, color, width);
    drawLine(x, y - size, x, y + size, color, width);
    
    // Draw diagonal lines for a star effect
    drawLine(x - size/2, y - size/2, x + size/2, y + size/2, color, width);
    drawLine(x - size/2, y + size/2, x + size/2, y - size/2, color, width);
}

// Text drawing function
void EInkDisplay::drawString(int x, int y, const char* text, sFONT* font, UBYTE foreground, UBYTE background) {
    // Simple clipping
    if (x >= _bufferWidth || y >= _bufferHeight ||
        x + strlen(text) * font->Width < 0 || y + font->Height < 0) {
        return;
    }
    
    // Clip coordinates if needed
    x = std::max(0, std::min(_bufferWidth - 1, x));
    y = std::max(0, std::min(_bufferHeight - 1, y));
    
    Paint_DrawString_EN(x, y, text, font, foreground, background);
}

// Update the entire display
void EInkDisplay::updateDisplay() {
    // For a full display update, we use the display function that shows the
    // buffer contents at the top-left of the screen
    EPD_13IN3E_Display(_buffer);
} 