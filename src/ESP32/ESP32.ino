/*
 * ESP32-S3 Residual Current Display
 * 
 * This project visualizes residual current on a 13.3" 6-color E-Ink display.
 * 
 * MEMORY OPTIMIZATION NOTES:
 * The ESP32-S3 doesn't have enough RAM to buffer the entire 13.3" e-ink display at once,
 * which would require around 322,560 bytes (960x672 / 2 bytes per pixel). Instead, we use 
 * a tile-based approach, where we only allocate memory for one small section (tile) of the
 * display at a time (200x200 pixels = 20,000 bytes). We update tiles individually
 * and use EPD_13IN3E_DisplayPart() to send these partial updates to the display.
 * 
 * COLOR MAPPING:
 * The e-paper display supports 6 colors with the following codes:
 *   - BLACK  (0x0)
 *   - WHITE  (0x1)
 *   - YELLOW (0x2)
 *   - RED    (0x3)
 *   - BLUE   (0x5)
 *   - GREEN  (0x6)
 * 
 * Each tile visualizes the current value with appropriate colors to ensure
 * the best visibility on the e-ink display.
 */

#include <Adafruit_ADS1X15.h>
#include <Wire.h> // Needed for I2C
#include "EPD_13in3e.h"
#include "GUI_Paint.h"
#include "fonts.h"
#include "ImageData.h"

/* Pin connections for ESP32-S3-Nano to 13.3inch e-Paper HAT+:
 * e-Paper HAT+ Pin | ESP32-S3-Nano Pin
 * -------------------------------------
 * BUSY             | D6
 * RST              | D8
 * DC               | D7
 * CS-S             | D9
 * MOSI             | D11
 * CLK              | D13
 * CS-M             | D10
 * PWR              | D5
 * GND              | GND
 * 3.3V             | 3.3V
 */

// ADS1115 & I2C Settings
#define I2C_SDA D3           // Using GPIO6 for SDA
#define I2C_SCL D4           // Using GPIO7 for SCL
TwoWire s3i2c = TwoWire(0); // Use I2C bus 0
Adafruit_ADS1115 ads;
float V_MULTIPLIER = 0.1875F; // ADS1115 @ +/- 6.144V gain (16-bit results)

// Change detection parameters (ported from differential.ino)
#define WINDOW_SIZE 5
#define MIN_THRESHOLD 0.1875
#define MAX_THRESHOLD 50.0
#define ADAPTIVE_FACTOR 1.1
#define STABILITY_THRESHOLD 0.1
// Note: Rate limiting parameters omitted for now

// Change detection variables
float values[WINDOW_SIZE];
int valueIndex = 0;
float lastAverage = 0;
bool bufferFilled = false;

// Historical Voltage Tracking
float historicalMin = 99999.0;
float historicalMax = -99999.0;

// E-Ink Image Buffer & Row Tracking
UBYTE *Image = NULL; // Pointer for the image buffer
// EPD_13IN3E_WIDTH = 960, EPD_13IN3E_HEIGHT = 672
// Size calculation: (Width / 2 pixels per byte) * Height
const UWORD IMAGE_BUFFER_SIZE = ((EPD_13IN3E_WIDTH / 2) * EPD_13IN3E_HEIGHT);
// Reduced buffer size for partial updates
// Instead of trying to buffer the entire display (322,560 bytes)
const UWORD TILE_WIDTH = 200;  // Match the example code's dimensions
const UWORD TILE_HEIGHT = 200; // Match the example code's dimensions
// Calculate buffer size for a tile
const UWORD SMALL_IMAGE_BUFFER_SIZE = 20000; // Same as example code's buffer size
bool useSmallBuffer = true; // Use the smaller buffer approach

// Define how we divide the screen into tiles
const int TILES_X = (EPD_13IN3E_WIDTH + TILE_WIDTH - 1) / TILE_WIDTH;   // Ceiling division for X tiles
const int TILES_Y = (EPD_13IN3E_HEIGHT + TILE_HEIGHT - 1) / TILE_HEIGHT; // Ceiling division for Y tiles
const int TOTAL_TILES = TILES_X * TILES_Y;
bool tilesFilled[TILES_X * TILES_Y]; // Track which tiles are filled
int filledTileCount = 0;

// Row tracking (now used within tiles)
bool rowsFilled[EPD_13IN3E_HEIGHT];
int filledRowCount = 0;

// Periodic Update Timer
unsigned long lastDisplayUpdateTime = 0;
unsigned long currentDisplayUpdateInterval = 90000; // Default, will be randomized

// --- Simulation Mode ---
//#define SIMULATE_ADS true // Set to false when using the real ADS1115

// --- Clear Mode ---
//#define CLEAR_MODE false // Set to true to clear display once and halt

#if SIMULATE_ADS
float simTime = 0.0;
float simBaseline = 10.0; // Simulated baseline voltage in mV
float simAmplitude = 50.0; // Amplitude of sine wave in mV
float simFrequency = 0.1;  // Frequency of sine wave
float simNoise = 5.0;      // Max random noise in mV
#endif

// --- Helper Function Declarations ---
float calculateAverage();
float calculateStdDev(float avg);
bool isSignalStable(float stdDev);
void displaySimpleText(float value);
void drawEnergyLineInTile(float value, int tileX, int tileY);
void displayValueInTile(float value, int tileX, int tileY);

// --- Refactored Loop Functions ---
void handleScreenReset();
float readSensorValue();
void updateAndCheckChangeDetection(float currentValue);
void handlePeriodicUpdate(float currentValue);

void setup()
{
    Serial.begin(115200); // Use a faster baud rate
    while (!Serial)
        ;                      // Wait for serial connection

    #ifdef CLEAR_MODE
    // --- Clear Mode Logic ---
    Debug("Entering Clear Mode...\r\n");
    Debug("Initializing E-Paper Module...\r\n");
    DEV_Module_Init();
    Debug("Initializing E-Paper Display...\r\n");
    EPD_13IN3E_Init();
    Debug("Clearing Display...\r\n");
    EPD_13IN3E_Clear(EPD_13IN3E_WHITE); // Clear display physically
    DEV_Delay_ms(500); // Give time for clear command
    Debug("Putting Display to Sleep...\r\n");
    EPD_13IN3E_Sleep();
    Debug("Powering Down Module...\r\n");
    DEV_Module_Exit();
    Debug("Clear Mode complete. Halting execution.\r\n");
    while(1); // Busy wait to halt execution
    Debug("*** ERROR: Execution continued past while(1) in CLEAR_MODE! ***\r\n"); // This should never appear

    #else
    // --- Normal Visualization Logic ---

    randomSeed(analogRead(0)); // Seed the random number generator (assuming pin 0 is floating or provides noise)
    Debug("Initializing System...\r\n");

    // --- Initialize I2C and ADS1115 ---
    #ifndef SIMULATE_ADS
    Debug("Initializing I2C & ADS1115...\r\n");
    s3i2c.begin(I2C_SDA, I2C_SCL, 100000);
    if (!ads.begin(ADS1X15_ADDRESS, &s3i2c))
    {
        Debug("Failed to initialize ADS. Halting.\r\n");
        while (1)
            ;
    }
    // Optional: Set Gain if needed, default is GAIN_TWOTHIRDS (+/- 6.144V)
    // ads.setGain(GAIN_ONE); V_MULTIPLIER = 0.125F;
    Debug("ADS1115 Initialized.\r\n");
    #else
    Debug("ADS1115 Simulation Mode: Skipping hardware initialization.\r\n");
    #endif // SIMULATE_ADS

    // --- Initialize Change Detection Variables ---
    for (int i = 0; i < WINDOW_SIZE; i++)
    {
        values[i] = 0;
    }
    valueIndex = 0;
    lastAverage = 0;
    bufferFilled = false;
    historicalMin = 99999.0;
    historicalMax = -99999.0;

    // --- Initialize E-Ink ---
    Debug("Initializing E-Paper Module...\r\n");
    DEV_Module_Init(); // Initializes GPIOs based on EPD_Driver.h pins

    Debug("Initializing E-Paper Display...\r\n");
    EPD_13IN3E_Init();
    EPD_13IN3E_Clear(EPD_13IN3E_WHITE); // Clear display physically
    DEV_Delay_ms(500);

    // --- Create and Initialize Image Buffer ---
    Debug("Allocating Image Buffer...\r\n");
    if (useSmallBuffer) {
        // Test with a smaller buffer size for partial updates
        Debug("Using smaller test buffer of %d bytes for %dx%d area\r\n", 
              SMALL_IMAGE_BUFFER_SIZE, TILE_WIDTH, TILE_HEIGHT);
        
        if ((Image = (UBYTE *)malloc(SMALL_IMAGE_BUFFER_SIZE)) == NULL) {
            Debug("Failed to allocate memory for small test buffer! Halting.\r\n");
            while (1);
        }
        
        Debug("Small buffer allocated successfully.\r\n");
        
        // Initialize the buffer with white
        memset(Image, 0xFF, SMALL_IMAGE_BUFFER_SIZE);
        
        // Initialize the Paint object with proper dimensions
        Paint_NewImage(Image, TILE_WIDTH, TILE_HEIGHT, 0, EPD_13IN3E_WHITE);
        Paint_SelectImage(Image);
        Paint_Clear(EPD_13IN3E_WHITE);
        
        // Draw a test pattern with multiple colors
        Debug("Drawing test pattern with multiple colors...\r\n");
        
        // Draw colored rectangles to test the display's color capabilities
        const UBYTE colors[] = {
            EPD_13IN3E_BLACK,
            EPD_13IN3E_YELLOW,
            EPD_13IN3E_RED,
            EPD_13IN3E_BLUE,
            EPD_13IN3E_GREEN
        };
        
        int rectWidth = TILE_WIDTH / 5;
        for (int i = 0; i < 5; i++) {
            Paint_DrawRectangle(i * rectWidth, 0, (i + 1) * rectWidth, TILE_HEIGHT / 2, 
                               colors[i], DOT_PIXEL_1X1, DRAW_FILL_FULL);
        }
        
        // Draw a checkerboard pattern in the bottom half
        for (int y = TILE_HEIGHT/2; y < TILE_HEIGHT; y += 20) {
            for (int x = 0; x < TILE_WIDTH; x += 20) {
                UBYTE color = ((x + y) / 20) % 2 == 0 ? EPD_13IN3E_BLACK : EPD_13IN3E_WHITE;
                Paint_DrawRectangle(x, y, x + 19, y + 19, color, DOT_PIXEL_1X1, DRAW_FILL_FULL);
            }
        }
        
        // Display the partial area
        Debug("Sending test pattern to display (top-left)...\r\n");
        EPD_13IN3E_DisplayPart(Image, 0, 0, TILE_WIDTH, TILE_HEIGHT);
        
        Debug("Pattern displayed. Now testing text display...\r\n");
        DEV_Delay_ms(2000);
        
        // Now test text display
        displaySimpleText(123.45);
    } else {
        // Original full-buffer approach
        if ((Image = (UBYTE *)malloc(IMAGE_BUFFER_SIZE)) == NULL) {
            Debug("Failed to allocate memory for full image buffer! Halting.\r\n");
            while (1);
        }
        Debug("Full Image Buffer Allocated. Size: %d bytes\r\n", IMAGE_BUFFER_SIZE);
    Paint_NewImage(Image, EPD_13IN3E_WIDTH, EPD_13IN3E_HEIGHT, 0, WHITE); // Set buffer dimensions and default rotation/color
    Paint_SelectImage(Image);                                             // Point drawing functions to this buffer
    Paint_Clear(WHITE);                                                   // Fill the buffer with white
    }

    // --- Initialize Row Tracking and Tile Tracking ---
    Debug("Initializing Row and Tile Tracking...\r\n");
    for (int i = 0; i < EPD_13IN3E_HEIGHT; i++) {
        rowsFilled[i] = false;
    }
    filledRowCount = 0;
    
    for (int i = 0; i < TOTAL_TILES; i++) {
        tilesFilled[i] = false;
    }
    filledTileCount = 0;

    Debug("Displaying Initial Blank Screen...\r\n");
    // Use factory-approved sequence for clearing the screen
    memset(Image, 0, IMAGE_BUFFER_SIZE); // Zero out buffer first
    EPD_13IN3E_Display(Image); // Show the cleared buffer on the display
    DEV_Delay_ms(1000); // Allow time for display update

    Debug("Setup Complete. Entering Loop.\r\n");

    // Calculate the first random update interval
    currentDisplayUpdateInterval = random(60000, 120001); // Random between 60000ms and 120000ms
    Debug("First update interval: %lu ms\r\n", currentDisplayUpdateInterval);
    lastDisplayUpdateTime = millis(); // Start the timer now

    // --- REMOVED Original Setup Drawing/Sleep Code ---
    // #if 1 // show bmp ... #endif
    // Debug("Clear..."); ...
    EPD_13IN3E_Sleep();
    //DEV_Module_Exit();

    #endif // CLEAR_MODE
}

void loop()
{
#ifdef CLEAR_MODE
    return; // Do nothing in loop if clear mode is active
#endif

    handleScreenReset();

    float currentValue = readSensorValue();

    updateAndCheckChangeDetection(currentValue);

    // Find a tile that isn't filled yet and draw to it
    if (filledTileCount < TOTAL_TILES) {
        // Find a random unfilled tile
        int tileIndex;
        int attempts = 0;
        do {
            tileIndex = random(TOTAL_TILES);
            attempts++;
        } while (tilesFilled[tileIndex] && attempts < TOTAL_TILES * 2);
        
        if (!tilesFilled[tileIndex]) {
            int tileX = tileIndex % TILES_X;
            int tileY = tileIndex / TILES_X;
            
            Debug("Drawing energy in tile %d,%d (tile %d of %d)\r\n", 
                  tileX, tileY, filledTileCount + 1, TOTAL_TILES);
            
            drawEnergyLineInTile(currentValue, tileX, tileY);
            tilesFilled[tileIndex] = true;
            filledTileCount++;
            
            // Reset periodic update timer after a tile draw
            lastDisplayUpdateTime = millis();
            currentDisplayUpdateInterval = random(60000, 120001);
        }
    }

    // Periodic update is still handled separately
    handlePeriodicUpdate(currentValue);

    yield(); // Allow background tasks/watchdog

    delay(5000); // Slow down main loop polling significantly
}

// --- Refactored Function Implementations ---

#ifndef CLEAR_MODE // Don't compile these functions in clear mode

void handleScreenReset() {
    // --- Check if All Tiles are Full -> Reset ---
    if (filledTileCount >= TOTAL_TILES) {
        Debug("All tiles filled. Clearing and restarting visualization.\r\n");
        
        // Full clear of the display
        EPD_13IN3E_Init();
        EPD_13IN3E_Clear(EPD_13IN3E_WHITE);

        // Reset tracking variables
        filledTileCount = 0;
        for (int i = 0; i < TOTAL_TILES; i++) {
            tilesFilled[i] = false;
        }
        
        filledRowCount = 0;
        for (int i = 0; i < EPD_13IN3E_HEIGHT; i++) {
            rowsFilled[i] = false;
        }

        DEV_Delay_ms(1000); // Pause briefly after clearing
        lastDisplayUpdateTime = millis(); // Reset update timer after clear
    }
}

float readSensorValue() {
    // --- Read ADS1115 ---
#ifndef SIMULATE_ADS
    int16_t results = ads.readADC_Differential_0_1();
    float currentValue = results * V_MULTIPLIER; // Convert to mV
#else
    // --- Simulate ADS1115 Reading ---
    simTime += 0.1; // Increment simulation time
    float sineValue = sin(simTime * simFrequency) * simAmplitude;
    float noiseValue = random(-(simNoise * 100), (simNoise * 100) + 1) / 100.0; // Random noise between -simNoise and +simNoise
    float currentValue = simBaseline + sineValue + noiseValue;
#endif
    return currentValue;
}

void updateAndCheckChangeDetection(float currentValue) {
    // --- Update Historical Min/Max ---
    if (currentValue < historicalMin)
    {
        historicalMin = currentValue;
    }
    if (currentValue > historicalMax)
    {
        historicalMax = currentValue;
    }

    // --- Update Change Detection Buffer ---
    values[valueIndex] = currentValue;
    valueIndex = (valueIndex + 1) % WINDOW_SIZE;
    if (!bufferFilled && valueIndex == 0)
    {
        bufferFilled = true; // Buffer is now full for the first time
        Debug("Buffer filled.\r\n");
    }

    // --- Calculate Average (still needed for lastAverage) ---
    float avgValue = calculateAverage();

    // --- Calculate Change ---
    float changeAmount = 0;
    if (bufferFilled)
    { // Calculate change only when buffer has meaningful data
        changeAmount = abs(avgValue - lastAverage);
    }
    lastAverage = avgValue; // Keep track of the last average regardless

    // --- Debug Print ---
    Serial.printf("V: %.2f, Avg: %.2f\r\n", 
                  currentValue, avgValue);
}

void handlePeriodicUpdate(float currentValue) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastDisplayUpdateTime;
    // Prevent underflow if millis() has rolled over
    unsigned long remainingTimeMs = (elapsedTime > currentDisplayUpdateInterval) ? 0 : currentDisplayUpdateInterval - elapsedTime;
    int remainingSeconds = remainingTimeMs / 1000;

    if (currentTime - lastDisplayUpdateTime >= currentDisplayUpdateInterval)
    {
        Debug("Periodic display update timer elapsed.\r\n");
        
        // Find a random unfilled tile for the periodic update
        if (filledTileCount < TOTAL_TILES) {
            int tileIndex;
            int attempts = 0;
            do {
                tileIndex = random(TOTAL_TILES);
                attempts++;
            } while (tilesFilled[tileIndex] && attempts < TOTAL_TILES * 2);
            
            if (!tilesFilled[tileIndex]) {
                int tileX = tileIndex % TILES_X;
                int tileY = tileIndex / TILES_X;
                
                Debug("Periodic update: Using tile %d,%d\r\n", tileX, tileY);
                
                // Display the value in this tile
                displayValueInTile(currentValue, tileX, tileY);
                
                // Mark this tile as filled
                tilesFilled[tileIndex] = true;
                filledTileCount++;
            } else {
                Debug("Could not find unfilled tile for periodic update.\r\n");
                // Could display in a corner of the screen instead
            }
        } else {
            Debug("All tiles filled, skipping periodic update.\r\n");
        }
        
        lastDisplayUpdateTime = currentTime; // Reset the timer
        currentDisplayUpdateInterval = random(60000, 120001); // Calculate next random interval
        Debug("Next periodic update in %lu ms\r\n", currentDisplayUpdateInterval);
    }
}

// Draws an energy line in a specific tile using the display's color capabilities
void drawEnergyLineInTile(float value, int tileX, int tileY) {
    // Validate tile coordinates
    if (tileX < 0 || tileX >= TILES_X || tileY < 0 || tileY >= TILES_Y) {
        Debug("Invalid tile coordinates: %d,%d\r\n", tileX, tileY);
        return;
    }

    // Calculate tile's position on screen
    int tileStartX = tileX * TILE_WIDTH;
    int tileStartY = tileY * TILE_HEIGHT;
    
    // Allocate the buffer if not already allocated
    if (Image == NULL) {
        if ((Image = (UBYTE *)malloc(SMALL_IMAGE_BUFFER_SIZE)) == NULL) {
            Debug("Failed to allocate memory for tile buffer! Skipping draw.\r\n");
            return;
        }
    }
    
    // Initialize the buffer with white
    memset(Image, 0xFF, SMALL_IMAGE_BUFFER_SIZE);
    
    // Initialize the Paint system for this tile
    Paint_NewImage(Image, TILE_WIDTH, TILE_HEIGHT, 0, EPD_13IN3E_WHITE);
    Paint_SelectImage(Image);
    Paint_Clear(EPD_13IN3E_WHITE);
    
    // Find available rows within this tile
    int tileRowStart = tileY * TILE_HEIGHT;
    int tileRowEnd = min((int)(tileRowStart + TILE_HEIGHT), EPD_13IN3E_HEIGHT);
    
    // Count unfilled rows in this tile
    int unfilledRowsInTile = 0;
    for (int y = tileRowStart; y < tileRowEnd; y++) {
        if (!rowsFilled[y]) {
            unfilledRowsInTile++;
        }
    }
    
    // If no unfilled rows, just return (shouldn't happen with proper tracking)
    if (unfilledRowsInTile == 0) {
        Debug("No unfilled rows in tile %d,%d\r\n", tileX, tileY);
        return;
    }
    
    // Pick an unfilled row within this tile
    int targetRow;
    int attempts = 0;
    do {
        int relativeRow = random(tileRowEnd - tileRowStart);
        targetRow = tileRowStart + relativeRow;
        attempts++;
    } while (rowsFilled[targetRow] && attempts < (tileRowEnd - tileRowStart) * 2);
    
    if (rowsFilled[targetRow]) {
        // If we couldn't find a random unfilled row, find the first available
        for (targetRow = tileRowStart; targetRow < tileRowEnd; targetRow++) {
            if (!rowsFilled[targetRow]) break;
        }
        if (targetRow == tileRowEnd) {
            Debug("Could not find any unfilled row in tile! This shouldn't happen.\r\n");
            return;
        }
    }
    
    // Final validation check for target row
    if (targetRow < 0 || targetRow >= EPD_13IN3E_HEIGHT) {
        Debug("Target row %d is outside the valid display height (0-%d)\r\n", 
              targetRow, EPD_13IN3E_HEIGHT - 1);
        return;
    }
    
    // Mark this row as filled
    rowsFilled[targetRow] = true;
    filledRowCount++;
    
    // Convert the global row position to a local tile row
    int localTargetRow = targetRow - tileRowStart;
    
    // Validate the local row coordinate
    if (localTargetRow < 0 || localTargetRow >= TILE_HEIGHT) {
        Debug("Local row %d (from global %d) is outside valid tile height (0-%d)\r\n", 
              localTargetRow, targetRow, TILE_HEIGHT - 1);
        return;
    }
    
    Debug("Drawing energy line at row %d (global) / %d (local in tile)\r\n", 
          targetRow, localTargetRow);
    
    // Determine the color set for this energy line based on the value
    UBYTE lineColor, accentColor, bgColor;
    
    // Map value to a color scheme
    if (value > 100) {
        lineColor = EPD_13IN3E_RED;      // Red for high values
        accentColor = EPD_13IN3E_YELLOW; // Yellow accent
        bgColor = EPD_13IN3E_WHITE;      // White background
    } else if (value > 50) {
        lineColor = EPD_13IN3E_YELLOW;   // Yellow for medium-high values
        accentColor = EPD_13IN3E_RED;    // Red accent
        bgColor = EPD_13IN3E_WHITE;      // White background
    } else if (value > 0) {
        lineColor = EPD_13IN3E_GREEN;    // Green for positive values
        accentColor = EPD_13IN3E_BLUE;   // Blue accent
        bgColor = EPD_13IN3E_WHITE;      // White background
    } else if (value > -50) {
        lineColor = EPD_13IN3E_BLUE;     // Blue for negative values
        accentColor = EPD_13IN3E_GREEN;  // Green accent
        bgColor = EPD_13IN3E_WHITE;      // White background
    } else {
        lineColor = EPD_13IN3E_BLACK;    // Black for very negative values
        accentColor = EPD_13IN3E_BLUE;   // Blue accent
        bgColor = EPD_13IN3E_WHITE;      // White background
    }
    
    // Draw a horizontal line across the full width using the primary color
    if (localTargetRow >= 0 && localTargetRow < TILE_HEIGHT) {
        Paint_DrawLine(0, localTargetRow, TILE_WIDTH, localTargetRow, 
                      lineColor, DOT_PIXEL_3X3, LINE_STYLE_SOLID);
    } else {
        Debug("Cannot draw main line - row %d out of bounds\r\n", localTargetRow);
        return; // Skip the rest if main line can't be drawn
    }
    
    // Create energy pulse patterns along the line
    const int numPulses = 5;
    int pulseSpacing = TILE_WIDTH / numPulses;
    
    for (int i = 0; i < numPulses; i++) {
        int pulseCenter = i * pulseSpacing + pulseSpacing/2;
        int pulseSize = 8 + abs(value) / 10; // Size based on value magnitude
        if (pulseSize > 20) pulseSize = 20;  // Cap the maximum pulse size
        
        // Ensure the pulse stays within tile boundaries
        if (pulseCenter - pulseSize >= 0 && pulseCenter + pulseSize < TILE_WIDTH && 
            localTargetRow - pulseSize >= 0 && localTargetRow + pulseSize < TILE_HEIGHT) {
            // Draw a circle pulse
            Paint_DrawCircle(pulseCenter, localTargetRow, pulseSize, 
                            accentColor, DOT_PIXEL_1X1, DRAW_FILL_FULL);
            
            // Add a smaller inner circle with the line color
            Paint_DrawCircle(pulseCenter, localTargetRow, pulseSize/2, 
                            lineColor, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        } else {
            Debug("Skipping pulse %d - would exceed display boundaries\r\n", i);
        }
    }
    
    // Draw indicator of value magnitude
    int valueIndicatorHeight = min(abs(value), 50.0f); // Cap at 50 pixels
    
    // Draw a small bar chart on the side of the tile to indicate value
    if (value >= 0) {
        // Positive values: bar goes up from the line
        // Check boundaries
        if (TILE_WIDTH - 10 >= 0 && TILE_WIDTH - 5 < TILE_WIDTH && 
            localTargetRow - valueIndicatorHeight >= 0 && localTargetRow < TILE_HEIGHT) {
            Paint_DrawRectangle(TILE_WIDTH - 10, localTargetRow - valueIndicatorHeight, 
                              TILE_WIDTH - 5, localTargetRow, 
                              lineColor, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        } else {
            Debug("Exceeding display boundaries for value indicator\r\n");
        }
    } else {
        // Negative values: bar goes down from the line
        // Check boundaries
        if (TILE_WIDTH - 10 >= 0 && TILE_WIDTH - 5 < TILE_WIDTH && 
            localTargetRow >= 0 && localTargetRow + valueIndicatorHeight < TILE_HEIGHT) {
            Paint_DrawRectangle(TILE_WIDTH - 10, localTargetRow, 
                              TILE_WIDTH - 5, localTargetRow + valueIndicatorHeight, 
                              lineColor, DOT_PIXEL_1X1, DRAW_FILL_FULL);
        } else {
            Debug("Exceeding display boundaries for value indicator\r\n");
        }
    }
    
    // Draw amplitude marker lines
    const int numMarkers = 8;
    int markerSize = 5;
    int markerSpacing = TILE_WIDTH / (numMarkers + 1);
    float amplitude = min(abs(value) / 5, 15.0f); // Cap at 15 pixels
    
    for (int i = 1; i <= numMarkers; i++) {
        int markerX = i * markerSpacing;
        float offset = sin(i * 0.8) * amplitude; // Sine wave pattern
        
        // Check boundaries before drawing markers
        if (markerX >= 0 && markerX < TILE_WIDTH && 
            localTargetRow - markerSize >= 0 && localTargetRow + markerSize < TILE_HEIGHT) {
            // Draw marker lines perpendicular to the main line
            Paint_DrawLine(markerX, localTargetRow - markerSize, 
                          markerX, localTargetRow + markerSize, 
                          accentColor, DOT_PIXEL_1X1, LINE_STYLE_SOLID);
        } else {
            Debug("Exceeding display boundaries for marker\r\n");
        }
        
        // Check boundaries for sine wave dots
        int yOffset = (int)offset;
        if (markerX >= 0 && markerX < TILE_WIDTH && 
            localTargetRow + yOffset >= 0 && localTargetRow + yOffset < TILE_HEIGHT) {
            // Add sine wave effect with smaller dots
            Paint_DrawPoint(markerX, localTargetRow + yOffset, 
                           lineColor, DOT_PIXEL_2X2, DOT_FILL_AROUND);
        }
    }
    
    // Add value indicator text
    char valueStr[10];
    sprintf(valueStr, "%.1f", value);
    // Check boundaries for text
    if (5 >= 0 && localTargetRow - 12 >= 0 && localTargetRow - 12 + 12 < TILE_HEIGHT) {
        Paint_DrawString_EN(5, localTargetRow - 12, valueStr, &Font12, 
                           lineColor, bgColor);
    } else {
        Debug("Exceeding display boundaries for text\r\n");
    }
    
    // Display the tile on the e-paper
    EPD_13IN3E_DisplayPart(Image, tileStartX, tileStartY, TILE_WIDTH, TILE_HEIGHT);
    
    // Debug output
    Debug("Energy line drawn in tile %d,%d at row %d with value %.2f\r\n", 
          tileX, tileY, localTargetRow, value);
}

// Displays a value in a specific tile with proper color formatting
void displayValueInTile(float value, int tileX, int tileY) {
    // Validate tile coordinates
    if (tileX < 0 || tileX >= TILES_X || tileY < 0 || tileY >= TILES_Y) {
        Debug("Invalid tile coordinates: %d,%d\r\n", tileX, tileY);
        return;
    }
    
    // Calculate tile's position on screen
    int tileStartX = tileX * TILE_WIDTH;
    int tileStartY = tileY * TILE_HEIGHT;
    
    // Allocate the buffer if not already allocated
    if (Image == NULL) {
        if ((Image = (UBYTE *)malloc(SMALL_IMAGE_BUFFER_SIZE)) == NULL) {
            Debug("Failed to allocate memory for tile buffer! Skipping draw.\r\n");
            return;
        }
    }
    
    // Convert value to string
    char valueStr[30];
    sprintf(valueStr, "Value: %.2f mV", value);
    Debug("Displaying in tile %d,%d: %s\r\n", tileX, tileY, valueStr);
    
    // Clear buffer with white
    memset(Image, 0xFF, SMALL_IMAGE_BUFFER_SIZE);
    
    // Initialize the Paint system for this tile
    Paint_NewImage(Image, TILE_WIDTH, TILE_HEIGHT, 0, EPD_13IN3E_WHITE);
    Paint_SelectImage(Image);
    //Paint_Clear(EPD_13IN3E_WHITE);
    
    // Determine color based on value 
    UBYTE textColor = EPD_13IN3E_BLACK;  // Default color
    UBYTE bgColor = EPD_13IN3E_WHITE;    // Default background
    
    if (value > 100) {
        textColor = EPD_13IN3E_RED;      // Red for high values
        bgColor = EPD_13IN3E_YELLOW;     // Yellow background for emphasis
    } else if (value < 0) {
        textColor = EPD_13IN3E_BLUE;     // Blue for negative values
    } else {
        textColor = EPD_13IN3E_GREEN;    // Green for normal values
    }
    
    // Draw background
    Paint_DrawRectangle(0, 0, TILE_WIDTH, TILE_HEIGHT, bgColor, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    
    // Draw text - ensure it fits within boundaries
    // Font24 is approximately 24 pixels tall
    if (10 >= 0 && 10 + strlen(valueStr) * 12 < TILE_WIDTH && 10 + 24 < TILE_HEIGHT) {
        Paint_DrawString_EN(10, 10, valueStr, &Font24, textColor, bgColor);
    } else {
        // Try with a smaller font if there's not enough room
        if (10 >= 0 && 10 + strlen(valueStr) * 6 < TILE_WIDTH && 10 + 12 < TILE_HEIGHT) {
            Paint_DrawString_EN(10, 10, valueStr, &Font12, textColor, bgColor);
        } else {
            Debug("Text would exceed tile boundaries - skipping text\r\n");
        }
    }
    
    // Add a visual indicator - circle with radius proportional to value
    int radius = abs(value) / 2;
    if (radius > 80) radius = 80;  // Cap the maximum radius
    if (radius < 10) radius = 10;  // Minimum radius
    
    // Check if circle would fit within the tile boundaries
    int circleX = TILE_WIDTH/2;
    int circleY = TILE_HEIGHT/2 + 20;
    if (circleX - radius >= 0 && circleX + radius < TILE_WIDTH &&
        circleY - radius >= 0 && circleY + radius < TILE_HEIGHT) {
        Paint_DrawCircle(circleX, circleY, radius, textColor, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    } else {
        Debug("Circle would exceed tile boundaries - using smaller radius\r\n");
        // Use a safe radius that fits within the tile
        int safeRadius = min((int)(min(circleX, TILE_WIDTH - circleX)),
                            (int)(min(circleY, TILE_HEIGHT - circleY))) - 1;
        if (safeRadius > 0) {
            Paint_DrawCircle(circleX, circleY, safeRadius, textColor, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
        }
    }
    
    // Add border - only draw if we're within bounds
    if (TILE_WIDTH > 1 && TILE_HEIGHT > 1) {
        Paint_DrawRectangle(0, 0, TILE_WIDTH-1, TILE_HEIGHT-1, EPD_13IN3E_BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    }
    
    // Display on the e-paper
    EPD_13IN3E_DisplayPart(Image, tileStartX, tileStartY, TILE_WIDTH, TILE_HEIGHT);
}

// Function to display a simple text message with proper color formatting
void displaySimpleText(float value) {
    if (Image == NULL) {
        if ((Image = (UBYTE *)malloc(SMALL_IMAGE_BUFFER_SIZE)) == NULL) {
            Debug("Failed to allocate memory for text buffer! Skipping.\r\n");
            return;
        }
    }
    
    // Convert value to string
    char valueStr[30];
    sprintf(valueStr, "Value: %.2f mV", value);
    Debug("Displaying test message: %s\r\n", valueStr);
    
    // Clear the buffer with white
    memset(Image, 0xFF, SMALL_IMAGE_BUFFER_SIZE);
    
    // Initialize for painting
    Paint_NewImage(Image, TILE_WIDTH, TILE_HEIGHT, 0, EPD_13IN3E_WHITE);
    Paint_SelectImage(Image);
    Paint_Clear(EPD_13IN3E_WHITE);
    
    // Determine color based on value (for demonstration)
    UBYTE textColor = EPD_13IN3E_BLACK;  // Default color
    if (value > 100) {
        textColor = EPD_13IN3E_RED;      // Red for high values
    } else if (value < 0) {
        textColor = EPD_13IN3E_BLUE;     // Blue for negative values
    } else {
        textColor = EPD_13IN3E_GREEN;    // Green for normal values
    }
    
    // Draw background rectangle
    if (TILE_WIDTH > 0 && 50 <= TILE_HEIGHT) {
        Paint_DrawRectangle(0, 0, TILE_WIDTH, 50, EPD_13IN3E_YELLOW, DOT_PIXEL_1X1, DRAW_FILL_FULL);
    }
    
    // Draw text with boundary checking
    if (10 >= 0 && 10 + 24 <= TILE_HEIGHT && 10 + strlen(valueStr) * 12 <= TILE_WIDTH) {
        Paint_DrawString_EN(10, 10, valueStr, &Font24, textColor, EPD_13IN3E_YELLOW);
    } else {
        Debug("Text would exceed boundaries in displaySimpleText\r\n");
    }
    
    // Draw a border with boundary checking
    if (TILE_WIDTH > 1 && TILE_HEIGHT > 1) {
        Paint_DrawRectangle(0, 0, TILE_WIDTH-1, TILE_HEIGHT-1, EPD_13IN3E_BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    }
    
    // Display the partial area
    EPD_13IN3E_DisplayPart(Image, 0, 0, TILE_WIDTH, TILE_HEIGHT);
    
    // Put display to sleep after update
    EPD_13IN3E_Sleep();
}

// --- Helper Function Implementations ---

float calculateAverage()
{
    float sum = 0;
    int count = bufferFilled ? WINDOW_SIZE : valueIndex;
    if (count == 0)
        return 0;
    for (int i = 0; i < count; i++)
    {
        sum += values[i];
    }
    return sum / count;
}

float calculateStdDev(float avg)
{
    float sumSquares = 0;
    int count = bufferFilled ? WINDOW_SIZE : valueIndex;
    if (count <= 1)
        return 0;
    for (int i = 0; i < count; i++)
    {
        float diff = values[i] - avg;
        sumSquares += diff * diff;
    }
    // Use population standard deviation (N) if buffer is small,
    // or sample standard deviation (N-1) if WINDOW_SIZE is larger?
    // Using N-1 for sample standard deviation:
    return sqrt(sumSquares / (count - 1));
}

bool isSignalStable(float stdDev)
{
    return stdDev < STABILITY_THRESHOLD;
}

#endif // CLEAR_MODE (End wrap for normal loop and helper functions)