#include <Adafruit_ADS1X15.h>
#include <Wire.h> // Needed for I2C
#include "EPD_13in3e.h"
#include "GUI_Paint.h"
#include "fonts.h"
#include "ImageData.h"
#include "MetalCrystalRenderer.h" // Renamed from EPD_Test.h - metal crystal implementation

// --- Mode Flags ---
#define TEST_MODE_CRYSTAL true  // Set to true to draw test crystal on startup, false for normal operation
//#define SIMULATE_ADS true     // Set to false when using the real ADS1115
//#define CLEAR_MODE false      // Set to true to clear display once and halt

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
// Size calculation: (Width / 2 pixels per byte) * Height
const UWORD IMAGE_BUFFER_SIZE = ((EPD_13IN3E_WIDTH / 2) * EPD_13IN3E_HEIGHT);
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
void HSVtoRGB(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b);
UBYTE mapRGBtoEinkColor(uint8_t r, uint8_t g, uint8_t b);
void displaySimpleText(float value);

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

    //Create a new image cache named IMAGE_BW and fill it with white
    // try to use global image UBYTE *Image;
    UWORD Imagesize = ((EPD_13IN3E_WIDTH % 2 == 0)? (EPD_13IN3E_WIDTH / 2 ): (EPD_13IN3E_WIDTH / 2 + 1)) * EPD_13IN3E_HEIGHT;
    Debug("Allocating image buffer of size %d bytes...\r\n", Imagesize);
    
    if((Image = (UBYTE *)malloc(Imagesize)) == NULL) {
        Debug("Failed to apply for black memory...\r\n");
        DEV_Module_Exit();
        while(1);
    }
    Debug("Successfully allocated image buffer at address %p\r\n", Image);
    Debug("NewImage:Image\r\n");
    Paint_NewImage(Image, EPD_13IN3E_WIDTH, EPD_13IN3E_HEIGHT, 0, WHITE);
    
    //Select Image
    Debug("Selecting image buffer for initial drawing\r\n");
    Paint_SelectImage(Image);
    Paint_Clear(WHITE);

#if 0 // show bmp
    printf("show bmp1-----------------\r\n");
    EPD_13IN3E_DisplayPart(BMP_1, 400, 500, 400, 600);
    DEV_Delay_ms(3000);
#else  // Drawing on the image
    Paint_NewImage(Image, EPD_13IN3E_WIDTH, EPD_13IN3E_HEIGHT, 0, EPD_13IN3E_WHITE);
    Paint_SetScale(6);
    // 1.Select Image
    Debug("SelectImage:Image\r\n");
    Paint_SelectImage(Image);
    Paint_Clear(EPD_13IN3E_WHITE);

    // 2.Drawing on the image
    Debug("Drawing:Image\r\n");
    
#if TEST_MODE_CRYSTAL
    // Test mode - Draw metal crystal instead of text
    Debug("TEST MODE: Drawing metal crystal on startup\r\n");
    
    // Use fixed test values for reliable testing
    float testValue1 = 25.5;  // First ADS value for testing
    float testValue2 = 45.8;  // Second ADS value for testing
    
    // Initialize ADS buffers with test values
    for (int i = 0; i < ADS_BUFFER_SIZE; i++) {
        adsBuffer1.values[i] = testValue1 + ((i % 3) - 1) * 2.5; // Add some variation
        adsBuffer2.values[i] = testValue2 + ((i % 3) - 1) * 3.8;
    }
    adsBuffer1.index = 0;
    adsBuffer1.filled = true;
    adsBuffer1.prevUpdateValue = testValue1;
    adsBuffer1.lastUpdateTime = millis() - 10000; // Pretend last update was 10 seconds ago
    
    adsBuffer2.index = 0;
    adsBuffer2.filled = true;
    adsBuffer2.prevUpdateValue = testValue2;
    adsBuffer2.lastUpdateTime = millis() - 10000;
    
    Debug("Displaying test crystal with values: %.1f, %.1f\r\n", testValue1, testValue2);
    displayMetalCrystal(EPD_13IN3E_WIDTH, EPD_13IN3E_HEIGHT, testValue1, testValue2);
#else
    // Normal mode - draw the original "Residual Current" text
    Paint_DrawString_EN(145, 105, "Residual Current", &Font16, EPD_13IN3E_RED, EPD_13IN3E_WHITE);
#endif

    Debug("EPD_Display\r\n");
    EPD_13IN3E_Display(Image);
    Debug("Initial display complete\r\n");
    DEV_Delay_ms(3000);
#endif
#endif
}

void loop()
{
#ifdef CLEAR_MODE
    return; // Do nothing in loop if clear mode is active
#endif

    handleScreenReset();

    float currentValue = readSensorValue();

    updateAndCheckChangeDetection(currentValue);

    // Periodic update is handled separately now, only call if no trigger occurred?
    // Or allow both potentially?
    // For now, let periodic update run regardless of trigger.
    handlePeriodicUpdate(currentValue);

    yield(); // Allow background tasks/watchdog

    delay(5000); // Slow down main loop polling significantly
}

// --- Refactored Function Implementations ---

#ifndef CLEAR_MODE // Don't compile these functions in clear mode

void handleScreenReset() {
    // --- Check if Screen is Full -> Reset ---
    if (filledRowCount >= EPD_13IN3E_HEIGHT)
    {
        Debug("Screen full. Clearing and restarting visualization.\r\n");
        
        // Check if Image buffer is valid
        if (Image == NULL) {
            Debug("FATAL ERROR: Image buffer is NULL in handleScreenReset!\r\n");
            return;
        }
        
        Paint_SelectImage(Image); // Ensure drawing happens in the buffer
        Paint_Clear(WHITE);       // Clear the image buffer
        EPD_13IN3E_Display(Image); // Update the physical display to blank

        // Reset tracking variables
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

// New function to read second ADS channel
float readSecondSensorValue() {
    // --- Read ADS1115 Second Channel ---
#ifndef SIMULATE_ADS
    int16_t results2 = ads.readADC_Differential_2_3();
    float currentValue2 = results2 * V_MULTIPLIER; // Convert to mV
#else
    // --- Simulate ADS1115 Second Channel Reading ---
    // Use different frequency for second channel
    float sineValue2 = cos(simTime * simFrequency * 0.7) * simAmplitude * 0.8;
    float noiseValue2 = random(-(simNoise * 80), (simNoise * 80) + 1) / 100.0; // Different noise profile
    float currentValue2 = simBaseline * 0.9 + sineValue2 + noiseValue2;
#endif
    return currentValue2;
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

    // Optional: Print remaining time here instead of main change detection print?
    // Serial.printf("RefreshIn: %d s\r\n", remainingSeconds);

    if (currentTime - lastDisplayUpdateTime >= currentDisplayUpdateInterval)
    {
        Debug("Starting periodic display update, free heap: %d\r\n", ESP.getFreeHeap());
        
        EPD_13IN3E_Init();
        Debug("re-init after sleep complete\r\n");
        
        Debug("Periodic display update timer elapsed.\r\n");
        
        // Check if Image buffer is valid - it should never be NULL since we allocate it in setup
        if (Image == NULL) {
            Debug("FATAL ERROR: Image buffer is NULL! This should never happen.\r\n");
            Debug("Aborting display update. Will try again next cycle.\r\n");
            return;
        }
        
        // Make sure we're using the same image buffer that was allocated in setup
        Debug("Selecting image buffer for drawing\r\n");
        Paint_SelectImage(Image);
        
        Debug("Reading second sensor value\r\n");
        // Read second sensor value
        float secondValue = readSecondSensorValue();
        Debug("Sensor values: %.2f, %.2f\r\n", currentValue, secondValue);
        
        Debug("Processing ADS buffer values\r\n");
        // Get averaged/processed values 
        float processedValue1 = addToADSBuffer(&adsBuffer1, currentValue);
        float processedValue2 = addToADSBuffer(&adsBuffer2, secondValue);
        Debug("Processed values: %.2f, %.2f\r\n", processedValue1, processedValue2);
        
        Debug("Calling displayMetalCrystal\r\n");
        // Use metal crystal visualization instead of simple text
        displayMetalCrystal(EPD_13IN3E_WIDTH, EPD_13IN3E_HEIGHT, processedValue1, processedValue2);
        
        Debug("Updating display with EPD_13IN3E_Display\r\n");
        EPD_13IN3E_Display(Image);
        Debug("Display update complete\r\n");
        
        lastDisplayUpdateTime = millis(); // Reset the timer *after* the update attempt
        currentDisplayUpdateInterval = random(120000, 180001); // Calculate next random interval
        Debug("Periodic display update complete. Next interval: %lu ms now going to sleep..\r\n", currentDisplayUpdateInterval);
        
        Debug("Entering sleep mode, free heap: %d\r\n", ESP.getFreeHeap());
        EPD_13IN3E_Sleep();
        Debug("Sleeping\r\n");
    }
}

// Displays the provided value as text in the center of the screen
void displaySimpleText(float value) {
    Debug("Updating display with simple text: %.2f\r\n", value);
    
    // Check if Image buffer is valid
    if (Image == NULL) {
        Debug("FATAL ERROR: Image buffer is NULL in displaySimpleText!\r\n");
        return;
    }
    
    // Make sure we're using the correct image buffer
    Paint_SelectImage(Image);
    
    // Clear the buffer
    Paint_Clear(EPD_13IN3E_WHITE);
    Debug("cleared display\r\n");
    
    // Format the value into a string
    char valueStr[20];
    snprintf(valueStr, sizeof(valueStr), "Value: %.2f mV", value);
    Debug("generated string\r\n");

    // Calculate center position (adjust font size as needed)
    // Using Font24 for visibility
    int textWidth = strlen(valueStr) * Font24.Width;
    int textHeight = Font24.Height;
    int centerX = (EPD_13IN3E_WIDTH - textWidth) / 2;
    int centerY = (EPD_13IN3E_HEIGHT - textHeight) / 2;

    // Ensure coordinates are non-negative
    centerX = max(0, centerX);
    centerY = max(0, centerY);

    // Draw the string
    Paint_DrawString_EN(centerX, centerY, valueStr, &Font24, WHITE, BLACK); // Black text on white background
    Debug("Drew string, gonna display!\r\n");
    
    // Update the display
    EPD_13IN3E_Display(Image);
    Debug("Simple text display update command sent.\r\n");
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

// Placeholder - needs actual HSV to RGB math
void HSVtoRGB(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b)
{
    // Simple approximation for now - replace with proper conversion
    if (s == 0)
    {
        r = g = b = v * 255;
        return;
    } // achromatic (grey)
    h /= 60; // sector 0 to 5
    int i = floor(h);
    float f = h - i; // factorial part of h
    float p = v * (1 - s);
    float q = v * (1 - s * f);
    float t = v * (1 - s * (1 - f));
    v *= 255;
    p *= 255;
    q *= 255;
    t *= 255;
    switch (i)
    {
    case 0:
        r = v;
        g = t;
        b = p;
        break;
    case 1:
        r = q;
        g = v;
        b = p;
        break;
    case 2:
        r = p;
        g = v;
        b = t;
        break;
    case 3:
        r = p;
        g = q;
        b = v;
        break;
    case 4:
        r = t;
        g = p;
        b = v;
        break;
    default:
        r = v;
        g = p;
        b = q;
        break; // case 5
    }
}

// Placeholder - needs mapping to EPD_13IN3E color constants
UBYTE mapRGBtoEinkColor(uint8_t r, uint8_t g, uint8_t b)
{
    // Define the basic E-Ink colors in RGB
    // These might need adjustment based on perceived color
    const uint8_t colors[][3] = {
        {0, 0, 0},       // BLACK
        {255, 255, 255}, // WHITE
        {255, 0, 0},     // RED
        {0, 0, 255},     // BLUE
        {0, 255, 0},     // GREEN
        {255, 255, 0},   // YELLOW
        // Add Orange if available for EPD_13IN3E? {255, 165, 0} // ORANGE
        // Add other colors if the display supports them
    };
    const UBYTE einkColors[] = {
        EPD_13IN3E_BLACK,
        EPD_13IN3E_WHITE,
        EPD_13IN3E_RED,
        EPD_13IN3E_BLUE,
        EPD_13IN3E_GREEN,
        EPD_13IN3E_YELLOW,
        // EPD_13IN3E_ORANGE,
    };
    const int numColors = sizeof(einkColors) / sizeof(einkColors[0]);

    long minDistSq = -1;
    int bestColorIndex = 1; // Default to white

    for (int i = 0; i < numColors; ++i)
    {
        long dr = (long)r - colors[i][0];
        long dg = (long)g - colors[i][1];
        long db = (long)b - colors[i][2];
        long distSq = dr * dr + dg * dg + db * db;

        if (minDistSq == -1 || distSq < minDistSq)
        {
            minDistSq = distSq;
            bestColorIndex = i;
        }
    }
    return einkColors[bestColorIndex];
}

// Draws the energy line to the buffer and updates the display
// void drawEnergyLine(float value)
// {
//     if (filledRowCount >= EPD_13IN3E_HEIGHT)
//     {
//         Debug("All rows filled.\r\n");
//         return;
//     }
//
//     // Select a random unfilled row
//     int y;
//     int attempts = 0; // Prevent infinite loop if something is wrong
//     do
//     {
//         y = random(EPD_13IN3E_HEIGHT);
//         attempts++;
//     } while (rowsFilled[y] && attempts < EPD_13IN3E_HEIGHT * 2);
//
//     if (rowsFilled[y])
//     {
//         Debug("Could not find an unfilled row!\r\n");
//         // Maybe find the first available instead?
//         for (y = 0; y < EPD_13IN3E_HEIGHT; ++y)
//         {
//             if (!rowsFilled[y])
//                 break;
//         }
//         if (y == EPD_13IN3E_HEIGHT)
//             return; // Should not happen if check above is correct
//     }
//
//     rowsFilled[y] = true;
//     filledRowCount++;
//     Debug("Drawing line at row %d (%d/%d filled)\r\n", y, filledRowCount, EPD_13IN3E_HEIGHT);
//
//     // Calculate base Hue (0-360) based on historical range
//     float hue = 0;
//     if (historicalMax > historicalMin && historicalMax != historicalMin)
//     { // Avoid division by zero
//         // Map value linearly between min and max to hue range 0-240 (avoiding wrap around magenta for voltage)
//         hue = map(value, historicalMin, historicalMax, 0, 240);
//         hue = constrain(hue, 0, 240); // Ensure it stays within the target range
//     }
//     else
//     {
//         // Default hue if range is zero or invalid (e.g., use middle like green/cyan)
//         hue = 120;
//     }
//
//     // Draw the line pixel by pixel into the buffer
//     float halfWidth = EPD_13IN3E_WIDTH / 2.0;
//     for (int x = 0; x < EPD_13IN3E_WIDTH; ++x)
//     {
//         // Calculate saturation falloff (1.0 in center, maybe 0.2 at edges)
//         float distFromCenter = abs(x - halfWidth);
//         float saturation = 1.0 - (distFromCenter / halfWidth) * 0.8; // Scale factor controls edge saturation
//         saturation = constrain(saturation, 0.2, 1.0);
//
//         // Calculate RGB color (Value/Brightness is 1.0)
//         uint8_t r, g, b;
//         HSVtoRGB(hue, saturation, 1.0, r, g, b);
//
//         // Map to the closest available e-ink color
//         UBYTE einkColor = mapRGBtoEinkColor(r, g, b);
//
//         // Set the pixel in the image buffer
//         // Paint_SetPixel needs the image buffer pointer if not using the global selected one
//         Paint_SetPixel(x, y, einkColor);
//     }
//
//     // Update the full display with the modified buffer
//     Debug("Updating display...\r\n");
//     EPD_13IN3E_Display(Image);
//     lastDisplayUpdateTime = millis();                     // Reset periodic timer after a line draw update
//     currentDisplayUpdateInterval = random(60000, 120001); // Calculate next random interval
//     Debug("Display update complete. Next interval: %lu ms\r\n", currentDisplayUpdateInterval);
// }

#endif // CLEAR_MODE (End wrap for normal loop and helper functions)
