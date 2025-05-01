#include <Adafruit_ADS1X15.h>
#include <Wire.h> // Needed for I2C
#include <esp_heap_caps.h> // For PSRAM allocation
#include <esp_task_wdt.h> // For watchdog timer management

// Include e-ink display libraries
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
#define I2C_SDA 6           // Using GPIO6 for SDA
#define I2C_SCL 7           // Using GPIO7 for SCL
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
UBYTE *Image = NULL;      // Pointer for the full-screen image buffer
UBYTE *TextImage = NULL;  // Pointer for the small text image buffer (preallocated)
uint16_t TextImageSize = 20000; // Original demo uses this exact size
// EPD_13IN3E_WIDTH = 960, EPD_13IN3E_HEIGHT = 672
// Size calculation: (Width / 2 pixels per byte) * Height
const UWORD IMAGE_BUFFER_SIZE = ((EPD_13IN3E_WIDTH / 2) * EPD_13IN3E_HEIGHT);
bool rowsFilled[EPD_13IN3E_HEIGHT];
int filledRowCount = 0;

// Periodic Update Timer
unsigned long lastDisplayUpdateTime = 0;
unsigned long currentDisplayUpdateInterval = 90000; // Default, will be randomized
unsigned long lastUpdateAttemptTime = 0; // Track when we last tried (even if it crashed)
const unsigned long MIN_RETRY_INTERVAL = 30000; // Don't retry for at least 30 seconds after a potential crash
bool lastUpdateSucceeded = true; // Assume first update will succeed

// --- Simulation Mode ---
#define SIMULATE_ADS false // Set to false when using the real ADS1115

// --- Clear Mode ---
// #define CLEAR_MODE false // Set to true to clear display once and halt

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

// Enable PSRAM support
#if CONFIG_IDF_TARGET_ESP32S3
  #define USE_PSRAM true
#else
  #define USE_PSRAM false
#endif

// Watchdog configuration
#define WDT_TIMEOUT 30  // Watchdog timeout in seconds
#define MAX_BUSY_WAIT 25000 // Maximum time in ms to wait for e-paper busy states

void setup()
{
    Serial.begin(115200); // Use a faster baud rate
    while (!Serial);

    // Configure and start Watchdog Timer
    Debug("Configuring watchdog timer...\r\n");
    esp_task_wdt_init(WDT_TIMEOUT, true); // Enable panic on timeout
    esp_task_wdt_add(NULL); // Add current thread to WDT watch
    Debug("Watchdog started with %d second timeout\r\n", WDT_TIMEOUT);

    // Print all configuration flags
    Debug("====== CONFIGURATION FLAGS ======\r\n");
    #ifdef CLEAR_MODE
    Debug("CLEAR_MODE: defined\r\n");
    #else
    Debug("CLEAR_MODE: not defined\r\n");
    #endif

    #if SIMULATE_ADS
    Debug("SIMULATE_ADS: true\r\n");
    #else
    Debug("SIMULATE_ADS: false\r\n");
    #endif

    #if USE_PSRAM
    Debug("USE_PSRAM: true\r\n");
    #else
    Debug("USE_PSRAM: false\r\n");
    #endif

    Debug("IMAGE_BUFFER_SIZE: %d bytes\r\n", IMAGE_BUFFER_SIZE);
    Debug("EPD_13IN3E_WIDTH: %d px\r\n", EPD_13IN3E_WIDTH);
    Debug("EPD_13IN3E_HEIGHT: %d px\r\n", EPD_13IN3E_HEIGHT);
    Debug("UPDATE_INTERVAL: %d-%d seconds\r\n", 60, 120);
    Debug("================================\r\n");

    #if USE_PSRAM
    // Check if PSRAM is enabled
    if (psramInit()) {
        Debug("PSRAM is enabled and available\r\n");
        Debug("Total PSRAM: %d bytes\r\n", ESP.getPsramSize());
        Debug("Free PSRAM: %d bytes\r\n", ESP.getFreePsram());
    } else {
        Debug("PSRAM is not available. Falling back to regular memory\r\n");
    }
    #endif

    #ifdef CLEAR_MODE
    // --- Clear Mode Logic ---
    Debug("Entering Clear Mode...\r\n");
    Debug("Initializing E-Paper Module...\r\n");
    Debug("Before DEV_Module_Init()...\r\n");
    DEV_Module_Init();
    Debug("After DEV_Module_Init()...\r\n");
    esp_task_wdt_reset(); // Feed watchdog
    delay(100);
    Debug("Initializing E-Paper Display...\r\n");
    Debug("Before EPD_13IN3E_Init()...\r\n");
    EPD_13IN3E_Init();
    Debug("After EPD_13IN3E_Init()...\r\n");
    esp_task_wdt_reset(); // Feed watchdog
    delay(500); // Give some time for initialization to settle
    Debug("Before EPD_13IN3E_Clear()...\r\n");
    EPD_13IN3E_Clear(EPD_13IN3E_WHITE); // Clear display physically
    Debug("After sending clear command...\r\n");
    esp_task_wdt_reset(); // Feed watchdog
    Debug("Waiting for clear operation to complete (with timeout)...\r\n");
    unsigned long clearStartTime = millis();
    bool clearTimedOut = false;
    while (DEV_Digital_Read(EPD_BUSY_PIN) == 0) {
        esp_task_wdt_reset();
        if (millis() - clearStartTime > MAX_BUSY_WAIT) {
            Debug("Clear operation TIMED OUT after %d ms!\r\n", MAX_BUSY_WAIT);
            clearTimedOut = true;
            break;
        }
        delay(100);
    }
    Debug("After EPD_13IN3E_Clear()...\r\n");
    esp_task_wdt_reset(); // Feed watchdog
    delay(1000); // Longer delay after clear
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
    Debug("Before DEV_Module_Init()...\r\n");
    DEV_Module_Init(); // Initializes GPIOs based on EPD_Driver.h pins
    Debug("After DEV_Module_Init()...\r\n");
    esp_task_wdt_reset(); // Feed watchdog
    delay(100);

    Debug("Initializing E-Paper Display...\r\n");
    Debug("Before EPD_13IN3E_Init()...\r\n");
    EPD_13IN3E_Init();
    Debug("After EPD_13IN3E_Init()...\r\n");
    esp_task_wdt_reset(); // Feed watchdog
    delay(500); // Give some time for initialization to settle

    Debug("Before EPD_13IN3E_Clear()...\r\n");
    EPD_13IN3E_Clear(EPD_13IN3E_WHITE); // Clear display physically
    Debug("After EPD_13IN3E_Clear()...\r\n");
    esp_task_wdt_reset(); // Feed watchdog
    delay(1000); // Longer delay after clear

    // --- Create and Initialize Image Buffer ---
    Debug("Allocating Image Buffer...\r\n");
    Debug("IMAGE_BUFFER_SIZE: %d bytes (calculated)\r\n", IMAGE_BUFFER_SIZE);
    uint32_t maxPsramToUse = ESP.getFreePsram() - 100000; // Leave 100KB safety margin
    Debug("Maximum safe PSRAM to use: %d bytes\r\n", maxPsramToUse);

    #if USE_PSRAM
    Debug("Using PSRAM for image buffer allocation\r\n");
    if((Image = (UBYTE *)heap_caps_malloc(IMAGE_BUFFER_SIZE, MALLOC_CAP_SPIRAM)) == NULL) {
    #else
    Debug("Using standard memory for image buffer allocation\r\n");
    if((Image = (UBYTE *)malloc(IMAGE_BUFFER_SIZE)) == NULL) {
    #endif
        Debug("Failed to allocate memory for image buffer! Halting.\r\n");
        // Consider DEV_Module_Exit(); ?
        while (1)
            ;
    }
    Debug("Image Buffer Allocated. Size: %d bytes\r\n", IMAGE_BUFFER_SIZE);
    Debug("Image Buffer Address: 0x%08X\r\n", (uint32_t)Image);
    Debug("Before Paint_NewImage()...\r\n");
    Paint_NewImage(Image, EPD_13IN3E_WIDTH, EPD_13IN3E_HEIGHT, 0, WHITE); // Set buffer dimensions and default rotation/color
    Debug("After Paint_NewImage()...\r\n");
    Debug("Before Paint_SelectImage()...\r\n");
    Paint_SelectImage(Image);                                             // Point drawing functions to this buffer
    Debug("After Paint_SelectImage()...\r\n");
    Debug("Before Paint_SetRotate()...\r\n");
    Paint_SetRotate(ROTATE_90); // Apply rotation to match physical display orientation
    Debug("After Paint_SetRotate()...\r\n");
    Debug("Before Paint_Clear()...\r\n");
    Paint_Clear(WHITE);                                                   // Fill the buffer with white
    Debug("After Paint_Clear()...\r\n");

    // --- Pre-allocate the text buffer to avoid memory allocation during updates ---
    Debug("Pre-allocating text buffer...\r\n");
    #if USE_PSRAM
    Debug("Using PSRAM for text canvas allocation\r\n");
    if((TextImage = (UBYTE *)heap_caps_malloc(TextImageSize, MALLOC_CAP_SPIRAM)) == NULL) {
    #else
    Debug("Using standard memory for text canvas allocation\r\n");
    if((TextImage = (UBYTE *)malloc(TextImageSize)) == NULL) {
    #endif
        Debug("Failed to allocate text image buffer! Continuing without text display.\r\n");
        // Continue without text capability but don't halt
    } else {
        Debug("Text Buffer Allocated. Size: %d bytes\r\n", TextImageSize);
        Debug("Text Buffer Address: 0x%08X\r\n", (uint32_t)TextImage);
    }

    // --- Initialize Row Tracking ---
    Debug("Initializing Row Tracking...\r\n");
    for (int i = 0; i < EPD_13IN3E_HEIGHT; i++)
    {
        rowsFilled[i] = false;
    }
    filledRowCount = 0;

    Debug("Displaying Initial Blank Screen...\r\n");
    // Use factory-approved sequence for clearing the screen
    // Don't touch the large buffer
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
        Paint_SelectImage(Image); // Ensure drawing happens in the buffer
        Paint_Clear(WHITE);       // Clear the image buffer
        EPD_13IN3E_Display(Image); // Update the physical display to blank

        // Reset tracking variables
        filledRowCount = 0;
        for (int i = 0; i < EPD_13IN3E_HEIGHT; i++) {
            rowsFilled[i] = false;
        }

        // Reset change detection state optionally?
        // bufferFilled = false; 
        // baselineEstablished = false; 
        // currentThreshold = MIN_THRESHOLD;
        // historicalMin = 99999.0; // Reset historical? Maybe not desirable.
        // historicalMax = -99999.0;

        DEV_Delay_ms(1000); // Pause briefly after clearing
        lastDisplayUpdateTime = millis(); // Reset update timer after clear
        // No return here, continue the loop
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
    esp_task_wdt_reset(); // Feed watchdog
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastDisplayUpdateTime;
    // Prevent underflow if millis() has rolled over
    unsigned long remainingTimeMs = (elapsedTime > currentDisplayUpdateInterval) ? 0 : currentDisplayUpdateInterval - elapsedTime;
    int remainingSeconds = remainingTimeMs / 1000;
    unsigned long timeSinceLastAttempt = currentTime - lastUpdateAttemptTime;

    // Safety check - if the previous update attempt was recent and failed, wait longer
    if (!lastUpdateSucceeded && timeSinceLastAttempt < MIN_RETRY_INTERVAL) {
        Debug("Skipping update - last attempt may have crashed, waiting %d more seconds before retry\r\n", 
              (MIN_RETRY_INTERVAL - timeSinceLastAttempt) / 1000);
        return;
    }

    // Optional: Print remaining time here instead of main change detection print?
    // Serial.printf("RefreshIn: %d s\r\n", remainingSeconds);

    if (currentTime - lastDisplayUpdateTime >= currentDisplayUpdateInterval)
    {
        lastUpdateAttemptTime = currentTime; // Mark when we're attempting this update
        lastUpdateSucceeded = false; // Set to false now, will be set to true if this completes

        esp_task_wdt_reset(); // Feed watchdog
        // Removed filledRowCount check - no longer relevant
        EPD_13IN3E_Init();
        Debug("re-init after sleep\r\n");
        esp_task_wdt_reset(); // Feed watchdog
        Debug("Periodic display update timer elapsed.\r\n");
        displaySimpleText(currentValue); // Update with simple text
        lastDisplayUpdateTime = currentTime; // Reset the timer *after* the update attempt
        currentDisplayUpdateInterval = random(60000, 120001); // Calculate next random interval
        Debug("Periodic display update complete. Next interval: %lu ms now going to sleep..\r\n", currentDisplayUpdateInterval);
        esp_task_wdt_reset(); // Feed watchdog
        EPD_13IN3E_Sleep();
        Debug("Sleeping\r\n");
        lastUpdateSucceeded = true; // If we got here, update succeeded
    }
    esp_task_wdt_reset(); // Feed watchdog
}

// Displays the provided value as text in the center of the screen
void displaySimpleText(float value) {
    Debug("Updating display with simple text: %.2f\r\n", value);
    esp_task_wdt_reset(); // Feed watchdog

    // Check if we have the pre-allocated text buffer
    if (TextImage == NULL) {
        Debug("No text buffer available, skipping text display\r\n");
        return;
    }
    Debug("Using pre-allocated text buffer at address: 0x%08X\r\n", (uint32_t)TextImage);
    
    // Match the exact initialization from the demo
    Debug("Creating small text image buffer\r\n");
    Debug("Before Paint_NewImage()...\r\n");
    Paint_NewImage(TextImage, 200, 200, 0, WHITE);
    Debug("After Paint_NewImage()...\r\n");
    Debug("Before Paint_SelectImage()...\r\n");
    Paint_SelectImage(TextImage);
    Debug("After Paint_SelectImage()...\r\n");
    Debug("Before Paint_SetRotate()...\r\n");
    Paint_SetRotate(ROTATE_90); // Add rotation to match physical display orientation
    Debug("After Paint_SetRotate()...\r\n");
    Debug("Before Paint_Clear()...\r\n");
    Paint_Clear(WHITE);
    Debug("After Paint_Clear()...\r\n");
    
    // Simple drawing similar to the demo
    char valueStr[32];
    snprintf(valueStr, sizeof(valueStr), "Value: %.2f mV", value);
    Debug("Value string created: %s\r\n", valueStr);
    
    // Add basic drawings from the demo
    Debug("Drawing test points...\r\n");
    Paint_DrawPoint(10, 80, BLACK, DOT_PIXEL_1X1, DOT_STYLE_DFT);
    Paint_DrawPoint(10, 90, BLACK, DOT_PIXEL_2X2, DOT_STYLE_DFT);
    Paint_DrawPoint(10, 100, BLACK, DOT_PIXEL_3X3, DOT_STYLE_DFT);
    
    // Draw a box
    Debug("Drawing rectangle...\r\n");
    Paint_DrawRectangle(20, 70, 70, 120, BLACK, DOT_PIXEL_1X1, DRAW_FILL_EMPTY);
    
    // Draw the text
    Debug("Drawing text...\r\n");
    Paint_DrawString_EN(30, 30, valueStr, &Font16, BLACK, WHITE);
    Paint_DrawString_EN(30, 50, "RC Monitor", &Font16, BLACK, WHITE);
    Debug("All drawing completed.\r\n");
    
    // Display at the exact position used in the demo
    Debug("Displaying part at position (400, 500)\r\n");
    Debug("Before EPD_13IN3E_DisplayPart()...\r\n");
    esp_task_wdt_reset(); // Feed watchdog
    EPD_13IN3E_DisplayPart(TextImage, 400, 500, 200, 200);

    // Wait for busy with timeout
    Debug("Waiting for display part operation to complete (with timeout)...\r\n");
    unsigned long displayStartTime = millis();
    bool displayTimedOut = false;
    while (DEV_Digital_Read(EPD_BUSY_PIN) == 0) {
        esp_task_wdt_reset(); // Feed watchdog
        if (millis() - displayStartTime > MAX_BUSY_WAIT) {
            Debug("Display part operation TIMED OUT after %d ms!\r\n", MAX_BUSY_WAIT);
            displayTimedOut = true;
            break;
        }
        delay(100);
    }

    Debug("After EPD_13IN3E_DisplayPart()...\r\n");
    esp_task_wdt_reset(); // Feed watchdog
    
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