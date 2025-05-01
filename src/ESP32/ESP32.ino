/*
 * ESP32-S3 Residual Current Display
 * 
 * This project visualizes residual current on a 13.3" 6-color E-Ink display using
 * generative algorithms to create growing crystal/metal structures based on energy readings.
 * 
 * MEMORY APPROACH:
 * The ESP32-S3 can't buffer the entire 13.3" display at once, so we:
 * 1. Use a smaller buffer for our generative approach
 * 2. Focus on drawing within the buffer's boundaries
 * 3. Build visual structures over time using simple generative algorithms
 * 
 * COLOR MAPPING:
 * The e-paper display supports 6 colors with the following codes:
 *   - BLACK  (0x0)
 *   - WHITE  (0x1)
 *   - YELLOW (0x2)
 *   - RED    (0x3)
 *   - BLUE   (0x5)
 *   - GREEN  (0x6)
 */

// Define this to use the actual ADS1115 ADC hardware
//#define USE_ADS1115

#ifdef USE_ADS1115
#include <Adafruit_ADS1X15.h>
#include <Wire.h> // Needed for I2C
#endif

// Include our custom e-ink display abstraction layer
#include "EInkDisplay.h"

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

// ADS1115 & I2C Settings (only if using the actual hardware)
#ifdef USE_ADS1115
#define I2C_SDA D3           // Using GPIO6 for SDA
#define I2C_SCL D4           // Using GPIO7 for SCL
TwoWire s3i2c = TwoWire(0);  // Use I2C bus 0
Adafruit_ADS1115 ads;
float V_MULTIPLIER = 0.1875F; // ADS1115 @ +/- 6.144V gain (16-bit results)
#endif

// Create our e-ink display controller
EInkDisplay display;

// Crystal growth parameters
#define MAX_CRYSTAL_POINTS 300
#define POINT_INFLUENCE_RADIUS 150
#define GROWTH_RATE 2

// Generative algorithm parameters
typedef struct {
    int x;
    int y;
    float energy;
    float size;
    UBYTE color;
    bool active;
} CrystalPoint;

CrystalPoint crystalPoints[MAX_CRYSTAL_POINTS];
int pointCount = 0;
float historicalMin = 99999.0;
float historicalMax = -99999.0;

// Simulation mode for testing without hardware
#ifndef USE_ADS1115
float simTime = 0.0;
float simBaseline = 10.0;  // Simulated baseline voltage in mV
float simAmplitude = 50.0; // Amplitude of sine wave in mV
float simFrequency = 0.1;  // Frequency of sine wave
float simNoise = 5.0;      // Max random noise in mV
#endif

// Forward function declarations
float getRandomFloat(float min, float max);
int getRandomInt(int min, int max);
UBYTE getColorForEnergy(float energy);
void addNewCrystalPoint(float energy);
void initializePoint(int index, float energy);
void updateCrystals(float energy);
void renderCrystals();
float readSensorValue();
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("Initializing Residual Current Crystal Visualization");

    // Initialize I2C and ADS1115 (if using real hardware)
    #ifdef USE_ADS1115
    Serial.println("Initializing I2C & ADS1115...");
    s3i2c.begin(I2C_SDA, I2C_SCL, 100000);
    if (!ads.begin(ADS1X15_ADDRESS, &s3i2c)) {
        Serial.println("Failed to initialize ADS. Halting.");
        while (1);
    }
    Serial.println("ADS1115 Initialized.");
    #else
    Serial.println("ADS1115 Simulation Mode");
    #endif

    // Initialize E-Ink Display with our abstraction layer
    Serial.println("Initializing E-Paper Display...");
    if (!display.begin(50000)) { // 50KB buffer size
        Serial.println("Failed to allocate memory buffer! Halting.");
        while (1);
    }
    
    // Clear to white
    display.clear(COLOR_WHITE);
    delay(500);

    // Initialize crystal points array
    for (int i = 0; i < MAX_CRYSTAL_POINTS; i++) {
        crystalPoints[i].active = false;
    }

    Serial.println("Initialization complete. Beginning visualization...");
}

void loop() {
    // Read sensor value
    float currentValue = readSensorValue();

    // Update historical min/max
    if (currentValue < historicalMin) historicalMin = currentValue;
    if (currentValue > historicalMax) historicalMax = currentValue;
    
    // Every 5 seconds, add a new crystal point
    static unsigned long lastAddTime = 0;
    if (millis() - lastAddTime > 5000) {
        addNewCrystalPoint(currentValue);
        lastAddTime = millis();
        Serial.printf("Added new crystal point. Value: %.2f, Total points: %d\n", currentValue, pointCount);
    }
    
    // Update crystal growth based on energy
    updateCrystals(currentValue);
    
    // Render crystals to display
    renderCrystals();
    
    // Debug info
    Serial.printf("Current: %.2f mV, Min: %.2f, Max: %.2f, Active points: %d\n", 
                  currentValue, historicalMin, historicalMax, pointCount);
    
    delay(1000); // Update every second
}

// Read values from the ADS1115 (or simulation)
float readSensorValue() {
#ifdef USE_ADS1115
    int16_t results = ads.readADC_Differential_0_1();
    float currentValue = results * V_MULTIPLIER; // Convert to mV
#else
    // Simulate ADS1115 Reading
    simTime += 0.1; // Increment simulation time
    float sineValue = sin(simTime * simFrequency) * simAmplitude;
    float noiseValue = random(-(simNoise * 100), (simNoise * 100) + 1) / 100.0; // Random noise
    float currentValue = simBaseline + sineValue + noiseValue;
#endif
    return currentValue;
}

// Add a new crystal seed point
void addNewCrystalPoint(float energy) {
    if (pointCount >= MAX_CRYSTAL_POINTS) {
        // If we've reached max points, find an inactive one to replace
        for (int i = 0; i < MAX_CRYSTAL_POINTS; i++) {
            if (!crystalPoints[i].active) {
                initializePoint(i, energy);
                return;
            }
        }
        
        // If all points are active, replace a random one
        int replaceIndex = random(MAX_CRYSTAL_POINTS);
        initializePoint(replaceIndex, energy);
    } else {
        // Add a new point
        initializePoint(pointCount, energy);
        pointCount++;
    }
}

// Initialize a crystal point at the given index
void initializePoint(int index, float energy) {
    // Limit coordinates to buffer size to ensure visibility
    int bufferWidth = 224;  // Standard working buffer size
    int bufferHeight = 224; // Standard working buffer size
    
    crystalPoints[index].x = random(bufferWidth);
    crystalPoints[index].y = random(bufferHeight);
    crystalPoints[index].energy = abs(energy);
    crystalPoints[index].size = 2.0 + abs(energy) / 10.0; // Base size on energy
    crystalPoints[index].color = getColorForEnergy(energy);
    crystalPoints[index].active = true;
}

// Update the growth and state of all crystal points
void updateCrystals(float currentValue) {
    // Normalize current value for growth calculation
    float growthFactor = mapFloat(abs(currentValue), 0, 100, 1, GROWTH_RATE * 2) / 10.0;
    
    for (int i = 0; i < pointCount; i++) {
        if (crystalPoints[i].active) {
            // Grow the crystal based on energy and current value
            crystalPoints[i].size += growthFactor;
            
            // Chance to spawn a new connected crystal point
            if (random(100) < 5 && pointCount < MAX_CRYSTAL_POINTS) {
                // Create a new point near the existing one
                float angle = getRandomFloat(0, 2 * PI);
                float distance = getRandomFloat(5, 20) + crystalPoints[i].size;
                
                // Limit new point to buffer boundaries
                int bufferWidth = 224;  // Standard working buffer size 
                int bufferHeight = 224; // Standard working buffer size
                
                crystalPoints[pointCount].x = constrain(
                    crystalPoints[i].x + cos(angle) * distance, 
                    0, bufferWidth - 1);
                crystalPoints[pointCount].y = constrain(
                    crystalPoints[i].y + sin(angle) * distance, 
                    0, bufferHeight - 1);
                
                // Inherit energy characteristics with some variation
                crystalPoints[pointCount].energy = crystalPoints[i].energy * getRandomFloat(0.8, 1.2);
                crystalPoints[pointCount].size = crystalPoints[i].size * 0.7;
                
                // Occasionally change color for visual interest
                if (random(100) < 30) {
                    crystalPoints[pointCount].color = getColorForEnergy(currentValue);
                } else {
                    crystalPoints[pointCount].color = crystalPoints[i].color;
                }
                
                crystalPoints[pointCount].active = true;
                pointCount++;
            }
            
            // Limit maximum size and chance to become inactive
            if (crystalPoints[i].size > 50 || random(1000) < 2) {
                // 0.2% chance to become inactive each update
                crystalPoints[i].active = false;
            }
        }
    }
}

// Render all crystals to the display
void renderCrystals() {
    // Draw crystals into our buffer
    for (int i = 0; i < pointCount; i++) {
        if (crystalPoints[i].active) {
            // Get crystal properties
            int x = crystalPoints[i].x;
            int y = crystalPoints[i].y;
            int size = (int)crystalPoints[i].size;
            if (size < 1) size = 1;
            UBYTE color = crystalPoints[i].color;
            
            // Choose shape based on energy properties
            int shapeType = ((int)crystalPoints[i].energy) % 4;
            
            switch (shapeType) {
                case 0: // Circle
                    display.drawCircle(x, y, size, color, DOT_PIXEL_1X1, DRAW_FILL_FULL);
                    break;
                    
                case 1: // Diamond
                    display.drawDiamond(x, y, size, color, DOT_PIXEL_1X1, DRAW_FILL_FULL);
                    break;
                    
                case 2: // Rectangle
                    display.drawRectangle(x - size, y - size, x + size, y + size,
                                        color, DOT_PIXEL_1X1, DRAW_FILL_FULL);
                    break;
                    
                case 3: // Star/Cross
                    display.drawStar(x, y, size, color, DOT_PIXEL_2X2);
                    break;
            }
            
            // Add some connection lines between nearby crystals
            for (int j = 0; j < pointCount; j++) {
                if (i != j && crystalPoints[j].active) {
                    // Calculate distance between points
                    float dx = crystalPoints[i].x - crystalPoints[j].x;
                    float dy = crystalPoints[i].y - crystalPoints[j].y;
                    float distance = sqrt(dx * dx + dy * dy);
                    
                    // Connect points that are close but not too close
                    if (distance < POINT_INFLUENCE_RADIUS && distance > 10) {
                        // Line style varies based on energy
                        LINE_STYLE style = (crystalPoints[i].energy > 50) ? 
                                          LINE_STYLE_SOLID : LINE_STYLE_DOTTED;
                        
                        display.drawLine(x, y, crystalPoints[j].x, crystalPoints[j].y, 
                                       color, DOT_PIXEL_1X1, style);
                    }
                }
            }
        }
    }
    
    // Update the display with the buffer contents
    display.updateDisplay();
    
    static unsigned long lastSleepTime = 0;
    if (millis() - lastSleepTime > 30000) { // Every 30 seconds
        // Put display to sleep briefly to preserve it
        display.sleep();
        delay(100);
        display.wakeUp();
        lastSleepTime = millis();
    }
}

// Get an appropriate color based on the energy value
UBYTE getColorForEnergy(float energy) {
    // Different color schemes based on energy value
    if (energy > 80) {
        return (random(100) < 70) ? COLOR_RED : COLOR_YELLOW; // Hot energy
    } else if (energy > 50) {
        return (random(100) < 70) ? COLOR_YELLOW : COLOR_GREEN; // Medium-high energy
    } else if (energy > 20) {
        return (random(100) < 70) ? COLOR_GREEN : COLOR_BLUE; // Medium energy
    } else if (energy > 0) {
        return (random(100) < 70) ? COLOR_BLUE : COLOR_BLACK; // Low energy
    } else if (energy > -20) {
        return (random(100) < 70) ? COLOR_BLACK : COLOR_BLUE; // Negative energy
    } else if (energy > -50) {
        return (random(100) < 70) ? COLOR_BLUE : COLOR_GREEN; // Medium negative energy
    } else {
        return (random(100) < 70) ? COLOR_BLACK : COLOR_RED; // Strong negative energy
    }
}

// Helper function - get random float in range
float getRandomFloat(float min, float max) {
    return min + (max - min) * (random(1000) / 1000.0);
}

// Helper function - get random int in range
int getRandomInt(int min, int max) {
    return random(min, max + 1);
}

// Map function for float values
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}