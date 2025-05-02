#include "Debug.h"
#include "DEV_Config.h"
#include "fonts.h"
#include <math.h> // Add math.h for sin, cos, sqrt functions
#include <limits.h> // For INT_MAX constant
#include <stdlib.h> // For malloc and free
#include <stdio.h> // For sprintf

// Define M_PI if not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Forward declarations
void displaySimpleText(float value);

// Voronoi seed point structure
typedef struct {
    int x;
    int y;
    int energy;
} SeedPoint;

// ADS value buffer for stable readings
#define ADS_BUFFER_SIZE 10
typedef struct {
    float values[ADS_BUFFER_SIZE];
    int index;
    bool filled;
    float prevUpdateValue;
    unsigned long lastUpdateTime;
} ADSBuffer;

// Initialize ADS buffers
ADSBuffer adsBuffer1 = {{0}, 0, false, 0, 0};
ADSBuffer adsBuffer2 = {{0}, 0, false, 0, 0};

// Add value to ADS buffer and return average
float addToADSBuffer(ADSBuffer* buffer, float value) {
    // Add the new value
    buffer->values[buffer->index] = value;
    buffer->index = (buffer->index + 1) % ADS_BUFFER_SIZE;
    
    if (buffer->index == 0) {
        buffer->filled = true;
    }
    
    // Calculate average
    float sum = 0;
    int count = buffer->filled ? ADS_BUFFER_SIZE : buffer->index;
    
    if (count == 0) return 0;
    
    for (int i = 0; i < count; i++) {
        sum += buffer->values[i];
    }
    return sum / count;
}

// Calculate median from buffer
float getMedianFromBuffer(ADSBuffer* buffer) {
    float tempValues[ADS_BUFFER_SIZE];
    int count = buffer->filled ? ADS_BUFFER_SIZE : buffer->index;
    
    if (count == 0) return 0;
    
    // Copy values to temporary array
    for (int i = 0; i < count; i++) {
        tempValues[i] = buffer->values[i];
    }
    
    // Simple bubble sort (efficient enough for small arrays)
    for (int i = 0; i < count - 1; i++) {
        for (int j = 0; j < count - i - 1; j++) {
            if (tempValues[j] > tempValues[j + 1]) {
                float temp = tempValues[j];
                tempValues[j] = tempValues[j + 1];
                tempValues[j + 1] = temp;
            }
        }
    }
    
    // Return median
    if (count % 2 == 0) {
        return (tempValues[count/2 - 1] + tempValues[count/2]) / 2.0;
    } else {
        return tempValues[count/2];
    }
}

// Calculate distance between two points
int calculateDistance(int x1, int y1, int x2, int y2) {
    int dx = x2 - x1;
    int dy = y2 - y1;
    return sqrt(dx*dx + dy*dy);
}

// Find closest seed point to given coordinates
int findClosestSeedPoint(int x, int y, SeedPoint* seeds, int seedCount) {
    int closestIndex = 0;
    int minDistance = calculateDistance(x, y, seeds[0].x, seeds[0].y);
    
    for (int i = 1; i < seedCount; i++) {
        int distance = calculateDistance(x, y, seeds[i].x, seeds[i].y);
        if (distance < minDistance) {
            minDistance = distance;
            closestIndex = i;
        }
    }
    
    return closestIndex;
}

// Map energy level to color
UBYTE energyToColor(int energy) {
    // Map energy levels to different colors
    if (energy < 25) {
        return EPD_13IN3E_BLUE;     // Low energy = blue
    } else if (energy < 50) {
        return EPD_13IN3E_GREEN;    // Medium-low energy = green
    } else if (energy < 75) {
        return EPD_13IN3E_RED;      // Medium-high energy = red
    } else if (energy < 100) {
        return EPD_13IN3E_YELLOW;   // High energy = yellow
    } else {
        return EPD_13IN3E_BLACK;    // Very high energy = black
    }
}

// Generate Voronoi cell boundaries
void drawVoronoiCells(SeedPoint* seeds, int seedCount, int width, int height, float energyFactor) {
    Debug("Starting drawVoronoiCells with %d seeds\r\n", seedCount);
    
    // Step size for boundary detection (smaller = more detailed but slower)
    int step = 8; // Increased step size for better performance on 1200x1600 display
    
    // Calculate actual buffer size needed - only need entries at step intervals
    int bufferSize = (width + step - 1) / step;
    Debug("Allocating memory for row buffers, reduced size=%d (full width=%d)\r\n", bufferSize, width);
    
    // Create a small buffer to track cell assignments for the current and previous rows
    // Only allocate what we need based on the step size
    int *currentRow = (int*)malloc(bufferSize * sizeof(int));
    int *previousRow = (int*)malloc(bufferSize * sizeof(int));
    
    if (!currentRow || !previousRow) {
        // Handle allocation failure
        Debug("ERROR: Failed to allocate memory for row buffers!\r\n");
        if (currentRow) free(currentRow);
        if (previousRow) free(previousRow);
        return;
    }
    
    Debug("Initializing first row\r\n");
    // Initialize the first row - using the proper buffer indexing
    for (int i = 0, x = 0; x < width; i++, x += step) {
        currentRow[i] = findClosestSeedPoint(x, 0, seeds, seedCount);
    }
    
    Debug("Processing rows to find boundaries\r\n");
    // Process rows and find boundaries
    for (int y = step; y < height; y += step) {
        // Swap buffers
        int *temp = previousRow;
        previousRow = currentRow;
        currentRow = temp;
        
        // Calculate current row - using proper buffer indexing
        for (int i = 0, x = 0; x < width; i++, x += step) {
            currentRow[i] = findClosestSeedPoint(x, y, seeds, seedCount);
            
            // Check for vertical boundary
            if (y >= step && currentRow[i] != previousRow[i]) {
                // Vertical boundary detected
                int closestIndex = currentRow[i];
                int energy = seeds[closestIndex].energy;
                UBYTE color = energyToColor(energy);
                
                // Draw boundary pixels
                for (int di = -1; di <= 1; di++) {
                    for (int dj = -1; dj <= 1; dj++) {
                        int bx = x + di;
                        int by = y + dj;
                        // Check bounds
                        if (bx >= 0 && bx < width && by >= 0 && by < height) {
                            Paint_SetPixel(bx, by, color);
                        }
                    }
                }
            }
            
            // Check for horizontal boundary if not at leftmost column
            if (i > 0) {
                if (currentRow[i] != currentRow[i-1]) {
                    // Horizontal boundary detected
                    int closestIndex = currentRow[i];
                    int energy = seeds[closestIndex].energy;
                    UBYTE color = energyToColor(energy);
                    
                    // Draw boundary pixels
                    for (int di = -1; di <= 1; di++) {
                        for (int dj = -1; dj <= 1; dj++) {
                            int bx = x + di;
                            int by = y + dj;
                            // Check bounds
                            if (bx >= 0 && bx < width && by >= 0 && by < height) {
                                Paint_SetPixel(bx, by, color);
                            }
                        }
                    }
                }
            }
        }
        
        // Debug output every 200 rows to show progress
        if (y % 200 == 0) {
            Debug("Processed row %d of %d\r\n", y, height);
        }
    }
    
    Debug("Freeing row buffers\r\n");
    // Free the buffers
    free(currentRow);
    free(previousRow);
    
    Debug("drawVoronoiCells completed\r\n");
}

// Add internal structure to Voronoi cells
void addCrystalStructure(SeedPoint* seeds, int seedCount, int width, int height) {
    Debug("Starting addCrystalStructure\r\n");
    
    // Limit the maximum number of seeds we'll process for performance
    const int maxProcessSeeds = 50;
    int seedsToProcess = (seedCount > maxProcessSeeds) ? maxProcessSeeds : seedCount;
    
    Debug("Processing %d of %d seeds for crystal structure\r\n", seedsToProcess, seedCount);
    
    // Draw lines from each seed point to create internal structure
    for (int i = 0; i < seedsToProcess; i++) {
        // Find a few closest neighbors to connect to
        int connections = 2 + (seeds[i].energy / 40); // Reduced number of connections
        if (connections > 3) connections = 3; // Maximum of 3 connections
        
        // Simple array to track connected seeds
        int connected[3] = {-1, -1, -1};
        
        // Find connections
        for (int c = 0; c < connections; c++) {
            int closestDist = INT_MAX;
            int closestIndex = -1;
            
            // Find next closest seed not already connected
            for (int j = 0; j < seedCount; j++) {
                if (i == j) continue; // Skip self
                
                // Skip already connected
                bool alreadyConnected = false;
                for (int k = 0; k < c; k++) {
                    if (connected[k] == j) {
                        alreadyConnected = true;
                        break;
                    }
                }
                if (alreadyConnected) continue;
                
                // Calculate distance
                int dist = calculateDistance(seeds[i].x, seeds[i].y, seeds[j].x, seeds[j].y);
                if (dist < closestDist) {
                    closestDist = dist;
                    closestIndex = j;
                }
            }
            
            // If found, draw connection and store
            if (closestIndex >= 0) {
                connected[c] = closestIndex;
                
                // Line thickness based on energy
                DOT_PIXEL thickness = DOT_PIXEL_1X1;
                if (seeds[i].energy > 70) thickness = DOT_PIXEL_2X2;
                if (seeds[i].energy > 120) thickness = DOT_PIXEL_3X3;
                
                // Line color based on average energy of two connected points
                int avgEnergy = (seeds[i].energy + seeds[closestIndex].energy) / 2;
                UBYTE color = energyToColor(avgEnergy);
                
                // Draw the connection
                Paint_DrawLine(seeds[i].x, seeds[i].y, 
                               seeds[closestIndex].x, seeds[closestIndex].y, 
                               color, thickness, LINE_STYLE_SOLID);
            }
        }
    }
    
    Debug("addCrystalStructure completed\r\n");
}

// Fill some Voronoi cells with color
void fillSelectedCells(SeedPoint* seeds, int seedCount, int width, int height) {
    Debug("Filling selected cells with color\r\n");
    
    // Only fill cells for seeds with high enough energy (to avoid filling everything)
    // and limit the number of filled cells for performance
    int maxFillCells = min(10, seedCount);
    int fillCount = 0;
    
    // First pass: count high energy cells
    for (int i = 0; i < seedCount && fillCount < maxFillCells; i++) {
        // Only fill cells with high enough energy
        if (seeds[i].energy > 80) {
            fillCount++;
            
            // Get the color based on energy
            UBYTE fillColor = energyToColor(seeds[i].energy);
            
            // Define the area to check around this seed (don't check the entire image for performance)
            int radius = calculateDistance(0, 0, width, height) / 5; // Use 1/5 of the diagonal as max radius
            int startX = max(0, seeds[i].x - radius);
            int startY = max(0, seeds[i].y - radius);
            int endX = min(width - 1, seeds[i].x + radius);
            int endY = min(height - 1, seeds[i].y + radius);
            
            // Check points in this area
            for (int y = startY; y <= endY; y += 4) { // Sample every 4 pixels for speed
                for (int x = startX; x <= endX; x += 4) {
                    // If this point is closest to our seed, fill it
                    int closestSeed = findClosestSeedPoint(x, y, seeds, seedCount);
                    if (closestSeed == i) {
                        Paint_SetPixel(x, y, fillColor);
                    }
                }
            }
            
            Debug("Filled cell %d with color\r\n", i);
        }
    }
    
    Debug("Cell filling complete\r\n");
}

// Display metal crystal visualization using Voronoi diagram
void displayMetalCrystal(int width, int height, float adsValue1, float adsValue2) {
    Debug("Starting displayMetalCrystal with dimensions: %d x %d\r\n", width, height);
    Debug("ADS values: %.2f, %.2f\r\n", adsValue1, adsValue2);
    
    // Calculate change since last update
    float energy1Change = 0;
    float energy2Change = 0;
    
    unsigned long currentTime = millis();
    if (adsBuffer1.lastUpdateTime > 0) {
        energy1Change = adsValue1 - adsBuffer1.prevUpdateValue;
        energy2Change = adsValue2 - adsBuffer2.prevUpdateValue;
    }
    
    // Store current values for next update comparison
    adsBuffer1.prevUpdateValue = adsValue1;
    adsBuffer2.prevUpdateValue = adsValue2;
    adsBuffer1.lastUpdateTime = currentTime;
    adsBuffer2.lastUpdateTime = currentTime;
    
    Debug("Clearing display buffer\r\n");
    // Clear the display - use the currently selected image buffer
    Paint_Clear(WHITE);
    
    // Number of seed points based on ADS value 1
    // Scale value to reasonable range (20-80 points)
    int seedCount = 30 + abs(adsValue1) / 10;
    if (seedCount < 20) seedCount = 20;
    if (seedCount > 80) seedCount = 80;
    
    Debug("Allocating memory for %d seed points\r\n", seedCount);
    // Allocate seed points
    SeedPoint* seeds = (SeedPoint*)malloc(seedCount * sizeof(SeedPoint));
    if (seeds == NULL) {
        // Handle memory allocation failure
        Debug("ERROR: Failed to allocate memory for seeds!\r\n");
        displaySimpleText(-999.99); // Use a special error value
        return;
    }
    
    Debug("Generating seed points\r\n");
    // Generate seed points
    // Use deterministic pattern influenced by ADS values
    for (int i = 0; i < seedCount; i++) {
        // Use deterministic placement but influenced by values
        float angle = (360.0 * i / seedCount) + (adsValue1 * 3.6);
        float radius = (height < width ? height : width) * 0.4 * (0.6 + 0.4 * sin(i * 0.1 + adsValue2 * 0.01));
        
        seeds[i].x = width / 2 + radius * cos(angle * M_PI / 180.0);
        seeds[i].y = height / 2 + radius * sin(angle * M_PI / 180.0);
        
        // Energy level influenced by the second ADS value and the change
        seeds[i].energy = 50 + 
                        adsValue2 * 3 + 
                        sin(i * 0.7) * 30 + 
                        energy2Change * 10;
                        
        // Ensure energy is within reasonable range
        if (seeds[i].energy < 10) seeds[i].energy = 10;
        if (seeds[i].energy > 150) seeds[i].energy = 150;
    }
    
    // Energy factor influenced by ADS value 2
    float energyFactor = 0.5 + abs(adsValue2) / 100.0;
    
    // Growth direction based on change (add subtle offset to seed positions)
    float xShift = energy1Change * 3;
    float yShift = energy2Change * 3;
    
    for (int i = 0; i < seedCount; i++) {
        seeds[i].x += xShift + sin(seeds[i].energy * 0.1) * 5;
        seeds[i].y += yShift + cos(seeds[i].energy * 0.1) * 5;
        
        // Ensure coordinates stay within bounds
        if (seeds[i].x < 0) seeds[i].x = 0;
        if (seeds[i].x >= width) seeds[i].x = width - 1;
        if (seeds[i].y < 0) seeds[i].y = 0;
        if (seeds[i].y >= height) seeds[i].y = height - 1;
    }
    
    // Fill some cell interiors with color (do this before drawing cell boundaries)
    Debug("Filling selected cells\r\n");
    fillSelectedCells(seeds, seedCount, width, height);
    
    Debug("Drawing Voronoi cells\r\n");
    // Draw Voronoi cells
    drawVoronoiCells(seeds, seedCount, width, height, energyFactor);
    
    Debug("Adding crystal structure\r\n");
    // Add crystalline internal structure
    addCrystalStructure(seeds, seedCount, width, height);
    
    Debug("Drawing energy centers\r\n");
    // Draw energy centers at seed points
    for (int i = 0; i < seedCount; i++) {
        // Draw circles with varying size based on energy
        DOT_PIXEL radius = DOT_PIXEL_1X1;
        if (seeds[i].energy > 50) radius = DOT_PIXEL_2X2;
        if (seeds[i].energy > 100) radius = DOT_PIXEL_3X3;
        
        // Use energy-based color for the center
        UBYTE color = energyToColor(seeds[i].energy);
        
        Paint_DrawCircle(seeds[i].x, seeds[i].y, 
                         radius * 2, color, 
                         radius, DRAW_FILL_FULL);
    }
    
    Debug("Drawing status text\r\n");
    // Draw status text with ADS values
    char statusText[100];
    sprintf(statusText, "ADS Values: %.1f | %.1f mV", adsValue1, adsValue2);
    Paint_DrawString_EN(10, 10, statusText, &Font16, BLACK, WHITE);
    
    // Add change information
    char changeText[100];
    sprintf(changeText, "Change: %.1f | %.1f mV", energy1Change, energy2Change);
    Paint_DrawString_EN(10, 40, changeText, &Font16, BLACK, WHITE);
    
    // Draw timestamp
    char timeInfo[50];
    unsigned long uptime = millis() / 1000; // seconds
    unsigned long hours = uptime / 3600;
    unsigned long minutes = (uptime % 3600) / 60;
    unsigned long seconds = uptime % 60;
    sprintf(timeInfo, "Uptime: %02lu:%02lu:%02lu", hours, minutes, seconds);
    Paint_DrawString_EN(10, 70, timeInfo, &Font16, BLACK, WHITE);
    
    Debug("Freeing seed memory\r\n");
    // Free allocated memory
    free(seeds);
    
    Debug("displayMetalCrystal completed successfully\r\n");
} 