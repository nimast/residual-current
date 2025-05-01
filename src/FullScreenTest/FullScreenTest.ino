/*
 * ESP32-S3 Full Screen E-Paper Test
 * 
 * This test displays three color bands across the entire 1600×1200 screen
 * using PSRAM to allocate the buffer memory.
 */

#include "EPD_13in3e.h"
#include "GUI_Paint.h"

// ESP32-S3 has 2MB of PSRAM, which is enough for our full-screen buffer
// We need to enable PSRAM in the Arduino IDE settings:
// Tools > PSRAM: "OPI PSRAM"

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("Full Screen E-Paper Test");
    
    // Initialize the display module
    DEV_Module_Init();
    
    // Initialize the e-Paper
    Serial.println("Initializing e-Paper...");
    EPD_13IN3E_Init();
    
    // Clear the display (white)
    Serial.println("Clearing display...");
    EPD_13IN3E_Clear(EPD_13IN3E_WHITE);
    DEV_Delay_ms(500);
    
    // Calculate buffer size for the full screen
    // The E-Paper display is 1600×1200, but the buffer format requires space
    // See the Waveshare documentation for details on the format
    UWORD Imagesize = ((EPD_13IN3E_WIDTH % 2 == 0) ? 
                     (EPD_13IN3E_WIDTH / 2) : 
                     (EPD_13IN3E_WIDTH / 2 + 1)) * EPD_13IN3E_HEIGHT;
    
    Serial.printf("Buffer size needed: %d bytes\n", Imagesize);
    
    // Allocate buffer in PSRAM
    UBYTE *Image;
    
    // Allocate using ps_malloc which uses PSRAM
    if((Image = (UBYTE *)ps_malloc(Imagesize)) == NULL) {
        Serial.println("Failed to allocate memory in PSRAM!");
        while(1);
    }
    
    Serial.println("Memory allocated in PSRAM successfully");
    
    // Initialize Paint with full-screen dimensions
    Paint_NewImage(Image, EPD_13IN3E_WIDTH, EPD_13IN3E_HEIGHT, 0, EPD_13IN3E_WHITE);
    
    // Select the image
    Paint_SelectImage(Image);
    
    // Clear the image
    Paint_Clear(EPD_13IN3E_WHITE);
    
    // Draw three horizontal color bands
    Serial.println("Drawing color bands...");
    
    // Top third: RED
    int bandHeight = EPD_13IN3E_HEIGHT / 3;
    for (int y = 0; y < bandHeight; y++) {
        for (int x = 0; x < EPD_13IN3E_WIDTH; x++) {
            Paint_SetPixel(x, y, EPD_13IN3E_RED);
        }
    }
    
    // Middle third: GREEN
    for (int y = bandHeight; y < 2 * bandHeight; y++) {
        for (int x = 0; x < EPD_13IN3E_WIDTH; x++) {
            Paint_SetPixel(x, y, EPD_13IN3E_GREEN);
        }
    }
    
    // Bottom third: BLUE
    for (int y = 2 * bandHeight; y < EPD_13IN3E_HEIGHT; y++) {
        for (int x = 0; x < EPD_13IN3E_WIDTH; x++) {
            Paint_SetPixel(x, y, EPD_13IN3E_BLUE);
        }
    }
    
    // Display the image
    Serial.println("Sending image to display (this may take a while)...");
    EPD_13IN3E_Display(Image);
    Delay(3000);
    // Free the memory
    free(Image);
    
    // Put display to sleep
    Serial.println("Display complete, entering sleep mode...");
    EPD_13IN3E_Sleep();
    
    Serial.println("Test complete!");
}

void loop() {
    // Nothing to do here
    delay(1000);
}