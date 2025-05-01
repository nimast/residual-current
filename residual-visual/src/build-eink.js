/**
 * build-eink.js
 * 
 * Converts a p5.js sketch to e-ink display code for ESP32
 * Usage: node build-eink.js <path-to-sketch>
 */

const fs = require('fs');
const path = require('path');
const acorn = require('acorn');
const walk = require('acorn-walk');

// Check if we have a sketch file path
const sketchPath = process.argv[2];
if (!sketchPath) {
  console.error('Please provide a path to a p5.js sketch file');
  process.exit(1);
}

// Check if the file exists
if (!fs.existsSync(sketchPath)) {
  console.error(`File not found: ${sketchPath}`);
  process.exit(1);
}

// Read the sketch file
const sketchCode = fs.readFileSync(sketchPath, 'utf-8');

// Parse the sketch code using acorn
let ast;
try {
  ast = acorn.parse(sketchCode, { 
    ecmaVersion: 2020,
    sourceType: 'script',
    locations: true
  });
} catch (error) {
  console.error(`Error parsing sketch code: ${error.message}`);
  process.exit(1);
}

// Objects to store setup and draw function information
const setupInfo = {
  canvasWidth: 600,
  canvasHeight: 800,
  hasBackground: false,
  backgroundColor: 255, // Default white
  strokeWeight: 1,
  hasFill: true,
  fillColor: 0 // Default black
};

// Array to store drawing commands
const drawCommands = [];

// Function to map p5.js colors to e-ink display color constants
function mapColor(color) {
  // Simple mapping for common colors
  if (typeof color === 'number') {
    if (color === 0) return 'EPD_13IN3E_BLACK';
    if (color === 255) return 'EPD_13IN3E_WHITE';
    return 'EPD_13IN3E_BLACK'; // Default to black for other values
  }
  
  // Handle color arrays [r, g, b]
  if (Array.isArray(color)) {
    const [r, g, b] = color;
    
    // Basic color mapping
    if (r === 255 && g === 0 && b === 0) return 'EPD_13IN3E_RED';
    if (r === 0 && g === 0 && b === 255) return 'EPD_13IN3E_BLUE';
    if (r === 0 && g === 255 && b === 0) return 'EPD_13IN3E_GREEN';
    if (r === 255 && g === 255 && b === 0) return 'EPD_13IN3E_YELLOW';
    if (r === 0 && g === 0 && b === 0) return 'EPD_13IN3E_BLACK';
    if (r === 255 && g === 255 && b === 255) return 'EPD_13IN3E_WHITE';
    
    // Default to black for other colors
    return 'EPD_13IN3E_BLACK';
  }
  
  return 'EPD_13IN3E_BLACK'; // Default
}

// Function to handle a drawing command and convert it to e-ink code
function processDrawingCommand(command, args, currentState) {
  switch (command) {
    case 'line':
      return `Paint_DrawLine(${args[0]}, ${args[1]}, ${args[2]}, ${args[3]}, ${currentState.color || 'EPD_13IN3E_BLACK'}, DOT_PIXEL_${currentState.strokeWeight || 1}X${currentState.strokeWeight || 1}, LINE_STYLE_SOLID);`;
    
    case 'rect':
      return `Paint_DrawRectangle(${args[0]}, ${args[1]}, ${args[0] + args[2]}, ${args[1] + args[3]}, ${currentState.color || 'EPD_13IN3E_BLACK'}, DOT_PIXEL_${currentState.strokeWeight || 1}X${currentState.strokeWeight || 1}, ${currentState.fill ? 'DRAW_FILL_FULL' : 'DRAW_FILL_EMPTY'});`;
    
    case 'ellipse':
      return `Paint_DrawCircle(${args[0]}, ${args[1]}, ${args[2] / 2}, ${currentState.color || 'EPD_13IN3E_BLACK'}, DOT_PIXEL_${currentState.strokeWeight || 1}X${currentState.strokeWeight || 1}, ${currentState.fill ? 'DRAW_FILL_FULL' : 'DRAW_FILL_EMPTY'});`;
    
    case 'text':
      return `Paint_DrawString_EN(${args[1]}, ${args[2]}, "${args[0]}", &Font${currentState.textSize || 12}, ${currentState.color || 'EPD_13IN3E_BLACK'}, EPD_13IN3E_WHITE);`;
    
    // Add more command conversions as needed
    
    default:
      return `// Unsupported command: ${command}`;
  }
}

// Analyze the AST to extract setup information and drawing commands
console.log('Analyzing p5.js sketch...');

// Extract canvas dimensions and other setup information
walk.simple(ast, {
  CallExpression(node) {
    if (node.callee.type === 'Identifier') {
      const funcName = node.callee.name;
      const args = node.arguments.map(arg => {
        if (arg.type === 'Literal') {
          return arg.value;
        }
        // For non-literal arguments, we would need more sophisticated handling
        return '/* complex expression */';
      });
      
      // Extract setup information
      if (funcName === 'createCanvas' && args.length >= 2) {
        setupInfo.canvasWidth = args[0];
        setupInfo.canvasHeight = args[1];
      } else if (funcName === 'background' && args.length >= 1) {
        setupInfo.hasBackground = true;
        setupInfo.backgroundColor = args[0];
      } else if (funcName === 'strokeWeight' && args.length === 1) {
        setupInfo.strokeWeight = args[0];
      } else if (funcName === 'noFill') {
        setupInfo.hasFill = false;
      } else if (funcName === 'fill' && args.length >= 1) {
        setupInfo.hasFill = true;
        setupInfo.fillColor = args;
      }
      
      // Add to drawing commands for conversion later
      // In a full implementation, we'd track the current context (setup vs draw)
      // and the drawing state (stroke color, fill, etc.)
      
      // This is a simplified version - in a real implementation we would
      // track which function we're in and process accordingly
    }
  }
});

console.log('Setup information:', setupInfo);

// Generate the e-ink display code
console.log('Generating e-ink display code...');

const templateCode = `
// Generated from ${path.basename(sketchPath)}
// by residual-visual build system

#include "EPD_13in3e.h"
#include "GUI_Paint.h"
#include "fonts.h"

void drawSketch() {
  // Initialize and setup
  UBYTE *Image;
  UWORD Imagesize = ((EPD_13IN3E_WIDTH % 2 == 0) ? (EPD_13IN3E_WIDTH / 2) : (EPD_13IN3E_WIDTH / 2 + 1)) * EPD_13IN3E_HEIGHT;
  
  if ((Image = (UBYTE *)malloc(Imagesize)) == NULL) {
    Debug("Failed to allocate memory...");
    return;
  }
  
  // Create a new image cache
  Paint_NewImage(Image, ${setupInfo.canvasWidth}, ${setupInfo.canvasHeight}, 0, EPD_13IN3E_WHITE);
  Paint_SelectImage(Image);
  Paint_Clear(EPD_13IN3E_WHITE);
  
  // Set scale (adjust if needed)
  Paint_SetScale(6);
  
  // Draw the sketch
  // TODO: Insert generated drawing commands here
  
  // Display the image
  EPD_13IN3E_DisplayPart(Image, 0, 0, ${setupInfo.canvasWidth}, ${setupInfo.canvasHeight});
  
  // Clean up
  free(Image);
}
`;

// Get output file path
const outputFileName = path.basename(sketchPath, '.js') + '.cpp';
const outputDir = path.join(path.dirname(sketchPath), '..', 'output', 'eink');
const outputPath = path.join(outputDir, outputFileName);

// Ensure output directory exists
if (!fs.existsSync(outputDir)) {
  fs.mkdirSync(outputDir, { recursive: true });
}

// Write output file
fs.writeFileSync(outputPath, templateCode);
console.log(`E-ink code generated: ${outputPath}`);

// Note: This is a skeleton implementation
console.log('NOTE: The e-ink code generator is a work in progress.');
console.log('Currently it only creates a template file without actual drawing commands.');
console.log('A full implementation would analyze your p5.js sketch and generate');
console.log('corresponding e-ink drawing commands for each shape.');

process.exit(0); 