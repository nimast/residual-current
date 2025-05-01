# Residual Visual

A development environment for creating visual programs that can be exported to both SVG and e-ink displays.

## Features

- Create visual designs using p5.js
- Live preview updates as you edit code
- Export designs to standard SVG format
- Generate e-ink display code for ESP32

## Installation

1. Clone this repository
2. Install dependencies:

```bash
cd residual-visual
npm install
```

## Usage

### Start the development server

```bash
npm start
```

This will start a server at http://localhost:3000 with live reloading.

### Creating sketches

Write p5.js sketches in the `sketches` directory. The development server will automatically detect changes and refresh the preview.

Example sketch:

```javascript
function setup() {
  createCanvas(600, 800);
  strokeWeight(2);
  noFill();
}

function draw() {
  background(255);
  stroke(0);
  line(50, 50, 100, 100);
  
  stroke(255, 0, 0); // Red
  rect(150, 50, 100, 75);
  
  noLoop(); // Static sketch - no animation needed
}
```

### Exporting to SVG

Click the "Export SVG" button in the preview window or run:

```bash
npm run build:svg sketches/your-sketch.js
```

SVG files will be generated in the `output/svg` directory.

### Generating e-ink code

Click the "Export e-ink" button in the preview window or run:

```bash
npm run build:eink sketches/your-sketch.js
```

C++ files will be generated in the `output/eink` directory.

## Important Notes

### Color Support

The e-ink display supports a limited color palette:
- BLACK
- WHITE
- RED
- BLUE
- GREEN
- YELLOW

Use these colors in your p5.js sketches for best results.

### Canvas Size

The e-ink display has a resolution of 1200x1600. Use a canvas size of 600x800 for a 1:2 scale preview.

## Deployment to GitHub Pages

This project can be deployed as a static website using GitHub Pages.

1.  **Push Code:** Ensure your latest code (including the `app/`, `lib/`, and `sketches/` directories) is pushed to your GitHub repository.
2.  **Configure Pages:**
    *   Go to your repository settings on GitHub.
    *   Navigate to the "Pages" section.
    *   Under "Build and deployment", select "Deploy from a branch".
    *   Choose the branch you want to deploy (e.g., `main`).
    *   Select the folder as `/ (root)`.
    *   Click "Save".
3.  **Access Site:** GitHub will build and deploy your site. The URL will be available in the Pages settings (e.g., `https://<your-username>.github.io/<repository-name>/`).

**GitHub Pages Limitations:**

*   **Static Only:** The Node.js server (`server.js`) does not run.
*   **No Live Reload:** Changes pushed to GitHub need to be redeployed by GitHub Pages; there's no automatic browser refresh like with the local server.
*   **E-ink Export Disabled:** The "Export e-ink" functionality will not work as it requires the server-side build script.
*   **SVG Export:** The client-side SVG export should work as expected.
*   **Path Issues:** If sketches or libraries fail to load, you may need to adjust the paths in `app/index.html` to be relative (e.g., change `/lib/p5.min.js` to `lib/p5.min.js`).

## Development

### Project Structure

```
residual-visual/
├── app/              # Web frontend
├── sketches/         # p5.js sketches 
├── output/           # Generated files
│   ├── svg/          # Exported SVG files
│   └── eink/         # Generated e-ink code
├── src/              # Build scripts
└── server.js         # Development server
```

## License

MIT 