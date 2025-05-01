console.log('[Main] main.js script started.');

// Main script for Residual Visual
const socket = io();
let p5Instance = null;
let currentSketch = 'example.js'; // Default sketch to load

// DOM elements
const sketchContainer = document.getElementById('sketch-container');
const statusElement = document.getElementById('status');
const dimensionsElement = document.getElementById('dimensions');
const exportSvgButton = document.getElementById('export-svg');
const exportEinkButton = document.getElementById('export-eink');

// Handle socket connection events
socket.on('connect', () => {
  statusElement.textContent = 'Connected, watching for changes...';
  loadSketch(currentSketch);
});

socket.on('disconnect', () => {
  statusElement.textContent = 'Disconnected from server';
});

// When a file changes, reload the sketch
socket.on('fileChanged', (data) => {
  statusElement.textContent = `File changed: ${data.path}`;
  // Extract the filename from the path
  const filename = data.path.split('/').pop();
  if (filename === currentSketch) {
    reloadSketch();
  }
});

// Load a sketch by filename
function loadSketch(filename) {
  console.log(`[Main] loadSketch called for: ${filename}`);
  // Clear previous sketch if it exists
  if (p5Instance) {
    p5Instance.remove();
    p5Instance = null;
  }
  
  // Remove any existing sketch container
  while (sketchContainer.firstChild) {
    sketchContainer.removeChild(sketchContainer.firstChild);
  }
  
  // Create a new iframe to isolate the sketch environment
  const iframe = document.createElement('iframe');
  iframe.style.width = '100%';
  iframe.style.height = '100%';
  iframe.style.border = 'none';
  sketchContainer.appendChild(iframe);
  console.log('[Main] Iframe element created and appended.');
  
  // --- Check for seed in URL --- 
  const urlParams = new URLSearchParams(window.location.search);
  const urlSeed = urlParams.get('seed');
  let seedScript = '';
  if (urlSeed && !isNaN(parseInt(urlSeed, 10))) {
      const seedValue = parseInt(urlSeed, 10);
      console.log(`[Main] Found seed in URL: ${seedValue}`);
      seedScript = `<script>window.initialSeed = ${seedValue}; console.log('[Iframe] Initial seed set from URL:', window.initialSeed);</script>`;
  } else if (urlSeed) {
      console.warn(`[Main] Ignoring invalid seed parameter: ${urlSeed}`);
  }
  // --- End seed check ---

  // Create a simple HTML document with p5.js and the sketch
  const iframeContent = `
    <!DOCTYPE html>
    <html>
    <head>
      <script src="/lib/p5.min.js"></script>
      <script src="/lib/p5.svg.js"></script>
      ${seedScript}
      <script>
        console.log('[Iframe] Script block started.');
        // --- SVG Export Listener ---
        window.addEventListener('message', async (event) => {
          console.log('[Iframe] Message received:', event.data, 'from origin:', event.origin);

          if (event.data && event.data.type === 'exportSVG') {
            console.log('[Iframe] Received exportSVG message');
            try {
              // --- SVG Generation Logic (Temporary Instance) ---
              console.log('[Iframe Temp Instance] Starting SVG generation...');
              if (typeof window.p5 === 'undefined' || typeof window.SVG === 'undefined') {
                  throw new Error('p5 or SVG renderer missing in iframe');
              }

              const originalCanvas = document.querySelector('canvas');
              if (!originalCanvas) throw new Error('Original canvas not found');
              const w = originalCanvas.width;
              const h = originalCanvas.height;
              const originalSeed = window.seed;
              const originalSetupFunction = window.setup; // CAPTURE ORIGINAL SETUP
              const originalDrawFunction = window.draw;   // CAPTURE ORIGINAL DRAW

              if (typeof originalSetupFunction !== 'function' || typeof originalDrawFunction !== 'function') {
                 throw new Error('Original setup or draw function not found');
              }
              console.log('[Iframe Temp Instance] Original sketch details captured.');

              const svgContainer = document.createElement('div');
              svgContainer.style.position = 'absolute';
              svgContainer.style.left = '-9999px';
              document.body.appendChild(svgContainer);
              console.log('[Iframe Temp Instance] Hidden container created.');

              let svgResult = null;
              const svgSketch = (p) => {
                p.setup = () => {
                  console.log('[Iframe Temp Instance] SVG p.setup started.');
                  p.createCanvas(w, h, p.SVG); // Use SVG renderer
                  if (typeof originalSeed !== 'undefined') {
                    p.randomSeed(originalSeed);
                    p.noiseSeed(originalSeed);
                  }
                  // Call original setup logic within the new context
                  console.log('[Iframe Temp Instance] Calling original setup function...');
                  originalSetupFunction.call(p);
                  console.log('[Iframe Temp Instance] Original setup finished.');
                  p.noLoop(); // Important: Prepare for single draw
                };

                p.draw = () => {
                  console.log('[Iframe Temp Instance] SVG p.draw started.');
                  try {
                    // Execute original draw function in the context of this SVG instance
                    // originalDrawFunction.call(p); // --- Temporarily comment out original draw

                    // --- TEST: Draw a simple shape directly ---
                    console.log('[Iframe Temp Instance] Drawing test rectangle...');
                    p.fill(255, 0, 0);
                    p.stroke(0);
                    p.strokeWeight(2);
                    p.rect(50, 50, 200, 100);
                    console.log('[Iframe Temp Instance] Test rectangle drawn.');
                    // --- End TEST ---

                    console.log('[Iframe Temp Instance] Original draw function executed (or skipped for test).');
                  } catch(drawError) {
                    console.error('[Iframe Temp Instance] Error during SVG export:', drawError);
                    event.source.postMessage({ type: 'svgResult', success: false, error: drawError.message }, event.origin);
                  }
                };
              };

              console.log('[Iframe Temp Instance] Creating temporary p5 instance...');
              const tempP5 = new window.p5(svgSketch, svgContainer);
              console.log('[Iframe Temp Instance] Temporary p5 instance created.');

              // Use a promise to wait for SVG generation (setup and one draw)
              const generationPromise = new Promise((resolve) => {
                  // Timeout allows setup and the single draw frame to complete
                  setTimeout(() => {
                      console.log('[Iframe Temp Instance] Extracting SVG after timeout...');
                      const svgElement = svgContainer.querySelector('svg');
                      if (svgElement) {
                          console.log('[Iframe Temp Instance] SVG Element Found. Children:', svgElement.children.length);
                           if (svgElement.children.length > 1 && svgElement.children[1].tagName.toLowerCase() === 'g') {
                               console.log('[Iframe Temp Instance] <g> tag children count:', svgElement.children[1].children.length);
                           }
                          const svgString = svgElement.outerHTML;
                          const preface = '<?xml version="1.0" standalone="no"?>\\r\\n';
                          const completeSvg = svgString.replace('<svg', '<svg xmlns="http://www.w3.org/2000/svg"');
                          resolve({ success: true, svg: preface + completeSvg });
                      } else {
                           console.error('[Iframe Temp Instance] SVG element not found!');
                          resolve({ success: false, error: 'SVG element not generated in iframe' });
                      }
                      // Cleanup
                      console.log('[Iframe Temp Instance] Cleaning up...');
                      if(tempP5 && typeof tempP5.remove === 'function') {
                         tempP5.remove();
                         console.log('[Iframe Temp Instance] tempP5 instance removed.');
                      }
                      if (document.body.contains(svgContainer)){
                           document.body.removeChild(svgContainer);
                           console.log('[Iframe Temp Instance] Hidden container removed.');
                      }
                  }, 2000); // Increased wait to 2s
              });

              svgResult = await generationPromise;
              console.log('[Iframe Temp Instance] SVG generation promise resolved.');
              // --- End SVG Generation Logic ---

              // Send result back to parent
              event.source.postMessage({ type: 'svgResult', ...svgResult }, event.origin);

            } catch (e) {
              console.error('[Iframe Temp Instance] Error during SVG export:', e);
              event.source.postMessage({ type: 'svgResult', success: false, error: e.message }, event.origin);
               // Attempt cleanup on error too
               const container = document.querySelector('#__svgExportContainer__'); // Assuming we add an ID
               if(container) document.body.removeChild(container);
            }
          }
        });
        console.log('[Iframe] Message listener attached.');
        // --- End SVG Export Listener ---
      </script>
      <style>
        body { 
          margin: 0; 
          overflow: hidden; 
          display: flex;
          justify-content: center;
          align-items: center;
        }
      </style>
    </head>
    <body>
      <script src="/sketches/${filename}?t=${Date.now()}"></script>
    </body>
    </html>
  `;
  
  iframe.srcdoc = iframeContent;
  console.log('[Main] Iframe srcdoc assigned.');
  
  iframe.onload = () => {
    console.log(`[Main] Iframe onload event fired for: ${filename}`);
    statusElement.textContent = `Loaded sketch: ${filename}`;
    
    // Try to get canvas dimensions
    try {
      const iframeCanvas = iframe.contentWindow.document.querySelector('canvas');
      if (iframeCanvas) {
        dimensionsElement.textContent = `Width: ${iframeCanvas.width}px, Height: ${iframeCanvas.height}px`;
      }
    } catch (e) {
      console.error('Error getting canvas dimensions:', e);
    }
  };
  
  currentSketch = filename;
}

// Reload the current sketch
function reloadSketch() {
  loadSketch(currentSketch);
}

// Export SVG button click handler - client-side implementation
exportSvgButton.addEventListener('click', () => {
  console.log('[Main] Export SVG button clicked (Direct Save Call).'); 
  statusElement.textContent = 'Triggering SVG save...';

  const iframe = sketchContainer.querySelector('iframe');
  if (!iframe || !iframe.contentWindow) {
    statusElement.textContent = 'Error: Sketch iframe not found or inaccessible.';
    return;
  }

  // Directly call the save function within the sketch's context
  if (typeof iframe.contentWindow.triggerSVGSave === 'function') {
    try {
      iframe.contentWindow.triggerSVGSave();
      statusElement.textContent = 'SVG save triggered.';
      // Note: Download is handled by the browser via save() call
    } catch (error) {
        console.error('[Main] Error calling triggerSVGSave in iframe:', error);
        statusElement.textContent = `Error triggering save: ${error.message}`;
    }
  } else {
    console.error('[Main] triggerSVGSave function not found in iframe window.');
    statusElement.textContent = 'Error: Save function not found in sketch.';
  }
});

// Export e-ink button click handler - still uses server-side approach
exportEinkButton.addEventListener('click', () => {
  statusElement.textContent = 'Exporting e-ink code...';
  
  // Send request to server to build e-ink code
  fetch(`/build/eink?sketch=${currentSketch}`, {
    method: 'POST'
  })
  .then(response => response.json())
  .then(data => {
    if (data.success) {
      statusElement.textContent = `E-ink code exported to: ${data.path}`;
    } else {
      statusElement.textContent = `Error: ${data.error}`;
    }
  })
  .catch(error => {
    statusElement.textContent = `Error: ${error.message}`;
  });
}); 