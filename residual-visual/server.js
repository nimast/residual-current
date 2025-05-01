const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const path = require('path');
const chokidar = require('chokidar');
const fs = require('fs');
const { exec } = require('child_process');

const app = express();
const server = http.createServer(app);
const io = socketIo(server);
const PORT = 3000;

// Configure Express for correct MIME types
app.use((req, res, next) => {
  if (req.path.endsWith('.js')) {
    res.type('application/javascript');
  }
  next();
});

// Serve static files
app.use(express.static(path.join(__dirname, 'app')));
app.use('/sketches', express.static(path.join(__dirname, 'sketches')));
app.use('/lib', express.static(path.join(__dirname, 'lib')));

// Watch for changes in the sketches directory
const watcher = chokidar.watch(path.join(__dirname, 'sketches'), {
  ignored: /(^|[\/\\])\../,
  persistent: true
});

// When a file changes, notify connected clients
watcher.on('change', (path) => {
  console.log(`File ${path} has changed`);
  io.emit('fileChanged', { path });
});

// Socket.io connection
io.on('connection', (socket) => {
  console.log('Client connected');
  socket.on('disconnect', () => {
    console.log('Client disconnected');
  });
});

// Build routes for SVG and e-ink exports
app.post('/build/svg', (req, res) => {
  const sketchName = req.query.sketch || 'example.js';
  const sketchPath = path.join(__dirname, 'sketches', sketchName);
  
  // Make sure output directory exists
  const outputDir = path.join(__dirname, 'output', 'svg');
  if (!fs.existsSync(outputDir)) {
    fs.mkdirSync(outputDir, { recursive: true });
  }
  
  // Run the SVG build script
  exec(`node src/build-svg.js ${sketchPath}`, (error, stdout, stderr) => {
    if (error) {
      console.error(`Error: ${error.message}`);
      return res.json({ success: false, error: error.message });
    }
    if (stderr) {
      console.error(`stderr: ${stderr}`);
      return res.json({ success: false, error: stderr });
    }
    
    const outputPath = path.join(outputDir, sketchName.replace('.js', '.svg'));
    console.log(`SVG export successful: ${outputPath}`);
    
    return res.json({ 
      success: true, 
      path: outputPath,
      message: stdout
    });
  });
});

app.post('/build/eink', (req, res) => {
  const sketchName = req.query.sketch || 'example.js';
  const sketchPath = path.join(__dirname, 'sketches', sketchName);
  
  // Make sure output directory exists
  const outputDir = path.join(__dirname, 'output', 'eink');
  if (!fs.existsSync(outputDir)) {
    fs.mkdirSync(outputDir, { recursive: true });
  }
  
  // Run the e-ink build script
  exec(`node src/build-eink.js ${sketchPath}`, (error, stdout, stderr) => {
    if (error) {
      console.error(`Error: ${error.message}`);
      return res.json({ success: false, error: error.message });
    }
    if (stderr) {
      console.error(`stderr: ${stderr}`);
      return res.json({ success: false, error: stderr });
    }
    
    const outputPath = path.join(outputDir, sketchName.replace('.js', '.cpp'));
    console.log(`E-ink export successful: ${outputPath}`);
    
    return res.json({ 
      success: true, 
      path: outputPath,
      message: stdout
    });
  });
});

// Add route to serve SVG outputs for preview
app.use('/output/svg', express.static(path.join(__dirname, 'output', 'svg')));

// Start server
server.listen(PORT, () => {
  console.log(`Server running at http://localhost:${PORT}`);
  console.log(`Watching for changes in ${path.join(__dirname, 'sketches')}`);
}); 