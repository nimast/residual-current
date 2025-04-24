// Media Strata - Generative Diagrams of Unseen Networks
// A generative diagram system inspired by geological cross-sections and media theory

// Configuration
let currentMode = 0; // 0: Strata, 1: Network, 2: Combinatorial
const MODES = ["STRATA", "NETWORK", "COMBINATORIAL"];
let seed; // Random seed for consistency

// Visual parameters
let strata = []; // For strata mode
let nodes = []; // For network mode
let combinations = []; // For combinatorial mode
let labels = []; // Text labels

// Colors for e-ink palette
const COLORS = {
  BLACK: [0, 0, 0],
  WHITE: [255, 255, 255],
  RED: [255, 0, 0],
  BLUE: [0, 0, 255],
  GREEN: [0, 255, 0],
  YELLOW: [255, 255, 0]
};

// Media theory terms for labels
const MEDIA_TERMS = [
  "Signal", "Noise", "Channel", "Code", "Receiver", 
  "Transmitter", "Archive", "Storage", "Interface",
  "Network", "Protocol", "Medium", "Message", "Bandwidth",
  "Substrate", "Encryption", "Resonance", "Feedback"
];

// Geological terms for labels
const GEO_TERMS = [
  "Strata", "Vein", "Deposit", "Shaft", "Tunnel", 
  "Fault", "Seam", "Bedrock", "Chamber", "Gallery",
  "Drift", "Outcrop", "Lode", "Formation", "Sediment"
];

function setup() {
  // Create canvas at e-paper dimensions (scaled down for preview)
  createCanvas(600, 800, SVG); // <<< USE SVG RENDERER
  
  // Set default drawing parameters
  strokeWeight(1.5);
  noFill();
  textSize(10);
  
  // Initialize with a random seed
  seed = floor(random(10000));
  initializeMode();
}

function draw() {
  background(255);
  
  // Draw the current mode
  switch(currentMode) {
    case 0:
      drawStrataMode();
      break;
    case 1:
      drawNetworkMode();
      break;
    case 2:
      drawCombinatorialMode();
      break;
  }
  
  // Draw mode label in corner
  fill(0);
  noStroke();
  textSize(14);
  text(`MODE: ${MODES[currentMode]}`, 20, 30);
  text(`SEED: ${seed}`, 20, 50);
  textSize(10);
  
  noLoop(); // Static image - no animation needed
}

function keyPressed() {
  if (key === ' ') {
    // Generate new diagram with new seed
    seed = floor(random(10000));
    initializeMode();
    redraw();
  } else if (key === 'm' || key === 'M') {
    // Switch to next mode
    currentMode = (currentMode + 1) % MODES.length;
    initializeMode();
    redraw();
  } else if (key === 's' || key === 'S') {
    // Save as image
    saveCanvas('media_strata', 'png');
  }
}

function initializeMode() {
  // Reset randomization with seed
  randomSeed(seed);
  noiseSeed(seed);
  
  // Clear all arrays
  strata = [];
  nodes = [];
  combinations = [];
  labels = [];
  
  // Initialize based on current mode
  switch(currentMode) {
    case 0:
      initializeStrata();
      break;
    case 1:
      initializeNetwork();
      break;
    case 2:
      initializeCombinatorial();
      break;
  }
}

// ===== STRATA MODE =====

function initializeStrata() {
  // Create layers of geological strata
  const numLayers = floor(random(5, 12));
  const baseHeight = height / numLayers;
  
  // Generate strata with varying heights and properties
  let currentY = 0;
  for (let i = 0; i < numLayers; i++) {
    const layerHeight = baseHeight * random(0.5, 1.5);
    
    // Choose a color for this stratum
    const colorKeys = Object.keys(COLORS);
    const colorKey = random([colorKeys[0], colorKeys[1], ...Array(3).fill(colorKeys[floor(random(2, colorKeys.length))])]);
    const strataColor = COLORS[colorKey];
    
    // Create strata with noise-based top edge
    strata.push({
      y: currentY,
      height: layerHeight,
      color: strataColor,
      noiseOffset: random(100),
      noiseScale: random(0.002, 0.01),
      noiseAmp: random(10, 40)
    });
    
    currentY += layerHeight;
  }
  
  // Generate mine shafts and tunnels
  const numShafts = floor(random(3, 8));
  for (let i = 0; i < numShafts; i++) {
    const x = random(width * 0.1, width * 0.9);
    const startY = random(height * 0.1);
    const endY = random(height * 0.5, height * 0.9);
    
    // Add vertical shaft
    strata.push({
      type: 'shaft',
      x: x,
      startY: startY,
      endY: endY,
      width: random(2, 8)
    });
    
    // Add some horizontal tunnels from this shaft
    const numTunnels = floor(random(1, 4));
    for (let j = 0; j < numTunnels; j++) {
      const tunnelY = map(j, 0, numTunnels, startY + (endY-startY)*0.2, endY);
      const direction = random() > 0.5 ? 1 : -1;
      const tunnelLength = random(50, 150);
      
      strata.push({
        type: 'tunnel',
        startX: x,
        startY: tunnelY,
        endX: x + (tunnelLength * direction),
        endY: tunnelY + random(-20, 20),
        width: random(1, 5)
      });
    }
  }
  
  // Add some text labels to strata
  for (let i = 0; i < floor(random(5, 12)); i++) {
    labels.push({
      x: random(width * 0.1, width * 0.9),
      y: random(height * 0.1, height * 0.9),
      text: random(GEO_TERMS),
      size: floor(random(8, 14)),
      rotation: random(-PI/6, PI/6)
    });
  }
}

function drawStrataMode() {
  // Draw base rectangle
  stroke(0);
  strokeWeight(2);
  noFill();
  rect(0, 0, width, height);
  
  // Draw strata layers
  for (const layer of strata) {
    if (layer.type === 'shaft') {
      // Draw vertical shaft
      stroke(0);
      strokeWeight(layer.width);
      line(layer.x, layer.startY, layer.x, layer.endY);
      
      // Draw little horizontal supports
      const numSupports = floor((layer.endY - layer.startY) / 20);
      strokeWeight(1);
      for (let i = 0; i < numSupports; i++) {
        const supportY = map(i, 0, numSupports-1, layer.startY, layer.endY);
        line(layer.x - 5, supportY, layer.x + 5, supportY);
      }
      
    } else if (layer.type === 'tunnel') {
      // Draw horizontal tunnel
      stroke(0);
      strokeWeight(layer.width);
      line(layer.startX, layer.startY, layer.endX, layer.endY);
      
    } else {
      // Draw geological stratum
      stroke(0);
      strokeWeight(1);
      
      // Create a wavy top edge for the stratum using noise
      beginShape();
      for (let x = 0; x < width; x += 5) {
        const noiseVal = noise(x * layer.noiseScale + layer.noiseOffset);
        const y = layer.y + noiseVal * layer.noiseAmp;
        vertex(x, y);
      }
      // Complete the rectangle
      vertex(width, layer.y + layer.height);
      vertex(0, layer.y + layer.height);
      endShape(CLOSE);
      
      // Apply a fill color with low opacity for the layer
      noStroke();
      fill(layer.color[0], layer.color[1], layer.color[2], 30);
      beginShape();
      for (let x = 0; x < width; x += 5) {
        const noiseVal = noise(x * layer.noiseScale + layer.noiseOffset);
        const y = layer.y + noiseVal * layer.noiseAmp;
        vertex(x, y);
      }
      vertex(width, layer.y + layer.height);
      vertex(0, layer.y + layer.height);
      endShape(CLOSE);
      
      // Add some texture lines within the strata
      stroke(0, 40);
      strokeWeight(0.5);
      const numLines = floor(random(3, 8));
      for (let i = 0; i < numLines; i++) {
        const lineY = layer.y + layer.height * map(i, 0, numLines-1, 0.2, 0.9);
        beginShape();
        for (let x = 0; x < width; x += 10) {
          const noiseVal = noise(x * layer.noiseScale * 2 + layer.noiseOffset + 100);
          const y = lineY + noiseVal * layer.noiseAmp * 0.3;
          vertex(x, y);
        }
        endShape();
      }
    }
  }
  
  // Draw labels
  for (const label of labels) {
    push();
    translate(label.x, label.y);
    rotate(label.rotation);
    fill(0);
    noStroke();
    textSize(label.size);
    text(label.text, 0, 0);
    pop();
  }
}

// ===== NETWORK MODE =====

function initializeNetwork() {
  // Create nodes at different levels/strata
  const numLevels = floor(random(3, 6));
  const numNodesPerLevel = floor(random(3, 8));
  
  // Level names - from deep to surface
  const levelNames = [
    "Physical", "Technical", "Symbolic", "Cultural", "Cognitive"
  ];
  
  // Create nodes organized by levels
  for (let level = 0; level < numLevels; level++) {
    const levelY = map(level, 0, numLevels-1, height * 0.8, height * 0.2);
    
    // Add level label
    labels.push({
      x: width * 0.05,
      y: levelY,
      text: levelNames[level % levelNames.length],
      size: 12,
      rotation: 0,
      isLevel: true
    });
    
    // Create nodes for this level
    for (let i = 0; i < numNodesPerLevel; i++) {
      const nodeX = map(i, 0, numNodesPerLevel-1, 
                       width * 0.2 + random(-20, 20), 
                       width * 0.9 + random(-20, 20));
      
      // Choose a node style/type based on level
      const nodeTypes = ['circle', 'triangle', 'rect', 'diamond'];
      const nodeType = nodeTypes[floor(random(nodeTypes.length))];
      
      // Choose a color
      const colorKeys = Object.keys(COLORS);
      const colorKey = random(colorKeys.slice(2)); // Skip black and white
      const nodeColor = COLORS[colorKey];
      
      nodes.push({
        x: nodeX,
        y: levelY + random(-15, 15),
        size: random(10, 25),
        level: level,
        type: nodeType,
        color: nodeColor,
        label: random(MEDIA_TERMS)
      });
    }
  }
  
  // Create connections between nodes
  // Each node connects to 1-3 nodes in adjacent levels
  for (let i = 0; i < nodes.length; i++) {
    const node = nodes[i];
    const connectionsCount = floor(random(1, 4));
    
    // Find nodes in adjacent levels
    const possibleTargets = nodes.filter(n => 
      (n.level === node.level + 1 || n.level === node.level - 1) && 
      dist(node.x, node.y, n.x, n.y) < width * 0.4
    );
    
    // If we found possible targets, connect to some of them
    if (possibleTargets.length > 0) {
      for (let c = 0; c < min(connectionsCount, possibleTargets.length); c++) {
        const targetIndex = floor(random(possibleTargets.length));
        const target = possibleTargets[targetIndex];
        
        // Remove this target from possible targets to avoid duplicate connections
        possibleTargets.splice(targetIndex, 1);
        
        // Create a connection with a control point for curved lines
        combinations.push({
          startNode: i,
          endNode: nodes.indexOf(target),
          controlX: (node.x + target.x) / 2 + random(-40, 40),
          controlY: (node.y + target.y) / 2 + random(-30, 30),
          weight: random(0.5, 2)
        });
      }
    }
  }
  
  // Add a few cross-level connections (jumping multiple levels)
  const numCrossConnections = floor(random(2, 5));
  for (let i = 0; i < numCrossConnections; i++) {
    const startNode = floor(random(nodes.length));
    const endNode = floor(random(nodes.length));
    
    // Only connect if they're not on adjacent levels and not the same node
    if (startNode !== endNode && 
        abs(nodes[startNode].level - nodes[endNode].level) > 1) {
      combinations.push({
        startNode: startNode,
        endNode: endNode,
        controlX: (nodes[startNode].x + nodes[endNode].x) / 2 + random(-60, 60),
        controlY: (nodes[startNode].y + nodes[endNode].y) / 2 + random(-40, 40),
        weight: random(0.5, 2),
        style: 'dashed'
      });
    }
  }
}

function drawNetworkMode() {
  // Draw base rectangle
  stroke(0);
  strokeWeight(2);
  noFill();
  rect(0, 0, width, height);
  
  // Draw horizontal lines for each level
  for (const label of labels) {
    if (label.isLevel) {
      stroke(0, 50);
      strokeWeight(1);
      line(width * 0.15, label.y, width * 0.95, label.y);
      
      // Draw level label
      fill(0);
      noStroke();
      textSize(label.size);
      textAlign(RIGHT);
      text(label.text, width * 0.12, label.y + 4);
      textAlign(LEFT);
    }
  }
  
  // Draw connections first (so they appear behind nodes)
  for (const conn of combinations) {
    const start = nodes[conn.startNode];
    const end = nodes[conn.endNode];
    
    // Style based on connection type
    if (conn.style === 'dashed') {
      drawingContext.setLineDash([5, 3]);
      stroke(0, 150);
    } else {
      drawingContext.setLineDash([]);
      stroke(0, 200);
    }
    
    strokeWeight(conn.weight);
    
    // Draw curved line
    noFill();
    beginShape();
    vertex(start.x, start.y);
    quadraticVertex(conn.controlX, conn.controlY, end.x, end.y);
    endShape();
    
    // Reset dash pattern
    drawingContext.setLineDash([]);
  }
  
  // Draw nodes
  for (const node of nodes) {
    // Draw node shape
    fill(node.color);
    stroke(0);
    strokeWeight(1);
    
    if (node.type === 'circle') {
      ellipse(node.x, node.y, node.size);
    } else if (node.type === 'triangle') {
      triangle(
        node.x, node.y - node.size/2,
        node.x - node.size/2, node.y + node.size/2,
        node.x + node.size/2, node.y + node.size/2
      );
    } else if (node.type === 'rect') {
      rectMode(CENTER);
      rect(node.x, node.y, node.size, node.size);
      rectMode(CORNER);
    } else if (node.type === 'diamond') {
      quad(
        node.x, node.y - node.size/2,
        node.x + node.size/2, node.y,
        node.x, node.y + node.size/2,
        node.x - node.size/2, node.y
      );
    }
    
    // Draw node label
    fill(0);
    noStroke();
    textSize(8);
    textAlign(CENTER);
    text(node.label, node.x, node.y + node.size/2 + 10);
    textAlign(LEFT);
  }
}

// ===== COMBINATORIAL MODE =====

function initializeCombinatorial() {
  // Create two sets of items to connect
  const leftSide = [];
  const rightSide = [];
  
  const numLeftItems = floor(random(8, 15));
  const numRightItems = floor(random(5, 10));
  
  // Create left side items
  for (let i = 0; i < numLeftItems; i++) {
    leftSide.push({
      x: width * 0.15,
      y: map(i, 0, numLeftItems-1, height * 0.15, height * 0.85),
      label: random(GEO_TERMS),
      index: i
    });
  }
  
  // Create right side items
  for (let i = 0; i < numRightItems; i++) {
    rightSide.push({
      x: width * 0.85,
      y: map(i, 0, numRightItems-1, height * 0.2, height * 0.8),
      label: random(MEDIA_TERMS),
      index: i + numLeftItems
    });
  }
  
  // Combine left and right items into nodes array
  nodes = [...leftSide, ...rightSide];
  
  // Generate connections between items
  // This creates the combinatorial pattern
  
  // One approach: connect every left to every right
  for (let i = 0; i < leftSide.length; i++) {
    for (let j = 0; j < rightSide.length; j++) {
      // Create a combination
      const leftItem = leftSide[i];
      const rightItem = rightSide[j];
      
      // Only connect some pairs for sparser diagram
      if (random() < 0.7) {
        combinations.push({
          startNode: leftItem.index,
          endNode: rightItem.index,
          weight: random(0.5, 1.5),
          controlY: (leftItem.y + rightItem.y) / 2 + random(-20, 20)
        });
      }
    }
  }
  
  // Add a title
  labels.push({
    x: width / 2,
    y: height * 0.07,
    text: "TABULA COMBINATORIA MEDIA-GEOLOGICA",
    size: 16,
    rotation: 0,
    isTitle: true
  });
  
  // Add section labels
  labels.push({
    x: width * 0.15,
    y: height * 0.1,
    text: "TERRESTRIAL STRATA",
    size: 12,
    rotation: 0
  });
  
  labels.push({
    x: width * 0.85,
    y: height * 0.1,
    text: "MEDIA SIGNALS",
    size: 12,
    rotation: 0
  });
  
  // Add some descriptive text
  labels.push({
    x: width * 0.5,
    y: height * 0.95,
    text: "Showing hidden connections between geological formations and media operations",
    size: 10,
    rotation: 0,
    isBottom: true
  });
}

function drawCombinatorialMode() {
  // Draw background
  background(255);
  
  // Draw border
  stroke(0);
  strokeWeight(2);
  noFill();
  rect(0, 0, width, height);
  
  // Draw inner decorative frame
  stroke(0);
  strokeWeight(1);
  rect(width * 0.05, height * 0.05, width * 0.9, height * 0.9);
  
  // Draw connections between nodes
  for (const conn of combinations) {
    const start = nodes[conn.startNode];
    const end = nodes[conn.endNode];
    
    stroke(0, 150);
    strokeWeight(conn.weight);
    
    // Draw curved or straight line
    noFill();
    if (conn.controlY) {
      beginShape();
      vertex(start.x, start.y);
      quadraticVertex((start.x + end.x) / 2, conn.controlY, end.x, end.y);
      endShape();
    } else {
      line(start.x, start.y, end.x, end.y);
    }
  }
  
  // Draw nodes
  for (const node of nodes) {
    // Draw node marker
    fill(0);
    noStroke();
    ellipse(node.x, node.y, 5, 5);
    
    // Draw node label
    textAlign(node.x < width/2 ? RIGHT : LEFT);
    textSize(10);
    text(node.label, node.x + (node.x < width/2 ? -10 : 10), node.y + 4);
  }
  
  // Draw title and labels
  for (const label of labels) {
    fill(0);
    noStroke();
    textSize(label.size);
    textAlign(CENTER);
    
    if (label.isTitle) {
      push();
      translate(label.x, label.y);
      rotate(label.rotation);
      text(label.text, 0, 0);
      // Add decorative line under title
      stroke(0);
      strokeWeight(1);
      line(-textWidth(label.text)/2, 10, textWidth(label.text)/2, 10);
      pop();
    } else if (label.isBottom) {
      text(label.text, label.x, label.y);
    } else {
      push();
      translate(label.x, label.y);
      rotate(label.rotation);
      textAlign(CENTER);
      text(label.text, 0, 0);
      pop();
    }
  }
  
  // Add decorative elements in corners
  drawDecorativeCorners();
}

function drawDecorativeCorners() {
  // Draw decorative corner elements
  stroke(0);
  strokeWeight(1);
  
  const cornerSize = 20;
  
  // Top-left
  line(width * 0.05, height * 0.05 + cornerSize, width * 0.05, height * 0.05);
  line(width * 0.05, height * 0.05, width * 0.05 + cornerSize, height * 0.05);
  
  // Top-right
  line(width * 0.95 - cornerSize, height * 0.05, width * 0.95, height * 0.05);
  line(width * 0.95, height * 0.05, width * 0.95, height * 0.05 + cornerSize);
  
  // Bottom-left
  line(width * 0.05, height * 0.95 - cornerSize, width * 0.05, height * 0.95);
  line(width * 0.05, height * 0.95, width * 0.05 + cornerSize, height * 0.95);
  
  // Bottom-right
  line(width * 0.95 - cornerSize, height * 0.95, width * 0.95, height * 0.95);
  line(width * 0.95, height * 0.95, width * 0.95, height * 0.95 - cornerSize);
}

// Add this function to be called from the parent window
function triggerSVGSave() {
  const filename = `media_strata_seed_${seed}.svg`; // Include seed in filename
  console.log(`[Sketch] Triggering SVG save as: ${filename}`);
  save(filename);
} 