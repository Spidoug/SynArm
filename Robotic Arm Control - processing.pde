// =====================================================================
//  SynArm
//  Robotic Arm Control – Leap Motion · Joystick · Keyboard · WebSockets
// ---------------------------------------------------------------------
//  ✉ Douglas Santana (SPIDOUG)
// =====================================================================

// ==============================
// 🏷 Imports and Libraries
// ==============================
import de.voidplus.leapmotion.*;
import processing.serial.*;
import org.gamecontrolplus.*;        
import net.java.games.input.*;       
import websockets.*;               //  ←  NEW
import java.util.List;
import javax.swing.JFrame;
import javax.swing.ImageIcon;
import java.awt.Image;
import java.io.File;

// ==============================
// 🌐 WebSocket Server
// ==============================
WebsocketServer wsServer;
final int WS_PORT = 9000;
final String WS_PATH = "/";

// ==============================
// 📡 Serial & Sensor variables
// ==============================
String   latestSerialLine = "";    // last raw line from Arduino
JSONObject latestSensors  = new JSONObject();

// ==============================
// 🎮 Joystick and Control
// ==============================
ControlIO control;
ControlDevice joystick;
boolean joystickConnected = false;
float[] joystickPos = new float[4]; // [x, y, z, r]

// ==============================
// ⚙️ System and Configuration
// ==============================
boolean arduinoReady = false;
boolean leapAvailable = false;
long lastServoCommandTime = 0;
int movementTimeout = 500;

// ==============================
// 🚧 Leap‑Motion stability constants (NEW)
// ==============================
final int LEAP_UPDATE_PERIOD = 30;   // ms between Leap→servo pushes
final int ANGLE_DEADBAND     = 2;    // ignore ±2 deg micro‑variations
int       lastLeapUpdateTime = 0;

LeapMotion leap;
Serial myPort;
ArrayList<Box3D> boundingBoxes = new ArrayList<Box3D>();
ArrayList<Box3D> collidingBoxes = new ArrayList<Box3D>();
ArrayList<String> pendingWsCommands = new ArrayList<String>();

// ==============================
// 📦 Serial Port
// ==============================
String[] ports;
int portIndex = 0;
Serial testPort;
int arduinoStartTime = 0;

// ==============================
// 🧠 System States
// ==============================
int initializationStep = 0;
boolean waiting = false;
int waitDuration = 0;
int waitStartTime = 0;
String[] logMessages = new String[4];
String currentMessage = "🔄 Initializing system...";
float cameraRotationIncrement = 0;

String controlInstructions =
  "🎮 ARM CONTROL (KEYBOARD)\n" +
  "\n" +
  "  Q / A   →  Base        \n" +
  "  W / S   →  Shoulder    \n" +
  "  E / D   →  Elbow       \n" +
  "  R / F   →  Wrist Tilt  \n" +
  "  T / G   →  Wrist Rot.  \n" +
  "  Y / H   →  Gripper     \n" +
  "  ← / → : Rotate camera";

// ==============================
// 🤖 Servo Angles
// ==============================
int[] angles = {90, 90, 180, 90, 90, 140}; // Base, Arm, Forearm, Wrist V, Wrist R, Gripper
int[][] servoLimits = {
  {0, 359}, {0, 359}, {0, 359}, {0, 359}, {0, 359}, {0, 359}
}; // [servo][min, max]
final int BASE       = 0;
final int ARM        = 1;
final int FOREARM    = 2;
final int WRIST_VERT = 3;
final int WRIST_ROT  = 4;
final int GRIPPER    = 5;

// ==============================
// 🎹 Keyboard
// ==============================
boolean[] keyPressedStates = new boolean[6];
int[] keyboardIncrement   = {0, 0, 0, 0, 0, 0};

// ==============================
// 🎥 3D Camera
// ==============================
float cameraRotationY = 0;
int selectedPart = -1;

// ==============================
// 📼 Samples (Recording/Playback)
// ==============================
@SuppressWarnings("unchecked")
  ArrayList<int[]>[] samples = (ArrayList<int[]>[]) new ArrayList[5];
boolean[] playingBack = new boolean[5];
int[] playbackIndex = new int[5];
int[] lastFrameTime = new int[5];
int playbackInterval = 100;
int selectedSample = -1;
boolean recording = false;

// ==============================
// NEW: Flag to control bounding box updates
// ==============================
boolean boundingBoxesDirty = true; // Mark bounding boxes as needing refresh

// ----------------------------------------------------------
//  SETUP  – window, WebSocket, logs, joystick, initial state
// ----------------------------------------------------------
void setup() {
  // --------------------------------------------------------
  // Window & renderer
  // --------------------------------------------------------
  size(1100, 650, P3D);
  surface.setResizable(false);

  // --------------------------------------------------------
  // WebSocket server (processing‑websockets lib)
  // --------------------------------------------------------
  try {
    wsServer = new WebsocketServer(this, WS_PORT, WS_PATH);
    println("🌐 WebSocket server started on ws://localhost:" + WS_PORT);
    updateMessage("🌐 WS server online (port " + WS_PORT + ")");
  } catch (Exception e) {
    println("❌ Could not start WS server: " + e.getMessage());
    updateMessage("⚠️ WS server failed");
  }

  // --------------------------------------------------------
  // Basic UI parameters
  // --------------------------------------------------------
  textSize(14);
  fill(0);

  // --------------------------------------------------------
  // Logs, limits, sample buffers
  // --------------------------------------------------------
  initializeLog();
  loadServoLimits();

  for (int i = 0; i < samples.length; i++) {
    samples[i] = new ArrayList<int[]>();
  }

  updateMessage("🔄 Initializing …");

  // --------------------------------------------------------
  // GameControlPlus  – joystick detection / XML mapping
  // --------------------------------------------------------
  try {
    control = ControlIO.getInstance(this);

    // ➊  Ensure we have a mapping XML
    File xmlFile = new File(sketchPath("data/gamecontrol.xml"));
    if (!xmlFile.exists()) {
      println("⚠️  gamecontrol.xml not found – generating default mapping …");
      if (control.getNumberOfDevices() > 0) {
        ControlDevice dev = control.getDevice(0);   // first HID device
        createDefaultJoystickXML(dev);              // helper defined elsewhere
      } else {
        println("⛔ No HID devices detected – joystick disabled.");
      }
    }

    // ➋  Load joystick by the default XML name, else fall back
    joystick = control.getDevice("Generic-GCP-Joystick");   // name given in auto‑XML

    if (joystick == null) {
      println("⚠️  Mapping name not found; falling back to first HID device.");
      if (control.getNumberOfDevices() > 0) {
        joystick = control.getDevice(0);
      }
    }

    joystickConnected = (joystick != null);

    if (joystickConnected) {
      println("🕹️  Joystick connected: " + joystick.getName());
      for (ControlInput inp : joystick.getInputs()) {
        println("   • " + inp.getName());
      }
    } else {
      println("⚠️  No joystick detected.");
    }

  } catch (Exception e) {
    println("❌ GameControlPlus initialization error: " + e.getMessage());
    e.printStackTrace();
    joystickConnected = false;
  }
}

// ==============================
// 🎬 Draw
// ==============================
void draw() {
  background(230);
  drawSidebar();
  lights();

  // Recompute bounding boxes ONLY if needed
  if (boundingBoxesDirty) {
    collectArmBoundingBoxes();
    boundingBoxesDirty = false;
  }

  // 3D transformations for viewing
  pushMatrix();
  translate(700, height / 2 + 100);
  rotateX(PI / -6);
  rotateY(cameraRotationY);
  drawArm3D();
  drawCollisionBoxes();
  popMatrix();

  cameraRotationY += cameraRotationIncrement;

  drawHUD();
  drawProgressBar();
  drawControlInstructions();
  drawTextInterface();

  // Based on your initialization steps
  switch (initializationStep) {
  case 0:
    checkLeapMotion();
    break;
  case 1:
    checkLastPort();
    break;
  case 2:
    listSerialPorts();
    break;
  case 3:
    testSerialPorts();
    break;
  case 4:
    connectArduino();
    break;
  case 99:
    // No Arduino found
    break;
  case 100:
    // Normal operation
    updateControls();
    break;
  }
  processPendingWsCommands();
}

// ==============================
// 🔄 System Control Handlers
// ==============================
void updateControls() {
  updateWithLeap();
  updateWithJoystick();
  updateWithKeyboard();
  handleRecording();
  handlePlayback();
}

void updateWithLeap() {
  if (!leapAvailable) return;

  // temporal limiter — 30 ms ≈ 33 fps max for servo updates
  if (millis() - lastLeapUpdateTime < LEAP_UPDATE_PERIOD) return;
  lastLeapUpdateTime = millis();

  for (Hand hand : leap.getHands()) {
    if (hand.isRight()) {
      int newWristV  = (int) map(hand.getPitch(), -60, 60, 0, 180);
      int newWristR  = (int) map(hand.getRoll(),  -60, 60, 0, 180);
      int newGripper = (int) map(hand.getPinchStrength(), 0, 1, 100, 180);

      maybeSetAngle(13, smooth(angles[WRIST_VERT], newWristV, 0.15f));
      maybeSetAngle(14, smooth(angles[WRIST_ROT], newWristR, 0.15f));
      maybeSetAngle(15, smooth(angles[GRIPPER],    newGripper, 0.15f));
    }
    else if (hand.isLeft()) {
      int newBase    = (int) map(hand.getYaw(),   60, -60, 0, 180);
      int newArm     = (int) map(hand.getPitch(), -60, 60, 0, 180);
      int newForearm = (int) map(hand.getRoll(),  -60, 60, 0, 180);

      maybeSetAngle(0, smooth(angles[BASE],    newBase,    0.15f));
      maybeSetAngle(1, smooth(angles[ARM],     newArm,     0.15f));
      maybeSetAngle(2, smooth(angles[FOREARM], newForearm, 0.15f));
    }
  }
}

void updateWithJoystick() {
  if (!joystickConnected || joystick == null) return;

  ControlSlider sx = safeSlider("axis_x", 0);
  ControlSlider sy = safeSlider("axis_y", 1);
  ControlSlider sz = safeSlider("axis_z", 2);
  ControlSlider sr = safeSlider("axis_r", 3);

  if (sx == null || sy == null) return;

  setAngle(0, angles[BASE]        + (int)(sx.getValue() * 2));
  setAngle(1, angles[ARM]         - (int)(sy.getValue() * 2));
  if (sz != null) setAngle(2,  angles[FOREARM] + (int)(sz.getValue() * 2));
  if (sr != null) setAngle(13, angles[WRIST_VERT] - (int)(sr.getValue() * 2));

  ControlButton bClose = safeButton("gripper_close_button", 0);
  ControlButton bOpen  = safeButton("gripper_open_button",  1);
  if (bClose != null && bClose.pressed()) setAngle(15, angles[GRIPPER] + 1);
  if (bOpen  != null &&  bOpen.pressed()) setAngle(15, angles[GRIPPER] - 1);
}

void updateWithKeyboard() {
  // Only every 2 frames
  if (frameCount % 2 != 0) return;
  if (keyPressedStates[0]) setAngle(0, angles[BASE]         + keyboardIncrement[0]);
  if (keyPressedStates[1]) setAngle(1, angles[ARM]          + keyboardIncrement[1]);
  if (keyPressedStates[2]) setAngle(2, angles[FOREARM]      + keyboardIncrement[2]);
  if (keyPressedStates[3]) setAngle(13, angles[WRIST_VERT]  + keyboardIncrement[3]);
  if (keyPressedStates[4]) setAngle(14, angles[WRIST_ROT]   + keyboardIncrement[4]);
  if (keyPressedStates[5]) setAngle(15, angles[GRIPPER]     + keyboardIncrement[5]);
}

void handleRecording() {
  if (recording && selectedSample >= 0 && frameCount % 6 == 0) {
    int[] frame = {
      angles[BASE], angles[ARM], angles[FOREARM],
      angles[WRIST_VERT], angles[WRIST_ROT], angles[GRIPPER]
    };
    samples[selectedSample].add(frame);
  }
}

void handlePlayback() {
  for (int i = 0; i < samples.length; i++) {
    if (playingBack[i] && millis() - lastFrameTime[i] > playbackInterval) {
      if (playbackIndex[i] < samples[i].size()) {
        int[] frame = samples[i].get(playbackIndex[i]);
        setAngle(0, frame[BASE]);
        setAngle(1, frame[ARM]);
        setAngle(2, frame[FOREARM]);
        setAngle(13, frame[WRIST_VERT]);
        setAngle(14, frame[WRIST_ROT]);
        setAngle(15, frame[GRIPPER]);

        playbackIndex[i]++;
        lastFrameTime[i] = millis();
      } else {
        playingBack[i] = false;
        updateMessage("✅ Sample " + (i + 1) + " playback finished.");
      }
    }
  }
}

int smooth(int current, int target, float factor) {
  return int(current * (1 - factor) + target * factor);
}

// ==============================
// 🎮 Input Events
// ==============================
void keyPressed() {
  switch (key) {
  case 'q':
    keyPressedStates[0] = true;
    keyboardIncrement[0] = 1;
    break;
  case 'a':
    keyPressedStates[0] = true;
    keyboardIncrement[0] = -1;
    break;
  case 'w':
    keyPressedStates[1] = true;
    keyboardIncrement[1] = 1;
    break;
  case 's':
    keyPressedStates[1] = true;
    keyboardIncrement[1] = -1;
    break;
  case 'e':
    keyPressedStates[2] = true;
    keyboardIncrement[2] = 1;
    break;
  case 'd':
    keyPressedStates[2] = true;
    keyboardIncrement[2] = -1;
    break;
  case 'r':
    keyPressedStates[3] = true;
    keyboardIncrement[3] = 1;
    break;
  case 'f':
    keyPressedStates[3] = true;
    keyboardIncrement[3] = -1;
    break;
  case 't':
    keyPressedStates[4] = true;
    keyboardIncrement[4] = 1;
    break;
  case 'g':
    keyPressedStates[4] = true;
    keyboardIncrement[4] = -1;
    break;
  case 'y':
    keyPressedStates[5] = true;
    keyboardIncrement[5] = 1;
    break;
  case 'h':
    keyPressedStates[5] = true;
    keyboardIncrement[5] = -1;
    break;

  case CODED:
    if (keyCode == LEFT)  cameraRotationIncrement = -0.02;
    else if (keyCode == RIGHT) cameraRotationIncrement = 0.02;
    break;
  }
}

void keyReleased() {
  switch (key) {
  case 'q':
  case 'a':
    keyPressedStates[0] = false;
    keyboardIncrement[0] = 0;
    break;
  case 'w':
  case 's':
    keyPressedStates[1] = false;
    keyboardIncrement[1] = 0;
    break;
  case 'e':
  case 'd':
    keyPressedStates[2] = false;
    keyboardIncrement[2] = 0;
    break;
  case 'r':
  case 'f':
    keyPressedStates[3] = false;
    keyboardIncrement[3] = 0;
    break;
  case 't':
  case 'g':
    keyPressedStates[4] = false;
    keyboardIncrement[4] = 0;
    break;
  case 'y':
  case 'h':
    keyPressedStates[5] = false;
    keyboardIncrement[5] = 0;
    break;

  case CODED:
    if (keyCode == LEFT || keyCode == RIGHT) cameraRotationIncrement = 0;
    break;
  }
}

void mouseWheel(MouseEvent event) {
  float scroll = event.getCount();
  cameraRotationY += scroll * 0.05;
}

// ==============================
// 🔄 Collision & Bounding Boxes
// ==============================
class Box3D {
  PVector position;
  float w, h, d;
  Box3D(PVector pos, float w, float h, float d) {
    this.position = pos.copy();
    this.w = w;
    this.h = h;
    this.d = d;
  }
  boolean intersects(Box3D other) {
    return abs(this.position.x - other.position.x) < (this.w / 2 + other.w / 2) &&
      abs(this.position.y - other.position.y) < (this.h / 2 + other.h / 2) &&
      abs(this.position.z - other.position.z) < (this.d / 2 + other.d / 2);
  }
}

// Tolerance-based intersection
boolean intersectsWithTolerance(Box3D a, Box3D b, float tolerance) {
  return abs(a.position.x - b.position.x) < ((a.w + b.w) / 2 - tolerance) &&
    abs(a.position.y - b.position.y) < ((a.h + b.h) / 2 - tolerance) &&
    abs(a.position.z - b.position.z) < ((a.d + b.d) / 2 - tolerance);
}

boolean checkCollision() {
  collidingBoxes.clear();
  boolean anyCollision = false;
  for (int i = 0; i < boundingBoxes.size(); i++) {
    for (int j = i + 1; j < boundingBoxes.size(); j++) {
      if (abs(i - j) == 1) continue; // skip immediate neighbors if desired
      Box3D b1 = boundingBoxes.get(i);
      Box3D b2 = boundingBoxes.get(j);
      if (intersectsWithTolerance(b1, b2, 4)) {
        collidingBoxes.add(b1);
        collidingBoxes.add(b2);
        anyCollision = true;
      }
    }
  }
  return anyCollision;
}

void collectArmBoundingBoxes() {
  boundingBoxes.clear();

  pushMatrix();
  translate(700, height / 2 + 100);
  rotateX(-PI / 6);

  // Base
  boundingBoxes.add(new Box3D(modelToWorld(0, -36, 0), 76, 72, 76));

  // Move up to the servo base
  translate(0, -64, 0);
  rotateY(radians(angles[BASE]));
  boundingBoxes.add(new Box3D(modelToWorld(0, 0, 0), 12.8f, 51.2f, 12.8f));
  translate(0, -25.6f, 0);

  // Arm (servo 1)
  pushMatrix();
  rotateZ(radians(angles[ARM] - 90));
  translate(0, -38.4f, 0);
  boundingBoxes.add(new Box3D(modelToWorld(0, 0, 0), 12.8f, 76.8f, 12.8f));
  translate(0, -38.4f, 0);

  // Forearm (servo 2)
  pushMatrix();
  rotateZ(radians(angles[FOREARM] - 180));
  translate(0, -32, 0);
  boundingBoxes.add(new Box3D(modelToWorld(0, 0, 0), 12.8f, 70, 12.8f));
  translate(0, -32, 0);

  // Wrist vertical (servo 13)
  pushMatrix();
  rotateZ(radians(angles[WRIST_VERT] - 90));
  translate(0, -16, 0);
  boundingBoxes.add(new Box3D(modelToWorld(0, 0, 0), 8, 32, 8));
  translate(0, -16, 0);

  // Wrist rotation (servo 14)
  rotateY(radians(angles[WRIST_ROT] - 90));

  // Gripper (servo 15)
  pushMatrix();
  translate(0, -6.4f, 0);

  // Left gripper
  pushMatrix();
  translate(6.4f, 0, 0);
  rotateZ(radians(-(angles[GRIPPER] - 140)));
  translate(0, -6.4f, 0);
  boundingBoxes.add(new Box3D(modelToWorld(0, 0, 0), 2.4f, 32, 4));
  popMatrix();

  // Right gripper
  pushMatrix();
  translate(-6.4f, 0, 0);
  rotateZ(radians(angles[GRIPPER] - 140));
  translate(0, -6.4f, 0);
  boundingBoxes.add(new Box3D(modelToWorld(0, 0, 0), 2.4f, 32, 4));
  popMatrix();

  popMatrix(); // end gripper
  popMatrix(); // end wrist
  popMatrix(); // end forearm
  popMatrix(); // end arm
  popMatrix(); // end base

  // After building, check collisions
  checkCollision();
}

// Purely for drawing the robot arm (no boundingBoxes logic)
void drawArm3D() {
  pushMatrix();
  fill(80);
  drawCylinder(38.4f, 72);
  translate(0, -64, 0);
  rotateY(radians(angles[BASE]));
  box(12.8f, 51.2f, 12.8f);
  translate(0, -25.6f, 0);

  pushMatrix();
  rotateZ(radians(angles[ARM] - 90));
  translate(0, -38.4f, 0);
  fill(200, 100, 100);
  box(12.8f, 76.8f, 12.8f);
  translate(0, -38.4f, 0);

  pushMatrix();
  rotateZ(radians(angles[FOREARM] - 180));
  translate(0, -32, 0);
  fill(100, 200, 100);
  box(12.8f, 70, 12.8f);
  translate(0, -32, 0);

  pushMatrix();
  rotateZ(radians(angles[WRIST_VERT] - 90));
  translate(0, -16, 0);
  fill(100, 100, 200);
  box(8, 32, 8);
  translate(0, -16, 0);

  rotateY(radians(angles[WRIST_ROT] - 90));

  pushMatrix();
  translate(0, -6.4f, 0);

  // Left gripper
  pushMatrix();
  translate(6.4f, 0, 0);
  rotateZ(radians(-(angles[GRIPPER] - 140)));
  translate(0, -6.4f, 0);
  fill(200);
  box(2.4f, 32, 4);
  popMatrix();

  // Right gripper
  pushMatrix();
  translate(-6.4f, 0, 0);
  rotateZ(radians(angles[GRIPPER] - 140));
  translate(0, -6.4f, 0);
  fill(200);
  box(2.4f, 32, 4);
  popMatrix();

  popMatrix(); // gripper
  popMatrix(); // wrist
  popMatrix(); // forearm
  popMatrix(); // arm
  popMatrix(); // base
}

// NEW: draw collision boxes (red) to visualize them
void drawCollisionBoxes() {
  for (Box3D b : collidingBoxes) {
    pushMatrix();
    resetMatrix();
    translate(b.position.x, b.position.y, b.position.z);

    stroke(255, 0, 0);
    fill(255, 0, 0, 80);
    box(b.w, b.h, b.d);
    popMatrix();
  }
}

void drawCylinder(float r, float h) {
  int sides = 30;
  float ang, x, z;
  beginShape(TRIANGLE_FAN);
  vertex(0, -h / 2, 0);
  for (int i = 0; i <= sides; i++) {
    ang = TWO_PI * i / sides;
    x = cos(ang) * r;
    z = sin(ang) * r;
    vertex(x, -h / 2, z);
  }
  endShape();

  beginShape(QUAD_STRIP);
  for (int i = 0; i <= sides; i++) {
    ang = TWO_PI * i / sides;
    x = cos(ang) * r;
    z = sin(ang) * r;
    vertex(x, -h / 2, z);
    vertex(x, h / 2, z);
  }
  endShape();

  beginShape(TRIANGLE_FAN);
  vertex(0, h / 2, 0);
  for (int i = 0; i <= sides; i++) {
    ang = TWO_PI * i / sides;
    x = cos(ang) * r;
    z = sin(ang) * r;
    vertex(x, h / 2, z);
  }
  endShape();
}

PVector modelToWorld(float x, float y, float z) {
  PMatrix3D mat = new PMatrix3D();
  getMatrix(mat);
  float[] src = { x, y, z };
  float[] dst = new float[3];
  mat.mult(src, dst);
  return new PVector(dst[0], dst[1], dst[2]);
}

// ==============================
// 🖥️ UI Elements
// ==============================
void drawSidebar() {
  fill(255);
  stroke(200);
  strokeWeight(1);
  rect(0, 0, 300, height);

  for (int i = 0; i < 5; i++) {
    String label = (selectedSample == i && recording) ? "⏹ Stop Sample " + (i + 1)
      : "▶ Sample " + (i + 1);
    boolean isSelected = (selectedSample == i);
    drawButton(20, 20 + i * 50, 260, 35, label, recording, isSelected);
  }

  drawButton(20, height - 60, 260, 35, "Reconnect", false, false);
}

void drawButton(int x, int y, int w, int h, String label, boolean isRecording, boolean isSelected) {
  if (isRecording && isSelected) {
    fill(255, 100, 100);
  } else {
    fill(240);
  }

  stroke(150);
  strokeWeight(2);
  rect(x, y, w, h, 8);

  fill(0);
  textAlign(CENTER, CENTER);
  text(label, x + w / 2, y + h / 2);
}

void drawHUD() {
  pushMatrix();
  hint(DISABLE_DEPTH_TEST);
  camera();
  textAlign(LEFT, TOP);
  textSize(12);
  fill(20);

  text("📸 Camera Y Rotation: " + nf(degrees(cameraRotationY), 1, 1) + "°", 630, 20);

  color statusColor = color(255);
  String statusLabel = "Disconnected";

  if (arduinoReady) {
    boolean recentlyMoved = (millis() - lastServoCommandTime) < movementTimeout;
    statusColor = recentlyMoved ? color(50, 150, 255) : color(0, 200, 100);
    statusLabel = recentlyMoved ? "Moving" : "Idle";

    fill(70);
    textAlign(LEFT, CENTER);
    text("Last move: " + (millis() - lastServoCommandTime) + " ms ago", 340, height - 45);
  }

  noStroke();
  fill(statusColor);
  ellipse(width - 40, 30, 16, 16);

  fill(0);
  textAlign(RIGHT, CENTER);
  text("Arm Status: " + statusLabel, width - 60, 27);

  hint(ENABLE_DEPTH_TEST);
  popMatrix();
}

void drawControlInstructions() {
  pushMatrix();
  hint(DISABLE_DEPTH_TEST);
  camera();
  textAlign(LEFT, TOP);
  fill(20, 20, 20, 220);
  textSize(12);
  textLeading(16);
  text(controlInstructions, 320, 20);
  hint(ENABLE_DEPTH_TEST);
  popMatrix();
}

void drawTextInterface() {
  textAlign(LEFT, BASELINE);
  fill(0);

  text("🔧 Servo Angles:", 20, 290);
  text("• Base:           " + angles[0], 20, 310);
  text("• Arm:            " + angles[1], 20, 330);
  text("• Forearm:        " + angles[2], 20, 350);
  text("• Wrist Vertical: " + angles[3], 20, 370);
  text("• Wrist Rotation: " + angles[4], 20, 390);
  text("• Gripper:        " + angles[5], 20, 410);

  text("📝 Logs:", 20, 440);
  for (int i = 0; i < logMessages.length; i++) {
    text(logMessages[i], 5, 460 + i * 16);
  }

  if (leapAvailable) {
    text("🖐️ Hands detected: " + leap.countHands(), 20, height - 100);
  } else {
    text("❌ Leap Motion not available", 20, height - 100);
  }

  if (joystickConnected) {
    text("🕹️ Joystick connected", 20, height - 80);
  } else {
    text("⚠️ Joystick not detected", 20, height - 80);
  }
}

// ==============================
// Mouse Interaction
// ==============================
void mousePressed() {

  // 🔁 Reconnect button
  if (mouseX >= 20 && mouseX <= 280 &&
      mouseY >= height - 60 && mouseY <= height - 25) {

    updateMessage("🔄 Restarting connection...");
    initializationStep = 0;
    arduinoReady     = false;
    leapAvailable    = false;
    initializeLog();

    portIndex        = 0;
    arduinoStartTime = 0;
    waiting          = false;
    waitDuration     = 0;
    waitStartTime    = 0;

    if (testPort != null) {
      try { testPort.stop(); }
      catch (Exception e) {
        println("⚠️ Error stopping testPort:");
        e.printStackTrace();
      }
      testPort = null;
    }

    if (myPort != null) {
      try { myPort.stop(); }
      catch (Exception e) {
        println("⚠️ Error stopping myPort:");
        e.printStackTrace();
      }
      myPort = null;
    }

    updateMessage("🔁 Restarting device detection...");
  }

  // ▶️ Sample record / play buttons
  for (int i = 0; i < 5; i++) {
    int y1 = 20 + i * 50;

    if (mouseX >= 20 && mouseX <= 280 &&
        mouseY >= y1 && mouseY <= y1 + 35) {

      // ——— STOP recording ———
      if (recording && selectedSample == i) {
        recording = false;

        File file = new File(sketchPath("data/samples/sample_" + i + ".json"));

        if (!file.exists()) {
          JSONArray jsonArray = new JSONArray();
          for (int[] frame : samples[i]) {
            JSONArray frameArray = new JSONArray();
            for (int val : frame) frameArray.append(val);
            jsonArray.append(frameArray);
          }

          try {
            saveJSONArray(jsonArray, file.getAbsolutePath());
            updateMessage("💾 Sample " + (i + 1) + " saved successfully.");
          } catch (Exception e) {
            println("❌ Error saving sample " + (i + 1) + ": " + e.getMessage());
            e.printStackTrace();
            updateMessage("⚠️ Failed to save sample " + (i + 1));
          }
        } else {
          updateMessage("⚠️ File already exists. Recording not overwritten.");
        }

        selectedSample = -1;
      }

      // ——— START playback or recording ———
      else if (!recording && !isPlaying()) {

        // ▶ Playback
        if (samples[i].size() > 0) {
          playingBack[i]   = true;
          playbackIndex[i] = 0;
          lastFrameTime[i] = millis();
          updateMessage("▶️ Playing sample " + (i + 1));
        }

        // 🔴 Record
        else {
          File file = new File(sketchPath("data/samples/sample_" + i + ".json"));
          if (file.exists()) {
            updateMessage("⚠️ Slot " + (i + 1) + " already has a recording.");
          } else {
            recording      = true;
            selectedSample = i;
            samples[i].clear();
            updateMessage("🔴 Recording sample " + (i + 1));
          }
        }
      }

      // ——— Busy with another sample ———
      else {
        updateMessage("⚠️ Cannot record or play while another sample is active.");
      }
    }
  }
}

// ==============================
// 📋 System Messages
// ==============================
void drawProgressBar() {
  if (initializationStep < 100) {
    float progress = constrain(map(initializationStep, 0, 5, 0, 1), 0, 1);

    float barWidth = 200;
    float barHeight = 15;

    float barX = width / 2 + 160;
    float barY = height / 2 + 220;

    pushMatrix();
    hint(DISABLE_DEPTH_TEST);
    camera();

    fill(200);
    stroke(50);
    rectMode(CENTER);
    rect(barX, barY, barWidth, barHeight, 5);

    noStroke();
    fill(70, 160, 255);
    rectMode(CORNER);
    rect(barX - barWidth / 2, barY - barHeight / 2, barWidth * progress, barHeight, 5);

    fill(20);
    textAlign(CENTER, BOTTOM);
    textSize(14);
    text("Initializing... " + int(progress * 100) + "%", barX, barY - barHeight / 2 - 8);

    hint(ENABLE_DEPTH_TEST);
    popMatrix();
  }
}

void updateMessage(String newMsg) {
  for (int i = logMessages.length - 1; i > 0; i--) {
    logMessages[i] = logMessages[i - 1];
  }
  logMessages[0] = newMsg;
  currentMessage = newMsg;
}

void initializeLog() {
  for (int i = 0; i < logMessages.length; i++) {
    logMessages[i] = "";
  }
  currentMessage = "🔄 Initializing system...";
}

void clearLog() {
  for (int i = 0; i < logMessages.length; i++) {
    logMessages[i] = "";
  }
}

// ==============================
// 💾 Port Management
// ==============================
String loadLastPort() {
  String[] lines = loadStrings("data/lastPt.txt");
  if (lines != null && lines.length > 0) {
    return lines[0];
  }
  return null;
}

void saveLastPort(String port) {
  String[] data = { port };
  saveStrings("data/lastPt.txt", data);
}

// ==============================
// 📊 Servo Limits Loader
// ==============================
void loadServoLimits() {
  File file = new File(sketchPath("data/limits.json"));
  if (!file.exists()) {
    JSONArray jsonArray = new JSONArray();
    int[][] defaultLimits = {
      {0, 180}, {0, 180}, {10, 180}, {0, 180}, {0, 180}, {100, 180}
    };
    for (int i = 0; i < defaultLimits.length; i++) {
      JSONObject obj = new JSONObject();
      obj.setInt("min", defaultLimits[i][0]);
      obj.setInt("max", defaultLimits[i][1]);
      jsonArray.append(obj);
    }
    saveJSONArray(jsonArray, sketchPath("data/limits.json"));
  }

  try {
    JSONArray jsonArray = loadJSONArray("data/limits.json");
    for (int i = 0; i < jsonArray.size() && i < 6; i++) {
      JSONObject obj = jsonArray.getJSONObject(i);
      servoLimits[i][0] = obj.getInt("min");
      servoLimits[i][1] = obj.getInt("max");
    }
  }
  catch (Exception e) {
    for (int i = 0; i < 6; i++) {
      servoLimits[i][0] = 0;
      servoLimits[i][1] = 180;
    }
  }
}

// ==============================
// 🛠️ Initialization
// ==============================
void checkLeapMotion() {
  updateMessage("🔍 Checking Leap Motion...");

  try {
    leap = new LeapMotion(this);
    leapAvailable = true;
    updateMessage("🟢 Leap Motion detected.");
  }
  catch (Exception e) {
    leapAvailable = false;
    leap = null;

    updateMessage("⚠️ Leap Motion NOT detected.");

    // 🧠 Robust Diagnostic Information:
    println("❌ Error initializing Leap Motion:");
    println("  • Type: " + e.getClass().getSimpleName());
    println("  • Message: " + e.getMessage());
    e.printStackTrace();  // Full stack trace for debugging
  }

  startWait(1000);
  initializationStep++;
}

void checkLastPort() {
  if (checkWait()) {
    updateMessage("📁 Checking last used port...");
    String lastPort = loadLastPort();
    ports = Serial.list();

    if (lastPort != null) {
      for (int i = 0; i < ports.length; i++) {
        if (ports[i].equals(lastPort)) {
          updateMessage("🔁 Attempting to reconnect to last port: " + lastPort);
          try {
            testPort = new Serial(this, lastPort, 115200);
            delay(100);
            while (testPort.available() > 0) testPort.read();  // flush buffer
            testPort.bufferUntil('\n');

            arduinoStartTime = millis();
            portIndex = i;
            startWait(500);
            initializationStep = 4;
            return;
          }
          catch (Exception e) {
            println("❌ Error while trying to reconnect to port " + lastPort + ": " + e.getMessage());
            e.printStackTrace();
            updateMessage("⚠ Failed to automatically reconnect to the last used port.");
            break;
          }
        }
      }
    }

    // If reconnection fails or lastPort is null, proceed with standard detection
    startWait(1000);
    initializationStep = 2;
  }
}

void listSerialPorts() {
  if (checkWait()) {
    updateMessage("🔌 Searching for serial ports...");
    ports = Serial.list();
    updateMessage("🔎 " + ports.length + " ports found.");
    startWait(1000);
    initializationStep = 3;
  }
}

void testSerialPorts() {
  if (checkWait()) {
    if (portIndex < ports.length) {
      updateMessage("🔌 Testing port: " + ports[portIndex]);
      try {
        testPort = new Serial(this, ports[portIndex], 115200);
        testPort.clear();
        testPort.bufferUntil('\n');
        arduinoStartTime = millis();
        initializationStep = 4;
      }
      catch (Exception e) {
        println("❌ Error opening port " + ports[portIndex] + ": " + e.getMessage());
        e.printStackTrace();
        updateMessage("❌ Failed to open port: " + ports[portIndex]);
        portIndex++;
        startWait(500);
      }
    } else {
      initializationStep = 99;
      updateMessage("⛔ No Arduino port found.");
    }
  }
}

void connectArduino() {
  if (testPort != null && testPort.available() > 0) {
    String msg = testPort.readStringUntil('\n');
    if (msg != null && msg.trim().equals("SynArm")) {
      myPort = testPort;
      arduinoReady = true;
      clearLog();
      updateMessage("✅ Arduino connected on port: " + ports[portIndex]);
      saveLastPort(ports[portIndex]);

      delay(200);
      myPort.write("READY?\n");

      boolean handshakeOk = false;
      int timeout = millis() + 2000;

      while (millis() < timeout) {
        if (myPort.available() > 0) {
          String reply = myPort.readStringUntil('\n');
          if (reply != null && reply.trim().equals("READY!")) {
            handshakeOk = true;
            break;
          }
        }
      }

      if (!handshakeOk) {
        updateMessage("⚠️ Handshake failed.");
        arduinoReady = false;
        return;
      }

      // Send initial servo angles
      for (int i = 0; i < 6; i++) {
        int channel = (i < 3) ? i : i + 10;
        setAngle(channel, angles[i]);
      }

      // Load any previously saved samples from the "data" folder
      for (int i = 0; i < samples.length; i++) {
        samples[i].clear();
        String samplePath = sketchPath("data/samples/sample_" + i + ".json");
        File sampleFile = new File(samplePath);

        if (sampleFile.exists()) {
          try {
            JSONArray jsonArray = loadJSONArray(samplePath);
            if (jsonArray != null) {
              for (int j = 0; j < jsonArray.size(); j++) {
                JSONArray frameArray = jsonArray.getJSONArray(j);
                int[] frame = new int[frameArray.size()];
                for (int k = 0; k < frame.length; k++) {
                  frame[k] = frameArray.getInt(k);
                }
                samples[i].add(frame);
              }
            } else {
              println("⚠️ JSON array empty or invalid in: sample_" + i + ".json");
            }
          }
          catch (Exception e) {
            println("⚠️ Error loading sample " + i + ": " + e.getMessage());
            e.printStackTrace();
          }
        } else {
          println("ℹ️ Sample file not found: sample_" + i + ".json");
        }
      }


      initializationStep = 100;
    }
  }

  // Timeout: try next port
  if (millis() - arduinoStartTime > 3000) {
    if (testPort != null) {
      try {
        testPort.stop();
      }
      catch (Exception e) {
        println("⚠️ Error stopping testPort: " + e.getMessage());
        e.printStackTrace();
      }
    }
    portIndex++;
    initializationStep = 2;
    startWait(500);
  }
}

// ==============================
// ⏳ Timing Helpers
// ==============================
void startWait(int duration) {
  waiting = true;
  waitDuration = duration;
  waitStartTime = millis();
}

boolean checkWait() {
  if (waiting && millis() - waitStartTime >= waitDuration) {
    waiting = false;
    return true;
  }
  return false;
}

// ==============================
// CHANGED: setAngle to mark boundingBoxesDirty = true
// and handle collision checks
// ==============================
void setAngle(int channel, int value) {
  int index = mapServoIndex(channel);
  if (index == -1) return;

  int constrained = constrain(value, servoLimits[index][0], servoLimits[index][1]);

  // Store old angle
  int oldAngle = angles[index];

  // Build bounding boxes from "current" angles
  boolean wasColliding = checkCollision();

  // Temporarily set new angle and rebuild bounding boxes
  angles[index] = constrained;
  collectArmBoundingBoxes();
  boolean stillColliding = checkCollision();

  // Decide if we allow this move
  if (wasColliding && !stillColliding) {
    // Escaping a collision
    moveServo(channel, constrained);
    boundingBoxesDirty = true;
    updateMessage("✅ Escaping collision on servo " + index);
    lastServoCommandTime = millis();
    return;
  } else if (wasColliding && abs(constrained - oldAngle) <= 4) {
    // Small move to attempt to get out
    moveServo(channel, constrained);
    boundingBoxesDirty = true;
    updateMessage("⚠ Small adjustment allowed in collision (servo " + index + ")");
    lastServoCommandTime = millis();
    return;
  } else if (!stillColliding) {
    // Normal move with no collision
    moveServo(channel, constrained);
    boundingBoxesDirty = true;
    lastServoCommandTime = millis();
    return;
  }

  // Otherwise, revert the angle (blocked by collision)
  angles[index] = oldAngle;
  collectArmBoundingBoxes();
  updateMessage("⛔ Collision: blocked movement on servo " + index);
}

int mapServoIndex(int channel) {
  switch (channel) {
  case 0:
    return 0;
  case 1:
    return 1;
  case 2:
    return 2;
  case 13:
    return 3;
  case 14:
    return 4;
  case 15:
    return 5;
  default:
    return -1;
  }
}

boolean isPlaying() {
  for (boolean pb : playingBack) {
    if (pb) return true;
  }
  return false;
}

boolean isJoystickActive() {
  if (!joystickConnected || joystick == null) return false;
  try {
    float threshold = 0.1;
    if (abs(joystick.getSlider("axis_x").getValue()) > threshold) return true;
    if (abs(joystick.getSlider("axis_y").getValue()) > threshold) return true;
    if (abs(joystick.getSlider("axis_z").getValue()) > threshold) return true;
    if (abs(joystick.getSlider("axis_r").getValue()) > threshold) return true;
    if (joystick.getButton("gripper_close_button").pressed()) return true;
    if (joystick.getButton("gripper_open_button").pressed())  return true;
  }
  catch (Exception e) {
    println("⚠️ Joystick read error: " + e.getMessage());
  }
  return false;
}

//--------------------------------------------------
// 🌐 WebSocket callback (library v0.4.x)
//--------------------------------------------------
void webSocketServerEvent(String msg) {
  pendingWsCommands.add(msg.trim());
}

//--------------------------------------------------
// 📮 Pending WS commands → servo moves / raw serial
//--------------------------------------------------
void processPendingWsCommands() {
  while (!pendingWsCommands.isEmpty()) {
    String cmd = pendingWsCommands.remove(0);
    try {
      JSONObject j = JSONObject.parse(cmd);
      if (j.hasKey("servo") && j.hasKey("angle")) {
        setAngle(j.getInt("servo"), j.getInt("angle"));
      } else if (j.hasKey("serial")) {
        // forward raw command to Arduino
        if (arduinoReady && myPort != null) myPort.write(j.getString("serial") + "\n");
      }
    } catch (Exception e) {
      println("⚠️ Invalid WS msg: " + cmd);
    }
  }
}

//--------------------------------------------------
// 📢 Broadcast every servo move + helper
//--------------------------------------------------
void moveServo(int channel, int angle) {
  if (arduinoReady && myPort != null) {
    myPort.write("S" + channel + " P" + angle + "\n");
  }
  if (wsServer != null) {
    JSONObject j = new JSONObject();
    j.setInt("servo", channel);
    j.setInt("angle", angle);
    j.setInt("timestamp", millis());
    wsServer.sendMessage(j.toString());
  }
}

//--------------------------------------------------
// 📥  Serial event – capture ALL lines
//--------------------------------------------------
void serialEvent(Serial port) {
  if (port != myPort) return;      // ignore other ports

  String line = port.readStringUntil('\n');
  if (line == null) return;
  line = trim(line);

  latestSerialLine = line;         // save last line for UI if needed

  // 1) Broadcast raw serial to WS clients
  if (wsServer != null) {
    JSONObject j = new JSONObject();
    j.setString("serial", line);
    wsServer.sendMessage(j.toString());
  }

  // 2) If the line contains sensor data, parse it
  if (line.startsWith("#SENS|")) parseSensorPacket(line);
}

//--------------------------------------------------
// 🔎 Parse sensor packet like  #SENS|MPU:ax,ay,az|AN:0,1,2
//--------------------------------------------------
void parseSensorPacket(String raw) {
  JSONObject sens = new JSONObject();
  String payload = raw.substring(6);                // remove "#SENS|"
  String[] blocks = split(payload, '|');
  for (String b : blocks) {
    String[] kv = split(b, ':');
    if (kv.length == 2) sens.setString(kv[0], kv[1]);
  }
  latestSensors = sens;   // overwrite global; user can display/use later
}

// ------------------------------------------------------------------
// 🔘 Wrapper: issue servo command only if change > dead‑band
// ------------------------------------------------------------------
void maybeSetAngle(int channel, int value) {
  int idx = mapServoIndex(channel);
  if (idx == -1) return;
  if (abs(value - angles[idx]) >= ANGLE_DEADBAND) {
    setAngle(channel, value);
  }
}

// ---------- Safe helpers (no array API) ----------
ControlSlider safeSlider(String name, int fallbackIdx) {

  try { return joystick.getSlider(name); }
  catch (Exception ignored) { }

  return (fallbackIdx < joystick.getNumberOfSliders())
         ? joystick.getSlider(fallbackIdx)
         : null;
}

ControlButton safeButton(String name, int fallbackIdx) {
  try { return joystick.getButton(name); }
  catch (Exception ignored) { }

  return (fallbackIdx < joystick.getNumberOfButtons())
         ? joystick.getButton(fallbackIdx)
         : null;
}

// ========= generates a minimal XML mapping =========
void createDefaultJoystickXML(ControlDevice dev){
  String[] lines = {
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>",
    "<gameControlPlus>",
    "  <controller name=\"Generic-GCP-Joystick\" type=\"joystick\">",
    "    <slider name=\"axis_x\" index=\"0\" min=\"-1\" max=\"1\" />",
    "    <slider name=\"axis_y\" index=\"1\" min=\"-1\" max=\"1\" />",
    "    <slider name=\"axis_z\" index=\"2\" min=\"-1\" max=\"1\" />",
    "    <slider name=\"axis_r\" index=\"3\" min=\"-1\" max=\"1\" />",
    "    <button name=\"gripper_close_button\" index=\"0\" />",
    "    <button name=\"gripper_open_button\"  index=\"1\" />",
    "  </controller>",
    "</gameControlPlus>"
  };
  saveStrings("data/gamecontrol.xml", lines);
  println("✅ Default gamecontrol.xml created for device: "+dev.getName());
}
