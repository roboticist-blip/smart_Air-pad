# Smart Air-Pad 🎨✨

**Turn your finger or pen into a virtual drawing tool - no touchscreen required!**

A real-time 2D position tracking system using two VL53L0X Time-of-Flight sensors and trilateration geometry to capture air movements and visualize them on screen.

[![Air Pen Prototype](images/Air-pen3.png)](images/Air-pen3.png)

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python](https://img.shields.io/badge/Python-3.7+-blue.svg)](https://www.python.org/downloads/)
[![Platform](https://img.shields.io/badge/Platform-ESP32-red.svg)](https://www.espressif.com/en/products/socs/esp32)

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Key Features](#-key-features)
- [Hardware Requirements](#-hardware-requirements)
- [Software Requirements](#-software-requirements)
- [Installation](#-installation)
- [Hardware Setup](#-hardware-setup)
- [Usage](#-usage)
- [How It Works](#️-how-it-works)
- [Configuration & Tuning](#️-configuration--tuning)
- [Troubleshooting](#-troubleshooting)
- [Project Structure](#-project-structure)
- [Mathematical Background](#-mathematical-background)
- [Future Enhancements](#-future-enhancements)
- [Contributing](#-contributing)
- [License](#-license)

---

## 🎯 Overview

Smart Air-Pad is an innovative touchless interface that tracks the 2D position of a pen tip (or your finger) in real-time using dual VL53L0X ToF sensors. The system employs sophisticated multi-stage filtering to provide smooth, accurate tracking and visualization.

### ✨ Key Features

- ✅ **Real-time position tracking** using trilateration geometry
- ✅ **Multi-stage filtering pipeline** (Median → Kalman → Velocity Gating → EMA)
- ✅ **Intelligent pen up/down detection** for natural stroke management
- ✅ **Multi-stroke recording** with visual trail history
- ✅ **Live debug monitoring** with dual-panel visualization
- ✅ **Highly configurable** parameters for different environments
- ✅ **Low-cost hardware** (~$15 in components)
- ✅ **Easy to build** with minimal soldering

---

## 🔧 Hardware Requirements

### Essential Components

| Component | Quantity | Estimated Cost | Purchase Link |
|-----------|----------|----------------|---------------|
| XIAO ESP32-S3 | 1 | ~$7 | [Seeed Studio](https://www.seeedstudio.com/XIAO-ESP32S3-p-5627.html) |
| VL53L0X ToF Sensor | 2 | ~$3-4 each | [Amazon](https://www.amazon.com/s?k=VL53L0X) / AliExpress |
| Jumper Wires | 10+ | ~$2 | Any electronics store |
| USB-C Cable | 1 | ~$3 | Any USB-C cable |
| Breadboard (optional) | 1 | ~$3 | For prototyping |

**Total Cost: ~$15-20**

### VL53L0X Sensor Specifications

- **Measurement Range:** 30mm to 2000mm
- **Accuracy:** ±3% up to 1000mm
- **Field of View:** 25°
- **Interface:** I2C (address configurable)
- **Supply Voltage:** 2.6V to 3.5V
- **Update Rate:** Up to 50Hz

---

## 💻 Software Requirements

### Development Environment

**Arduino IDE:**
- Arduino IDE 1.8.x or 2.x
- ESP32 Board Support Package
- Libraries: Pololu VL53L0X

**Python Environment:**
- Python 3.7 or higher
- pip (Python package manager)

### Python Dependencies

```bash
pip install pyserial numpy matplotlib
```

**Or install from requirements file:**
```bash
pip install -r requirements.txt
```

---

## 📥 Installation

### Step 1: Clone the Repository

```bash
git clone https://github.com/roboticist-blip/smart_Air-pad.git
cd smart_Air-pad
```

### Step 2: Install Arduino Libraries

**Via Arduino Library Manager:**
1. Open Arduino IDE
2. Go to **Sketch** → **Include Library** → **Manage Libraries**
3. Search for **"VL53L0X"**
4. Install **"VL53L0X by Pololu"**

**Or manually download:**
- [VL53L0X Library](https://github.com/pololu/vl53l0x-arduino)

### Step 3: Install Python Dependencies

**Using pip:**
```bash
pip install pyserial numpy matplotlib
```

**Using requirements.txt:**
```bash
pip install -r requirements.txt
```

### Step 4: Upload ESP32 Code

1. Open `Air-pen/Air-pen.ino` in Arduino IDE
2. Select **Board:** "XIAO_ESP32S3"
3. Select **Port:** Your ESP32's serial port
4. Click **Upload** (or press Ctrl+U)

### Step 5: Configure Serial Port

**Linux:**
```bash
# Find your device
ls /dev/ttyUSB* /dev/ttyACM*

# Grant permissions
sudo chmod 666 /dev/ttyUSB0

# Or add user to dialout group (recommended)
sudo usermod -a -G dialout $USER
# Log out and log back in for changes to take effect
```

**Windows:**
- Open **Device Manager** → **Ports (COM & LPT)**
- Note the COM port (e.g., COM3, COM4)

**macOS:**
```bash
ls /dev/tty.usb*
# Usually /dev/tty.usbserial-XXXX or /dev/tty.usbmodem-XXXX
```

---

## 🔌 Hardware Setup

### Pin Connections

**XIAO ESP32-S3 Wiring Diagram:**

```
VL53L0X Sensor 1 (Left - Anchor A):
┌─────────────┬──────────────────┐
│ VL53L0X Pin │ ESP32-S3 Pin     │
├─────────────┼──────────────────┤
│ VCC         │ 3.3V             │
│ GND         │ GND              │
│ SDA         │ GPIO 5 (I2C SDA) │
│ SCL         │ GPIO 6 (I2C SCL) │
│ XSHUT       │ GPIO 7           │
└─────────────┴──────────────────┘

VL53L0X Sensor 2 (Right - Anchor C):
┌─────────────┬──────────────────┐
│ VL53L0X Pin │ ESP32-S3 Pin     │
├─────────────┼──────────────────┤
│ VCC         │ 3.3V             │
│ GND         │ GND              │
│ SDA         │ GPIO 5 (I2C SDA) │
│ SCL         │ GPIO 6 (I2C SCL) │
│ XSHUT       │ GPIO 8           │
└─────────────┴──────────────────┘
```

### Physical Placement Guide

```
     Sensor A (0,0)              Sensor C (140mm,0)
         ●━━━━━━━━━━━━━━━━━━━━━━━━━━━●
         │    Baseline: 14cm          │
         │                            │
         │      ↓ 10-30cm ↓           │
         │                            │
         │         Pen/Finger         │
         │            ●               │
         │         (x, y)             │
         └────────────────────────────┘
         
     VALID TRACKING ZONE (shaded area)
```

**Setup Tips:**
- ✅ Mount sensors **exactly 14cm (140mm) apart**
- ✅ Both sensors must face the **same direction**
- ✅ Ensure sensors are at the **same height**
- ✅ Keep area **clear of obstacles**
- ✅ Stable mounting (cardboard, wood, or 3D printed holder)
- ✅ Optimal tracking distance: **10-30cm** from baseline

---

## 🚀 Usage

### Running the Visualizer

**Navigate to project directory:**
```bash
cd smart_Air-pad
python Air-pen/air_pen_visualizer.py
```

### Expected Console Output

```
Air-Pen Visualizer with Full Pipeline
============================================================
Pipeline: Raw → Median → Triangulation → Kalman → Velocity Gate → EMA → Pen State
Baseline L = 140 mm
Serial Port = /dev/ttyUSB0
Baud Rate = 115200
============================================================
Connected to /dev/ttyUSB0

Raw: R1=156.0, R2=189.0 → Pos: (62.3, -143.2) → Final: (62.5, -143.0)
Raw: R1=158.0, R2=187.0 → Pos: (63.1, -144.8) → Final: (63.0, -144.5)
✓ PEN DOWN
Movement: 2.3 mm
Velocity: 46.2 mm/s
Strokes: 0
```

### Understanding the Interface

**Left Panel (Main Visualization):**
- 🔴 **Red Dot:** Anchor A (left sensor position)
- 🔵 **Blue Dot:** Anchor C (right sensor position)
- 🟢 **Green Dot:** Real-time pen position
- **Green Line:** Current stroke being drawn
- **Blue Lines:** Previously completed strokes
- **Dashed Lines:** Distance measurements (R1, R2)

**Right Panel (Debug Monitor):**
- 🔴 **Red Line:** Raw unfiltered distance
- 🟢 **Green Line:** Filtered distance output
- 🔵 **Blue Line:** Velocity measurement

**Status Overlay:**
- Pen state indicator (✓ PEN DOWN / ✗ PEN UP)
- Real-time movement distance
- Current velocity in mm/s
- Number of completed strokes

### Basic Operation

1. **Power on** the ESP32 with sensors connected
2. **Run** the Python visualizer
3. **Position your hand/pen** 10-30cm from the baseline
4. **Move slowly** to draw - pen automatically detects down state
5. **Lift quickly** to end stroke (pen up detection)
6. **View trails** of multiple strokes in real-time

---

## ⚙️ How It Works

### Complete System Architecture

```
┌───────────────────────────────────────────────────┐
│              HARDWARE LAYER                       │
│                                                   │
│  VL53L0X #1 (R1) ──┐                            │
│                     ├──→ XIAO ESP32-S3           │
│  VL53L0X #2 (R2) ──┘        │                   │
│                             ↓                     │
│                    Serial @ 115200 baud          │
└───────────────────────────────────────────────────┘
                      ↓
┌───────────────────────────────────────────────────┐
│           SOFTWARE PROCESSING PIPELINE            │
│                                                   │
│  1️⃣  Raw R1, R2 values from sensors              │
│           ↓                                       │
│  2️⃣  Median Filter (window=5)                    │
│       → Removes sensor spikes & noise            │
│           ↓                                       │
│  3️⃣  Trilateration Algorithm                     │
│       → Calculates (x, y) position               │
│           ↓                                       │
│  4️⃣  Kalman Filter (state prediction)            │
│       → Smooths position & estimates velocity    │
│           ↓                                       │
│  5️⃣  Velocity Gating (max 800 mm/s)              │
│       → Rejects unrealistic jumps                │
│           ↓                                       │
│  6️⃣  EMA Smoothing (α=0.3)                       │
│       → Final position refinement                │
│           ↓                                       │
│  7️⃣  Pen State Detection                         │
│       → Determines UP/DOWN from movement         │
│           ↓                                       │
│  8️⃣  Stroke Management                           │
│       → Records & displays drawing trails        │
└───────────────────────────────────────────────────┘
                      ↓
┌───────────────────────────────────────────────────┐
│            VISUALIZATION LAYER                    │
│  • Real-time matplotlib rendering                │
│  • Dual-panel display (main + debug)             │
│  • Live status indicators                        │
└───────────────────────────────────────────────────┘
```

### Trilateration Mathematics

The system uses geometric trilateration to find the pen position from two distance measurements:

**Given:**
- Anchor A at position (0, 0)
- Anchor C at position (L, 0) where L = 140mm
- Distance R1 from A to pen
- Distance R2 from C to pen

**Equations:**
```
R1² = x² + y²           ... Circle centered at A
R2² = (x - L)² + y²     ... Circle centered at C

Solving for x:
x = (L² + R1² - R2²) / (2L)

Solving for y:
y = -√(R1² - x²)        ... Negative for pen below baseline
```

**See detailed derivation in:** [`include/air_pen_geometry.pdf`](include/air_pen_geometry.pdf)

---

## 🎛️ Configuration & Tuning

### Core Parameters

Edit `air_pen_visualizer.py` to adjust these values:

```python
# ========== BASIC CONFIGURATION ==========
L = 140                    # Baseline distance in mm
SERIAL_PORT = '/dev/ttyUSB0'  # Change based on your OS
BAUD_RATE = 115200

# ========== SENSOR VALIDATION ==========
MIN_VALID_DISTANCE = 20    # Minimum sensor reading (mm)
MAX_VALID_DISTANCE = 2000  # Maximum sensor reading (mm)
```

### Filter Fine-Tuning

#### 🎯 Median Filter
```python
MEDIAN_WINDOW = 5  # Recommended: 3-7
# Higher = smoother but slower response
```

#### 🎯 Kalman Filter
```python
PROCESS_NOISE = 0.5      # How much we trust the model
MEASUREMENT_NOISE = 8.0  # How much we trust sensors
# Lower process_noise = smoother tracking
# Higher measurement_noise = less reactive to sensor jumps
```

#### 🎯 Velocity Gating
```python
MAX_VELOCITY = 800  # mm/s - reject faster movements
MIN_VELOCITY = 5    # mm/s - ignore tiny vibrations
# Adjust MAX_VELOCITY based on your drawing speed
```

#### 🎯 EMA Smoothing
```python
EMA_ALPHA = 0.3  # Range: 0.0 to 1.0
# 0.0 = maximum smoothing (very slow response)
# 1.0 = no smoothing (raw Kalman output)
# Recommended: 0.2 - 0.4
```

#### 🎯 Pen State Detection
```python
PEN_DOWN_THRESHOLD = 15   # mm - small movements = drawing
PEN_UP_THRESHOLD = 50     # mm - large movements = lifting
STATIONARY_FRAMES = 3     # Confirmation frames
# Decrease thresholds for more sensitive detection
```

### Distance Calibration

If your measurements don't match reality:

```python
DISTANCE_SCALE = 1.0   # Multiply all distances
DISTANCE_OFFSET = 0    # Add constant offset (mm)

# Example: If distances appear 10% too large
DISTANCE_SCALE = 0.9

# Example: If there's a 5mm systematic error
DISTANCE_OFFSET = -5
```

### Performance Optimization

```python
# Update rate (in FuncAnimation)
interval = 50  # milliseconds (20 Hz)
# Lower = faster but more CPU intensive

# Trail settings
MAX_TRAIL_POINTS = 500  # Max points per stroke
TRAIL_LENGTH = 100      # Visible trail length
```

---

## 🔍 Troubleshooting

### ❌ Serial Port Connection Failed

**Error:** `Error opening serial port: [Errno 2] No such file or directory`

**Solutions:**

**Linux:**
```bash
# List available ports
ls /dev/tty{USB,ACM}*

# Fix permissions (temporary)
sudo chmod 666 /dev/ttyUSB0

# Fix permissions (permanent)
sudo usermod -a -G dialout $USER
# Then logout and login again
```

**Windows:**
- Device Manager → Ports → Note COM number
- Update `SERIAL_PORT = 'COM3'` (or your port)

**macOS:**
```bash
ls /dev/tty.usb*
# Update to actual device path
```

---

### ❌ Sensors Reading 8191mm (Out of Range)

**Console shows:** `Out of range: R1=8191.0, R2=8191.0`

**Causes:**
- 8191 = VL53L0X "no object detected" error code
- Pen/hand too far from sensors (>2m)
- Sensors not powered correctly
- Wiring issue

**Solutions:**
1. ✅ **Check wiring** - Verify all connections, especially VCC and GND
2. ✅ **Move closer** - Position hand/pen 10-30cm from baseline
3. ✅ **Test individually:**
   ```cpp
   // Upload minimal test sketch
   Serial.println(sensor1.readRangeContinuousMillimeters());
   Serial.println(sensor2.readRangeContinuousMillimeters());
   ```
4. ✅ **Check I2C** - Ensure both sensors respond on I2C bus
5. ✅ **Power issue** - ESP32 USB power might be insufficient for two sensors

---

### ❌ Invalid Geometry Errors

**Console shows:** `→ Invalid geometry!`

**Explanation:** Triangle inequality violated (impossible sensor readings)

**Causes:**
- `R1 + R2 < L` (sensors too close)
- `|R1 - R2| > L` (pen outside valid zone)
- Incorrect baseline measurement

**Solutions:**
1. ✅ **Verify L value** - Physically measure sensor spacing
   ```python
   L = 140  # Must match actual distance in mm!
   ```
2. ✅ **Stay in valid zone** - Keep pen in front of both sensors
   ```
   ❌ BAD: Pen to the side
   ✅ GOOD: Pen centered between sensors
   ```
3. ✅ **Check sensor angles** - Both must face same direction

---

### ❌ Noisy/Jittery Movement

**Symptoms:** Position jumps around, unstable trails

**Solutions:**

```python
# Increase smoothing (slower but stabler)
EMA_ALPHA = 0.2
PROCESS_NOISE = 0.3
MEASUREMENT_NOISE = 10.0
MEDIAN_WINDOW = 7

# Reduce velocity limits
MAX_VELOCITY = 500
```

**Environmental factors:**
- ❌ Bright sunlight (affects ToF sensors)
- ❌ Reflective surfaces nearby
- ❌ Unstable sensor mounting

---

### ❌ Movement Too Slow/Laggy

**Symptoms:** Pen position trails behind actual movement

**Solutions:**

```python
# Reduce smoothing (faster but less stable)
EMA_ALPHA = 0.5
PROCESS_NOISE = 0.8
MEASUREMENT_NOISE = 5.0

# Faster update rate
interval = 30  # in FuncAnimation call
```

---

### ❌ Pen State Not Detecting Correctly

**Symptoms:** Doesn't recognize when drawing or lifting

**Solutions:**

```python
# For more sensitive detection
PEN_DOWN_THRESHOLD = 10   # Lower threshold
PEN_UP_THRESHOLD = 60     # Higher threshold
STATIONARY_FRAMES = 5     # More confirmation

# For less sensitive (fewer false triggers)
PEN_DOWN_THRESHOLD = 20
PEN_UP_THRESHOLD = 40
STATIONARY_FRAMES = 2
```

---

### ❌ Distances Seem Wrong

**Symptoms:** Direction correct, but scale is off

**Solutions:**

```python
# If everything appears too large
DISTANCE_SCALE = 0.9

# If everything appears too small
DISTANCE_SCALE = 1.1

# Measure actual baseline with ruler!
L = <actual_measured_distance_in_mm>
```

---

## 📁 Project Structure

```
smart_Air-pad/
│
├── Air-pen/
│   ├── Air-pen.ino              # ESP32 sensor code
│   └── air_pen_visualizer.py    # Python visualization
│
├── images/
│   ├── Air-pen3.png             # Project photos
│   └── ...
│
├── include/
│   └── air_pen_geometry.pdf     # Mathematical derivation
│
├── requirements.txt             # Python dependencies
├── LICENSE                      # MIT License
└── README.md                    # This file
```

---

## 📐 Mathematical Background

### Trilateration Geometry

**Problem:** Given two circles with known centers and radii, find their intersection point.

**Setup:**
- Circle 1: Center A(0, 0), Radius R1
- Circle 2: Center C(L, 0), Radius R2

**Derivation:**

```
Step 1: Define circle equations
  x² + y² = R1²                    ... (1)
  (x - L)² + y² = R2²              ... (2)

Step 2: Expand equation (2)
  x² - 2Lx + L² + y² = R2²

Step 3: Substitute (1) into expanded (2)
  R1² - 2Lx + L² = R2²

Step 4: Solve for x
  x = (R1² - R2² + L²) / (2L)

Step 5: Substitute x back into (1)
  y² = R1² - x²
  y = ±√(R1² - x²)

Step 6: Choose sign based on geometry
  y = -√(R1² - x²)  for pen below baseline
```

**Numerical Stability:**
- Discriminant `R1² - x²` must be ≥ 0
- If negative → Invalid geometry (reject measurement)

**Complete mathematical proof:** See [`include/air_pen_geometry.pdf`](include/air_pen_geometry.pdf)

### Kalman Filter Implementation

**State Vector:** `[x, y, vx, vy]`
- (x, y) = Position
- (vx, vy) = Velocity

**State Transition (Constant Velocity Model):**
```
x_{k+1} = x_k + vx_k * Δt
y_{k+1} = y_k + vy_k * Δt
vx_{k+1} = vx_k * decay
vy_{k+1} = vy_k * decay
```

**Measurement Model:**
Direct position measurement from trilateration.

---

## 📊 Performance Metrics

### Typical Performance

| Metric | Value |
|--------|-------|
| Update Rate | 20-50 Hz |
| Position Accuracy | ±5-10mm |
| System Latency | 50-100ms |
| Working Range | 50mm - 1500mm |
| Angular Coverage | ~120° cone |

### Accuracy vs Distance

| Distance from Baseline | Typical Accuracy |
|------------------------|------------------|
| 50-200mm | ±3-5mm |
| 200-500mm | ±5-10mm |
| 500-1000mm | ±10-20mm |
| 1000-2000mm | ±20-50mm |

---

## 🎨 Use Cases & Applications

1. **✏️ Air Drawing** - Digital art without physical contact
2. **👋 Gesture Control** - Touchless interface for presentations
3. **📍 Position Tracking** - Monitor object movement in 2D
4. **🤖 Robotics** - Simple 2D localization system
5. **🖼️ Interactive Displays** - Museum exhibits, kiosks
6. **📚 Education** - Demonstrate geometry & physics concepts
7. **♿ Accessibility** - Hands-free control for assistive tech

---

## 🔮 Future Enhancements

### Planned Features
- [ ] **3D Tracking** - Add third sensor for Z-axis
- [ ] **Wireless Mode** - WiFi/Bluetooth connectivity
- [ ] **Mobile App** - Android/iOS visualization
- [ ] **Gesture Recognition** - ML-based gesture classification
- [ ] **Multi-User** - Track multiple pens simultaneously
- [ ] **Export Formats** - Save drawings as SVG/PNG
- [ ] **Web Interface** - Browser-based control panel
- [ ] **Auto-Calibration** - Wizard for easy setup

### Community Ideas
Have an idea? [Open an issue](https://github.com/roboticist-blip/smart_Air-pad/issues) or submit a pull request!

---

## 🤝 Contributing

Contributions are welcome! Here's how:

1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/amazing-feature`)
3. **Commit** your changes (`git commit -m 'Add amazing feature'`)
4. **Push** to the branch (`git push origin feature/amazing-feature`)
5. **Open** a Pull Request

### Contribution Guidelines
- Follow existing code style
- Add comments for complex logic
- Test thoroughly before submitting
- Update documentation as needed

---

## 🙏 Acknowledgments

- **Pololu** for the excellent VL53L0X Arduino library
- **Trilateration Research** community for mathematical foundations
- **ESP32 Community** for hardware support and examples
- **Contributors** who helped test and improve this project

---

## 📞 Support & Contact

**Need Help?**
- 📝 [Open an Issue](https://github.com/roboticist-blip/smart_Air-pad/issues)
- 💬 [Start a Discussion](https://github.com/roboticist-blip/smart_Air-pad/discussions)
- ⭐ Star this repo if you find it useful!

**Found a Bug?**
Please report it with:
- Your hardware setup
- Console output
- Steps to reproduce
- Expected vs actual behavior

---

## 🚦 Quick Start Checklist

Before opening an issue, ensure you've completed:

- [ ] ✅ Hardware wired correctly per diagram
- [ ] ✅ Sensors mounted exactly 140mm apart
- [ ] ✅ Arduino IDE configured for XIAO ESP32-S3
- [ ] ✅ VL53L0X library installed
- [ ] ✅ ESP32 code uploaded successfully
- [ ] ✅ Python 3.7+ installed
- [ ] ✅ Python dependencies installed (`pip install -r requirements.txt`)
- [ ] ✅ Serial port permissions granted
- [ ] ✅ Correct serial port configured in code
- [ ] ✅ Baseline distance L measured and verified
- [ ] ✅ Console shows valid sensor readings (not 8191)
- [ ] ✅ Parameters tuned for your environment

---

## ⭐ Star History

If you find this project useful, please consider giving it a star! ⭐

[![Star History Chart](https://api.star-history.com/svg?repos=roboticist-blip/smart_Air-pad&type=Date)](https://star-history.com/#roboticist-blip/smart_Air-pad&Date)

---

**Made with ❤️ by [roboticist-blip](https://github.com/roboticist-blip)**

**Happy Air Drawing! 🎨✨**
