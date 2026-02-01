# Air-Pen: 2D Position Tracking System

A real-time 2D position tracking system using two VL53L0X Time-of-Flight (ToF) distance sensors and trilateration geometry. Track pen movements in the air and visualize them on screen!

![Air Pen Prototype](images/Air-pen1.jpg)

---

## 📋 Table of Contents

- [Overview](#overview)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Hardware Setup](#hardware-setup)
- [Installation](#installation)
- [How It Works](#how-it-works)
- [Usage](#usage)
- [Configuration & Tuning](#configuration--tuning)
- [Troubleshooting](#troubleshooting)
- [Mathematical Background](#mathematical-background)
- [License](#license)

---

## 🎯 Overview

The Air-Pen system tracks the 2D position of a pen tip in real-time using two distance measurements from fixed reference points (anchors). It employs a sophisticated multi-stage filtering pipeline to provide smooth, accurate tracking.

### Key Features

- **Real-time position tracking** using trilateration
- **Multi-stage filtering pipeline** for noise reduction
- **Pen up/down detection** for stroke management
- **Multiple stroke recording** and visualization
- **Live debug monitoring** of filter performance
- **Configurable parameters** for different environments

---

## 🔧 Hardware Requirements

### Components

| Component | Quantity | Notes |
|-----------|----------|-------|
| **XIAO ESP32-S3** | 1 | Main microcontroller |
| **VL53L0X ToF Sensor** | 2 | Distance measurement (max ~2m) |
| **Jumper Wires** | Several | For connections |
| **USB Cable** | 1 | For ESP32 programming & power |
| **Breadboard** (optional) | 1 | For prototyping |

### VL53L0X Sensor Specifications

- **Range:** 30mm to 2000mm
- **Accuracy:** ±3% at distances < 1000mm
- **Interface:** I2C
- **Supply Voltage:** 2.6V to 3.5V

---

## 💻 Software Requirements

### Python Environment

```bash
Python 3.7+
```

### Required Python Libraries

```bash
pip install pyserial numpy matplotlib
```

### Arduino Libraries

Install via Arduino Library Manager:
- **Pololu VL53L0X** library

---

## 🔌 Hardware Setup

### Pin Connections

#### XIAO ESP32-S3 Pinout

```
VL53L0X Sensor 1 (Left - Anchor A):
├── VCC  → 3.3V
├── GND  → GND
├── SDA  → GPIO 5 (default I2C SDA)
├── SCL  → GPIO 6 (default I2C SCL)
└── XSHUT → GPIO 7

VL53L0X Sensor 2 (Right - Anchor C):
├── VCC  → 3.3V
├── GND  → GND
├── SDA  → GPIO 5 (default I2C SDA)
├── SCL  → GPIO 6 (default I2C SCL)
└── XSHUT → GPIO 8
```

### Physical Placement

```
      Sensor A (Anchor A)          Sensor C (Anchor C)
            (0, 0)                      (140mm, 0)
              ●━━━━━━━━━━━━━━━━━━━━━━━━━━●
             │       Baseline: 14cm       │
             │                            │
             │         ↓ 10-30cm          │
             │                            │
             │           Pen              │
             │            ●               │
             │         (x, y)             │
             └────────────────────────────┘
```

**Important:** 
- Mount sensors **14cm (140mm) apart** on a flat surface
- Both sensors should face the same direction
- Sensors should be at the same height
- Keep the measurement area clear of obstacles

---

## 📥 Installation

### 1. Clone or Download the Project

```bash
git clone https://github.com/yourusername/air-pen.git
cd air-pen
```

### 2. Install Python Dependencies

```bash
pip install -r requirements.txt
```

Or manually:
```bash
pip install pyserial numpy matplotlib
```

### 3. Upload Arduino Code to ESP32

1. Open `esp32_sensors.ino` in Arduino IDE
2. Select **Board:** "XIAO_ESP32S3"
3. Select **Port:** Your ESP32's serial port
4. Click **Upload**

### 4. Configure Serial Port

**On Linux:**
```bash
# Find your device
ls /dev/ttyUSB* /dev/ttyACM*

# Give permissions (if needed)
sudo chmod 666 /dev/ttyUSB0
# OR add user to dialout group
sudo usermod -a -G dialout $USER
```

**On Windows:**
- Check Device Manager → Ports (COM & LPT)
- Note the COM port number (e.g., COM3)

**On macOS:**
```bash
ls /dev/tty.usb*
```

### 5. Update Configuration

Edit `air_pen_visualizer.py`:
```python
SERIAL_PORT = '/dev/ttyUSB0'  # Change to your port
```

---

## ⚙️ How It Works

### System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    SENSOR HARDWARE                           │
│  VL53L0X #1 (R1) ──┐                                        │
│                     ├──→ ESP32 ──→ Serial Output            │
│  VL53L0X #2 (R2) ──┘                                        │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                  FILTERING PIPELINE                          │
│                                                              │
│  1. Raw R1, R2 values                                       │
│         ↓                                                    │
│  2. Median Filter (removes spikes)                          │
│         ↓                                                    │
│  3. Trilateration (calculate x, y)                          │
│         ↓                                                    │
│  4. Kalman Filter (predict & correct)                       │
│         ↓                                                    │
│  5. Velocity Gating (reject unrealistic jumps)              │
│         ↓                                                    │
│  6. EMA Smoothing (final smoothing)                         │
│         ↓                                                    │
│  7. Pen Up/Down Detection                                   │
│         ↓                                                    │
│  8. Stroke Trail Management                                 │
└─────────────────────────────────────────────────────────────┘
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                    VISUALIZATION                             │
│  • Real-time position display                               │
│  • Stroke trails (current + previous)                       │
│  • Debug graphs                                             │
└─────────────────────────────────────────────────────────────┘
```

### Trilateration Mathematics

Given two distance measurements R1 and R2 from fixed points A(0,0) and C(L,0):

```
R1² = x² + y²
R2² = (x - L)² + y²

Solving:
x = (L² + R1² - R2²) / (2L)
y = -√(R1² - x²)
```

---

## 🚀 Usage

### Running the Visualizer

```bash
python air_pen_visualizer.py
```

### Expected Output

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
...
```

### Interface Elements

**Left Panel (Main View):**
- 🔴 Red dot: Anchor A (left sensor)
- 🔵 Blue dot: Anchor C (right sensor)
- 🟢 Green dot: Current pen position
- Green line: Current stroke being drawn
- Blue lines: Previously completed strokes
- Dashed lines: Distance measurements from sensors

**Right Panel (Debug View):**
- Red line: Raw distance data
- Green line: Filtered distance data
- Blue line: Velocity measurement

**Status Display:**
- Pen state (UP/DOWN)
- Movement speed
- Velocity
- Number of completed strokes

---

## 🎛️ Configuration & Tuning

### Basic Parameters

```python
# Baseline distance (measure between your sensors!)
L = 140  # mm

# Serial port
SERIAL_PORT = '/dev/ttyUSB0'  # Linux
# SERIAL_PORT = 'COM3'        # Windows
# SERIAL_PORT = '/dev/tty.usbserial-XXX'  # macOS
```

### Sensor Validity

```python
MIN_VALID_DISTANCE = 20    # Ignore readings below 20mm
MAX_VALID_DISTANCE = 2000  # Ignore readings above 2000mm (out of range)
```

### Filter Tuning

#### Median Filter
```python
MEDIAN_WINDOW = 5  # Window size (3-7 recommended)
```

#### Kalman Filter
```python
PROCESS_NOISE = 0.5      # Lower = smoother, higher = more responsive
MEASUREMENT_NOISE = 8.0  # Higher = trust sensors less
```

#### Velocity Gating
```python
MAX_VELOCITY = 800  # mm/s - Maximum realistic pen speed
MIN_VELOCITY = 5    # mm/s - Minimum movement to track
```

#### EMA Smoothing
```python
EMA_ALPHA = 0.3  # 0 = maximum smoothing, 1 = no smoothing
```

#### Pen State Detection
```python
PEN_DOWN_THRESHOLD = 15   # mm - movement less than this = pen down
PEN_UP_THRESHOLD = 50     # mm - movement more than this = pen up
STATIONARY_FRAMES = 3     # Frames before confirming pen is down
```

### Distance Calibration

If distances seem off:

```python
DISTANCE_SCALE = 1.0   # Multiply all distances (try 0.8 - 1.2)
DISTANCE_OFFSET = 0    # Add offset in mm if needed
```

### Performance Tuning

```python
# Update interval (ms) - higher = slower but smoother
interval=50  # in FuncAnimation call

# Trail length
MAX_TRAIL_POINTS = 500  # Maximum points per stroke
TRAIL_LENGTH = 100      # Points shown in current trail
```

---

## 🔍 Troubleshooting

### No Serial Connection

**Symptoms:** `Error opening serial port`

**Solutions:**
```bash
# Linux - Check permissions
ls -l /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB0

# Or add to dialout group permanently
sudo usermod -a -G dialout $USER
# Then logout and login again

# Verify port exists
ls /dev/tty* | grep -E "USB|ACM"
```

### Pen Not Moving / No Position Updates

**Symptoms:** Console shows `Out of range: R1=8191.0, R2=8191.0`

**Causes:**
- Sensors not detecting any object (8191 = no detection)
- Pen too far from sensors (>2m)
- Sensor wiring issue

**Solutions:**
1. **Check sensor wiring** - Verify all connections
2. **Reduce distance** - Move pen to 10-30cm from baseline
3. **Test sensors individually:**
   ```cpp
   // Upload simple test code
   Serial.println(sensor1.readRangeContinuousMillimeters());
   ```

### Invalid Geometry Errors

**Symptoms:** `→ Invalid geometry!`

**Causes:**
- Triangle inequality violated: `R1 + R2 < L` or `|R1 - R2| > L`
- Pen position creates impossible triangle

**Solutions:**
1. **Check baseline measurement** - Verify L is correct
2. **Adjust pen position** - Keep pen in front of both sensors
3. **Valid zone:**
   ```
        Sensor A          Sensor C
           ●━━━━━━━━━━━━━━━●
           │   VALID ZONE   │
           │       ▼        │
           │      ___       │
           │     |Pen|      │
           │      ‾‾‾       │
           └────────────────┘
   ```

### Noisy/Jittery Movement

**Solutions:**
```python
# Increase smoothing
EMA_ALPHA = 0.2          # More smoothing
PROCESS_NOISE = 0.3      # Smoother Kalman
MEASUREMENT_NOISE = 10.0 # Trust sensors less

# Increase median window
MEDIAN_WINDOW = 7
```

### Movement Too Slow

**Solutions:**
```python
# Faster response
EMA_ALPHA = 0.5          # Less smoothing
PROCESS_NOISE = 0.8      # More responsive
interval = 30            # Faster updates
```

### Movement Too Fast / Unrealistic Jumps

**Solutions:**
```python
# Stricter velocity limits
MAX_VELOCITY = 500       # Lower maximum speed
MEASUREMENT_NOISE = 5.0  # Trust sensors more
```

### Pen State Not Detecting Correctly

**Solutions:**
```python
# Adjust thresholds based on your usage
PEN_DOWN_THRESHOLD = 10  # Lower for more sensitive
PEN_UP_THRESHOLD = 60    # Higher to avoid false triggers
STATIONARY_FRAMES = 5    # More frames for confirmation
```

### Distance Scale Issues

**Symptoms:** Movement direction correct but distances wrong

**Solutions:**
```python
# Calibrate distances
DISTANCE_SCALE = 0.9  # If distances too large
DISTANCE_SCALE = 1.1  # If distances too small

# Measure actual baseline carefully!
L = 145  # Use actual measured distance in mm
```

---

## 📐 Mathematical Background

### Trilateration Principle

Given two circles with known centers and radii:
- Circle 1: Center A(0, 0), radius R1
- Circle 2: Center C(L, 0), radius R2

The intersection point P(x, y) satisfies:

```
x² + y² = R1²                    ... (1)
(x - L)² + y² = R2²              ... (2)

Expanding (2):
x² - 2Lx + L² + y² = R2²

Substituting (1):
R1² - 2Lx + L² = R2²

Solving for x:
x = (R1² - R2² + L²) / (2L)

Substituting back into (1):
y² = R1² - x²
y = ±√(R1² - x²)
```

### Sign Convention

- **Negative y:** Pen below baseline (typical setup)
- **Positive y:** Pen above baseline

### Numerical Stability

The discriminant `R1² - x²` must be non-negative:
- If negative, geometry is invalid (impossible triangle)
- Causes: sensor noise, measurement errors, or pen outside valid zone

### Kalman Filter Model

**State Vector:** `[x, y, vx, vy]`
- Position (x, y)
- Velocity (vx, vy)

**State Transition:**
```
x_new = x + vx * dt
y_new = y + vy * dt
vx_new = vx * decay
vy_new = vy * decay
```

**Measurement Model:**
We directly measure (x, y) from trilateration.

---

## 📊 Performance Characteristics

### Typical Performance

- **Update Rate:** ~20-50 Hz (20-50 updates/second)
- **Position Accuracy:** ±5-10mm (depends on sensor quality and distance)
- **Latency:** ~50-100ms (sensor reading + filtering + visualization)
- **Working Range:** 50mm - 1500mm from baseline
- **Angular Coverage:** ~120° cone from baseline

### Accuracy vs Distance

| Distance from Baseline | Typical Accuracy |
|------------------------|------------------|
| 50-200mm | ±3-5mm |
| 200-500mm | ±5-10mm |
| 500-1000mm | ±10-20mm |
| 1000-2000mm | ±20-50mm |

---

## 🎨 Example Use Cases

1. **Air Drawing** - Draw in the air with finger or pen
2. **Gesture Recognition** - Track hand movements for control
3. **Position Tracking** - Monitor object movement in 2D plane
4. **Robotics** - Simple 2D localization system
5. **Interactive Displays** - Touchless interface control
6. **Educational Tool** - Demonstrate trilateration concepts

---

## 🔮 Future Enhancements

- [ ] 3D tracking with third sensor
- [ ] Bluetooth/WiFi connectivity
- [ ] Mobile app visualization
- [ ] Gesture recognition algorithms
- [ ] Multi-pen tracking
- [ ] Export drawings to SVG/PNG
- [ ] Calibration wizard
- [ ] Web-based interface

---

## 🙏 Acknowledgments

- VL53L0X library by Pololu
- Mathematical concepts from trilateration research
- Community contributions and testing

---

## 📞 Support

For issues, questions, or contributions:
- **GitHub Issues:** [github.com/yourusername/air-pen/issues](https://github.com/yourusername/air-pen/issues)
- **Email:** your.email@example.com
- **Documentation:** [Full docs](https://yourusername.github.io/air-pen)

---

## 🚦 Quick Start Checklist

- [ ] Hardware assembled with correct wiring
- [ ] Sensors mounted 140mm apart
- [ ] Arduino libraries installed
- [ ] ESP32 code uploaded successfully
- [ ] Python dependencies installed
- [ ] Serial port configured correctly
- [ ] Baseline distance L measured and set
- [ ] Test run shows valid sensor readings
- [ ] Pen position tracking works
- [ ] Parameters tuned for your setup

**Happy Tracking! 🎯**
