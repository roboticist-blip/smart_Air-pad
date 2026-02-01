import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

# ==================== CONFIGURATION ====================
SERIAL_PORT = '/dev/ttyACM0'  # Change to your port (e.g., '/dev/ttyUSB0' on Linux)
BAUD_RATE = 115200
L = 140  # Baseline distance in mm (14 cm)

# Median Filter Parameters
MEDIAN_WINDOW = 5

# Kalman Filter Parameters
PROCESS_NOISE = 0.5
MEASUREMENT_NOISE = 8.0

# Velocity Gating Parameters
MAX_VELOCITY = 800  # mm/s - maximum realistic pen speed
MIN_VELOCITY = 5    # mm/s - minimum to consider as movement

# EMA Smoothing Parameter
EMA_ALPHA = 0.3  # 0 = no smoothing, 1 = no filtering

# Pen Up/Down Detection
PEN_DOWN_THRESHOLD = 15  # mm - movement below this is "pen down"
PEN_UP_THRESHOLD = 50    # mm - movement above this is "pen up"
STATIONARY_FRAMES = 3    # Frames to confirm pen is stationary

# Display Parameters
SCALE = 3  # pixels per mm
MAX_TRAIL_POINTS = 500  # Maximum points per stroke

# Distance Calibration
DISTANCE_SCALE = 1.0  # Adjust if distances seem off
DISTANCE_OFFSET = 0   # Offset in mm if needed

# ==================== MEDIAN FILTER ====================
class MedianFilter:
    def __init__(self, window_size=5):
        self.window_size = window_size
        self.buffer_r1 = deque(maxlen=window_size)
        self.buffer_r2 = deque(maxlen=window_size)
    
    def filter(self, r1, r2):
        self.buffer_r1.append(r1)
        self.buffer_r2.append(r2)
        
        if len(self.buffer_r1) < self.window_size:
            return r1, r2
        
        r1_filtered = np.median(list(self.buffer_r1))
        r2_filtered = np.median(list(self.buffer_r2))
        
        return r1_filtered, r2_filtered

# ==================== KALMAN FILTER ====================
class KalmanFilter2D:
    def __init__(self, process_noise=0.5, measurement_noise=8.0):
        # State vector: [x, y, vx, vy]
        self.state = np.array([70.0, -100.0, 0.0, 0.0])  # Start at center
        
        # State covariance matrix
        self.P = np.eye(4) * 100
        
        # State transition matrix (constant velocity model)
        self.dt = 0.05  # Assume ~50ms between updates
        self.F = np.array([
            [1, 0, self.dt, 0],
            [0, 1, 0, self.dt],
            [0, 0, 0.95, 0],  # Velocity decay
            [0, 0, 0, 0.95]
        ])
        
        # Measurement matrix
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # Process noise covariance
        self.Q = np.eye(4) * process_noise
        
        # Measurement noise covariance
        self.R = np.eye(2) * measurement_noise
        
    def predict(self):
        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + self.Q
        
    def update(self, measurement):
        y = measurement - (self.H @ self.state)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        
    def get_position(self):
        return self.state[0], self.state[1]
    
    def get_velocity(self):
        vx, vy = self.state[2], self.state[3]
        return np.sqrt(vx**2 + vy**2)

# ==================== VELOCITY GATING ====================
class VelocityGate:
    def __init__(self, max_velocity=800, min_velocity=5, dt=0.05):
        self.max_velocity = max_velocity
        self.min_velocity = min_velocity
        self.dt = dt
        self.last_pos = None
    
    def validate(self, x, y):
        if self.last_pos is None:
            self.last_pos = (x, y)
            return True
        
        dx = x - self.last_pos[0]
        dy = y - self.last_pos[1]
        distance = np.sqrt(dx**2 + dy**2)
        velocity = distance / self.dt
        
        # Check if velocity is realistic
        if velocity > self.max_velocity:
            return False  # Too fast, likely noise
        
        self.last_pos = (x, y)
        return True
    
    def reset(self):
        self.last_pos = None

# ==================== EMA SMOOTHING ====================
class EMAFilter:
    def __init__(self, alpha=0.3):
        self.alpha = alpha
        self.x_smooth = None
        self.y_smooth = None
    
    def filter(self, x, y):
        if self.x_smooth is None:
            self.x_smooth = x
            self.y_smooth = y
            return x, y
        
        self.x_smooth = self.alpha * x + (1 - self.alpha) * self.x_smooth
        self.y_smooth = self.alpha * y + (1 - self.alpha) * self.y_smooth
        
        return self.x_smooth, self.y_smooth
    
    def reset(self):
        self.x_smooth = None
        self.y_smooth = None

# ==================== PEN STATE DETECTION ====================
class PenStateDetector:
    def __init__(self, down_threshold=15, up_threshold=50, stationary_frames=3):
        self.down_threshold = down_threshold
        self.up_threshold = up_threshold
        self.stationary_frames = stationary_frames
        
        self.is_down = False
        self.last_pos = None
        self.stationary_count = 0
        self.last_movement = 0
    
    def update(self, x, y):
        if self.last_pos is None:
            self.last_pos = (x, y)
            return self.is_down
        
        dx = x - self.last_pos[0]
        dy = y - self.last_pos[1]
        movement = np.sqrt(dx**2 + dy**2)
        self.last_movement = movement
        
        # Pen down: small, consistent movement
        if movement < self.down_threshold:
            self.stationary_count += 1
            if self.stationary_count >= self.stationary_frames:
                self.is_down = True
        else:
            self.stationary_count = 0
        
        # Pen up: large, sudden movement
        if movement > self.up_threshold:
            self.is_down = False
            self.stationary_count = 0
        
        self.last_pos = (x, y)
        return self.is_down
    
    def get_movement(self):
        return self.last_movement

# ==================== POSITION CALCULATION ====================
def calculate_position(r1, r2, L):
    """Calculate (x, y) position with calibration"""
    # Apply calibration
    r1 = r1 * DISTANCE_SCALE + DISTANCE_OFFSET
    r2 = r2 * DISTANCE_SCALE + DISTANCE_OFFSET
    
    # Calculate x coordinate
    x = (L**2 + r1**2 - r2**2) / (2 * L)
    
    # Calculate y coordinate
    discriminant = r1**2 - x**2
    
    if discriminant < 0:
        return None
    
    y = -np.sqrt(discriminant)  # Negative y (pen below baseline)
    
    return x, y

# ==================== SERIAL DATA PARSER ====================
def parse_serial_line(line):
    """Parse serial line format"""
    try:
        parts = line.split('|')
        r1_str = parts[0].split(':')[1].strip().replace('mm', '').strip()
        r2_str = parts[1].split(':')[1].strip().replace('mm', '').strip()
        
        r1 = float(r1_str)
        r2 = float(r2_str)
        
        return r1, r2
    except:
        return None

# ==================== VISUALIZATION ====================
class AirPenVisualizer:
    def __init__(self):
        self.fig, (self.ax_main, self.ax_debug) = plt.subplots(1, 2, figsize=(16, 8))
        
        # Initialize all filters
        self.median_filter = MedianFilter(MEDIAN_WINDOW)
        self.kalman = KalmanFilter2D(PROCESS_NOISE, MEASUREMENT_NOISE)
        self.velocity_gate = VelocityGate(MAX_VELOCITY, MIN_VELOCITY)
        self.ema_filter = EMAFilter(EMA_ALPHA)
        self.pen_state = PenStateDetector(PEN_DOWN_THRESHOLD, PEN_UP_THRESHOLD, STATIONARY_FRAMES)
        
        # Stroke storage
        self.current_stroke = []
        self.all_strokes = []
        
        # Debug data
        self.debug_raw_x = deque(maxlen=100)
        self.debug_raw_y = deque(maxlen=100)
        self.debug_filtered_x = deque(maxlen=100)
        self.debug_filtered_y = deque(maxlen=100)
        self.debug_velocity = deque(maxlen=100)
        
        # Setup main plot - store lines as instance variables
        self.anchor_a, = self.ax_main.plot([0], [0], 'ro', markersize=12, label='Anchor A', zorder=5)
        self.anchor_c, = self.ax_main.plot([L], [0], 'bo', markersize=12, label='Anchor C', zorder=5)
        self.pen_point, = self.ax_main.plot([], [], 'go', markersize=15, label='Pen Tip', zorder=10)
        self.current_trail, = self.ax_main.plot([], [], 'g-', alpha=0.6, linewidth=2, zorder=3)
        self.distance_line1, = self.ax_main.plot([], [], 'k--', alpha=0.3, zorder=1)
        self.distance_line2, = self.ax_main.plot([], [], 'k--', alpha=0.3, zorder=1)
        self.pen_state_text = self.ax_main.text(0.02, 0.98, '', transform=self.ax_main.transAxes, 
                                                verticalalignment='top', fontsize=12, 
                                                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # Store stroke lines separately
        self.stroke_lines = []
        
        self.ax_main.set_xlim(-50, L + 50)
        self.ax_main.set_ylim(-300, 50)
        self.ax_main.set_aspect('equal')
        self.ax_main.grid(True, alpha=0.3)
        self.ax_main.legend()
        self.ax_main.set_xlabel('X (mm)')
        self.ax_main.set_ylabel('Y (mm)')
        self.ax_main.set_title('Air-Pen Position Tracking')
        
        # Setup debug plot
        self.debug_line1, = self.ax_debug.plot([], [], 'r-', alpha=0.5, label='Raw')
        self.debug_line2, = self.ax_debug.plot([], [], 'g-', linewidth=2, label='Filtered')
        self.debug_line3, = self.ax_debug.plot([], [], 'b-', label='Velocity')
        self.ax_debug.set_xlim(0, 100)
        self.ax_debug.set_ylim(0, 100)
        self.ax_debug.legend()
        self.ax_debug.set_title('Filter Pipeline Debug')
        self.ax_debug.grid(True, alpha=0.3)
        
        # Serial connection
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            print(f"Connected to {SERIAL_PORT}")
        except Exception as e:
            print(f"Error opening serial port: {e}")
            self.ser = None
    
    def update(self, frame):
        if self.ser is None or not self.ser.is_open:
            return
        
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                
                # Step 1: Parse raw data
                result = parse_serial_line(line)
                if result is None:
                    return
                r1_raw, r2_raw = result
                
                # Step 2: Median filter
                r1_median, r2_median = self.median_filter.filter(r1_raw, r2_raw)
                
                # Step 3: Triangulation
                pos = calculate_position(r1_median, r2_median, L)
                if pos is None:
                    return
                x_raw, y_raw = pos
                
                # Store raw position for debug
                self.debug_raw_x.append(x_raw)
                self.debug_raw_y.append(abs(y_raw))
                
                # Step 4: Kalman filter
                self.kalman.predict()
                self.kalman.update(np.array([x_raw, y_raw]))
                x_kalman, y_kalman = self.kalman.get_position()
                
                # Step 5: Velocity gating
                if not self.velocity_gate.validate(x_kalman, y_kalman):
                    return  # Skip this frame if velocity is unrealistic
                
                # Step 6: EMA smoothing
                x_final, y_final = self.ema_filter.filter(x_kalman, y_kalman)
                
                # Store filtered position for debug
                self.debug_filtered_x.append(x_final)
                self.debug_filtered_y.append(abs(y_final))
                
                # Get velocity
                velocity = self.kalman.get_velocity()
                self.debug_velocity.append(min(velocity / 10, 100))  # Scale for display
                
                # Step 7: Pen up/down logic
                is_pen_down = self.pen_state.update(x_final, y_final)
                
                # Step 8: Stroke trail update
                if is_pen_down:
                    self.current_stroke.append((x_final, y_final))
                    if len(self.current_stroke) > MAX_TRAIL_POINTS:
                        self.current_stroke.pop(0)
                else:
                    if len(self.current_stroke) > 5:  # Save stroke if it has enough points
                        self.all_strokes.append(list(self.current_stroke))
                    self.current_stroke = []
                
                # Update visualization
                self.pen_point.set_data([x_final], [y_final])
                
                # Update current stroke trail
                if self.current_stroke:
                    stroke_x, stroke_y = zip(*self.current_stroke)
                    self.current_trail.set_data(stroke_x, stroke_y)
                else:
                    self.current_trail.set_data([], [])
                
                # Draw all previous strokes
                # Remove old stroke lines
                for line in self.stroke_lines:
                    line.remove()
                self.stroke_lines = []
                
                # Redraw all strokes
                for stroke in self.all_strokes:
                    if len(stroke) > 1:
                        sx, sy = zip(*stroke)
                        line, = self.ax_main.plot(sx, sy, 'b-', alpha=0.4, linewidth=1.5, zorder=2)
                        self.stroke_lines.append(line)
                
                # Update distance lines
                self.distance_line1.set_data([0, x_final], [0, y_final])
                self.distance_line2.set_data([L, x_final], [0, y_final])
                
                # Update status text
                state_str = "✓ PEN DOWN" if is_pen_down else "✗ PEN UP"
                movement = self.pen_state.get_movement()
                self.pen_state_text.set_text(f'{state_str}\nMovement: {movement:.1f} mm\nVelocity: {velocity:.1f} mm/s\nStrokes: {len(self.all_strokes)}')
                
                # Update debug plot
                self.debug_line1.set_data(range(len(self.debug_raw_y)), list(self.debug_raw_y))
                self.debug_line2.set_data(range(len(self.debug_filtered_y)), list(self.debug_filtered_y))
                self.debug_line3.set_data(range(len(self.debug_velocity)), list(self.debug_velocity))
                
        except Exception as e:
            print(f"Error: {e}")
    
    def run(self):
        ani = FuncAnimation(self.fig, self.update, interval=50, blit=False)  # Slower update rate
        plt.show()
    
    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

# ==================== MAIN ====================
if __name__ == "__main__":
    print("Air-Pen Visualizer with Full Pipeline")
    print("=" * 60)
    print(f"Pipeline: Raw → Median → Triangulation → Kalman → Velocity Gate → EMA → Pen State")
    print(f"Baseline L = {L} mm")
    print(f"Serial Port = {SERIAL_PORT}")
    print(f"Baud Rate = {BAUD_RATE}")
    print("=" * 60)
    
    visualizer = AirPenVisualizer()
    
    try:
        visualizer.run()
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        visualizer.close()
        print("Closed.")
