#!/usr/bin/env python3
"""
Luminous Flexure - Full Physical Simulation
Animates the actual kinetic sculpture motion in 2D space
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Rectangle
import math

# --- PATTERN DEFINITIONS ---
class MotionPattern:
    def __init__(self, name, period_ms, phase_difference, amplitude, duration_seconds):
        self.name = name
        self.period_ms = period_ms
        self.phase_difference = phase_difference
        self.amplitude = amplitude
        self.duration_seconds = duration_seconds

PATTERNS = [
    MotionPattern("Gentle Breathing", 8000, 0, 0.30, 30),
    MotionPattern("Slow Wave", 8000, 60, 0.40, 40),
    MotionPattern("Quick Ripple", 3000, 45, 0.25, 25),
    MotionPattern("Figure-8", 10000, 180, 0.45, 45),
    MotionPattern("Deep Meditation", 15000, 90, 0.35, 50),
    MotionPattern("Synchronized Pump", 5000, 0, 0.50, 30),
    MotionPattern("Asymmetric Wave", 7000, 120, 0.38, 35),
]

# --- PHYSICAL PARAMETERS (matching Arduino) ---
SERVO_SEPARATION = 25.0  # cm
ARM_LENGTH = 16.0        # cm (increased for more spatial drama)
LED_MAX_LENGTH = 35.0    # cm (increased to allow fuller motion envelope)
LED_MIN_LENGTH = 8.0     # cm

CENTER_ANGLE = 90        # degrees (straight up)
MIN_ANGLE = 0
MAX_ANGLE = 180

TRANSITION_DURATION = 5000  # ms

# --- EASING FUNCTIONS ---
def smoothstep(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)

def eased_sine(sin_value):
    normalized = (sin_value + 1.0) / 2.0
    eased = smoothstep(normalized)
    return (eased * 2.0) - 1.0

def lerp(a, b, t):
    return a + (b - a) * t

# --- PHYSICAL GEOMETRY ---
def servo_angle_to_cartesian(servo_pos_x, servo_pos_y, angle_degrees, arm_length):
    """
    Convert servo angle to arm endpoint position

    Args:
        servo_pos_x, servo_pos_y: Servo base position
        angle_degrees: Servo angle (90 = straight DOWN, 0 = right, 180 = left)
        arm_length: Length of arm

    Returns:
        (x, y) endpoint position
    """
    # Servo mounted on baseline, arms point DOWNWARD
    # 0° points right, 90° points DOWN, 180° points left
    # Negate angle so 90° points down instead of up
    angle_rad = math.radians(-angle_degrees)

    # Calculate endpoint (y goes downward from baseline)
    x = servo_pos_x + arm_length * math.cos(angle_rad)
    y = servo_pos_y + arm_length * math.sin(angle_rad)

    return x, y

def calculate_led_length(x1, y1, x2, y2):
    """Calculate distance between two points"""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def constrain_angle2(angle1, angle2, servo1_pos, servo2_pos):
    """
    Constrain angle2 based on LED length limits (matching Arduino logic)
    """
    # Calculate endpoint positions
    x1, y1 = servo_angle_to_cartesian(servo1_pos[0], servo1_pos[1], angle1, ARM_LENGTH)
    x2, y2 = servo_angle_to_cartesian(servo2_pos[0], servo2_pos[1], angle2, ARM_LENGTH)

    led_length = calculate_led_length(x1, y1, x2, y2)

    # Check if valid
    if LED_MIN_LENGTH <= led_length <= LED_MAX_LENGTH:
        return angle2

    # Search for nearest valid angle
    center = 90
    offset = abs(angle2 - center)

    for test_offset in range(int(offset), -1, -2):
        test_angle = center + test_offset if angle2 >= center else center - test_offset
        test_angle = np.clip(test_angle, MIN_ANGLE, MAX_ANGLE)

        x2_test, y2_test = servo_angle_to_cartesian(servo2_pos[0], servo2_pos[1], test_angle, ARM_LENGTH)
        test_length = calculate_led_length(x1, y1, x2_test, y2_test)

        if LED_MIN_LENGTH <= test_length <= LED_MAX_LENGTH:
            return test_angle

    return center  # Fallback

def generate_catenary(x1, y1, x2, y2, num_points=20, sag_factor=0.15):
    """
    Generate catenary curve for hanging LED filament with gravity

    Args:
        x1, y1: Start point
        x2, y2: End point
        num_points: Number of points along curve
        sag_factor: How much the cable sags (0 = straight line, higher = more sag)

    Returns:
        xs, ys: Arrays of x, y coordinates
    """
    # Linear interpolation for x and y
    xs = np.linspace(x1, x2, num_points)
    ys = np.linspace(y1, y2, num_points)

    # Calculate distance
    distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    # Simple catenary approximation using parabola
    # (true catenary is more complex, but parabola looks good enough)
    ts = np.linspace(0, 1, num_points)

    # Parabolic sag - always points downward (negative y direction) due to gravity
    sag = sag_factor * distance * (ts * (1 - ts) * 4)  # Max sag at t=0.5

    # Apply sag downward in y direction (gravity pulls down)
    ys = ys - sag  # Subtract to go downward

    return xs, ys

# --- SIMULATION CLASS ---
class SculptureSimulator:
    def __init__(self, pattern1, pattern2=None, show_transition=True, total_time_sec=20):
        self.pattern1 = pattern1
        self.pattern2 = pattern2 if pattern2 else pattern1
        self.show_transition = show_transition
        self.total_time_ms = total_time_sec * 1000

        # Servo positions (baseline)
        self.servo1_pos = (0, 0)
        self.servo2_pos = (SERVO_SEPARATION, 0)

        # Phase accumulation
        self.accumulated_phase = 0.0
        self.last_time = 0

        # Transition timing
        self.transition_start_ms = self.total_time_ms // 3 if show_transition else 999999
        self.transition_end_ms = self.transition_start_ms + TRANSITION_DURATION

        # Setup figure
        self.setup_figure()

    def setup_figure(self):
        """Setup matplotlib figure and axes"""
        self.fig, self.ax = plt.subplots(figsize=(14, 12), facecolor='black')

        # Set up the plot area - arms hang DOWN from baseline (y=0)
        x_margin = ARM_LENGTH + 5  # Allow full arm swing left/right
        y_top_margin = 5  # Small margin above baseline
        y_bottom_margin = ARM_LENGTH * 2 + 10  # Room for arms hanging down + LED sag
        self.ax.set_xlim(-x_margin, SERVO_SEPARATION + x_margin)
        self.ax.set_ylim(-y_bottom_margin, y_top_margin)  # Baseline at y=0, arms below
        self.ax.set_aspect('equal')
        self.ax.set_facecolor('black')  # Dark background like sculpture would be displayed
        self.ax.grid(True, alpha=0.15, color='gray')
        self.ax.set_xlabel('Distance (cm)', fontsize=12, color='white')
        self.ax.set_ylabel('Height (cm)', fontsize=12, color='white')
        self.ax.tick_params(colors='white')
        for spine in self.ax.spines.values():
            spine.set_color('gray')

        # Draw baseline reference (servos mounted here)
        self.ax.axhline(y=0, color='gray', linewidth=4, alpha=0.6, linestyle='-', label='Baseline (Servos)')

        # Visual elements - servos
        self.servo1_circle = Circle(self.servo1_pos, 1.0, color='darkgray', ec='white', linewidth=2, zorder=10)
        self.servo2_circle = Circle(self.servo2_pos, 1.0, color='darkgray', ec='white', linewidth=2, zorder=10)
        self.ax.add_patch(self.servo1_circle)
        self.ax.add_patch(self.servo2_circle)

        # Baseline
        self.ax.plot([self.servo1_pos[0], self.servo2_pos[0]],
                     [self.servo1_pos[1], self.servo2_pos[1]],
                     color='gray', linewidth=2, alpha=0.4, zorder=1)

        # Arms (will be updated) - dimmer so LED stands out
        self.arm1_line, = self.ax.plot([], [], color='steelblue', linewidth=4, label='Arm 1', zorder=5, alpha=0.5)
        self.arm2_line, = self.ax.plot([], [], color='indianred', linewidth=4, label='Arm 2', zorder=5, alpha=0.5)

        # LED filament with glow effect
        self.led_glow, = self.ax.plot([], [], 'yellow', linewidth=15, zorder=2, alpha=0.3)
        self.led_line, = self.ax.plot([], [], 'yellow', linewidth=7,
                                       label='COB LED Filament', zorder=3, alpha=1.0)

        # Info text
        self.info_text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes,
                                       fontsize=12, verticalalignment='top',
                                       bbox=dict(boxstyle='round', facecolor='#333333',
                                                edgecolor='yellow', alpha=0.9),
                                       family='monospace', color='white')

        self.ax.legend(loc='upper right', fontsize=11, facecolor='#333333',
                      edgecolor='gray', labelcolor='white')
        self.ax.set_title('Luminous Flexure - Physical Simulation',
                         fontsize=16, fontweight='bold', color='yellow', pad=20)

    def calculate_state(self, t_ms):
        """Calculate sculpture state at time t"""
        dt_ms = t_ms - self.last_time if self.last_time > 0 else 0
        self.last_time = t_ms

        # Determine active pattern parameters
        if t_ms < self.transition_start_ms:
            active_period = self.pattern1.period_ms
            active_amplitude = self.pattern1.amplitude
            active_phase = self.pattern1.phase_difference
            pattern_name = self.pattern1.name
            in_transition = False
            progress = 0

        elif t_ms < self.transition_end_ms:
            progress = (t_ms - self.transition_start_ms) / TRANSITION_DURATION
            progress_eased = smoothstep(progress)

            active_period = lerp(self.pattern1.period_ms, self.pattern2.period_ms, progress_eased)
            active_amplitude = lerp(self.pattern1.amplitude, self.pattern2.amplitude, progress_eased)
            active_phase = lerp(self.pattern1.phase_difference, self.pattern2.phase_difference, progress_eased)
            pattern_name = f"Morphing ({progress*100:.0f}%)"
            in_transition = True

        else:
            active_period = self.pattern2.period_ms
            active_amplitude = self.pattern2.amplitude
            active_phase = self.pattern2.phase_difference
            pattern_name = self.pattern2.name
            in_transition = False
            progress = 1.0

        # Phase accumulation
        phase_increment = (dt_ms / active_period) * (2.0 * math.pi)
        self.accumulated_phase += phase_increment

        # Calculate servo angles from phase
        sin1 = math.sin(self.accumulated_phase)
        sin2 = math.sin(self.accumulated_phase + math.radians(active_phase))

        sin1 = eased_sine(sin1)
        sin2 = eased_sine(sin2)

        swing = ((MAX_ANGLE - MIN_ANGLE) / 2.0) * active_amplitude
        angle1 = CENTER_ANGLE + (sin1 * swing)
        angle2 = CENTER_ANGLE + (sin2 * swing)

        # Apply geometric constraints
        angle1 = np.clip(angle1, MIN_ANGLE, MAX_ANGLE)
        angle2 = constrain_angle2(angle1, angle2, self.servo1_pos, self.servo2_pos)

        return angle1, angle2, pattern_name, in_transition, progress

    def update(self, frame):
        """Animation update function"""
        t_ms = frame * 50  # 50ms per frame = 20 FPS

        if t_ms > self.total_time_ms:
            return self.arm1_line, self.arm2_line, self.led_glow, self.led_line, self.info_text

        angle1, angle2, pattern_name, in_transition, progress = self.calculate_state(t_ms)

        # Calculate arm endpoints
        x1, y1 = servo_angle_to_cartesian(self.servo1_pos[0], self.servo1_pos[1], angle1, ARM_LENGTH)
        x2, y2 = servo_angle_to_cartesian(self.servo2_pos[0], self.servo2_pos[1], angle2, ARM_LENGTH)

        # Update arms
        self.arm1_line.set_data([self.servo1_pos[0], x1], [self.servo1_pos[1], y1])
        self.arm2_line.set_data([self.servo2_pos[0], x2], [self.servo2_pos[1], y2])

        # Update LED filament with catenary curve
        led_length = calculate_led_length(x1, y1, x2, y2)
        xs, ys = generate_catenary(x1, y1, x2, y2, num_points=40, sag_factor=0.15)

        # Update both glow and main line
        self.led_glow.set_data(xs, ys)
        self.led_line.set_data(xs, ys)

        # Update LED color based on length (visual feedback for constraints)
        if led_length < LED_MIN_LENGTH * 1.1:
            color = 'orange'  # Too short
        elif led_length > LED_MAX_LENGTH * 0.95:
            color = 'red'  # Almost too long
        else:
            color = 'yellow'  # Good

        self.led_line.set_color(color)
        self.led_glow.set_color(color)

        # Update info text
        info = f"Time: {t_ms/1000:.1f}s\n"
        info += f"Pattern: {pattern_name}\n"
        info += f"Servo 1: {angle1:.0f}°\n"
        info += f"Servo 2: {angle2:.0f}°\n"
        info += f"LED Length: {led_length:.1f} cm\n"

        if led_length >= LED_MAX_LENGTH:
            info += "⚠ MAX LENGTH!"
        elif led_length <= LED_MIN_LENGTH:
            info += "⚠ MIN LENGTH!"

        self.info_text.set_text(info)

        return self.arm1_line, self.arm2_line, self.led_glow, self.led_line, self.info_text

    def animate(self, save_filename=None):
        """Run the animation"""
        num_frames = int(self.total_time_ms / 50)  # 20 FPS

        anim = FuncAnimation(self.fig, self.update, frames=num_frames,
                           interval=50, blit=True, repeat=True)

        if save_filename:
            print(f"Saving animation to {save_filename}...")
            anim.save(save_filename, writer='pillow', fps=20)
            print("Done!")

        plt.show()
        return anim

# --- MAIN ---
if __name__ == "__main__":
    print("=" * 70)
    print("LUMINOUS FLEXURE - PHYSICAL SIMULATION")
    print("=" * 70)
    print("\nThis simulation shows the actual kinetic sculpture motion!")
    print("- Baseline (y=0): Horizontal line where servos are mounted")
    print("- Servo arms: Hang DOWNWARD from baseline (90° = straight down)")
    print("- COB LED filament: Yellow glowing curve suspended below baseline")
    print("- Gravity: LED naturally sags downward between the arms")
    print("- Color changes: Orange/Red when near length limits")
    print("- Dark background: Simulates gallery display\n")

    # Choose patterns to simulate
    print("Available patterns:")
    for i, p in enumerate(PATTERNS):
        print(f"  {i+1}. {p.name} ({p.period_ms}ms, {p.phase_difference}°, amp={p.amplitude})")

    print("\n" + "=" * 70)
    print("SIMULATING: Deep Meditation → Quick Ripple")
    print("(The problematic 5x speed-up, now fixed with phase accumulation)")
    print("=" * 70 + "\n")

    sim = SculptureSimulator(
        pattern1=PATTERNS[4],  # Deep Meditation
        pattern2=PATTERNS[2],  # Quick Ripple
        show_transition=True,
        total_time_sec=20
    )

    print("Starting animation... (close window to exit)")
    sim.animate()

    print("\n✓ Try changing patterns in the code to see different motions!")
