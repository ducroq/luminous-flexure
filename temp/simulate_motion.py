#!/usr/bin/env python3
"""
Luminous Flexure Motion Simulator
Visualizes servo motion patterns and transitions to debug morphing behavior
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math

# --- PATTERN DEFINITIONS (matching Arduino code) ---
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
]

# --- MOTION CONSTANTS ---
CENTER_ANGLE = 90
MIN_ANGLE = 0
MAX_ANGLE = 180
TRANSITION_DURATION = 5000  # ms

# --- EASING FUNCTIONS ---
def smoothstep(x):
    """S-curve easing function"""
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)

def eased_sine(sin_value):
    """Apply S-curve easing to sine wave"""
    normalized = (sin_value + 1.0) / 2.0
    eased = smoothstep(normalized)
    return (eased * 2.0) - 1.0

def calculate_angle(time_ms, phase_offset, period_ms, amplitude, apply_easing=True):
    """Calculate servo angle (matching Arduino implementation)"""
    angle_rad = (time_ms / period_ms) * (2.0 * math.pi)
    phase_rad = (phase_offset / 360.0) * (2.0 * math.pi)
    sin_value = math.sin(angle_rad + phase_rad)

    if apply_easing:
        sin_value = eased_sine(sin_value)

    full_range = MAX_ANGLE - MIN_ANGLE
    swing = (full_range / 2.0) * amplitude
    angle = CENTER_ANGLE + (sin_value * swing)

    return np.clip(angle, MIN_ANGLE, MAX_ANGLE)

def lerp(a, b, t):
    """Linear interpolation"""
    return a + (b - a) * t

# --- SIMULATION WITH PHASE ACCUMULATION ---
def simulate_transition_phase_accumulation(pattern1, pattern2, total_time_ms=20000, dt_ms=50):
    """
    Simulate transition using PHASE ACCUMULATION (the fix!)
    """
    times = np.arange(0, total_time_ms, dt_ms)

    transition_start_ms = total_time_ms // 3
    transition_end_ms = transition_start_ms + TRANSITION_DURATION

    servo1_angles = []
    servo2_angles = []
    in_transition = []

    accumulated_phase = 0.0  # The key: track phase continuously

    for i, t in enumerate(times):
        # Calculate dt
        dt = dt_ms if i > 0 else 0

        if t < transition_start_ms:
            # Before transition
            active_period = pattern1.period_ms
            active_amplitude = pattern1.amplitude
            active_phase = pattern1.phase_difference
            in_trans = False

        elif t < transition_end_ms:
            # During transition - MORPH parameters
            progress = (t - transition_start_ms) / TRANSITION_DURATION
            progress = smoothstep(progress)

            active_period = lerp(pattern1.period_ms, pattern2.period_ms, progress)
            active_amplitude = lerp(pattern1.amplitude, pattern2.amplitude, progress)
            active_phase = lerp(pattern1.phase_difference, pattern2.phase_difference, progress)
            in_trans = True

        else:
            # After transition
            active_period = pattern2.period_ms
            active_amplitude = pattern2.amplitude
            active_phase = pattern2.phase_difference
            in_trans = False

        # PHASE ACCUMULATION: Increment based on current period
        phase_increment = (dt / active_period) * (2.0 * math.pi)
        accumulated_phase += phase_increment

        # Calculate angles from phase
        sin1 = math.sin(accumulated_phase)
        sin2 = math.sin(accumulated_phase + (active_phase / 360.0) * 2.0 * math.pi)

        sin1 = eased_sine(sin1)
        sin2 = eased_sine(sin2)

        swing = ((MAX_ANGLE - MIN_ANGLE) / 2.0) * active_amplitude
        angle1 = CENTER_ANGLE + (sin1 * swing)
        angle2 = CENTER_ANGLE + (sin2 * swing)

        servo1_angles.append(np.clip(angle1, MIN_ANGLE, MAX_ANGLE))
        servo2_angles.append(np.clip(angle2, MIN_ANGLE, MAX_ANGLE))
        in_transition.append(in_trans)

    return times, servo1_angles, servo2_angles, in_transition, transition_start_ms, transition_end_ms

# --- OLD METHOD (for comparison) ---
def simulate_transition_old(pattern1, pattern2, total_time_ms=20000, dt_ms=50):
    """Old time-based method (shows the problem)"""
    times = np.arange(0, total_time_ms, dt_ms)
    transition_start_ms = total_time_ms // 3
    transition_end_ms = transition_start_ms + TRANSITION_DURATION

    servo1_angles = []
    servo2_angles = []
    in_transition = []

    for t in times:
        if t < transition_start_ms:
            angle1 = calculate_angle(t, 0, pattern1.period_ms, pattern1.amplitude)
            angle2 = calculate_angle(t, pattern1.phase_difference, pattern1.period_ms, pattern1.amplitude)
            in_trans = False

        elif t < transition_end_ms:
            progress = (t - transition_start_ms) / TRANSITION_DURATION
            progress = smoothstep(progress)

            morphed_period = lerp(pattern1.period_ms, pattern2.period_ms, progress)
            morphed_amplitude = lerp(pattern1.amplitude, pattern2.amplitude, progress)
            morphed_phase = lerp(pattern1.phase_difference, pattern2.phase_difference, progress)

            angle1 = calculate_angle(t, 0, morphed_period, morphed_amplitude)
            angle2 = calculate_angle(t, morphed_phase, morphed_period, morphed_amplitude)
            in_trans = True

        else:
            angle1 = calculate_angle(t, 0, pattern2.period_ms, pattern2.amplitude)
            angle2 = calculate_angle(t, pattern2.phase_difference, pattern2.period_ms, pattern2.amplitude)
            in_trans = False

        servo1_angles.append(angle1)
        servo2_angles.append(angle2)
        in_transition.append(in_trans)

    return times, servo1_angles, servo2_angles, in_transition, transition_start_ms, transition_end_ms

# --- VISUALIZATION ---
def plot_comparison(pattern1, pattern2):
    """Compare OLD vs NEW (phase accumulation) methods"""
    # OLD method (broken)
    times_old, s1_old, s2_old, _, trans_start, trans_end = simulate_transition_old(pattern1, pattern2)

    # NEW method (fixed!)
    times_new, s1_new, s2_new, _, _, _ = simulate_transition_phase_accumulation(pattern1, pattern2)

    times_sec = np.array(times_old) / 1000.0
    trans_start_sec = trans_start / 1000.0
    trans_end_sec = trans_end / 1000.0

    fig, axes = plt.subplots(2, 2, figsize=(16, 10))
    fig.suptitle(f'OLD vs NEW: {pattern1.name} → {pattern2.name}', fontsize=16, fontweight='bold')

    # OLD Method - Servo 1
    axes[0, 0].plot(times_sec, s1_old, 'b-', linewidth=2)
    axes[0, 0].axvspan(trans_start_sec, trans_end_sec, alpha=0.2, color='red')
    axes[0, 0].set_title('OLD: Time-Based (BROKEN)', fontsize=14, fontweight='bold', color='red')
    axes[0, 0].set_ylabel('Servo 1 Angle (°)', fontsize=11)
    axes[0, 0].set_ylim(MIN_ANGLE - 10, MAX_ANGLE + 10)
    axes[0, 0].grid(True, alpha=0.3)

    # OLD Method - Servo 2
    axes[1, 0].plot(times_sec, s2_old, 'r-', linewidth=2)
    axes[1, 0].axvspan(trans_start_sec, trans_end_sec, alpha=0.2, color='red')
    axes[1, 0].set_ylabel('Servo 2 Angle (°)', fontsize=11)
    axes[1, 0].set_xlabel('Time (seconds)', fontsize=11)
    axes[1, 0].set_ylim(MIN_ANGLE - 10, MAX_ANGLE + 10)
    axes[1, 0].grid(True, alpha=0.3)

    # NEW Method - Servo 1
    axes[0, 1].plot(times_sec, s1_new, 'b-', linewidth=2)
    axes[0, 1].axvspan(trans_start_sec, trans_end_sec, alpha=0.2, color='green')
    axes[0, 1].set_title('NEW: Phase Accumulation (FIXED)', fontsize=14, fontweight='bold', color='green')
    axes[0, 1].set_ylabel('Servo 1 Angle (°)', fontsize=11)
    axes[0, 1].set_ylim(MIN_ANGLE - 10, MAX_ANGLE + 10)
    axes[0, 1].grid(True, alpha=0.3)

    # NEW Method - Servo 2
    axes[1, 1].plot(times_sec, s2_new, 'r-', linewidth=2)
    axes[1, 1].axvspan(trans_start_sec, trans_end_sec, alpha=0.2, color='green')
    axes[1, 1].set_ylabel('Servo 2 Angle (°)', fontsize=11)
    axes[1, 1].set_xlabel('Time (seconds)', fontsize=11)
    axes[1, 1].set_ylim(MIN_ANGLE - 10, MAX_ANGLE + 10)
    axes[1, 1].grid(True, alpha=0.3)

    plt.tight_layout()
    return fig

if __name__ == "__main__":
    print("=" * 70)
    print("LUMINOUS FLEXURE MOTION SIMULATOR")
    print("=" * 70)
    print("\nTesting problematic transition: Deep Meditation → Quick Ripple")
    print("This is the 5x speed-up that caused frantic flipping!\n")

    # Test the problematic transition mentioned by user
    deep_med = PATTERNS[4]  # Deep Meditation: 15000ms period
    quick_ripple = PATTERNS[2]  # Quick Ripple: 3000ms period

    print(f"Pattern 1: {deep_med.name}")
    print(f"  Period: {deep_med.period_ms}ms, Phase: {deep_med.phase_difference}°, Amp: {deep_med.amplitude}")
    print(f"\nPattern 2: {quick_ripple.name}")
    print(f"  Period: {quick_ripple.period_ms}ms, Phase: {quick_ripple.phase_difference}°, Amp: {quick_ripple.amplitude}")
    print(f"\nPeriod change: {deep_med.period_ms / quick_ripple.period_ms:.1f}x speed-up!")

    print("\n" + "=" * 70)
    print("THE PROBLEM:")
    print("=" * 70)
    print("When morphing period from 15000ms to 3000ms:")
    print("  - The sine wave 'frequency' changes dramatically")
    print("  - Same time value maps to different phase positions")
    print("  - This causes PHASE DISCONTINUITIES = jumpy motion")
    print("  - Look at the RED (OLD) graphs - see the chaos!")

    print("\n" + "=" * 70)
    print("THE SOLUTION:")
    print("=" * 70)
    print("Use PHASE ACCUMULATION instead of absolute time:")
    print("  - Track phase angle continuously")
    print("  - Increment phase based on CURRENT period each frame")
    print("  - Phase stays continuous even as period changes")
    print("  - Look at the GREEN (NEW) graphs - smooth as butter!")

    print("\n" + "=" * 70)
    print("Generating visualization...")
    print("=" * 70 + "\n")

    fig = plot_comparison(deep_med, quick_ripple)
    plt.show()

    print("\n✓ The Arduino code has been updated with phase accumulation!")
    print("  Upload it to see smooth morphing between patterns.")
