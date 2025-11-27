# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Kinetic light sculptures controlled by an Arduino Nano (ATmega328) using servos to actuate COB LED filament (3V). Multiple design concepts are available.

## Code Versions

Three implementations are available:

1. **`src/light_whip.cpp`** - Light Whip concept **← CURRENTLY ACTIVE**
   - Single servo with expressive gestures
   - Sharp snaps, whips, recoils, tremors
   - Variable timing with dramatic pauses
   - Emphasis on motion trails (light painting in long-exposure)
   - Mechanical precision creating ephemeral drawings in space
   - COB LED mounted at tip of flexible arm (carbon fiber rod, spring steel, etc.)

2. **`src/main_refactored.cpp`** - Smooth Morphing (two-servo original concept)
   - Two SG90 servos, 25cm apart, controlling suspended COB LED filament
   - Gentle wave patterns, figure-eights, Lissajous curves
   - Smooth morphing between patterns with phase accumulation
   - Geometric constraints for LED length
   - Modular class-based architecture

3. **`src/main.cpp`** - Original implementation of smooth morphing
   - First iteration (kept for reference)
   - Organic code growth, single file

Switch versions by editing `platformio.ini` (lines 11-15).

## Build System: PlatformIO

This project uses **PlatformIO** (not Arduino IDE) for building and uploading:

```bash
# Build the project
pio run

# Upload to Arduino Nano
pio run --target upload

# Open serial monitor (115200 baud)
pio device monitor

# Clean build files
pio run --target clean
```

Target board: `nanoatmega328` (Arduino Nano with ATmega328)

## Hardware Configuration

- **Servos**: Connected to digital pins 9 and 10 (`SERVO_PIN_1`, `SERVO_PIN_2` in src/main.cpp:6-7)
- **LED Filament**: Powered separately (not Arduino-controlled), 3V COB flexible LED
- **Servo Motion Range**: 0-180 degrees (`MIN_ANGLE`, `MAX_ANGLE` in src/main.cpp:11-12)

**Critical**: Adjust `MIN_ANGLE` and `MAX_ANGLE` values to prevent physical binding or over-stretching of the LED filament. Test motion range carefully with physical hardware.

## Architecture

### Light Whip System (src/light_whip.cpp) **← CURRENT**

The Light Whip uses a **gesture-based approach** rather than continuous motion:

1. **Gesture Types** (8 defined gestures):
   - `WHIP_LEFT/RIGHT`: Sharp snaps to create motion trails
   - `DOUBLE_SNAP`: Quick succession for staccato effect
   - `SLOW_SWEEP`: Contemplative arc through space
   - `RECOIL`: Wind-up and release for dramatic tension
   - `TREMOR`: Small rapid vibrations
   - `PENDULUM_SWING`: Wide smooth swing with momentum
   - `STACCATO_BURST`: Series of unpredictable movements

2. **Gesture Engine** (`src/light_whip.cpp:49`):
   - Fast movement: 5ms between steps, 3° per step
   - Slow movement: 20ms between steps, 1° per step
   - Each gesture is a choreographed sequence of positions and pauses

3. **Choreographer** (`src/light_whip.cpp:111`):
   - Intelligent gesture selection (avoids repeating same gesture)
   - Variable pause duration:
     - Quick succession: 0.8-1.6s (40% of time)
     - Medium pause: 1.5-2.5s (40% of time)
     - Long anticipation: 3-4s (20% of time)
   - Creates dramatic rhythm of stillness and sudden motion

4. **Hardware Setup**:
   - Single servo on pin 9
   - COB LED mounted at tip of flexible arm (50cm+ recommended)
   - Arm material: carbon fiber rod, spring steel, or similar
   - LED whips through space, creating light trails in long-exposure photography

5. **Modifying Gestures**:
   Add new gestures in `GestureEngine::perform()` (src/light_whip.cpp:90):
   ```cpp
   case MY_NEW_GESTURE:
       snapTo(angle);    // Fast movement
       delay(100);
       sweepTo(angle);   // Slow movement
       returnToCenter(); // Back to rest position
       break;
   ```

6. **Adjusting Timing**:
   Modify constants in `Config` namespace (src/light_whip.cpp:20):
   ```cpp
   const int STEP_DELAY_FAST = 5;      // Lower = faster whips
   const int DEGREES_PER_STEP = 3;     // Higher = more aggressive
   const long MIN_PAUSE_MS = 800;      // Minimum stillness
   const long MAX_PAUSE_MS = 4000;     // Maximum anticipation
   ```

### Organic Motion System (src/main.cpp and src/main_refactored.cpp)

The sculpture uses an **organic motion system** with multiple layers of smoothness and variation:

1. **S-Curve Easing**: All motion uses smoothstep easing (src/main.cpp:212-236)
   - Eliminates harsh velocity changes at direction reversals
   - Creates natural acceleration/deceleration like pendulum motion
   - Applied via `easedSine()` function to sine wave output

2. **Parameter Morphing**: Transitions morph pattern characteristics (src/main.cpp:238-276)
   - **Not cross-fading**: Period, amplitude, and phase gradually evolve
   - Creates single motion that transforms rather than two motions blending
   - 5-second ease-in-out interpolation using `calculateBlendedAngles()`
   - Motion continuously evolves from one quality to another

3. **Pattern Library**: `PATTERNS[]` array (src/main.cpp:50) defines 7 motion presets:
   - Period (oscillation speed)
   - Phase difference (coordination between servos)
   - Amplitude (motion range as 0.0-1.0)
   - Duration (how long to display pattern)

4. **Randomness Layers**:
   - **Pattern selection**: Random order, never repeats same pattern twice consecutively
   - **Parameter variation**: Each instance gets ±20% period, ±15% amplitude/phase, ±25% duration
   - **Organic drift**: Layered smooth noise adds subtle continuous variation (src/main.cpp:193-200)
   - **Unique seed**: Uses analog pin noise for different behavior on each power-up

5. **Geometric Constraints**: `constrainAngle2()` (src/main.cpp:160) dynamically limits servo angles:
   - Calculated LED length between arm endpoints
   - Maximum stretch limit (30 cm)
   - Minimum length to prevent slack (8 cm)

6. **Real-time Physics**: `calculateLEDLength()` (src/main.cpp:123) computes actual distance using trigonometry

### Adding New Motion Patterns

Edit the `PATTERNS[]` array (src/main.cpp:50-58):

```cpp
{"Pattern Name", period_ms, phase_degrees, amplitude, duration_seconds}
```

**Pattern parameters:**
- **period_ms**: Time for one sine wave cycle (3000-15000ms recommended)
- **phase_degrees**: Servo coordination (0=sync, 90=wave, 180=opposite)
- **amplitude**: Motion range (0.25-0.50 recommended for hanging LED)
- **duration_seconds**: How long to display before transitioning

**Example patterns:**
- Breathing: `{8000, 0, 0.30, 30}` - synchronized gentle motion
- Wave: `{8000, 60, 0.40, 40}` - rolling wave effect
- Figure-8: `{10000, 180, 0.45, 45}` - opposite motion creates curves

### Adjusting Motion Characteristics

**S-Curve Easing Strength:**
To adjust or disable S-curve easing, modify `easedSine()` (src/main.cpp:222-236):
```cpp
// For stronger easing (more pronounced acceleration):
return x * x * (3.0 - 2.0 * x);  // Standard smoothstep

// For gentler easing:
return x * x * x * (x * (x * 6 - 15) + 10);  // Smootherstep

// To disable easing (pure sine motion):
// In calculateAngle(), change apply_easing default to false
```

**Randomness Intensity:**
Modify `generatePatternVariations()` (src/main.cpp:108):
```cpp
current_period_variation = randomFloat(0.8, 1.2);      // ±20% - increase range for more variation
current_amplitude_variation = randomFloat(0.85, 1.15); // ±15%
current_phase_variation = random(-15, 16);             // ±15°
current_duration_variation = randomFloat(0.75, 1.25);  // ±25%
```

**Organic Drift Strength:**
Modify noise scaling in `calculateAngle()` (src/main.cpp:197):
```cpp
float drift = (drift1 * 0.1 + drift2 * 0.05);  // Increase multipliers for stronger drift
```

**Morphing Duration:**
Change `TRANSITION_DURATION` constant (src/main.cpp:69):
```cpp
const long TRANSITION_DURATION = 5000;  // 5 seconds (default)
const long TRANSITION_DURATION = 10000; // 10 seconds for slower morphing
```

### Extending the System

1. **Add new waveforms**: Modify `calculateAngle()` to use triangle, square, or Lissajous functions
2. **Dynamic amplitude**: Modulate amplitude over time for breathing effects
3. **Multiple frequencies**: Layer different sine waves for complex harmonics
4. **External control**: Add serial commands to switch patterns manually or adjust randomness
5. **Disable randomness**: Set all variation ranges to `(1.0, 1.0)` and use sequential pattern selection

## Serial Debugging

Serial monitor is configured at **115200 baud** (platformio.ini:5). Add `Serial.begin(115200)` in `setup()` and use `Serial.print()` for debugging servo positions or timing.
