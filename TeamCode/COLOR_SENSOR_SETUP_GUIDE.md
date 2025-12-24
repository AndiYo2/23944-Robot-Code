# Color Sensor Setup Guide

## Overview

Your robot now uses **4 color sensors** with a dual-sensor system for better ball detection:
- **2 sensors on the spindexer** (offset to avoid ball holes)
- **2 sensors below the shooter flipper**

## Hardware Configuration

### 1. Update your Robot Configuration

In the Driver Station app, configure these 4 sensors:
```
spindexerSensor1  (I2C port - First spindexer sensor)
spindexerSensor2  (I2C port - Second spindexer sensor, offset from first)
shooterSensor1    (I2C port - First shooter sensor)
shooterSensor2    (I2C port - Second shooter sensor)
```

### 2. Sensor Positioning

**Spindexer Sensors (offset placement):**
```
     [Ball with holes]
        /         \
     Sensor1    Sensor2
```
- Mount the two sensors **offset from each other**
- This ensures that if one sensor sees through a hole, the other will see the ball
- Spacing: Approximately 30-45 degrees apart works well

**Shooter Sensors:**
- Mount both sensors **below the flipper**
- Position them to detect balls before they're shot
- Use similar offset strategy

## How It Works

### DualColorSensor Class

The `DualColorSensor` combines readings from 2 sensors:

**Detection Strategy (Default):**
- If **EITHER** sensor detects a color → Ball detected ✅
- This prevents holes from causing missed detections

**Alternative Strategy (Conservative):**
```java
// Only returns color if BOTH sensors agree
robot.spindexerColorSensor.getBallColorConservative();
```

## Usage Examples

### Basic Usage
```java
// Refresh sensors before reading
robot.spindexerColorSensor.refreshScan();

// Get detected color
BallColor color = robot.spindexerColorSensor.getBallColor();

if (color == BallColor.Purple) {
    // Do something with purple ball
} else if (color == BallColor.Green) {
    // Do something with green ball
}
```

### Check for Ball Entry
```java
robot.spindexerColorSensor.refreshScan();
if (robot.spindexerColorSensor.ballJustEntered()) {
    // A ball just entered the sensor area
    spindexer.catalogBall(0, robot.spindexerColorSensor.getBallColor());
}
```

### Access Individual Sensors (for debugging)
```java
// Get individual sensor readings
ColorSensorReader sensor1 = robot.spindexerColorSensor.getSensor1();
ColorSensorReader sensor2 = robot.spindexerColorSensor.getSensor2();

telemetry.addData("Sensor 1 Color", sensor1.getBallColor());
telemetry.addData("Sensor 2 Color", sensor2.getBallColor());
telemetry.addData("Combined", robot.spindexerColorSensor.getBallColor());
```

### Telemetry Helper
```java
// Quick telemetry output showing both sensors
telemetry.addData("Spindexer", robot.spindexerColorSensor.getTelemetryString());
// Output: "S1: Purple | S2: None | Combined: Purple"
```

## Available Sensors in RobotHardware

```java
// Individual sensors (if you need direct access)
robot.spindexerSensor1
robot.spindexerSensor2
robot.shooterSensor1
robot.shooterSensor2

// Dual sensor readers (recommended to use these)
robot.spindexerColorSensor  // Combines spindexerSensor1 + spindexerSensor2
robot.shooterColorSensor     // Combines shooterSensor1 + shooterSensor2
```

## Tuning Color Thresholds

If you need to adjust detection thresholds, edit `RobotConstants.java`:

```java
public static class ColorSensor {
    // Color detection thresholds [red, green, blue]
    public static final double[] PURPLE_THRESHOLDS = {0.6, 0.5, 0.4};
    public static final double[] GREEN_THRESHOLDS = {0.3, 0.5, 0.2};
}
```

### How to Tune:
1. Run a test OpMode with telemetry showing raw sensor values
2. Place different colored balls in front of the sensors
3. Note the RGB values for each color
4. Adjust thresholds based on observed values

## Complete Example: Cataloging System

This is already integrated in your `CatalogManager`:

```java
// In TeleOpTemplate.java - This is automatic!
catalogManager.update(intakeActive);

// When you release the intake trigger:
// 1. Both spindexer sensors read the ball color
// 2. Color is cataloged in the spindexer pattern
// 3. Spindexer rotates to next slot
```

## Advanced: Using Shooter Sensors

```java
// Check if a ball is in shooting position
robot.shooterColorSensor.refreshScan();
BallColor ballInShooter = robot.shooterColorSensor.getBallColor();

if (ballInShooter != BallColor.None) {
    // Ready to shoot!
    shooter.triggerShot();
}
```

## Troubleshooting

### Problem: Sensors not detecting balls
**Solution:**
1. Check sensor wiring and I2C connections
2. Verify sensor names in robot configuration
3. Print raw RGB values to telemetry:
   ```java
   telemetry.addData("Detailed", robot.spindexerColorSensor.getDetailedColorData());
   ```
4. Adjust thresholds if needed

### Problem: False positives (detecting balls that aren't there)
**Solution:**
1. Use conservative detection mode:
   ```java
   robot.spindexerColorSensor.getBallColorConservative();
   ```
2. Increase threshold values
3. Check for ambient light interference

### Problem: One sensor works, the other doesn't
**Solution:**
1. Check individual sensors:
   ```java
   telemetry.addData("Sensor 1", robot.spindexerColorSensor.getSensor1().getColorDataString());
   telemetry.addData("Sensor 2", robot.spindexerColorSensor.getSensor2().getColorDataString());
   ```
2. Verify I2C addresses don't conflict
3. Try swapping sensors to isolate hardware issue

## Benefits of This System

✅ **No missed detections** - Holes in balls won't cause problems
✅ **More reliable** - Redundancy improves accuracy
✅ **Flexible** - Can use conservative or aggressive detection
✅ **Easy to debug** - Access to individual sensor data
✅ **Reusable** - Same code works for spindexer and shooter sensors

## Migration Notes

- Old `ColorSensorSubsytem` still exists for backward compatibility
- It now uses `spindexerSensor1` under the hood
- New code should use `DualColorSensor` from `RobotHardware`
