# Spindexer PIDF Tuning Guide
**Vertical Spindexer with Direction-Specific PIDs**

---

## Understanding Your Setup

Your spindexer is **vertically mounted** with 3 slots at 120° intervals:
- **Slot positions**: 23°, 143°, 263°
- **CW rotation (Clockwise)**: Gravity assists - faster, needs less power
- **CCW rotation (Counter-Clockwise)**: Fighting gravity - slower, needs more power

This is why you have **separate PID sets** for each direction!

---

## Equipment Needed

1. FTC Robot with charged battery
2. Driver station with gamepad
3. Notebook/laptop for recording values
4. Timer/stopwatch (optional but helpful)

---

## Pre-Tuning Preparation

### 1. Verify Hardware
- [ ] Spindexer servo is securely mounted
- [ ] Encoder is properly connected and reading values
- [ ] Spindexer can rotate freely without binding
- [ ] Check that all 3 slots are mechanically sound

### 2. Know Your Controls (in PID Tuning Mode)
- **BACK button**: Enter/Exit PID tuning mode
- **Y button**: Switch between Shooter/Spindexer tuning
- **X button**: Toggle CW/CCW PID (spindexer only)
- **D-Pad Left/Right**: Select parameter (P, I, D, F)
- **D-Pad Up/Down**: Adjust selected parameter
- **B button**: Cycle step size (10.0 → 1.0 → 0.1 → 0.01 → 0.001 → 0.0001)

### 3. Starting Point
Current values in RobotConstants.java:
```
CW (with gravity):
- P: 0.0089
- I: 0.0
- D: 0.0001
- F: 0.0

CCW (against gravity):
- P: 0.0055
- I: 0.02
- D: 0.0003
- F: 0.0003
```

---

## Part 1: Tuning CW Rotation (With Gravity)

CW rotation has gravity helping, so it needs **less aggressive** control to prevent overshooting.

### Step 1: Enter Tuning Mode
1. Press **BACK** button (short rumble = tuning mode ON)
2. Ensure you're tuning spindexer (if not, press **Y**)
3. Press **X** button until display shows **"CW (with gravity)"**

### Step 2: Zero Out PIDs (Start Fresh)
1. Press **B** to set step size to **0.01**
2. Use **D-Pad Left/Right** to select each parameter
3. Use **D-Pad Down** to reduce each to **0.000**
4. Your starting point: P=0, I=0, D=0, F=0

### Step 3: Tune P (Proportional)
**Goal**: Find minimum P that reaches target without excessive oscillation

1. Press **D-Pad Left/Right** until **kP** is highlighted (>>> kP <<<)
2. Set step size to **0.001** (press B until you see 0.001)
3. Trigger a CW rotation (press **RIGHT BUMPER** in normal mode)
4. Start adding P:
   - Press **D-Pad Up** to increment by 0.001
   - Test rotation after each increment
   - **Watch for**:
     - Too low: Rotation doesn't complete, stops short
     - Too high: Overshoots, oscillates back and forth
     - Just right: Reaches target smoothly, minor oscillation acceptable

5. **Expected range**: 0.005 - 0.012
6. **Record your value**: P = _______

**Behavior Guide**:
- If it stops 10°+ short → Increase P
- If it overshoots by 5°+ → Decrease P
- If it oscillates wildly → Decrease P significantly

### Step 4: Add D (Derivative) - Damping
**Goal**: Reduce overshoot and oscillation from P

1. Select **kD** with D-Pad Left/Right
2. Set step size to **0.0001**
3. Start at D = 0.0001
4. Test rotation, gradually increase:
   - Too low: Still overshoots/oscillates
   - Too high: Very slow approach, sluggish
   - Just right: Smooth approach, minimal overshoot

5. **Expected range**: 0.00005 - 0.0003
6. **Record your value**: D = _______

**Note**: Since gravity assists CW, you likely need very little D.

### Step 5: Evaluate I (Integral) - Usually Not Needed
**Goal**: Eliminate steady-state error

1. Test your current P and D values with 5-10 rotations
2. Check final position error (displayed in telemetry)
3. **If error < 3°**: Skip I, leave at 0.0
4. **If error > 3° consistently**:
   - Select kI
   - Set step size to 0.0001
   - Start at 0.0001 and increase slowly
   - **Watch for**: I can cause overshoot if too high
   - Expected range: 0.0 - 0.001

5. **Record your value**: I = _______

### Step 6: Test F (Feedforward) - Optional
**Goal**: Compensate for gravity/friction preemptively

1. **For CW with gravity assist**: Usually F = 0.0 works fine
2. If rotations are sluggish even with good P/D:
   - Try F = 0.0001 to 0.0005
3. Most likely you won't need this

4. **Record your value**: F = _______

### Step 7: Validate CW Tuning
1. Perform **20 test rotations** in a row (RIGHT BUMPER)
2. Monitor telemetry for:
   - Position error (should be < 5°)
   - Settling time (should be < 0.5 seconds)
   - Consistency (all rotations behave similarly)

3. **Pass criteria**:
   - [ ] Reaches target within 5° every time
   - [ ] No oscillation or single small bounce
   - [ ] Completes in reasonable time (< 1 second)

**Record Final CW Values**:
- P: _______
- I: _______
- D: _______
- F: _______

---

## Part 2: Tuning CCW Rotation (Against Gravity)

CCW rotation fights gravity, so it needs **more aggressive** control to overcome resistance.

### Step 1: Switch to CCW Mode
1. Press **X** button while in spindexer tuning mode
2. Display should show **"CCW (against gravity)"**
3. Notice the different starting values

### Step 2: Zero Out PIDs (Start Fresh)
1. Follow same process as CW Step 2
2. Set all parameters to 0.000

### Step 3: Tune P (Proportional)
**Goal**: Find P strong enough to overcome gravity

1. Select **kP**
2. Set step size to **0.001**
3. Trigger CCW rotation (press **LEFT BUMPER**)
4. Start adding P:
   - **Important**: CCW needs MORE P than CW!
   - Start at 0.003 and go up
   - **Watch for**:
     - Too low: Doesn't move or very slow, stops short
     - Too high: Jerky movement, overshoots significantly
     - Just right: Reaches target with authority

5. **Expected range**: 0.004 - 0.010
6. **Record your value**: P = _______

**Behavior Guide**:
- If it barely moves → Increase P significantly (by 0.002-0.003)
- If it's slow but steady → Increase P moderately (by 0.001)
- If it overshoots and bounces → Decrease P slightly

### Step 4: Add I (Integral) - Critical for CCW!
**Goal**: Provide sustained force to fight gravity

**This is where CCW differs most from CW!**

1. Select **kI**
2. Set step size to **0.001**
3. Start at I = 0.005
4. Test and adjust:
   - Too low: Stops short of target, can't make final climb
   - Too high: Slow buildup, then sudden overshoot
   - Just right: Provides steady "push" to complete rotation

5. **Expected range**: 0.01 - 0.03
6. **Record your value**: I = _______

**Pro Tip**: CCW usually needs I to complete the rotation against gravity!

### Step 5: Add D (Derivative) - Prevent Overshoot
**Goal**: Control the approach as it nears target

1. Select **kD**
2. Set step size to **0.0001**
3. Start at D = 0.0002
4. Test and adjust:
   - Too low: Overshoots when I kicks in
   - Too high: Slow, hesitant approach
   - Just right: Controlled deceleration as it approaches

5. **Expected range**: 0.0001 - 0.0005
6. **Record your value**: D = _______

### Step 6: Add F (Feedforward) - Gravity Compensation
**Goal**: Constant upward force to counteract gravity

1. Select **kF**
2. Set step size to **0.0001**
3. Start at F = 0.0002
4. Test and adjust:
   - Too low: Still struggles to lift
   - Too high: Overshoots easily
   - Just right: Smooth continuous lift

5. **Expected range**: 0.0001 - 0.0005
6. **Record your value**: F = _______

**Note**: F provides constant force regardless of error - useful for fighting gravity!

### Step 7: Fine-Tune the Balance
CCW is harder to tune because you're balancing 4 parameters:

1. **If it stops short**:
   - Increase I (primary fix)
   - Or increase F
   - Or increase P slightly

2. **If it overshoots**:
   - Increase D (primary fix)
   - Or decrease P
   - Or decrease I

3. **If it oscillates at target**:
   - Increase D
   - Or decrease P

4. **If it's too slow**:
   - Increase P
   - Or increase F

### Step 8: Validate CCW Tuning
1. Perform **20 test rotations** (LEFT BUMPER)
2. Monitor for:
   - Consistent completion (reaches target every time)
   - Reasonable speed (< 1.5 seconds)
   - Minimal overshoot (< 5°)

3. **Pass criteria**:
   - [ ] Reaches target within 5° every time
   - [ ] Smooth motion throughout
   - [ ] No stalling halfway
   - [ ] Minimal oscillation at end

**Record Final CCW Values**:
- P: _______
- I: _______
- D: _______
- F: _______

---

## Part 3: Combined Testing

### Test Both Directions Together
1. Perform alternating rotations: CW → CCW → CW → CCW
2. Run full 360° test: CCW → CCW → CCW (full rotation)
3. Run reverse 360°: CW → CW → CW (full rotation)

### Check Real-World Scenarios
1. **With balls loaded**: Test with spindexer full vs empty
2. **After running**: Test after robot has been on for 5+ minutes (heat)
3. **Different battery levels**: Test at 13.5V, 12.5V, 11.5V

### Watch For Issues
- [ ] Does CCW struggle when battery is lower?
- [ ] Does CW overshoot when battery is full?
- [ ] Are rotations consistent with weight?

---

## Part 4: Update Your Code

Once you have final values, update `RobotConstants.java`:

```java
// Spindexer PIDF coefficients - direction-specific for vertical mounting
// CW rotation (with gravity assist)
public static final double SPINDEXER_CW_P = _____;  // Your value
public static final double SPINDEXER_CW_I = _____;  // Your value
public static final double SPINDEXER_CW_D = _____;  // Your value
public static final double SPINDEXER_CW_F = _____;  // Your value

// CCW rotation (against gravity)
public static final double SPINDEXER_CCW_P = _____;  // Your value
public static final double SPINDEXER_CCW_I = _____;  // Your value
public static final double SPINDEXER_CCW_D = _____;  // Your value
public static final double SPINDEXER_CCW_F = _____;  // Your value
```

---

## Troubleshooting Guide

### Problem: CW overshoots significantly
**Solution**:
- Decrease P by 0.001-0.002
- Increase D by 0.00005-0.0001

### Problem: CW is too slow
**Solution**:
- Increase P by 0.001
- Decrease D slightly if it's sluggish

### Problem: CCW stops short of target
**Solution**:
- Increase I by 0.005
- Or increase F by 0.0001
- Or increase P by 0.001

### Problem: CCW overshoots
**Solution**:
- Increase D by 0.0001
- Or decrease I by 0.005
- Check if F is too high

### Problem: Both directions oscillate at target
**Solution**:
- Increase D for both
- Decrease P for both
- Verify ANGLE_RANGE tolerance (currently 7°)

### Problem: Inconsistent behavior
**Possible causes**:
- Battery voltage varying (test at consistent voltage)
- Mechanical binding (check servo, encoder)
- Encoder noise (check wiring)
- Temperature (servo performance changes when hot)

---

## Understanding PID Parameters

### P (Proportional)
- **What it does**: Provides force proportional to error
- **Effect**: Bigger error = more power
- **Too low**: Won't reach target
- **Too high**: Overshoots, oscillates

### I (Integral)
- **What it does**: Accumulates error over time, provides sustained push
- **Effect**: Eliminates steady-state error, fights constant forces (gravity)
- **Too low**: Stops short, can't overcome resistance
- **Too high**: Slow buildup then sudden overshoot, oscillation

### D (Derivative)
- **What it does**: Resists changes in error (damping)
- **Effect**: Slows down as it approaches target
- **Too low**: Overshoots
- **Too high**: Sluggish, slow approach

### F (Feedforward)
- **What it does**: Constant force regardless of error
- **Effect**: Preemptive compensation for known forces (gravity, friction)
- **Too low**: Struggles against constant resistance
- **Too high**: Constant overshoot tendency

---

## Expected Behavior Summary

### CW Rotation (Gravity Assist)
- **Speed**: Fast (gravity helps)
- **Overshoot tendency**: High (gravity accelerates)
- **Primary parameters**: P, D
- **Secondary parameters**: I (usually 0), F (usually 0)
- **Tuning difficulty**: Medium

### CCW Rotation (Against Gravity)
- **Speed**: Slower (fighting gravity)
- **Undershoot tendency**: High (gravity resists)
- **Primary parameters**: P, I, F
- **Secondary parameters**: D (still important)
- **Tuning difficulty**: High

---

## Tips for Success

1. **Tune one direction at a time** - Don't jump back and forth
2. **Make small changes** - Increment by step size, don't jump values
3. **Test thoroughly** - 5-10 rotations per change
4. **Record everything** - Write down what works and what doesn't
5. **Be patient** - Good PID tuning takes time
6. **Fresh battery** - Always tune with a well-charged battery (>12.5V)
7. **Consistent environment** - Same location, same temperature
8. **Watch telemetry** - Position error tells you everything
9. **Use step sizes wisely** - Start big (0.001), finish small (0.0001)
10. **Trust the process** - Follow the order: P → I/D → F

---

## Validation Checklist

Before considering tuning complete:

### CW Rotation
- [ ] Completes rotation in < 1 second
- [ ] Final position error < 5°
- [ ] No oscillation (or minimal single bounce)
- [ ] Consistent across 20 tests
- [ ] Works with empty and full spindexer
- [ ] Works at different battery levels (>11.5V)

### CCW Rotation
- [ ] Completes rotation in < 1.5 seconds
- [ ] Final position error < 5°
- [ ] Doesn't stall or stop short
- [ ] Consistent across 20 tests
- [ ] Works with empty and full spindexer
- [ ] Works at different battery levels (>11.5V)

### Both Directions
- [ ] Can complete full 360° rotation (3x CW or 3x CCW)
- [ ] Can alternate directions smoothly
- [ ] No mechanical binding or unusual sounds
- [ ] Encoder readings are stable and accurate

---

## Final Notes

- **Save your work**: Once tuned, commit the values to git immediately!
- **Document**: Note any special conditions (e.g., "works best at 12.5V+")
- **Share with team**: Make sure everyone knows the final values
- **Re-tune if needed**: If you change hardware, you'll need to re-tune
- **Competition day**: Verify tuning still works before matches

Good luck with your tuning! 🎯
