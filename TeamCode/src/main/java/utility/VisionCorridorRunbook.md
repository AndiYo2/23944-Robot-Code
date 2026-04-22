# Vision Corridor Runbook — Team 23944 Worlds Prep

Everything left for YOU to do, in order. Agent work (code edits) is already committed on branch `vision-corridor-worlds` (commits `ac357890`, `c49d665f`, `7cd8a8bf`). This file is the human-execution checklist.

---

## 0. Read this section first

**Branch state**: you are on a new branch `vision-corridor-worlds` off `origin/Main`. Three commits ahead. The failed `vision-overhaul` branch is untouched and ignored.

**Rollback at any time**: `git checkout Main` reverts everything in 30 seconds.

**Files modified on this branch** (don't edit by hand unless you have a reason):

- `TeamCode/src/main/java/utility/RobotHardware.java` — webcam config, camera control lock, Panels stream
- `TeamCode/src/main/java/vision/VisionConstants.java` — tunables (calibration, exposure, corridor, Panels fps)
- `TeamCode/src/main/java/vision/ArtifactDetector.java` — NMS fix for adjacent balls
- `TeamCode/src/main/java/vision/CorridorPlanner.java` — NEW, the sweep-heading planner
- `TeamCode/src/main/java/vision/CorridorSelector.java` — NEW (renamed from `LaneSelector`), frame capture + clustering + corridor decision
- `TeamCode/src/main/java/commands/VisionCollectCommand.java` — runtime path construction, 3-of-3 sensor check + geometric safety
- `TeamCode/src/main/java/commands/CommandSequenceBuilder.java` — `visionCollectAndCatalog` signatures dropped `PathChain[]`
- `TeamCode/src/main/java/Autos/RedVisionAuto.java` — removed the three pre-built lane paths
- `TeamCode/src/main/java/Autos/AutonTemplate.java` — telemetry uses `CorridorSelector.lastScanDebug`
- `TeamCode/src/main/java/tests/VisionTuningTeleOp.java` — NEW, the tuning TeleOp

**Deleted**: `vision/LaneSelector.java`, `vision/ChosenPath.java`.

---

## 1. Physical checks (5 min, before touching code)

Do these FIRST. They are the highest-leverage fixes if wrong and they cost nothing to verify.

1. **Logitech C920 USB cable**: must plug into the REV Control Hub's **USB-A 3.0** port (the one with **BLUE** plastic inside). NOT the USB-A 2.0 port. NOT the USB-C port.
   - Why: FTC docs confirm USB 2.0 has a known ESD issue that causes Wi-Fi disconnects mid-match. Losing Wi-Fi = losing robot control. This is a safety hazard.
2. **Cable strain relief**: verify the cable is not pulling on the connector and cannot catch on anything during a match.
3. **Camera mount solidity**: grab the camera, try to wiggle it. Any play means the calibration is wrong — camera pose must stay fixed relative to the intake.
4. **Lens clean**: microfiber cloth, no smudges or fingerprints.
5. **Camera tilted down 20°** (as of 2026-04-22). Measure with a digital level. The mount must be RIGID — any flex during a match craters accuracy. Adjustable in `VisionConstants.CAM_PITCH_DEG` — update the constant if you end up at a different angle.
   - NOTE: the current horizontal-width distance math does NOT use `CAM_PITCH_DEG` — tilting is purely a field-of-view decision to keep balls IN the image at your working range. The constant is there for documentation and future ground-plane-projection work.
   - At 20° down with a 3" lens height, the far edge of the image is ~75" from the lens and the near edge is ~3.5" from the lens — covers your 4-5 ft working range with margin. Because the tilt puts the whole image on the field (no horizon, no audience), the vertical ROI clip is also gone — ROI is now the full frame.

If any of these are wrong, fix them before anything else. Nothing below will work if the camera drops out mid-match.

---

## 2. Build and deploy (15-30 min)

You need these on your build machine:
- Android Studio (Flamingo / Hedgehog / whatever your team uses — the one that FTC SDK 11.x builds in)
- Java 17 (FTC SDK does NOT work with Java 21+; the agent's machine had Java 26-ea which is why the agent couldn't build)
- Android SDK configured with `ANDROID_HOME` set

### 2.1 Pull the branch

```bash
cd /Users/andysmith/Documents/GitHub/23944
git fetch origin                 # only if you have a remote you want to sync
git status                       # should show "On branch vision-corridor-worlds, nothing to commit, working tree clean"
git log --oneline -4             # should show the three new commits on top of bc0b24de
```

Expected top of log:
```
7cd8a8bf Revert to strict 3-of-3 sensor check for isFull
c49d665f Refinements: Panels preview, merged ball visibility, smarter path cut-short
ac357890 Vision overhaul: YCrCb SDK presets, camera lock, corridor planner
bc0b24de ACTUAL fix for shooting      ← this is Main's head
```

### 2.2 Build

Open the repo in Android Studio. Let it sync Gradle (may take 1-5 min first time). Then:

```bash
./gradlew assembleDebug
```

or use Android Studio's build button.

### 2.3 If the build fails

**The agent could not run `./gradlew` due to Java 26-ea on its environment, so the code has NOT been compile-verified.** Expect one or two compile errors. Most likely:

- **`PanelsCameraStream`** import — if the bylazar Panels library version on Main doesn't expose `INSTANCE.startStream(visionPortal, fps)` with that exact signature, you'll get a method-not-found. The original Main code at `RobotHardware.java` used it exactly like this, so it should be fine.
- **`android.util.Log`** in `CorridorSelector.java` — should be present on Android target, no extra import needed in most setups.
- **`org.json.JSONObject`** in `VisionTuningTeleOp.java` — should be on the Android classpath. If missing, remove the JSON read/write and use `java.util.Properties` instead.

If you see compile errors, paste them to the agent and it'll fix them.

### 2.4 Deploy

Via Android Studio's "Run" button to the Control Hub, or `./gradlew installDebug` with ADB connected.

---

## 3. First-boot smoke test (15 min)

On the driver station, you should now see two NEW items in the OpMode list:
- **Tests → Vision Tuning**
- **Autonomous → RedVisionAuto** (it's been modified, not added — but the behavior is different)

### 3.1 Run "Vision Tuning"

No need to point at balls yet. You're just verifying the pipeline boots.

Expected telemetry on screen:
```
LoopMs avg      : <less than 50 ms sustained>
Exposure (LB/RB): 5 ms
Gain (DPAD L/R) : 255
WB (LT/RT)      : 4500 K
X=apply  A=save  B=reload  START=scan
Green blobs (raw) : 0 (no balls in view yet)
Purple blobs (raw): 0
Merged balls : 0
Corridor: EMPTY
Report: (press START to scan)
```

If `LoopMs avg` is > 80 ms, something is wrong. Possible causes:
- Panels stream fps too high — set `VisionConstants.PANELS_STREAM_FPS = 0` (disables Panels preview entirely) and rebuild
- `enableLiveView(false)` got flipped back to true somewhere

### 3.2 Verify camera control lock

The exposure/WB/gain values shown in telemetry are the CONSTANTS values. The camera controls should also be applied — verify by:
1. Open Panels in a browser at the driver hub's IP:8001 (or whatever port your team uses)
2. Look at the camera preview — it should be slightly dim (5ms exposure) and color-stable (not auto-adjusting)
3. Walk the camera under different lighting — the image brightness should NOT visibly change. If it auto-adjusts, the camera control lock failed at init.

If the camera is auto-adjusting: the `while (getCameraState() != STREAMING)` loop in `RobotHardware.lockCameraControls()` might have timed out. Check adb logcat for the 3-second timeout. Usually means the USB connection is slow to enumerate.

---

## 4. Bench tune (2-3 hours) — do this in your home gym

### 4.1 Setup

- Clear space about 5 feet in front of the camera
- 3 balls: at least one of each color (purple and green)
- A tape measure
- Driver hub with a clear view of telemetry

Place the balls at tape-measured positions relative to the camera:
- Ball 1: 36" forward, 0" lateral (directly ahead)
- Ball 2: 48" forward, 8" right
- Ball 3: 60" forward, 4" left

### 4.2 Exposure tuning

Run `Vision Tuning`. Look at `Green blobs (raw)` and `Purple blobs (raw)`. These should match the number of balls of each color in view.

If raw counts are UNDER the actual count:
- **Too dark**: press `RB` a few times to bump exposure up (5 → 6 → 7 → ...). Press `X` to apply. Watch the blob counts in telemetry.
- Stop when you get stable correct counts for 10 seconds.

If raw counts are OVER the actual count (false positives):
- **Too bright**: press `LB` to reduce exposure. Press `X`.
- Or: press `DPAD_LEFT` to reduce gain (255 → 200 → 150 ...). Press `X`.

If green detects but purple doesn't (or vice versa):
- This is a WB issue. Press `RT` or `LT` to shift white balance. Press `X`.
- Try 4500K (default) → 4000K if purple looks too blue → 3500K if colors look cold.
- Or 5000K / 5500K if colors look too warm.

Goal: raw blob count = actual ball count, stable, no drops or spikes over 30 seconds.

### 4.3 Field-frame accuracy check

Place 1 ball at a known distance from the camera (e.g., exactly 48" forward). Press `START` to run a scan.

Telemetry should show:
```
Merged balls: 1
  [0] Green : x=55.98 y=5.81 conf=0.XX
```

Wait — why x=55.98? Because the TeleOp uses a zero robot pose, and the camera is mounted at `CAM_OFFSET_X=7.98, CAM_OFFSET_Y=5.81` from the robot center. So:
- **fieldX (robot frame forward, "how far ahead of center")** = 48" (ball distance from camera) + 7.98" (camera offset from center) = 55.98"
- **fieldY (robot frame left, "how far left of center")** = ball's lateral offset from camera + 5.81" = 5.81" if the ball is dead ahead.

Acceptance: field-frame value should match the tape-measured distance within **±2"**. If it's off by more:
- Check the calibration constants in `VisionConstants.java` (FX, FY, CX, CY) match the SDK's built-in C920 @ 640×480 values (622.001, 622.001, 319.803, 241.251)
- Check `CAM_OFFSET_X` and `CAM_OFFSET_Y` match where the camera is physically mounted (measure with tape from robot center)

**Note on distance math (changed 2026-04-22)**: distance is now computed from the ball's HORIZONTAL silhouette width, not the `minEnclosingCircle` diameter. This fixes the systematic overestimate at close/far range caused by the top/bottom of the ball being shadowed (missing contour pixels). The ball's left/right edges are the silhouette tangent points — they stay crisp even with asymmetric top/bottom shading. If distances are still off:
- Verify the ball's horizontal extent in the Panels camera preview (blob outline should hug the left and right edges of the ball)
- If the contour is consistently narrower than the ball (erosion eating too deep), lower `erodeSize` from 15 to 11 in `RobotHardware.java`
- Check `FX` in `VisionConstants.java` — if it's wrong, all distances are off by a constant ratio

### 4.4 Corridor behavior check

Place 3 balls roughly lined up in front of the camera (all within ±8" lateral of a straight line extending forward). Press `START`.

Expected:
```
Merged balls: 3
  [0] Green  ✓  x=... y=... conf=...
  [1] Purple ✓  x=... y=... conf=...
  [2] Green  ✓  x=... y=... conf=...
Corridor: hdg=<small angle>° cap=3 score=<something positive>
```

The `✓` means the planner included that ball in the corridor. If all 3 have ✓ → corridor captures all 3. 

Now spread one ball way off to the side (past 8" lateral). Press `START`.

Expected:
```
Merged balls: 3
  [0] Green  ✓  x=... y=... conf=...   ← in corridor
  [1] Purple ✓  x=... y=... conf=...   ← in corridor
  [2] Green     x=... y=... conf=...   ← NOT in corridor (no ✓)
Corridor: hdg=... cap=2 score=...
```

The planner found a 2-ball corridor and ignored the outlier. 

Now remove all balls. Press `START`.

Expected:
```
Merged balls: 0
Corridor: EMPTY
Report: EMPTY | merged:0 | raw:0/4f | blobs G:0 P:0
```

No crash, just an empty sweep. 

### 4.5 Save tuned values

Press `A`. Telemetry should show `SAVED → /sdcard/FIRST/vision_tuning.json`.

Power-cycle the robot (important — verify persistence). Run Vision Tuning again. Telemetry should show your tuned values are still active. If not, press `B` to reload manually.

### 4.6 End-to-end practice-field test

Go to a practice field (or a reasonable simulation). Run `RedVisionAuto`. Watch the three vision cycles (cycles 3, 4, 5).

Each cycle:
1. `visionPreScan` fires at the 30° heading after the shoot
2. Robot rotates to scan heading (0°)
3. `visionCollectAndCatalog` merges the pre-scan frames with fresh frames, picks a corridor, drives it straight
4. Intake scoops balls along the way
5. When all 3 sensors see a ball OR robot has passed the last corridor ball by 2", return to shoot pose

On the driver station, the `Vision Corridor` telemetry line shows the live debug:
```
hdg=3.0° cap=3 score=1.85 | merged:3 | raw:6/4f | blobs G:1 P:2
```
means: planner picked a heading of 3° from current, captured 3 balls, merged 3 distinct FieldBalls across 4 total frames with 6 raw detections, saw 1 green blob + 2 purple blobs in the last scan.

### 4.7 If auto misbehaves

**Robot doesn't drive after scan**:
- Telemetry shows `EMPTY` sweep → detection didn't find enough balls → go back to exposure tuning
- Telemetry shows low score → raise `CORRIDOR_MIN_SCORE` threshold wasn't met → lower it to 0.3 in `VisionConstants`

**Robot drives but misses balls**:
- Corridor geometry issue: the intake is 8" ahead of robot center. If balls are being missed, increase `CORRIDOR_APPROACH_EXTRA_IN` from 4 to 8 (drives farther past last ball for intake to scoop)
- Or: the calibration is wrong and balls are ending up in slightly wrong positions

**Robot overshoots and hits the field wall**:
- Reduce `CORRIDOR_MAX_TRAVEL_IN` from 40 to 30
- Reduce `CORRIDOR_HEADING_RANGE_DEG` from 45 to 20 (less aggressive turning)
- Increase `CORRIDOR_MIN_SCORE` to 0.8 (demands a more confident plan)

**Robot keeps driving after intake is full**:
- This is the sensor-latency issue. The 3-of-3 check fires only when ALL sensors see balls. If you want the 3rd ball (current behavior), leave it. The geometric `passedLastBall()` should fire at corridor end + 2".
- If you want to stop earlier and accept losing the 3rd ball sometimes, change `isFull()` in `VisionCollectCommand.java` to `count >= 2` — but you confirmed you'd rather wait for all 3.

---

## 5. At Worlds — venue pit tune (1-2 hours)

### 5.1 Plug in + power on

Follow the physical checks in section 1 again. Do not skip.

### 5.2 Lighting conditions

Run `Vision Tuning` under the actual match lighting (lights on, shades in their match position, spectators present if they cause shadows).

### 5.3 Re-tune if needed

Most likely: venue lighting is brighter or dimmer than your home gym. Bump `EXPOSURE_MS` up/down until blob counts match reality. Adjust `WB_KELVIN` if purple looks off (tends to go blue under fluorescent lighting — increase Kelvin).

Press `A` to save. Verify persistence across a power-cycle.

### 5.4 Practice match slot (if available)

Run `RedVisionAuto`. Watch it behave. If corridor fires and gets balls, you're done.

### 5.5 If something's wrong in the pit — quick knobs without rebuilding

All of these are in `VisionConstants.java` and are NON-FINAL, meaning they can be flipped via Bylazar Panels at runtime (if your team has Panels connected to the driver hub):

- `EXPOSURE_MS`, `GAIN`, `WB_KELVIN` — camera controls
- `CORRIDOR_MIN_SCORE` — raise to 0.8 if planner picks bad headings
- `CORRIDOR_HEADING_RANGE_DEG` — lower to 20 to limit how aggressively the robot can re-aim
- `CORRIDOR_APPROACH_EXTRA_IN` — raise if missing balls, lower if overshooting
- `CORRIDOR_FINISH_BUFFER_IN` — raise if `passedLastBall()` is firing too early
- `PANELS_STREAM_FPS` — lower to 5 (or 0) if camera loop time is an issue

### 5.6 Emergency rollback

If the vision auto is broken and you need to ship:
```bash
git checkout Main
./gradlew assembleDebug
# redeploy
```

30 seconds. You're back to the pre-corridor version that uses the 3-lane decision.

---

## 6. Known things the agent could not verify

These need YOUR eyes on them because the agent didn't have a build environment or a physical robot:

1. **The code compiles.** Agent couldn't run gradle. If there's a compile error, paste it back.
2. **`VisionTuningTeleOp` actually runs.** Agent verified the code structure, imports, and references, but did not observe it running on the hub. If it crashes at init, the most likely cause is a RobotHardware singleton state issue (double-init). Workaround: close any other OpMode first.
3. **`PanelsCameraStream.startStream(portal, 15)` is the correct signature on your Bylazar Panels version.** The original Main code used `.startStream(visionPortal, 75)` so it should be the same. If not, the catch-all try/catch in RobotHardware will log and skip — but you won't get Panels preview.
4. **Manual exposure + WB lock actually takes effect on the C920.** This varies by USB-UVC driver version. Verify per section 3.2.
5. **`setDrawContours(true)` actually overlays contours on the Panels stream.** Should Just Work with recent FTC SDK, but bear in mind if you don't see blob outlines on the Panels preview.
6. **The 3-of-3 isFull behavior is acceptable to you.** You confirmed this. If in testing you find it's too slow and you're regularly not getting a 3rd ball because the path ended (via the geometric safety net) before all 3 sensors fired, revisit.
7. **Ball positions are in the right frame.** Agent implemented the math as `camera → robot (via CAM_OFFSET_X/Y) → field (via rotation by robot heading)`. The `CAM_OFFSET_Z` is not used because the pinhole-by-diameter model doesn't need camera height. If balls are systematically mis-placed, the offsets may be wrong.

---

## 7. Tunable constants cheat-sheet

All in `TeamCode/src/main/java/vision/VisionConstants.java`. Marked non-final so Bylazar Panels can adjust at runtime.

| Constant | Default | What it does |
|---|---|---|
| `EXPOSURE_MS` | 5 | Camera exposure in milliseconds (0–204) |
| `GAIN` | 255 | Camera sensor gain (0–255) |
| `WB_KELVIN` | 4500 | White balance temperature in Kelvin (2000–6500) |
| `PANELS_STREAM_FPS` | 15 | Panels camera stream rate; 0 disables |
| `CORRIDOR_HALF_WIDTH_IN` | 8 | Half of the 16" intake width |
| `CORRIDOR_MAX_TRAVEL_IN` | 40 | Cap on corridor length |
| `CORRIDOR_HEADING_RANGE_DEG` | 45 | Max heading sweep ± from current |
| `CORRIDOR_HEADING_STEP_DEG` | 1 | Resolution of the heading sweep |
| `CORRIDOR_MAX_BALLS` | 3 | Max balls in one corridor |
| `CORRIDOR_K_TURN` | 0.5 | Score penalty per 90° of turning |
| `CORRIDOR_K_LENGTH` | 0.005 | Score penalty per inch of corridor length |
| `CORRIDOR_MIN_SCORE` | 0.3 | Minimum score to accept the corridor; below → no-plan |
| `CAM_PITCH_DEG` | 20.0 | Camera downward tilt (documentation / future use; not in current math) |
| `CAM_OFFSET_Z` | 3.0 | Camera lens-center height above field (inches) |
| `CORRIDOR_APPROACH_EXTRA_IN` | 4 | Path endpoint = last ball + this many inches |
| `CORRIDOR_FINISH_BUFFER_IN` | 2 | Geometric safety: end path when robot center passes last ball by this |

---

## 8. Worklist (check off as you go)

- [ ] USB-A 3.0 port verified
- [ ] Camera mount solid
- [ ] Lens clean
- [ ] Branch `vision-corridor-worlds` pulled/checked out
- [ ] `./gradlew assembleDebug` succeeds
- [ ] APK deployed to Control Hub
- [ ] `Vision Tuning` OpMode boots, LoopMs < 50 ms
- [ ] Camera controls locked (image doesn't auto-adjust)
- [ ] Bench test: exposure tuned for correct blob counts
- [ ] Bench test: field-frame accuracy ±2" at 48"
- [ ] Bench test: 3-ball corridor captures all 3
- [ ] Bench test: outlier 2-ball corridor works
- [ ] Bench test: empty scene returns EMPTY cleanly
- [ ] Tuned values saved to `/sdcard/FIRST/vision_tuning.json`
- [ ] Values persist across power-cycle
- [ ] Practice field: `RedVisionAuto` cycles 3-5 use corridor and pick up balls
- [ ] Worlds pit: re-tune exposure/WB under venue lighting
- [ ] Worlds pit: practice-match dry run succeeds

---

## 9. Agent contact points for fixes

When something breaks, paste the following back for a fix:
- The exact compile error or stack trace
- What telemetry showed at the moment of failure
- What you tried to adjust before calling for help

The agent has the full plan at `/Users/andysmith/.claude/plans/we-are-an-ftc-unified-flame.md` for reference. The three commits on `vision-corridor-worlds` are granular enough to revert individually if one refinement specifically misbehaves.

Good luck at Worlds.
