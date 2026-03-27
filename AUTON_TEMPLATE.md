# Autonomous OpMode Creation Instructions

## How to Invoke

Say: **"Read AUTON_TEMPLATE.md. Create `ClassName` in `FolderName/`, [Red/Blue] alliance. Here are the paths: [paste Paths class]"**

Optionally include:
- Which pasted paths should be combined into fluid multi-segment PathChains
- Custom delay values between cycles
- Custom speed values per path
- Whether to repeat a cycle's path for later cycles
- Any special notes (e.g., "cycle 4 and 5 reuse the same path")

---

## Step-by-Step: From Pasted Paths to Finished Auto

### 1. Parse the Pasted Paths Class

The user pastes a `public static class Paths` block from their path planner. Each path is a separate `follower.pathBuilder()...build()` call with:
- `BezierLine(startPose, endPose)` — straight line (2 points)
- `BezierCurve(startPose, controlPose, endPose)` — curve (3 points)
- `.setLinearHeadingInterpolation(startRad, endRad)` after each `.addPath()`

### 2. Combine Consecutive Segments into Fluid PathChains

**Rule:** If one path's endpoint equals the next path's start point, they are candidates for a single fluid PathChain. Combine them into one `follower.pathBuilder()` with multiple `.addPath()` calls and a single `.build()`.

The user will tell you which paths to combine. If not specified, combine paths whose names suggest a sequence (e.g., `shootToFirst1` + `shootToFirst2` -> `shootToFirst`).

**Before (pasted — two separate builds):**
```java
shootToFirst1 = follower.pathBuilder()
    .addPath(new BezierCurve(new Pose(87.800, 8.000), new Pose(91.000, 17.000), new Pose(105.000, 11.500)))
    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
    .build();

shootToSecond2 = follower.pathBuilder()
    .addPath(new BezierLine(new Pose(105.000, 11.500), new Pose(131.000, 11.500)))
    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
    .build();
```

**After (in the auto file — one fluid PathChain):**
```java
shootToFirst = follower.pathBuilder()
        .addPath(new BezierCurve(startPose, startCurveControl, cyclePrepPose))
        .setLinearHeadingInterpolation(startPose.getHeading(), cyclePrepPose.getHeading())
        .addPath(new BezierLine(cyclePrepPose, firstCyclePickupPose))
        .setLinearHeadingInterpolation(cyclePrepPose.getHeading(), firstCyclePickupPose.getHeading())
        .build();
```

### 3. Extract Coordinates into Named Pose Fields

Pull all coordinates out of inline `new Pose(x, y)` calls and into descriptive named fields at the class level.

- **Endpoint poses** (start, shoot, pickup, stop): include heading — `new Pose(x, y, Math.toRadians(deg))`
- **Control points** (middle point of BezierCurve): no heading — `new Pose(x, y)`
- **Group related poses** with comments (start, shoot, cycle prep, pickups, control points, end)

### 4. Determine Cycle Structure

Map pasted paths to cycles. The standard pattern:

| Cycle | Outbound Path | Return Path | Command Pattern |
|-------|--------------|-------------|-----------------|
| Preload | none | none | `.delay(.35).shoot()` |
| 1 | `shootToFirst` | `firstToShoot` | Explicit: `.moveTo()` + `.delay()` + `.parallel(moveTo + autoCatalog)` |
| 2 | `shootToSecond` | `secondToShoot` | Explicit: `.moveTo()` + `.delay()` + `.parallel(moveTo + autoCatalog)` |
| 3+ | `shootToThird` | (dynamic) | `.ballCollectMoveToAndCatalog(outbound, shootPose, speed, false, .3)` |
| Park | `shootToStop` | none | `.moveTo(shootToStop, maxSpeed, false)` |

**Cycles 1-2** use explicit outbound + return PathChains because the return path matters (curves, specific routes).

**Cycles 3+** use `ballCollectMoveToAndCatalog` which only needs the outbound PathChain — the return is generated dynamically as a straight line from wherever the robot ends up back to `shootPose`.

### 5. Detect Alliance Color

Determine from the class name or user specification:
- **Red**: `Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Red;`
- **Blue**: `Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.Blue;`

---

## File Template

```java
package Autos.{FolderName};

import Autos.AutonTemplate;
import Constants.EnumConstants;
import Constants.SpindexerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import commands.CommandSequenceBuilder;


@Autonomous(name = "{ClassName}")
public class {ClassName} extends AutonTemplate {
    public static double maxSpeed = 1;
    private PathChain shootToFirst, firstToShoot, shootToSecond, secondToShoot,
            shootToThird, shootToFourth, shootToFifth, shootToStop;

    // --- POSES (extracted from pasted paths) ---

    // Start pose
    private final Pose startPose = new Pose({x}, {y}, Math.toRadians({deg}));

    // Start curve control point (if cycle 1 outbound has a curve)
    private final Pose startCurveControl = new Pose({x}, {y});

    // Shoot position (where the robot returns to shoot)
    private final Pose shootPose = new Pose({x}, {y}, Math.toRadians({deg}));
    // postShootPose: same XY as shootPose but heading faces outbound direction
    private final Pose postShootPose = new Pose({x}, {y}, Math.toRadians({deg}));

    // Cycle prep & pickup poses
    private final Pose cyclePrepPose = new Pose({x}, {y}, Math.toRadians({deg}));
    private final Pose firstCyclePickupPose = new Pose({x}, {y}, Math.toRadians({deg}));
    // ... additional pickup/waypoint poses as needed

    // Control points for curved return paths
    private final Pose returnToShootControl = new Pose({x}, {y});

    // End pose
    private final Pose stopPose = new Pose({x}, {y}, Math.toRadians({deg}));

    @Override
    protected void buildPaths() {
        follower.setStartingPose(startPose);

        // Cycle 1 outbound (fluid: combine consecutive segments)
        shootToFirst = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, startCurveControl, cyclePrepPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), cyclePrepPose.getHeading())
                .addPath(new BezierLine(cyclePrepPose, firstCyclePickupPose))
                .setLinearHeadingInterpolation(cyclePrepPose.getHeading(), firstCyclePickupPose.getHeading())
                .build();

        // Cycle 1 return
        firstToShoot = follower.pathBuilder()
                .addPath(new BezierLine(firstCyclePickupPose, shootPose))
                .setLinearHeadingInterpolation(firstCyclePickupPose.getHeading(), shootPose.getHeading())
                .build();

        // Cycle 2 outbound
        shootToSecond = follower.pathBuilder()
                .addPath(new BezierCurve(postShootPose, secondCurveControl, secondPickupPose))
                .setLinearHeadingInterpolation(postShootPose.getHeading(), secondPickupPose.getHeading())
                .build();

        // Cycle 2 return
        secondToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(secondPickupPose, returnToShootControl, shootPose))
                .setLinearHeadingInterpolation(secondPickupPose.getHeading(), shootPose.getHeading())
                .build();

        // Cycles 3+ outbound only (return is dynamic)
        shootToThird = follower.pathBuilder()
                // ... segments from pasted paths
                .build();

        shootToFourth = follower.pathBuilder()
                // ... segments from pasted paths
                .build();

        shootToFifth = follower.pathBuilder()
                // ... segments from pasted paths (may reuse cycle 4 poses)
                .build();

        // Park
        shootToStop = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, stopPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), stopPose.getHeading())
                .build();
    }

    @Override
    public void init() {
        super.init();
        SpindexerConstants.currentMode = EnumConstants.ShootingMode.Fast;
        Constants.RobotConstants.Robot.allianceColor = Constants.EnumConstants.AllianceColor.{Red|Blue};

        autonomousCommand = new CommandSequenceBuilder(follower, intake, spindexer, limelight, shooter, turret)
                // Preload
                .delay(.35)
                .shoot()
                // Cycle 1 (explicit)
                .intakeStart()
                .moveTo(shootToFirst, maxSpeed, false)
                .delay(.75)
                .parallel(p -> p.moveTo(firstToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                // Cycle 2 (explicit)
                .intakeStart()
                .moveTo(shootToSecond, maxSpeed, false)
                .delay(.55)
                .parallel(p -> p.moveTo(secondToShoot, maxSpeed, false).autoCatalog())
                .shoot()
                // Cycle 3 (ballCollect)
                .intakeStart()
                .ballCollectMoveToAndCatalog(shootToThird, shootPose, .8, false, .3)
                .shoot()
                // Cycle 4 (ballCollect)
                .intakeStart()
                .ballCollectMoveToAndCatalog(shootToFourth, shootPose, maxSpeed, false, .3)
                .shoot()
                // Cycle 5 (ballCollect)
                .intakeStart()
                .ballCollectMoveToAndCatalog(shootToFifth, shootPose, maxSpeed, false, .3)
                .shoot()
                // Park
                .moveTo(shootToStop, maxSpeed, false)
                .build();
    }
}
```

---

## Key Details & Edge Cases

### Heading Conventions
- Use `Math.toRadians()` for all headings in Pose constructors and heading interpolation
- Control points (middle of BezierCurve) have NO heading — just `new Pose(x, y)`
- `postShootPose` has the same XY as `shootPose` but heading faces the next outbound direction (Red: 0deg, Blue: 180deg)

### Speed Values
- Default: `maxSpeed = 1`
- First `ballCollectMoveToAndCatalog` (cycle 3): typically `.8` speed
- Subsequent `ballCollectMoveToAndCatalog` cycles: `maxSpeed`
- All explicit moveTo calls: `maxSpeed`
- `holdEnd` is always `false` in these autos

### Delay Values
- Preload delay: `.35`
- Cycle 1 pickup delay: `.75` (longest — first pickup needs more time)
- Cycle 2 pickup delay: `.15` to `.55` (varies by path length)
- `ballCollectMoveToAndCatalog` delay param: `.3` (delay before return)

### When Cycles Reuse the Same Path
If the user says "cycle 5 reuses cycle 4's path," build a separate PathChain (`shootToFifth`) that references the same Pose fields but is its own `follower.pathBuilder()...build()` call. Each cycle needs its own PathChain object.

### Return Paths for ballCollectMoveToAndCatalog
Do NOT create return PathChains for cycles 3+. `ballCollectMoveToAndCatalog` internally uses `DynamicReturnPathCommand` which generates a straight-line return from the robot's current position to `shootPose`.

### Pasted Path Names May Be Ambiguous
The path planner sometimes generates generic names (`Path10`, `Path12`, `Path13`) or duplicates. Map them to meaningful names based on their role in the cycle sequence. Ask the user if the mapping is unclear.

### File Location
Place the file at: `TeamCode/src/main/java/Autos/{FolderName}/{ClassName}.java`
- Package declaration must match: `package Autos.{FolderName};`
- `@Autonomous(name = "{ClassName}")` annotation

### Folder Examples
- `PartnerAutos/` — competition-specific cycling autos
- `FifteenBalls/`, `EighteenBalls/`, `TwentyOneBalls/` — ball-count categories
- `Tests/` — path/movement testing
- User specifies the folder; default to `PartnerAutos/` if unspecified

---

## Reference: Complete Red Example (RedBackCycler)

See `TeamCode/src/main/java/Autos/PartnerAutos/RedBackCycler.java` for the canonical red-alliance implementation.

## Reference: Complete Blue Example (BlueEclipse)

See `TeamCode/src/main/java/Autos/PartnerAutos/BlueEclipse.java` for a blue-alliance implementation with multi-segment fourth cycle paths.
