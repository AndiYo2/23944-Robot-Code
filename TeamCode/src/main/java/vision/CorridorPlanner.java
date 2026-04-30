package vision;

import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Sweep-heading planner. Given a robot pose and a list of field-frame balls,
 * find the forward heading whose 16"-wide × MAX_TRAVEL-long corridor captures
 * the highest-scoring cluster of balls.
 *
 * Pure function — no side effects, no robot interaction. Unit-testable.
 *
 * Score = Σ(capturedConfidence)
 *       − K_TURN   × |Δheading| / 90°
 *       − K_LENGTH × maxAlongAxis
 */
public class CorridorPlanner {

    public static class Sweep {
        public final double         headingRad;
        public final Pose           endPose;
        public final List<FieldBall> captured;
        public final double         score;

        public Sweep(double headingRad, Pose endPose, List<FieldBall> captured, double score) {
            this.headingRad = headingRad;
            this.endPose    = endPose;
            this.captured   = captured;
            this.score      = score;
        }

        public static final Sweep EMPTY = new Sweep(
                0.0,
                new Pose(0, 0, 0),
                Collections.emptyList(),
                Double.NEGATIVE_INFINITY);

        public boolean isEmpty() {
            return this.score == Double.NEGATIVE_INFINITY || captured.isEmpty();
        }
    }

    private CorridorPlanner() {}

    /** Primary entry point. Uses defaults from VisionConstants. */
    public static Sweep plan(Pose robotPose, List<FieldBall> balls) {
        return plan(robotPose, balls,
                VisionConstants.CORRIDOR_MAX_BALLS,
                VisionConstants.CORRIDOR_MAX_TRAVEL_IN,
                VisionConstants.getCorridorFieldXLimit(),
                VisionConstants.CORRIDOR_Y_FLOOR_IN,
                VisionConstants.CORRIDOR_HALF_WIDTH_IN,
                Math.toRadians(VisionConstants.CORRIDOR_HEADING_RANGE_DEG),
                Math.toRadians(VisionConstants.CORRIDOR_HEADING_STEP_DEG),
                VisionConstants.CORRIDOR_K_TURN,
                VisionConstants.CORRIDOR_K_LENGTH,
                VisionConstants.CORRIDOR_APPROACH_EXTRA_IN);
    }

    /** Parametric entry point for unit tests. */
    public static Sweep plan(Pose robotPose, List<FieldBall> balls,
                             int maxBalls, double maxTravel, double fieldXLimit,
                             double fieldYFloor,
                             double halfWidth,
                             double halfRangeRad, double stepRad,
                             double kTurn, double kLength,
                             double approachExtra) {
        if (balls == null || balls.isEmpty()) return Sweep.EMPTY;

        final double baseHeading = robotPose.getHeading();
        // The intake's position at scan time is the robot's CURRENT pose
        // rotated by the CURRENT heading (baseHeading) — NOT by the candidate
        // sweep heading phi. The path will rotate the robot to phi during
        // travel, so the intake's end position uses R(phi); but the corridor
        // start position must use R(baseHeading) since that's where the
        // intake physically is at t=0. Bug: previously rotated by phi here,
        // which produced a (R(phi)-R(baseHeading))·(iox,ioy) lateral error
        // that grew with |phi - baseHeading|.
        final double cosBase = Math.cos(baseHeading);
        final double sinBase = Math.sin(baseHeading);
        final double iox = VisionConstants.INTAKE_OFFSET_X;
        final double ioy = VisionConstants.INTAKE_OFFSET_Y;
        final double intakeStartX = robotPose.getX() + iox * cosBase - ioy * sinBase;
        final double intakeStartY = robotPose.getY() + iox * sinBase + ioy * cosBase;

        Sweep best = Sweep.EMPTY;

        for (double phi = baseHeading - halfRangeRad;
             phi <= baseHeading + halfRangeRad + 1e-9;
             phi += stepRad) {

            final double ax =  Math.cos(phi), ay = Math.sin(phi);  // forward axis
            final double px = -Math.sin(phi), py = Math.cos(phi);  // perpendicular

            // Clamp max travel so the ROBOT CENTER's end pose doesn't cross
            // the field X boundary. The intake may end up past fieldXLimit
            // but that's expected (intake is already forward of center).
            // Works for both alliances: Red drives ax>0 against an upper limit
            // (~132); Blue drives ax<0 against a lower limit (~12). Sweeps with
            // ax≈0 (pure ±Y travel) skip the clamp — the X boundary doesn't
            // constrain lateral motion.
            double effectiveMaxTravel = maxTravel;
            if (Math.abs(ax) > 1e-6) {
                double distToXLimit = (fieldXLimit - robotPose.getX()) / ax;
                if (distToXLimit <= 0) continue;  // already at/past the boundary
                effectiveMaxTravel = Math.min(maxTravel, distToXLimit);
            }

            // Y floor clamp: robot CENTER must not drop below fieldYFloor (the
            // driver-station wall). Only relevant for headings that travel
            // toward -Y (ay < 0). Truncates corridor where it would cross the
            // floor; if the heading happens to also point along ±X (≈0/180°),
            // this naturally produces a wall-hugging corner-bound corridor.
            if (ay < -1e-6) {
                double distToYFloor = (fieldYFloor - robotPose.getY()) / ay;
                if (distToYFloor <= 0) continue;  // already at/past the floor
                effectiveMaxTravel = Math.min(effectiveMaxTravel, distToYFloor);
            }

            List<Candidate> inCorridor = new ArrayList<>();
            for (FieldBall ball : balls) {
                // Relative to the INTAKE's start position, not the robot center.
                double rx = ball.fieldX - intakeStartX;
                double ry = ball.fieldY - intakeStartY;
                double along = rx * ax + ry * ay;
                double cross = rx * px + ry * py;
                if (along <= 0 || along > effectiveMaxTravel) continue;
                if (Math.abs(cross) > halfWidth)              continue;
                inCorridor.add(new Candidate(ball, along));
            }
            if (inCorridor.isEmpty()) continue;

            inCorridor.sort(Comparator.comparingDouble(c -> c.along));
            int take = Math.min(maxBalls, inCorridor.size());

            double confSum  = 0;
            double maxAlong = 0;
            List<FieldBall> captured = new ArrayList<>(take);
            for (int i = 0; i < take; i++) {
                Candidate c = inCorridor.get(i);
                confSum += c.ball.confidence;
                maxAlong = Math.max(maxAlong, c.along);
                captured.add(c.ball);
            }

            double turnPenalty   = kTurn   * Math.abs(wrap(phi - baseHeading)) / (Math.PI / 2.0);
            double lengthPenalty = kLength * maxAlong;
            double score = confSum - turnPenalty - lengthPenalty;

            if (score > best.score) {
                // Drive the FULL corridor length (all the way to the field X
                // boundary). The isFull() sensor check ends the path early
                // when all 3 balls are in; otherwise we use every inch of
                // corridor to scoop any un-detected balls along the way.
                // approachExtra is kept as a param for API compatibility but
                // is no longer used in endDist.
                double endDist = effectiveMaxTravel;
                double endX = robotPose.getX() + ax * endDist;
                double endY = robotPose.getY() + ay * endDist;
                best = new Sweep(phi, new Pose(endX, endY, phi), captured, score);
            }
        }
        return best;
    }

    /** Wrap angle into [-π, π]. */
    private static double wrap(double a) {
        while (a >  Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private static class Candidate {
        final FieldBall ball;
        final double    along;
        Candidate(FieldBall b, double a) { this.ball = b; this.along = a; }
    }
}
