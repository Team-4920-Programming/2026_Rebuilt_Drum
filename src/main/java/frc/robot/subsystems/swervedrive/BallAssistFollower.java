package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import java.util.List;

/**
 * BallAssistFollower — Drives the robot along a committed ball-pickup path.
 *
 * DRIVER INTERFACE:
 *   Toggle button  → Arm/disarm (call toggle())
 *   Forward stick  → Proportional speed along the path
 *   Backward stick → Reverse along the path
 *   Rotation stick → Rotate chassis freely while following (aim intake)
 *   Toggle off     → Instant return to normal teleop
 *
 * STABILITY:
 *   - Commits to a path and tracks progress via nearest waypoint
 *   - Re-evaluates only after reaching a waypoint or traveling a minimum distance
 *   - Hysteresis: new path must score 25%+ better to replace current
 *   - Heading output is rate-limited for smooth transitions
 *
 * INTEGRATION:
 *   In RobotContainer or a Command:
 *
 *     private final BallPathCalculator calculator = new BallPathCalculator();
 *     private final BallAssistFollower follower = new BallAssistFollower();
 *
 *     // In teleopPeriodic:
 *     if (driver.getAButtonPressed()) follower.toggle();
 *
 *     if (follower.isActive()) {
 *         ChassisSpeeds speeds = follower.update(
 *             currentPose,
 *             ballPositions,
 *             calculator,
 *             -driver.getLeftY(),   // forward/back throttle [-1, 1]
 *             driver.getRightX()   // rotation [-1, 1]
 *         );
 *         swerve.driveFieldRelative(speeds);
 *
 *         // AdvantageScope logging
 *         Logger.recordOutput("BallAssist/Active", true);
 *         Logger.recordOutput("BallAssist/Path", follower.getTrajectory());
 *         Logger.recordOutput("BallAssist/TargetPose", follower.getTargetPose());
 *         Logger.recordOutput("BallAssist/WaypointIndex", follower.getWaypointIndex());
 *     }
 */
public class BallAssistFollower {

    // ── Tuning ──────────────────────────────────────────────────────────

    /** Max translation speed while following (m/s). */
    private double maxSpeed = 4.5;

    /** Max rotation speed while following (rad/s). */
    private double maxRotation = 3.0;

    /**
     * Lateral correction gain. Pulls robot back onto the path centerline
     * when drifting off. Higher = stiffer. Start at 2.0, increase if the
     * robot wanders, decrease if it oscillates laterally.
     */
    private double lateralKp = 2.0;

    /** Max lateral correction speed (m/s). Caps the correction force. */
    private double maxLateralCorrection = 1.5;

    /** Distance to a waypoint to consider it "reached" (meters). */
    private double waypointCaptureDist = 0.5;

    /**
     * Minimum distance to travel before re-evaluation is allowed (meters).
     * Prevents rapid oscillation between paths.
     */
    private double minCommitDistance = 1.0;

    /**
     * Hysteresis: a new path must score this fraction better than the
     * current path to trigger a switch. 0.25 = 25% better required.
     */
    private double hysteresisThreshold = 0.25;

    /**
     * Max heading slew rate (rad/s). Limits how fast the suggested path
     * heading can change between cycles for smooth transitions.
     */
    private double maxHeadingSlewRate = Math.toRadians(180.0); // 180°/s

    /**
     * Heading tracking P gain. Controls how aggressively the robot rotates
     * to face the path heading. Higher = snappier. Start at 3.0.
     * Units: rad/s per radian of error.
     */
    private double headingKp = 3.0;

    /**
     * Max driver rotation trim (rad/s). The stick adds this on top of
     * the auto-aim. Keep low so the driver makes fine adjustments,
     * not wild spins. ±1.0 rad/s ≈ ±57°/s is plenty for intake aiming.
     */
    private double maxDriverTrim = 1.0;

    /** Deadband for driver stick inputs. */
    private double stickDeadband = 0.05;

    /** Cycle time in seconds (default 20ms for standard FRC loop). */
    private double dtSeconds = 0.02;


    // ── State ───────────────────────────────────────────────────────────

    private boolean active = false;

    /** The currently committed path result. */
    private BallPathCalculator.PathResult committedPath = null;

    /** Current waypoint index we're driving toward. */
    private int waypointIndex = 0;

    /** Distance traveled since last re-evaluation. */
    private double distanceSinceEval = 0.0;

    /** Last robot position (for tracking distance traveled). */
    private Translation2d lastPosition = null;

    /** Smoothed heading output (for slew rate limiting). */
    private double smoothedHeading = 0.0;

    /** Whether we've done the initial path commit. */
    private boolean hasCommitted = false;


    // ── Public Interface ────────────────────────────────────────────────

    /** Toggle the system on/off. Resets all state when toggled off. */
    public void toggle() {
        active = !active;
        if (!active) {
            reset();
        }
    }

    /** Force the system off. */
    public void disable() {
        active = false;
        reset();
    }

    public void enable() {
    active = true;
    reset();
    }

    /** Is the system currently active? */
    public boolean isActive() {
        return active;
    }

    /**
     * Main update loop. Call this every cycle when active.
     *
     * @param robotPose      current robot pose from your pose estimator
     * @param ballPositions  current ball detections as Pose2d list
     * @param calculator     your BallPathCalculator instance
     * @param throttle       forward/back stick input [-1, 1], positive = forward
     * @param rotation       rotation stick input [-1, 1], positive = CCW
     * @return ChassisSpeeds to feed into swerve.driveFieldRelative()
     */
    public ChassisSpeeds update(
            Pose2d robotPose,
            List<Pose2d> ballPositions,
            BallPathCalculator calculator,
            double throttle,
            double rotation) {

        if (!active) {
            return new ChassisSpeeds();
        }

        Translation2d currentPos = robotPose.getTranslation();

        // ── Track distance traveled ─────────────────────────────────
        if (lastPosition != null) {
            distanceSinceEval += currentPos.getDistance(lastPosition);
        }
        lastPosition = currentPos;

        // ── Initial commit or re-evaluation ─────────────────────────
        if (!hasCommitted) {
            // First cycle: commit to the best path immediately
            committedPath = calculator.calculate(robotPose, ballPositions);
            smoothedHeading = committedPath.heading;
            hasCommitted = true;
            waypointIndex = 0;
            distanceSinceEval = 0;
        } else if (shouldReevaluate(currentPos)) {
            tryReevaluate(robotPose, ballPositions, calculator);
        }

        // ── No valid path → zero output ─────────────────────────────
        if (committedPath == null || !committedPath.isValid()) {
            return new ChassisSpeeds();
        }

        // ── Advance waypoint if reached ─────────────────────────────
        advanceWaypoint(currentPos);

        // ── Compute drive vector ────────────────────────────────────
        return computeSpeeds(robotPose, currentPos, throttle, rotation);
    }


    // ── Path Re-evaluation ──────────────────────────────────────────────

    /**
     * Should we try re-evaluating? Only after traveling enough distance
     * AND either reaching a waypoint or exhausting the path.
     */
    private boolean shouldReevaluate(Translation2d currentPos) {
        if (distanceSinceEval < minCommitDistance) return false;

        // Re-evaluate if we've reached the current waypoint
        if (committedPath != null && committedPath.isValid()) {
            if (waypointIndex >= committedPath.capturedBalls.size()) {
                return true; // exhausted all waypoints
            }
            Translation2d target = committedPath.capturedBalls.get(waypointIndex);
            if (currentPos.getDistance(target) < waypointCaptureDist) {
                return true; // just reached a waypoint
            }
        }

        return false;
    }

    /**
     * Evaluate a new path and compare against the current committed one.
     * Only switch if the new path is significantly better (hysteresis).
     */
    private void tryReevaluate(
            Pose2d robotPose,
            List<Pose2d> ballPositions,
            BallPathCalculator calculator) {

        BallPathCalculator.PathResult candidate = calculator.calculate(robotPose, ballPositions);

        // Calculate the effective remaining score of the current path
        // (only count balls we haven't passed yet)
        double currentRemainingScore = getRemainingScore();

        boolean shouldSwitch;
        if (currentRemainingScore <= 0 || !committedPath.isValid()) {
            // Current path is exhausted — take anything valid
            shouldSwitch = candidate.isValid();
        } else {
            // Hysteresis: new path must be meaningfully better
            shouldSwitch = candidate.score > currentRemainingScore * (1.0 + hysteresisThreshold);
        }

        if (shouldSwitch) {
            committedPath = candidate;
            waypointIndex = 0;
        }

        distanceSinceEval = 0;
    }

    /**
     * Estimate the remaining value of the current path based on
     * how many balls are still ahead of us.
     */
    private double getRemainingScore() {
        if (committedPath == null || !committedPath.isValid()) return 0;

        int remaining = committedPath.capturedBalls.size() - waypointIndex;
        if (remaining <= 0) return 0;

        // Rough proportional estimate
        double fraction = (double) remaining / committedPath.capturedBalls.size();
        return committedPath.score * fraction;
    }


    // ── Waypoint Tracking ───────────────────────────────────────────────

    private void advanceWaypoint(Translation2d currentPos) {
        if (committedPath == null || !committedPath.isValid()) return;

        while (waypointIndex < committedPath.capturedBalls.size()) {
            Translation2d target = committedPath.capturedBalls.get(waypointIndex);
            if (currentPos.getDistance(target) < waypointCaptureDist) {
                waypointIndex++;
            } else {
                break;
            }
        }
    }


    // ── Speed Computation ───────────────────────────────────────────────

    private ChassisSpeeds computeSpeeds(
            Pose2d robotPose,
            Translation2d currentPos,
            double throttle,
            double rotation) {

        // Apply deadband
        throttle = applyDeadband(throttle, stickDeadband);
        rotation = applyDeadband(rotation, stickDeadband);

        // ── Path heading (slew-rate limited) ────────────────────────
        double targetHeading = committedPath.heading;
        double headingError = normalizeAngle(targetHeading - smoothedHeading);
        double maxSlew = maxHeadingSlewRate * dtSeconds;
        headingError = MathUtil.clamp(headingError, -maxSlew, maxSlew);
        smoothedHeading = normalizeAngle(smoothedHeading + headingError);

        double cosH = Math.cos(smoothedHeading);
        double sinH = Math.sin(smoothedHeading);

        // ── Forward velocity along the path ─────────────────────────
        // Throttle directly controls speed along the committed heading.
        // Positive = forward along path, negative = reverse.
        double forwardSpeed = throttle * maxSpeed;

        double vxForward = forwardSpeed * cosH;
        double vyForward = forwardSpeed * sinH;

        // ── Lateral correction ──────────────────────────────────────
        // Pull the robot back toward the corridor centerline.
        // Perpendicular to the path heading (left-pointing).
        double perpX = -sinH;
        double perpY = cosH;

        // Lateral offset: how far off the centerline are we?
        double lateralOffset = 0;
        if (waypointIndex < committedPath.capturedBalls.size()) {
            double dx = currentPos.getX() - committedPath.robotPose.getX();
            double dy = currentPos.getY() - committedPath.robotPose.getY();
            lateralOffset = dx * perpX + dy * perpY;
        }

        // Correction velocity: push back toward centerline
        double correctionSpeed = -lateralOffset * lateralKp;
        correctionSpeed = MathUtil.clamp(correctionSpeed, -maxLateralCorrection, maxLateralCorrection);

        // Only apply lateral correction when actually moving
        double correctionScale = Math.min(1.0, Math.abs(throttle) * 2.0);
        double vxCorrection = correctionSpeed * perpX * correctionScale;
        double vyCorrection = correctionSpeed * perpY * correctionScale;

        // ── Combine translation ─────────────────────────────────────
        double vx = vxForward + vxCorrection;
        double vy = vyForward + vyCorrection;

        // Clamp total translation speed
        double totalSpeed = Math.sqrt(vx * vx + vy * vy);
        if (totalSpeed > maxSpeed) {
            vx = vx / totalSpeed * maxSpeed;
            vy = vy / totalSpeed * maxSpeed;
        }

        // ── Rotation: auto-aim to path heading + driver trim ────────
        // P controller snaps the robot to face the corridor heading.
        // Driver stick adds a small offset for fine intake aiming.
        double currentHeading = robotPose.getRotation().getRadians();
        double rotError = normalizeAngle(smoothedHeading - currentHeading);
        double autoRotation = rotError * headingKp;

        // Clamp auto-rotation so it doesn't fight the driver too hard
        autoRotation = MathUtil.clamp(autoRotation, -maxRotation, maxRotation);

        // Driver trim: small adjustments on top of auto-aim
        double driverTrim = rotation * maxDriverTrim;

        double omega = autoRotation + driverTrim;

        // Clamp total rotation
        omega = MathUtil.clamp(omega, -maxRotation, maxRotation);

        return new ChassisSpeeds(vx, vy, omega);
    }


    // ── Telemetry / AdvantageScope Outputs ───────────────────────────────

    /** Get the committed trajectory for AdvantageScope. */
    public Trajectory getTrajectory() {
        if (committedPath != null && committedPath.isValid()) {
            return committedPath.getTrajectory();
        }
        return new Trajectory();
    }

    /** Get the current target pose (next waypoint or path end). */
    public Pose2d getTargetPose() {
        if (committedPath != null && committedPath.isValid()
                && waypointIndex < committedPath.capturedBalls.size()) {
            return new Pose2d(
                committedPath.capturedBalls.get(waypointIndex),
                new Rotation2d(smoothedHeading)
            );
        }
        return new Pose2d();
    }

    /** Get corridor boundary poses for visualization. */
    public Pose2d[] getCorridorPoses() {
        if (committedPath != null && committedPath.isValid()) {
            return committedPath.getCorridorPoses();
        }
        return new Pose2d[0];
    }

    /** Get captured ball poses. */
    public Pose2d[] getCapturedBallPoses() {
        if (committedPath != null && committedPath.isValid()) {
            return committedPath.getBallPoses();
        }
        return new Pose2d[0];
    }

    /** Current waypoint index (how many balls we've passed). */
    public int getWaypointIndex() {
        return waypointIndex;
    }

    /** How many balls remain ahead on the committed path. */
    public int getRemainingBalls() {
        if (committedPath == null || !committedPath.isValid()) return 0;
        return Math.max(0, committedPath.capturedBalls.size() - waypointIndex);
    }

    /** Current smoothed heading output (radians). */
    public double getSmoothedHeading() {
        return smoothedHeading;
    }

    /** Committed path score. */
    public double getPathScore() {
        return committedPath != null ? committedPath.score : 0;
    }

    /** Distance traveled since last re-evaluation. */
    public double getDistanceSinceEval() {
        return distanceSinceEval;
    }


    // ── State Management ────────────────────────────────────────────────

    private void reset() {
        committedPath = null;
        waypointIndex = 0;
        distanceSinceEval = 0;
        lastPosition = null;
        hasCommitted = false;
        smoothedHeading = 0;
    }


    // ── Utility ─────────────────────────────────────────────────────────

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    private static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) return 0.0;
        return (value - Math.signum(value) * deadband) / (1.0 - deadband);
    }


    // ── Builder-style setters ───────────────────────────────────────────

    public BallAssistFollower withMaxSpeed(double metersPerSec) {
        this.maxSpeed = metersPerSec;
        return this;
    }

    public BallAssistFollower withMaxRotation(double radsPerSec) {
        this.maxRotation = radsPerSec;
        return this;
    }

    public BallAssistFollower withLateralKp(double kp) {
        this.lateralKp = kp;
        return this;
    }

    public BallAssistFollower withMaxLateralCorrection(double metersPerSec) {
        this.maxLateralCorrection = metersPerSec;
        return this;
    }

    public BallAssistFollower withWaypointCaptureDist(double meters) {
        this.waypointCaptureDist = meters;
        return this;
    }

    public BallAssistFollower withMinCommitDistance(double meters) {
        this.minCommitDistance = meters;
        return this;
    }

    public BallAssistFollower withHysteresis(double threshold) {
        this.hysteresisThreshold = threshold;
        return this;
    }

    public BallAssistFollower withHeadingSlewRate(double degreesPerSec) {
        this.maxHeadingSlewRate = Math.toRadians(degreesPerSec);
        return this;
    }

    public BallAssistFollower withStickDeadband(double deadband) {
        this.stickDeadband = deadband;
        return this;
    }

    public BallAssistFollower withHeadingKp(double kp) {
        this.headingKp = kp;
        return this;
    }

    public BallAssistFollower withMaxDriverTrim(double radsPerSec) {
        this.maxDriverTrim = radsPerSec;
        return this;
    }

    public BallAssistFollower withDt(double seconds) {
        this.dtSeconds = seconds;
        return this;
    }
}