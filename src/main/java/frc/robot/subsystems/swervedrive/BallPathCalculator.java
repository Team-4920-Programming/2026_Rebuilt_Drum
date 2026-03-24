package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import java.util.ArrayList;
import java.util.List;

/**
 * BallPathCalculator — Core path optimization for assisted ball pickup.
 *
 * Takes the robot's current Pose2d and a list of ball Pose2d positions,
 * sweeps candidate headings across the forward arc, and returns the
 * heading whose corridor captures the most/best balls.
 *
 * Outputs WPILib Trajectory + Pose2d[] for AdvantageScope visualization.
 *
 * Usage:
 *   BallPathCalculator calc = new BallPathCalculator();
 *   PathResult result = calc.calculate(robotPose, ballPositions);
 *   if (result.isValid()) {
 *       Logger.recordOutput("BallAssist/Path", result.getTrajectory());
 *       Logger.recordOutput("BallAssist/CapturedBalls", result.getBallPoses());
 *       Logger.recordOutput("BallAssist/Corridor", result.getCorridorPoses());
 *       Logger.recordOutput("BallAssist/AllBalls", result.getAllBallPoses(ballPositions));
 *   }
 *
 * All positions are field-relative. Heading 0 = +X, CCW positive (WPILib).
 */
public class BallPathCalculator {

    // ── Tuning ──────────────────────────────────────────────────────────

    private double corridorHalfWidth = 0.45;
    private double maxLookahead      = 5.0;
    private double minDistance        = 0.3;
    private double distanceDecay     = 1.2;
    private double centerlineBonus   = 0.4;
    private double sweepStepRad      = Math.toRadians(2.0);
    private double sweepHalfArc      = Math.toRadians(90.0);
    private double maxSpeed          = 4.5;
    private double maxAccel          = 3.0;


    // ── Result ──────────────────────────────────────────────────────────

    public static class PathResult {
        public final double heading;
        public final double relativeHeading;
        public final double score;
        public final List<Translation2d> capturedBalls;
        public final List<Double> capturedDistances;
        public final Pose2d robotPose;
        public final double corridorHalfWidth;

        public PathResult(
                double heading,
                double relativeHeading,
                double score,
                List<Translation2d> capturedBalls,
                List<Double> capturedDistances,
                Pose2d robotPose,
                double corridorHalfWidth) {
            this.heading = heading;
            this.relativeHeading = relativeHeading;
            this.score = score;
            this.capturedBalls = capturedBalls;
            this.capturedDistances = capturedDistances;
            this.robotPose = robotPose;
            this.corridorHalfWidth = corridorHalfWidth;
        }

        public boolean isValid() {
            return !capturedBalls.isEmpty();
        }

        public int ballCount() {
            return capturedBalls.size();
        }

        // ═════════════════════════════════════════════════════════════
        //  ADVANTAGESCOPE OUTPUTS
        // ═════════════════════════════════════════════════════════════

        /**
         * WPILib Trajectory along the corridor centerline, passing through
         * each captured ball (projected onto the line).
         *
         * In AdvantageScope:
         *   Add as "Trajectory" type under the 2D Field tab.
         *   Logger.recordOutput("BallAssist/Path", result.getTrajectory());
         */
        public Trajectory getTrajectory() {
            if (!isValid()) {
                return new Trajectory(List.of(
                    new Trajectory.State(0, 0, 0, robotPose, 0)
                ));
            }

            Rotation2d rot = new Rotation2d(heading);
            double cosH = Math.cos(heading);
            double sinH = Math.sin(heading);
            double rx = robotPose.getX();
            double ry = robotPose.getY();

            Pose2d start = new Pose2d(robotPose.getTranslation(), rot);

            // End point: just past the farthest captured ball
            double farthestDist = capturedDistances.get(capturedDistances.size() - 1);
            double endDist = farthestDist + 0.3;
            Pose2d end = new Pose2d(
                rx + endDist * cosH,
                ry + endDist * sinH,
                rot
            );

            // Interior waypoints: balls projected onto the centerline
            List<Translation2d> interior = new ArrayList<>();
            for (int i = 0; i < capturedBalls.size(); i++) {
                Translation2d ball = capturedBalls.get(i);
                double dx = ball.getX() - rx;
                double dy = ball.getY() - ry;
                double projDist = dx * cosH + dy * sinH;

                if (projDist < 0.2 || projDist > endDist - 0.2) continue;

                interior.add(new Translation2d(
                    rx + projDist * cosH,
                    ry + projDist * sinH
                ));
            }

            try {
                TrajectoryConfig config = new TrajectoryConfig(4.5, 3.0);
                return TrajectoryGenerator.generateTrajectory(start, interior, end, config);
            } catch (Exception e) {
                // Fallback: straight line if spline generation fails
                return TrajectoryGenerator.generateTrajectory(
                    start, List.of(), end, new TrajectoryConfig(4.5, 3.0)
                );
            }
        }

        /**
         * Captured ball positions as Pose2d[]. In AdvantageScope, add as
         * a "Ghost" with a custom ball model or just as Pose2d markers.
         *
         * Logger.recordOutput("BallAssist/CapturedBalls", result.getBallPoses());
         */
        public Pose2d[] getBallPoses() {
            Pose2d[] poses = new Pose2d[capturedBalls.size()];
            Rotation2d rot = new Rotation2d(heading);
            for (int i = 0; i < capturedBalls.size(); i++) {
                poses[i] = new Pose2d(capturedBalls.get(i), rot);
            }
            return poses;
        }

        /**
         * Corridor boundary as Pose2d[6] — forms a closed rectangle you can
         * trace in AdvantageScope as a path/ghost to see the intake sweep.
         *
         * Points: [leftStart, leftEnd, rightEnd, rightStart, leftStart, endCenter]
         * The last point (endCenter) gives you the target endpoint.
         *
         * Logger.recordOutput("BallAssist/Corridor", result.getCorridorPoses());
         */
        public Pose2d[] getCorridorPoses() {
            Rotation2d rot = new Rotation2d(heading);
            double cosH = Math.cos(heading);
            double sinH = Math.sin(heading);
            double perpX = -sinH;
            double perpY = cosH;

            double rx = robotPose.getX();
            double ry = robotPose.getY();
            double endDist = capturedDistances.isEmpty()
                ? 3.0
                : capturedDistances.get(capturedDistances.size() - 1) + 0.3;

            double hw = corridorHalfWidth;

            // Four corners of the corridor rectangle
            Pose2d leftStart = new Pose2d(
                rx + perpX * hw, ry + perpY * hw, rot);
            Pose2d leftEnd = new Pose2d(
                rx + endDist * cosH + perpX * hw,
                ry + endDist * sinH + perpY * hw, rot);
            Pose2d rightEnd = new Pose2d(
                rx + endDist * cosH - perpX * hw,
                ry + endDist * sinH - perpY * hw, rot);
            Pose2d rightStart = new Pose2d(
                rx - perpX * hw, ry - perpY * hw, rot);

            // Close the rectangle + add endpoint for reference
            Pose2d endCenter = new Pose2d(
                rx + endDist * cosH, ry + endDist * sinH, rot);

            return new Pose2d[] {
                leftStart, leftEnd, rightEnd, rightStart, leftStart, endCenter
            };
        }

        /**
         * ALL balls (captured + uncaptured) for full field visualization.
         * Captured balls use the corridor heading rotation, uncaptured use 0°,
         * so you can visually distinguish them in AdvantageScope.
         *
         * Logger.recordOutput("BallAssist/AllBalls", result.getAllBallPoses(balls));
         */
        public Pose2d[] getAllBallPoses(List<Pose2d> allBalls) {
            Pose2d[] poses = new Pose2d[allBalls.size()];
            Rotation2d capturedRot = new Rotation2d(heading);
            Rotation2d uncapturedRot = new Rotation2d();

            for (int i = 0; i < allBalls.size(); i++) {
                Translation2d t = allBalls.get(i).getTranslation();
                boolean isCaptured = false;
                for (Translation2d cb : capturedBalls) {
                    if (cb.getDistance(t) < 0.01) {
                        isCaptured = true;
                        break;
                    }
                }
                poses[i] = new Pose2d(t, isCaptured ? capturedRot : uncapturedRot);
            }
            return poses;
        }
    }


    // ── Core Algorithm ──────────────────────────────────────────────────

    public PathResult calculate(Pose2d robotPose, List<Pose2d> ballPositions) {

        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getRotation().getRadians();

        int n = ballPositions.size();
        double[] dx = new double[n];
        double[] dy = new double[n];
        double[] dist = new double[n];
        boolean[] inRange = new boolean[n];

        for (int i = 0; i < n; i++) {
            dx[i] = ballPositions.get(i).getX() - robotX;
            dy[i] = ballPositions.get(i).getY() - robotY;
            dist[i] = Math.sqrt(dx[i] * dx[i] + dy[i] * dy[i]);
            inRange[i] = dist[i] >= minDistance && dist[i] <= maxLookahead;
        }

        double bestScore = -1;
        double bestHeading = robotHeading;
        List<Integer> bestIndices = new ArrayList<>();

        double startAngle = robotHeading - sweepHalfArc;
        double endAngle = robotHeading + sweepHalfArc;

        for (double angle = startAngle; angle <= endAngle; angle += sweepStepRad) {
            double cosA = Math.cos(angle);
            double sinA = Math.sin(angle);
            double perpX = -sinA;
            double perpY = cosA;

            double corridorScore = 0;
            List<Integer> corridorIndices = new ArrayList<>();

            for (int i = 0; i < n; i++) {
                if (!inRange[i]) continue;

                double forward = dx[i] * cosA + dy[i] * sinA;
                if (forward < minDistance) continue;

                double lateral = dx[i] * perpX + dy[i] * perpY;
                double absLateral = Math.abs(lateral);
                if (absLateral > corridorHalfWidth) continue;

                double ballScore = 1.0 / Math.pow(dist[i], distanceDecay);
                double centerFactor = 1.0 - (absLateral / corridorHalfWidth);
                ballScore *= (1.0 + centerlineBonus * centerFactor);

                corridorScore += ballScore;
                corridorIndices.add(i);
            }

            if (corridorScore > bestScore) {
                bestScore = corridorScore;
                bestHeading = angle;
                bestIndices = corridorIndices;
            }
        }

        List<Translation2d> capturedBalls = new ArrayList<>();
        List<Double> capturedDistances = new ArrayList<>();

        bestIndices.sort((a, b) -> Double.compare(dist[a], dist[b]));

        for (int idx : bestIndices) {
            capturedBalls.add(ballPositions.get(idx).getTranslation());
            capturedDistances.add(dist[idx]);
        }

        double relativeHeading = normalizeAngle(bestHeading - robotHeading);

        return new PathResult(bestHeading, relativeHeading, bestScore,
                capturedBalls, capturedDistances, robotPose, corridorHalfWidth);
    }


    // ── Utility ─────────────────────────────────────────────────────────

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }


    // ── Builder-style setters ───────────────────────────────────────────

    public BallPathCalculator withCorridorWidth(double fullWidthMeters) {
        this.corridorHalfWidth = fullWidthMeters / 2.0;
        return this;
    }

    public BallPathCalculator withMaxLookahead(double meters) {
        this.maxLookahead = meters;
        return this;
    }

    public BallPathCalculator withMinDistance(double meters) {
        this.minDistance = meters;
        return this;
    }

    public BallPathCalculator withDistanceDecay(double exponent) {
        this.distanceDecay = exponent;
        return this;
    }

    public BallPathCalculator withCenterlineBonus(double bonus) {
        this.centerlineBonus = bonus;
        return this;
    }

    public BallPathCalculator withSweepResolution(double degrees) {
        this.sweepStepRad = Math.toRadians(degrees);
        return this;
    }

    public BallPathCalculator withSweepArc(double halfArcDegrees) {
        this.sweepHalfArc = Math.toRadians(halfArcDegrees);
        return this;
    }

    public BallPathCalculator withMaxSpeed(double metersPerSec) {
        this.maxSpeed = metersPerSec;
        return this;
    }

    public BallPathCalculator withMaxAccel(double metersPerSecSq) {
        this.maxAccel = metersPerSecSq;
        return this;
    }
}