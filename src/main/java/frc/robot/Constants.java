// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import edu.wpi.first.math.Matrix;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (140) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(14); //was 14.5
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

// Can IDs
  public static int Can_PDH = 1;
  public static int Can_FL_Drive = 2;
  public static int Can_FL_Angle = 3;
  public static int Can_FR_Drive = 4;
  public static int Can_FR_Angle = 5;
  public static int Can_BR_Drive = 6;
  public static int Can_BR_Angle = 7;
  public static int Can_BL_Drive = 8;
  public static int Can_BL_Angle = 9;
  public static int Can_Shooter1 = 10;
  public static int Can_Shooter2 = 11;
  public static int Can_Feeder = 12;
  public static int Can_Hood = 13;
  public static int Can_Rollers = 14;
  public static int Can_Tipper = 15;
  public static int Can_Intake1 = 16;
  public static int Can_Intake2 = 17;
  public static int Can_Pigeon = 18;
  public static int Can_Mitrocandria = 19;
  public static int Can_Climber = 20;
  public static int Can_TipperAbsEncoder = 21;

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }
public static final class DriveConstants {

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(24);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(24);  //27.5 less 3.5
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
  }
  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
  public static class Vision4920 {
    public static final String kFrontCam = "Front";  //Intake Camera
    // public static final String kRearCam = "RearCam"; //Shooter Camera
    public static final String kLeftCam = "Left";  //Climber Camera
    public static final String kRightCam = "Right"; //Right Camera
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.


    // positive x to the left, positive up
    public static final Transform3d kRobotToFrontCam =
            new Transform3d(new Translation3d(Units.inchesToMeters(1.75), Units.inchesToMeters(1.0), Units.inchesToMeters(19.5)), //0,-7.5,32
            new Rotation3d(Units.degreesToRadians(-1.4), Units.degreesToRadians(0.35), Units.degreesToRadians(0))); //

    public static final Transform3d kRobotToRightCam =
            new Transform3d(new Translation3d(Units.inchesToMeters(0.0), Units.inchesToMeters(-12.25), Units.inchesToMeters(17.25)), 
            new Rotation3d(Units.degreesToRadians(-1.5), Units.degreesToRadians(-2.5), Units.degreesToRadians(-90))); //

    public static final Transform3d kRobotToLeftCam =
            new Transform3d(new Translation3d(Units.inchesToMeters(0.25), Units.inchesToMeters(12.25), Units.inchesToMeters(17)), 
            new Rotation3d(Units.degreesToRadians(-1.0), Units.degreesToRadians(-7.5), Units.degreesToRadians(90))); //
  // public static final Transform3d kRobotToRearCam =
  //           new Transform3d(new Translation3d(Units.inchesToMeters(-.5), Units.inchesToMeters(2), Units.inchesToMeters(-32.25)), 
  //           new Rotation3d(Units.degreesToRadians(0), Units.degreesToRadians(-13), Units.degreesToRadians(180))); // 0.48     0,-10,180
  
  // public static final Transform3d ROBOT_TO_CAMERA_Front = kRobotToFrontCam.inverse();
  // public static final Transform3d ROBOT_TO_CAMERA_Rear = kRobotToRearCam.inverse();
  // public static final Transform3d ROBOT_TO_CAMERA_Right = kRobotToRightCam .inverse();
  // public static final Transform3d ROBOT_TO_CAMERA_Left = kRobotToLeftCam.inverse();
  
    
    //public static final AprilTagFieldLayout kTagLayout = new AprilTagFieldLayout(atag.getTags(), 17.548, 8.052);  
    public static final   AprilTagFieldLayout kTagLayout = AprilTagFields.k2026RebuiltWelded.loadAprilTagLayoutField(); 
    // public static final AprilTagFieldLayout kTagLayout =
        //         AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
   

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    //public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(.75, .75, 1.2);
    
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(.5, .5, 1);

// from Hemlock5712
    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;
    public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
    public static final double NOISY_DISTANCE_METERS = 2.5;
    public static final double DISTANCE_WEIGHT = 7;
    public static final int TAG_PRESENCE_WEIGHT = 10;

    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state estimates less. This
     * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then
     * meters.
     *
     * Note: These values are used as base multipliers in confidenceCalculator().
     * Tuned based on observed vision measurement errors:
     * - x, y: 0.5m std dev for close measurements, increases with distance
     * - theta: 0.2 rad std dev, relatively stable
     */
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS = VecBuilder
        .fill(
            // if these numbers are less than one, multiplying will do bad things
            0.5, // x - reduced from 1.0 for better trust in vision measurements
            0.5, // y - reduced from 1.0 for better trust in vision measurements
            0.2 * Math.PI // theta - reduced from 1.0*PI for better rotation estimates
        );

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global measurements from vision
     * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and
     * radians.
     */
   



    public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS = VecBuilder
        .fill(
            // if these numbers are less than one, multiplying will do bad things
            .1, // x
            .1, // y
            .1);



}

    public static class Tipper{
      public static final double TipperKp = 0.05;
      public static final double TipperKi = 0.0;
      public static final double TipperKd = 0.0;

    }
     public static class Hood{
      public static final double HoodKp = 0.05;
      public static final double HoodKi = 0.0;
      public static final double HoodKd = 0.0;

    }
}
