// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.DriveToTargetV0_1;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Shooter.*;
import frc.robot.subsystems.Intake.*;
import frc.robot.subsystems.Climber.*;
import frc.robot.subsystems.DataHighway.*;
import frc.robot.subsystems.DataHighway.DataHighway.MatchPhase;
import frc.robot.commands.shooter.auto.CmdA_AutoShoot;
import frc.robot.commands.shooter.auto.CmdA_ShootTillEmpty;
import frc.robot.commands.shooter.auto.CmdA_Snowblower;
import frc.robot.commands.shooter.tele.*;
import frc.robot.commands.Climber.tele.CmdT_Climb;
import frc.robot.commands.Climber.tele.CmdT_ClimberUp;
import frc.robot.commands.Drive.CmdT_DepotAutoAim;
import frc.robot.commands.Drive.CmdT_DisableAutoAim;
import frc.robot.commands.Drive.CmdT_EnableAutoAim;
import frc.robot.commands.Drive.CmdT_EnableAutoLock;
import frc.robot.commands.Drive.CmdT_EnableCornerAim;
import frc.robot.commands.Drive.CmdT_OutpostAutoAim;
import frc.robot.commands.Drive.CmdT_TrackBallDrive;
import frc.robot.commands.Drive.auto.CmdA_AutoAimRobot;
// import frc.robot.commands.shooter.Auto.*;
// import frc.robot.commands.Climber.Auto.*;
// import frc.robot.commands.Climber.Tele.*;
// import frc.robot.commands.Intake.Auto.*;
import frc.robot.commands.Intake.Tele.*;
import frc.robot.commands.Intake.auto.CmdA_HopperDownStart;
import frc.robot.commands.Intake.auto.CmdA_RunIntake;
import frc.robot.commands.Intake.auto.CmdA_StopIntake;
import frc.robot.commands.Intake.auto.CmdA_VerifyHopperDown;

import static edu.wpi.first.units.Units.RPM;

import java.io.File;

import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandJoystick toggleSwitchJoystick = new CommandJoystick(2);
    final CommandJoystick operatorJoystick = new CommandJoystick(1);


 
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/maxSwerve"));
  private final ShooterSubsystem Shooter = new ShooterSubsystem();
  //private final ShooterYAMS_SubSystem Shooter = new ShooterYAMS_SubSystem();
  private final IntakeSubsystem Intake = new IntakeSubsystem();
  private final ClimberSubsystem Climber =  new ClimberSubsystem();

  private final DataHighway DH = new DataHighway(drivebase,Shooter,Climber,Intake);

  private final SendableChooser<Command> autoChooser;
  
  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRawAxis(4) * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */ 
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true)
                                                                               .translationHeadingOffset(true)
                                                                               .translationHeadingOffset(Rotation2d.fromDegrees(
                                                                                   0));

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    //  
    
    DogLog.setOptions(new DogLogOptions().withCaptureDs(true).withCaptureNt(true));
    DogLog.setPdh(new PowerDistribution());

    NamedCommands.registerCommand("CmdA_ShootTillEmpty", new CmdA_AutoShoot (Shooter, drivebase, Intake).withTimeout(5));
    NamedCommands.registerCommand("CmdA_RunIntake", new CmdA_RunIntake (Intake));
    NamedCommands.registerCommand("CmdA_StopIntake", new CmdA_StopIntake (Intake));
    NamedCommands.registerCommand("CmdA_AutoAimRobot", new CmdA_AutoAimRobot (drivebase).withTimeout(5));
    NamedCommands.registerCommand("CmdA_EnableShooter", new CmdT_EnableShooter(Shooter));
    NamedCommands.registerCommand("CmdA_DisableShooter", new CmdT_DisableShooter(Shooter));
    NamedCommands.registerCommand("CmdA_VerifyHopperDown", new CmdA_VerifyHopperDown(Intake));
    NamedCommands.registerCommand("CmdA_HopperDownStart", new CmdA_HopperDownStart(Intake));
    NamedCommands.registerCommand("CmdA_Snowblower", new CmdA_Snowblower(Intake, Shooter, drivebase));
    // Configure the trigger bindings
    configureBindings();  
    DriverStation.silenceJoystickConnectionWarning(true);
 //   NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("AutoChooser", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

//     if (Robot.isSimulation())
//     {
//       Pose2d target = new Pose2d(new Translation2d(1, 4),
//                                  Rotation2d.fromDegrees(90));
//       //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
//       driveDirectAngleKeyboard.driveToPose(() -> target,
//                                            new ProfiledPIDController(5,
//                                                                      0,
//                                                                      0,
//                                                                      new Constraints(5, 2)),
//                                            new ProfiledPIDController(5,
//                                                                      0,
//                                                                      0,
//                                                                      new Constraints(Units.degreesToRadians(360),
//                                                                                      Units.degreesToRadians(180))
//                                            ));
//       driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
//       driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
//       driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
//                                                      () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

    //  driverXbox.b().whileTrue(
        //  drivebase.driveToPose(
        //      new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
        //                      );

//        driverXbox.b().whileTrue(
//            drivebase.driveToDistanceCommand(drivebase.getPose(),4.572, 2));

//     }
//     if (DriverStation.isTest())
//     {
//       drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

//       //driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
//       driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
//       driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
        driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
         //driverXbox.a().whileTrue(new DriveToTargetV0_1(drivebase));
        // driverXbox.a().whileTrue(new CmdT_ShootTillEmpty(Shooter));
              driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());


        //driverXbox.a().whileTrue(Shooter.setVelocity(RPM.of(5000)));
       // driverXbox.a().onFalse(Shooter.setVelocity(RPM.of(0)));
        
       // driverXbox.x().whileTrue(new CmdT_SetIntakeAngle(Intake, 70));
        // driverXbox.x().whileFalse(new CmdT_SetIntakeAngle(Intake, 5));
        // driverXbox.x().whileTrue(new CmdT_TipperUp(Intake));
        // driverXbox.b().whileTrue(new CmdT_TipperDown(Intake));
          driverXbox.x().whileTrue(new CmdT_TipperShooting(Intake));
          driverXbox.b().whileTrue(new CmdT_TipperIntaking(Intake));
          driverXbox.y().whileTrue(new CmdT_RunIntake(Intake));
       driverXbox.rightBumper().whileTrue(new CmdT_TipperTucked(Intake));
       // driverXbox.leftBumper().onTrue(new CmdT_DisableAutoAim(drivebase));
       driverXbox.leftTrigger().whileTrue(new CmdT_RampUpShooter(Shooter));
        driverXbox.rightTrigger().whileTrue(new CmdT_AutoShoot(Shooter, drivebase, Intake));
        //driverXbox.leftBumper().whileTrue(new CmdT_AutoAimTest(Shooter, drivebase, Intake));
        driverXbox.povUp().whileTrue(new CmdT_Manual_HoodUp(Shooter));
        driverXbox.povDown().whileTrue(new CmdT_Manual_HoodDown(Shooter));
        driverXbox.povLeft().whileTrue(new CmdT_Manual_SlowDownShooter(Shooter));
        driverXbox.povRight().whileTrue(new CmdT_Manual_SpeedUpShooter(Shooter));


        // driverXbox.povUp().whileTrue(new CmdT_Climb(Climber));
        // driverXbox.povDown().whileTrue(new CmdT_ClimberUp(Climber));
        // driverXbox.rightTrigger().whileTrue(new CmdT_OutpostAutoAim(drivebase));
       // driverXbox.leftTrigger().whileTrue(new CmdT_DepotAutoAim(drivebase));
         // Pre-match calibration routine - Back + Start buttons together
         // This ensures accidental activation is avoided during matches
         //driverXbox.x().onTrue(drivebase.getPreMatchCalibrationCommand());
        // driverXbox.leftBumper().whileTrue(new CmdT_TrackBallDrive(drivebase));
        // toggleSwitchJoystick.button(4).whileTrue(new CmdT_EnableSOTF(Shooter));
        // toggleSwitchJoystick.button(4).whileFalse(new CmdT_DisableSOTF(Shooter));
        

        operatorJoystick.button(3).whileTrue(new CmdT_ReverseIntake(Intake));
        operatorJoystick.button(2).whileTrue(new CmdT_TipperShooting(Intake));
        operatorJoystick.button(5).whileTrue(new CmdT_TipperIntaking(Intake));
        operatorJoystick.button(6).whileTrue(new CmdT_FlushOutRobot(Intake,Shooter));
          
//       driverXbox.back().whileTrue(drivebase.centerModulesCommand());
//       driverXbox.leftBumper().onTrue(Commands.none());
//       driverXbox.rightBumper().onTrue(Commands.none());
//       driverXbox.start().whileTrue(Commands.none());
//       driverXbox.back().whileTrue(Commands.none());
//       driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
//       driverXbox.rightBumper().onTrue(Commands.none());
//     }

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void startMatchTimer(){
    DH.DH_matchTimer.start();
  }
  public void stopMatchTimer(){
    DH.DH_matchTimer.stop();
  }
  public void resetMatchTimer(){
    DH.DH_matchTimer.reset();
  }

  public boolean hasMatchTimerStarted(){
    return DH.DH_matchTimer.isRunning();
  }
}
