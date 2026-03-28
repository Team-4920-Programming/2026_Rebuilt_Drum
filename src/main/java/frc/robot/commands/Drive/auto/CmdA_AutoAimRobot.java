// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdA_AutoAimRobot extends Command {
  /** Creates a new CmdA_AutoAimRobot. */
  SwerveSubsystem m_swerve;
  public CmdA_AutoAimRobot(SwerveSubsystem swerveSubsystem) {
    addRequirements(swerveSubsystem);
    m_swerve = swerveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.EnableAutoAim();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds v = m_swerve.getRobotVelocity();
       v = m_swerve.AutoAimVelocityPIDCalculation(v);
       v = new ChassisSpeeds(0, 0, v.omegaRadiansPerSecond);
       m_swerve.drive(v);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.DisableAutoAim();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_swerve.robotIsAimed() && Math.abs(m_swerve.getRobotVelocity().omegaRadiansPerSecond) <= 0.25;
  }
}
