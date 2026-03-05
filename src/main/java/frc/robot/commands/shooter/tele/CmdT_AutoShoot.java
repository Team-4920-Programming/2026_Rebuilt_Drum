// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.tele;

import static edu.wpi.first.units.Units.RPM;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

//import frc.robot.subsystems.Shooter.ShooterYAMS_SubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_AutoShoot extends Command {
  ShooterSubsystem m_shooter;
  SwerveSubsystem m_swerve;
  /** Creates a new CmdT_ShootTillEmpty. */
  public CmdT_AutoShoot(ShooterSubsystem m_ShooterSubsystem, SwerveSubsystem m_SwerveSubsystem) {
    //addRequirements(m_ShooterSubsystem);
    m_shooter = m_ShooterSubsystem;
   m_swerve = m_SwerveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   m_shooter.EnableShooter();
   m_swerve.EnableAutoLock();

   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //   m_shooter.SetShooterSpeeds( 0);
  //   //m_shooter.setVelocity(RPM.of(0));
  //   m_shooter.SetFeederSpeed(0);
  //   m_shooter.SetAugerSpeed(0);
  m_swerve.DisableAutoAim();
  m_shooter.DisableShooter();
   }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
