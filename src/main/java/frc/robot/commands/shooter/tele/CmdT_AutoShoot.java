// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.tele;

import static edu.wpi.first.units.Units.RPM;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
//import frc.robot.subsystems.Shooter.ShooterYAMS_SubSystem;
import frc.robot.subsystems.Intake.IntakeSubsystem.TipperState;
import frc.robot.Constants.Tipper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_AutoShoot extends Command {
  ShooterSubsystem m_shooter;
  SwerveSubsystem m_swerve;
  IntakeSubsystem m_intake;
  boolean TipperUp = false;
  Timer delay = new Timer();
  
  /** Creates a new CmdT_ShootTillEmpty. */
  public CmdT_AutoShoot(ShooterSubsystem m_ShooterSubsystem, SwerveSubsystem m_SwerveSubsystem, IntakeSubsystem m_IntakeSubsystem) {
    //addRequirements(m_ShooterSubsystem);
    m_shooter = m_ShooterSubsystem;
   m_swerve = m_SwerveSubsystem;
   m_intake = m_IntakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.EnableAutoAim();
    m_intake.OverrideTipperPID(true);
    m_intake.SetTipperSpeed(0);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   m_shooter.EnableShooter();
   
   //m_swerve.EnableAutoLock();
    if (!m_shooter.isShooterAtSpeed())
    {
      m_shooter.reverseFeeder();
    }
    if (m_shooter.isShooterAtSpeed() && m_swerve.robotIsAimed()){
  //   if (TipperUp && m_intake.TipperAtSetpoint())
  //   {
  //     m_intake.SetTipperState(TipperState.INTAKING);
  //     TipperUp = false;
  //   }
  //   else if (!TipperUp && m_intake.TipperAtSetpoint())
  //   {
  //     m_intake.SetTipperState(TipperState.SHOOTING);
  //     TipperUp = true;
  //   }
  
      m_intake.SetIntakeSpeed(-1);
      m_shooter.SetFeederSpeed(-0.8);
      m_shooter.SetRollerSpeed(-1.0);
      delay.start();

      if (!TipperUp && delay.hasElapsed(0.5))
      {
        m_intake.SetTipperSpeed(1.0);
        TipperUp = true;
      }
      else if (TipperUp && delay.hasElapsed(1.0))
      {
        m_intake.SetTipperSpeed(-1.0);
        TipperUp = false;
        delay.restart();
      }
      
   }
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  //   m_shooter.SetShooterSpeeds( 0);
  //   //m_shooter.setVelocity(RPM.of(0));
    m_shooter.SetFeederSpeed(0);
    m_shooter.SetRollerSpeed(0);
    m_intake.SetIntakeSpeed(0);
    m_intake.OverrideTipperPID(false);
    m_intake.SetTipperState(TipperState.INTAKING);
    m_swerve.DisableAutoAim();
    m_shooter.DisableShooter();
   }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
