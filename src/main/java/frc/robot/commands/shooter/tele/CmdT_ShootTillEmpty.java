// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.tele;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_ShootTillEmpty extends Command {
  ShooterSubsystem m_shooter;
  /** Creates a new CmdT_ShootTillEmpty. */
  public CmdT_ShootTillEmpty(ShooterSubsystem m_ShooterSubsystem) {
    addRequirements(m_ShooterSubsystem);
    m_shooter = m_ShooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DoubleSubscriber ShooterSpeed = DogLog.tunable("Shooter/ShooterSpeed", 3500.0,"rpm");
    DoubleSubscriber FeederSpeed = DogLog.tunable("Shooter/FeederSpeed", 0.5);
    DoubleSubscriber AugerSpeed = DogLog.tunable("Shooter/AugerSpeed",0.5);



    m_shooter.SetShooterSpeeds(ShooterSpeed.get());
    if (m_shooter.Shooter2AtSpeed() && m_shooter.Shooter1AtSpeed())
    {
      m_shooter.SetFeederSpeed(FeederSpeed.get());
      m_shooter.SetAugerSpeed(AugerSpeed.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.SetShooterSpeeds( 0);
    m_shooter.SetFeederSpeed(0);
    m_shooter.SetAugerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
