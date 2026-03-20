// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.auto;

import static edu.wpi.first.units.Units.RPM;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
//import frc.robot.subsystems.Shooter.ShooterYAMS_SubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdA_ShootTillEmpty extends Command {
  ShooterSubsystem m_shooter;
  /** Creates a new CmdT_ShootTillEmpty. */
  public CmdA_ShootTillEmpty(ShooterSubsystem m_ShooterSubsystem) {
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
    //DoubleSubscriber ShooterSpeed = DogLog.tunable("Shooter/ShooterSpeed", 5000.0,"rpm");
    //DoubleSubscriber FeederSpeed = DogLog.tunable("Shooter/FeederSpeed", 1.0);
    //DoubleSubscriber AugerSpeed = DogLog.tunable("Shooter/AugerSpeed",0.75);

    double ShooterSpeed = 5000;
    double FeederSpeed = 1;
    double RollerSpeed = .5;
    //m_shooter.setVelocity(RPM.of(3000));
    m_shooter.SetShooterSpeed(ShooterSpeed);
    //double CurrentShooter1Vel = m_shooter.getVelocity().magnitude();

    //m_shooter.SetShooterSpeed(ShooterSpeed.get());
    if (true)
    {
      m_shooter.SetFeederSpeed(FeederSpeed);
      m_shooter.SetRollerSpeed(RollerSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.SetShooterSpeed( 0);
    //m_shooter.setVelocity(RPM.of(0));
    m_shooter.SetFeederSpeed(0);
    m_shooter.SetRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
