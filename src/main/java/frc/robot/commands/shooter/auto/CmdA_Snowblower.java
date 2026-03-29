// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdA_Snowblower extends Command {
  /** Creates a new CmdA_Snowblower. */
  IntakeSubsystem m_intake;
  ShooterSubsystem m_shooter;
  SwerveSubsystem m_swerve;
  public CmdA_Snowblower(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intakeSubsystem;
    m_shooter = shooterSubsystem;
    m_swerve = swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.EnableShooter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.SetIntakeSpeed(-1);
    m_shooter.SetRollerSpeed(-1.0);

    if (m_swerve.DHOut_Aimed){
      m_shooter.SetFeederSpeed(-0.8);
    }
    else{
      m_shooter.SetFeederSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_intake.SetIntakeSpeed(0.0);
    m_shooter.SetRollerSpeed(0.0);
    m_shooter.SetFeederSpeed(0);
    m_shooter.DisableShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
