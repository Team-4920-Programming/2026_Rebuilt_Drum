// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.Tele;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdT_FlushOutRobot extends Command {
  /** Creates a new CmdT_ReverseIntake. */
  IntakeSubsystem m_intake;
  ShooterSubsystem m_shooter;
  public CmdT_FlushOutRobot(IntakeSubsystem intakeSubsystem,ShooterSubsystem shooterSubsystem) {
    m_intake = intakeSubsystem;
    m_shooter = shooterSubsystem;
    addRequirements(m_intake);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.SetIntakeSpeed(1.0);
    m_shooter.SetFeederSpeed(0.4);
     m_shooter.SetRollerSpeed(0.4);  // make this cmd
    //m_shooter.ReverseShooter();//make the cmd
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_intake.SetIntakeSpeed(0.0);
      m_shooter.SetFeederSpeed(0);
     m_shooter.SetRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
