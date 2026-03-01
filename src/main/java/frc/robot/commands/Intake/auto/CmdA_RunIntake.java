// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdA_RunIntake extends Command {
  /** Creates a new CmdT_RunIntake. */
    IntakeSubsystem Intake;
  double ReqSpeed = 0;
  public CmdA_RunIntake(IntakeSubsystem IntakeSS, double IntakeSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    ReqSpeed = IntakeSpeed;
    Intake = IntakeSS;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Intake.SetIntakeSpeed(ReqSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Intake.SetIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
