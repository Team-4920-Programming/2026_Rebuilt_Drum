// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem.TipperState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CmdA_HopperDownStart extends Command {
  /** Creates a new CmdA_HopperDownStart. */
  IntakeSubsystem m_intake;
  public CmdA_HopperDownStart(IntakeSubsystem intakeSubsystem) {
    m_intake = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.OverrideTipperPID(true);
    m_intake.SetTipperSpeed(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.SetTipperSpeed(-1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.SetTipperSpeed(0);
    m_intake.OverrideTipperPID(false);
    m_intake.SetTipperState(TipperState.INTAKING);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_intake.getTipperAngle() - TipperState.INTAKING.getAngle()) <= 15.0;
  }
}
