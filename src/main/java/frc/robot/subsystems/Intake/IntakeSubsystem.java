// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  //Motors
  SparkMax Mtr_Inkate = new SparkMax(17, MotorType.kBrushless);
  SparkMax Mtr_IntakeAngle = new SparkMax(18, MotorType.kBrushless);

  //Motor Encoders
  RelativeEncoder enc_Intake = Mtr_Inkate.getEncoder();
  RelativeEncoder enc_IntakeAngle = Mtr_IntakeAngle.getEncoder();

  //Absolute Enocders
  AbsoluteEncoder absEnc_IntakeAngle = Mtr_IntakeAngle.getAbsoluteEncoder();
  
  //PID Controllers
  PIDController PID_IntakeAngle = new PIDController(0.001, 0, 0);

  //Variables
  

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  public void SetIntakeAngle(Double Angle)
  {
    PID_IntakeAngle.setSetpoint(Angle);
  }

  public void SetIntakeSpeed(double Speed)
  {
    Mtr_Inkate.set(Speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
    public void UpdateDataHighway()
  {
    //Set Variables from Datahighway

    //Set Variable to DataHighway
  }
}
