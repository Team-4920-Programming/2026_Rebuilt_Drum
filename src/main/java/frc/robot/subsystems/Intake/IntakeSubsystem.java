// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import dev.doglog.DogLog;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;


import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  //Motors
  SparkFlex Mtr_Intake1 = new SparkFlex(Constants.Can_Intake1, MotorType.kBrushless);
  SparkFlex Mtr_Intake2 = new SparkFlex(Constants.Can_Intake2, MotorType.kBrushless);
  SparkFlex Mtr_Tipper = new SparkFlex(Constants.Can_Tipper, MotorType.kBrushless);

  SparkFlexConfig Intake1Config = new SparkFlexConfig();
SparkFlexConfig Intake2Config = new SparkFlexConfig();


  

  //Motor Encoders
  RelativeEncoder enc_Intake = Mtr_Intake1.getEncoder();
  RelativeEncoder enc_IntakeAngle = Mtr_Tipper.getEncoder();

  //Absolute Enocders
  AbsoluteEncoder absEnc_IntakeAngle = Mtr_Tipper.getAbsoluteEncoder();
  
  //PID Controllers
  PIDController PID_IntakeAngle = new PIDController(0.003, 0, 0);
   ArmFeedforward FF_IntakeAngle= new ArmFeedforward(0, 0, 0);

  DoubleSupplier IntakeAngKP = DogLog.tunable("Intake/Angle_kp", 0.003);
  DoubleSupplier IntakeAngKD = DogLog.tunable("Intake/Angle_kD", 0.0);
   DoubleSupplier IntakeAngKs = DogLog.tunable("Intake/Angle_ks", 0.0);
   DoubleSupplier IntakeAngKg = DogLog.tunable("Intake/Angle_kg", 0.0);
   DoubleSupplier IntakeAngKv = DogLog.tunable("Intake/Angle_kv", 0.0);

  //Variables
boolean Intake = false;
double IntakeSpeed = 0;
  //Datahighway
  boolean DHOut_IntakeOut = false;

  

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    PID_IntakeAngle.setSetpoint(absEnc_IntakeAngle.getPosition());
    PID_IntakeAngle.enableContinuousInput(0, 360);
  

        
        //Intake1Config.inverted(false);
        // Intake1Config.smartCurrentLimit(40);

        
        
        Intake2Config.inverted(true);
        Intake2Config.follow(Mtr_Intake1);
        Mtr_Intake2.configure(Intake2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }      


  public void SetIntakeAngle(Double Angle)
  {
    PID_IntakeAngle.setSetpoint(Angle);
  }

  public void RunIntake(){
Intake = true;
  }

  public void StopIntake(){
Intake = false;
  }

  public void SetIntakeSpeed(double Speed){
    Mtr_Intake1.set(Speed);
  }



  @Override
  public void periodic() {
    PID_IntakeAngle.setD(IntakeAngKD.getAsDouble());
    PID_IntakeAngle.setP(IntakeAngKP.getAsDouble());
    FF_IntakeAngle.setKg(IntakeAngKg.getAsDouble());
    FF_IntakeAngle.setKv(IntakeAngKv.getAsDouble());
    FF_IntakeAngle.setKs(IntakeAngKs.getAsDouble());
    
    double IntakePosRad = Units.degreesToRadians(PID_IntakeAngle.getSetpoint());
    double IntakePosDeg = absEnc_IntakeAngle.getPosition();
    Mtr_Tipper.set(PID_IntakeAngle.calculate(IntakePosDeg)+FF_IntakeAngle.calculate(IntakePosRad, 0));
    DogLog.log("intake/pos",absEnc_IntakeAngle.getPosition());
if (Intake){
  SetIntakeSpeed(1);
}
else if (!Intake){
  SetIntakeSpeed(0);
}



    // This method will be called once per scheduler run
  }
    public void UpdateDataHighway()
  {
    //Set Variables from Datahighway

    //Set Variable to DataHighway
  }
}
