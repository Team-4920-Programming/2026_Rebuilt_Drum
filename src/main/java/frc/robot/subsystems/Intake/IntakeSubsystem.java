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
import com.revrobotics.encoder.SplineEncoder;
import com.revrobotics.encoder.config.DetachedEncoderConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  
  public enum TipperState{
    TUCKED(70.0),
    SHOOTING(25.0),
    INTAKING(0.0);

    private final double angle;

    // Constructor
    TipperState(double angle) {
        this.angle = angle;
    }

    // Getter
    public double getAngle() {
        return angle;
    }

  }

  TipperState DHOut_tipperState = TipperState.INTAKING;
  boolean tipperOverride = false;
  //Motors
  SparkFlex intakeMotor1 = new SparkFlex(Constants.Can_Intake1, MotorType.kBrushless);
  SparkFlex intakeMotor2 = new SparkFlex(Constants.Can_Intake2, MotorType.kBrushless);
  SparkFlex tipperMotor = new SparkFlex(Constants.Can_Tipper, MotorType.kBrushless);

  SparkFlexConfig Intake1Config = new SparkFlexConfig();
  SparkFlexConfig Intake2Config = new SparkFlexConfig();
  SparkFlexConfig TipperConfig = new SparkFlexConfig();

  //Absolute Enocders
  //SplineEncoder tipAbsoluteEncoder = new SplineEncoder(Constants.Can_TipperAbsEncoder);
  private final CANcoder tipAbsoluteEncoder = new CANcoder(Constants.Can_TipperAbsEncoder);
  //PID Controllers
  PIDController tipperPID = new PIDController(Constants.Tipper.TipperKp, Constants.Tipper.TipperKi, Constants.Tipper.TipperKd);
  // DoubleSupplier tipperSetpoint = DogLog.tunable("Intake/TipperSetpoint", 45.0);
  // DoubleSupplier IntakeAngKP = DogLog.tunable("Intake/Angle_kp", 0.003);
  // DoubleSupplier IntakeAngKD = DogLog.tunable("Intake/Angle_kD", 0.0);
  // DoubleSupplier IntakeAngKs = DogLog.tunable("Intake/Angle_ks", 0.0);
  // DoubleSupplier IntakeAngKg = DogLog.tunable("Intake/Angle_kg", 0.0);
  // DoubleSupplier IntakeAngKv = DogLog.tunable("Intake/Angle_kv", 0.0);

  //Variables
  public boolean Intake = false;
  double IntakeSpeed = 0;
  double tipperOutput = 0;
  //Datahighway
  boolean DHOut_IntakeOut = false;

  

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    
        
    Intake1Config.inverted(true);
    Intake1Config.idleMode(IdleMode.kBrake);
    Intake1Config.smartCurrentLimit(40);
    intakeMotor1.configure(Intake1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    
    // Intake2Config.inverted(true);
    Intake2Config.idleMode(IdleMode.kBrake);
    Intake2Config.follow(intakeMotor1,true);
    intakeMotor2.configure(Intake2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    TipperConfig.idleMode(IdleMode.kCoast);
    TipperConfig.smartCurrentLimit(40);
    TipperConfig.disableFollowerMode();
    TipperConfig.inverted(true);
    tipperMotor.configure(TipperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // DetachedEncoderConfig dec = new DetachedEncoderConfig();
    // dec.positionConversionFactor(360);
    // dec.angleConversionFactor(360);
    // dec.dutyCycleZeroCentered(true);
    // dec.dutyCycleOffset(0.56874955);
    // dec.inverted(true);

//    tipAbsoluteEncoder.configure(dec, ResetMode.kNoResetSafeParameters);
tipperPID.enableContinuousInput(-180, 180);
tipperPID.setSetpoint(getTipperAngle());
   }      

public double getTipperAngle() {
        // return hoodMotor.getPosition().getValueAsDouble() / 360;
        double absolutePosition = tipAbsoluteEncoder.getAbsolutePosition().getValueAsDouble(); // Returns in Rotations (0-1)


        return absolutePosition * 360;
    }
  public void SetIntakeAngle(Double Angle)
  {
    tipperPID.setSetpoint(Angle);
  }

  public void RunIntake(){
    Intake = true;
  }

  public void StopIntake(){
    Intake = false;
  }

  public void SetIntakeSpeed(double Speed){
    intakeMotor1.set(Speed);
  }
  public void SetTipperSpeed(double Speed){
    tipperMotor.set(Speed);
  }
  public void OverrideTipperPID(boolean b){
    tipperOverride = b;
  }
  public void SetTipperState(TipperState state){
    DHOut_tipperState = state;
  }
  
  private void TuneTipperPID(){
    // tipperPID.setP(IntakeAngKP.getAsDouble());
    // tipperPID.setD(IntakeAngKD.getAsDouble());
    // tipperPID.setSetpoint(tipperSetpoint.getAsDouble());
  }

  public void setTipperAngle(double angle){
    tipperPID.setSetpoint(angle);
  }

  private void ProcessTipperState(){
    if (!tipperOverride){
      tipperPID.setSetpoint(DHOut_tipperState.getAngle());
    }
  }



  @Override
  public void periodic() {

    tipperOutput = tipperPID.calculate(getTipperAngle());
    tipperMotor.set(tipperOutput);

    if (Intake){
      SetIntakeSpeed(1);
    }
    else if (!Intake){
      SetIntakeSpeed(0);
    }
      ProcessTipperState();
      // TuneTipperPID();
      updateLogs();

    }

  private void updateLogs(){
    DogLog.log("Intake/TipperAngle", getTipperAngle());
    DogLog.log("Intake/TipperCurrent", tipperMotor.getOutputCurrent());
    DogLog.log("Intake/TipperPIDOutput", tipperOutput);
    DogLog.log("Intake/TipperPIDSetpoint",tipperPID.getSetpoint());
    DogLog.log("Intake/TipperPIDAtSetpoint",tipperPID.atSetpoint());
    DogLog.log("Intake/TipperState", DHOut_tipperState);
    DogLog.log("Intake/TipperOverride", tipperOverride);
  }
}