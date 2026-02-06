// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Newton;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
   //Motors
  SparkFlex Mtr_Shooter1 = new SparkFlex(10,MotorType.kBrushless);
  SparkFlex Mtr_Shooter2 = new SparkFlex(11,MotorType.kBrushless);
  SparkFlex Mtr_Feeder = new SparkFlex(12, MotorType.kBrushless);
  SparkFlex Mtr_Auger = new SparkFlex(13, MotorType.kBrushless);

  //SparkFlex Configs
  SparkFlexConfig mtrCfg_Shooter1 = new SparkFlexConfig();
  SparkFlexConfig mtrCfg_Shooter2 = new SparkFlexConfig();
  SparkFlexConfig mtrCfg_Feeder = new SparkFlexConfig();
  SparkFlexConfig mtrCfg_Auger = new SparkFlexConfig();

  //PIDs
  PIDController PID_Shooter1 = new PIDController(.1, 0, .1);
  PIDController PID_Shooter2 = new PIDController(.1, 0, .1);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    //Setup Motors
    mtrCfg_Shooter1.smartCurrentLimit(40);
    mtrCfg_Shooter1.idleMode(IdleMode.kCoast);
     Mtr_Shooter1.configure(mtrCfg_Shooter1,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mtrCfg_Shooter2.smartCurrentLimit(40);
    mtrCfg_Shooter2.idleMode(IdleMode.kCoast);
    Mtr_Shooter2.configure(mtrCfg_Shooter1,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mtrCfg_Feeder.smartCurrentLimit(40);
    mtrCfg_Feeder.idleMode(IdleMode.kCoast);
    Mtr_Feeder.configure(mtrCfg_Shooter1,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    mtrCfg_Auger.smartCurrentLimit(40);
    mtrCfg_Auger.idleMode( IdleMode.kCoast);
    Mtr_Auger.configure(mtrCfg_Shooter1,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    //Setup PIDs
    PID_Shooter1.setTolerance(100);
    PID_Shooter2.setTolerance(100);
    
  }

   



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
