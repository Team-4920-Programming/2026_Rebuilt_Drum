// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new Climber. */
 
 //Motors
  //SparkFlex Mtr_Climber = new SparkFlex(Constants.Can_Climber,MotorType.kBrushless);
  SparkMax Mtr_Climber = new SparkMax(Constants.Can_Climber,MotorType.kBrushless);

//Motor Configs
  SparkMaxConfig mtrCfg_Climber = new SparkMaxConfig();
  SparkMaxConfig mtrCfg_Hools = new SparkMaxConfig();

//Motor Encoders
  RelativeEncoder enc_Climber = Mtr_Climber.getEncoder();
 // RelativeEncoder enc_Hooks = Mtr_Hooks.getEncoder();

  //Absolute Encoders
  //AbsoluteEncoder absEnc_Hooks = Mtr_Hooks.getAbsoluteEncoder();

  
  public ClimberSubsystem() {

 }

  public void DeployHooks()
  {

  }

  public void RetractHooks()
  {

  }

  public void ClimberUp()
  {
Mtr_Climber.set(1);
  }

  public void Climb()
  {
    Mtr_Climber.set(-1);
  }

  public void ClimberStop(){
    Mtr_Climber.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    UpdateDataHighway();
  }
  public void UpdateDataHighway()
  {
    //Set Variables from Datahighway

    //Set Variable to DataHighway
  }
}
