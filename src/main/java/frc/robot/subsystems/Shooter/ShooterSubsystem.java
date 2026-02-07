// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Newton;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.config.ServoChannelConfig.BehaviorWhenDisabled;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;

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

  //Encoders
  RelativeEncoder enc_Shooter1 = Mtr_Shooter1.getEncoder();
  RelativeEncoder enc_Shooter2 = Mtr_Shooter2.getEncoder();

  //Other Speeds
  double AugerSpeed = 0;
  double FeederSpeed = 0;

  //Critical Motor Currents
  double AugerCurrent =0;
  double FeederCurrent = 0;
  double AugerJammedCurrent = 20;
  double FeederJammedCurrent = 20;
  boolean unJamAuger = false;
  boolean unJamFeeder = false;
  double  JamAugerTime = 0;
  double JamFdderTimer = 0;
    
  //Servo
  ServoHub Servos = new ServoHub(14);
  ServoHubConfig cfg_Servos = new ServoHubConfig();
  ServoChannel Hood1 = Servos.getServoChannel(ChannelId.kChannelId0);
  ServoChannel Hood2 = Servos.getServoChannel(ChannelId.kChannelId1);


  

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

    //setup Servohub
    cfg_Servos.channel0.pulseRange(500,1500,2500);
    cfg_Servos.channel1.pulseRange(500,1500,2500);
    cfg_Servos.channel0.disableBehavior(BehaviorWhenDisabled.kSupplyPower);
    cfg_Servos.channel1.disableBehavior(BehaviorWhenDisabled.kSupplyPower); 
    Servos.configure(cfg_Servos,ResetMode.kResetSafeParameters);


  }
  public void SetShooterSpeeds(double speed)
  {
    PID_Shooter1.setSetpoint(speed);
    PID_Shooter2.setSetpoint(speed);
  }
  public void SetAugerSpeed (double speed)
  {
    Mtr_Auger.set(speed);
  }
  public void SetFeederSpeed (double speed)
  {
    Mtr_Feeder.set(speed);
  }

  public boolean Shooter1AtSpeed ()
  {
    return PID_Shooter1.atSetpoint();
  }

  public boolean Shooter2AtSpeed ()
  {
    return PID_Shooter2.atSetpoint();
  }
public void SetHood(int HoodAngle)
{
  Hood1.setEnabled(true);
  Hood2.setEnabled(true);
int PulseWidth = (HoodAngle / 270) * (2500-500) + 500;
PulseWidth = PulseWidth *1000; // convert to microseconds
  Hood1.setPulseWidth(PulseWidth);
  Hood2.setPulseWidth(PulseWidth);
}

  @Override
  public void periodic() {
    if (PID_Shooter1.getSetpoint() > 500)
      Mtr_Shooter1.set(PID_Shooter1.calculate(enc_Shooter1.getVelocity()));
    else 
      Mtr_Shooter1.set(0);
    if (PID_Shooter2.getSetpoint() > 500)
        Mtr_Shooter2.set(PID_Shooter1.calculate(enc_Shooter2.getVelocity()));
    else 
      Mtr_Shooter2.set(0);


    Mtr_Auger.set(AugerSpeed);
    Mtr_Feeder.set(FeederSpeed);

    AugerCurrent = Mtr_Auger.getOutputCurrent();
    FeederCurrent = Mtr_Feeder.getOutputCurrent();

    if (AugerCurrent > AugerJammedCurrent && unJamAuger == false)
    {
      unJamAuger = true;
      JamAugerTime = RobotController.getTime();
      Mtr_Auger.set(-.25);

    }
    if (unJamAuger == true && (RobotController.getTime() - JamAugerTime) > 500000)
    {
      unJamAuger = false;
      Mtr_Auger.set(AugerSpeed);

    }

    
    // This method will be called once per scheduler run
  }
}
