// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Newton;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.config.ServoChannelConfig.BehaviorWhenDisabled;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.DataHighway.ShooterLookupTable;

public class ShooterSubsystem extends SubsystemBase {
   //Motors
  private final TalonFX shooterMotor1 = new TalonFX(Constants.Can_Shooter1);
  private final TalonFX shooterMotor2 = new TalonFX(Constants.Can_Shooter2);
  private final TalonFX feederMotor = new TalonFX(Constants.Can_Feeder);
  private final TalonFX rollerMotor = new TalonFX(Constants.Can_Rollers);
  private final SparkMax hoodMotor = new SparkMax(Constants.Can_Hood,MotorType.kBrushless);
 
  
  //SparkFlex Configs
  SparkFlexConfig mtrCfg_Feeder = new SparkFlexConfig();
  SparkFlexConfig mtrCfg_Roller = new SparkFlexConfig();

  //Encoders
  // RelativeEncoder enc_Shooter1 = Mtr_Shooter1.getEncoder();
  // RelativeEncoder enc_Shooter2 = Mtr_Shooter2.getEncoder();
  //   RelativeEncoder enc_Roller = Mtr_Roller.getEncoder();
  // RelativeEncoder enc_Feeder = Mtr_Feeder.getEncoder();

  //Other Speeds
  double RollerSpeed = -0.8;
  double FeederSpeed = 0.8;
  double m_shooterSpeed = 0.0;
  double HoodSpeed = 0.3;

  //Shooter Velocity Control
  private final VelocityVoltage m_shooterMotorVelocityRequest  = new VelocityVoltage(0).withSlot(0);

  //Critical Motor Currents
  double RollerCurrent =0;
  double FeederCurrent = 0;
  double Shooter1Current = 0;
  double Shooter2Current = 0;
  double RollerJammedCurrent = 20;
  double FeederJammedCurrent = 20;
  boolean unJamRoller = false;
  boolean unJamFeeder = false;
  double JamRollerTime = 0;
  double JamFdderTimer = 0;

  //Mechanism State Flags
  private boolean shooterEnabled = false;
  private boolean feederEnabled = false;
  private boolean rollerEnabled = false;


  public double DHIn_ShotDistance = 0;
  public boolean DHIn_AutoShoot = false;
  public double AngleToHub =0;
  public boolean DHIn_Aimed = false;

  public boolean DHIn_InAllianceZone = false;
  public boolean DHIn_InNeutralZone = false;
  public double DHIn_AngleToOutpost = 0;
  public double DHIn_AngleToDepot = 0;
  public double DHIn_CornerDistance = 0;
  public ChassisSpeeds DHIn_FieldVelocity = new ChassisSpeeds(0,0,0);
  public double DHIn_HubPoseX = 0;
  public double DHIn_HubPoseY =0;
  public double DHOut_reqRobotAngle =0;
  public ShooterLookupTable DHIn_ShooterLookupTable;
   

  DoubleSupplier ShooterKP = DogLog.tunable("Shooter/kp", 0.000167);
  DoubleSupplier ShooterKD = DogLog.tunable("Shooter/kD", 0.000);
  DoubleSupplier ShooterKv = DogLog.tunable("Shooter/kv", 0.00017);
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    //Setup Motors

    var talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.Slot0.kP = 0.000167; // Example P gain
    talonFXConfigs.Slot0.kI = 0.0;
    talonFXConfigs.Slot0.kD = 0.0;
    talonFXConfigs.Slot0.kV = 0.00017; // Example Velocity Feedforward V/rps
    talonFXConfigs.Slot0.kA = 0.0;
    shooterMotor1.getConfigurator().apply(talonFXConfigs);
    shooterMotor2.setControl(new Follower(shooterMotor1.getDeviceID(), MotorAlignmentValue.Opposed));


  }

public void SOTFCalc(){

  // double IdealSpeed = getShooterSpeedForDistance(DHIn_ShotDistance);
  // Translation2d targetPosition = new Translation2d(DHIn_HubPoseX, DHIn_HubPoseY);

  // Translation2d targetVector = targetPosition.div(DHIn_ShotDistance).times(IdealSpeed);
  
  // Translation2d robotVelocity = new Translation2d(DHIn_FieldVelocity.vxMetersPerSecond, DHIn_FieldVelocity.vyMetersPerSecond);
  // Translation2d shotVector = targetVector.minus(robotVelocity);

  // double requiredRobotAngle = shotVector.getAngle().getDegrees();
  // double requiredSpeed = shotVector.getNorm();
  // double shooterRPM = (requiredSpeed);
  // // DHOut_reqRobotAngle = requiredRobotAngle -180;

  // if (requiredRobotAngle > 0) {
  // DHOut_reqRobotAngle = requiredRobotAngle -182.5;
  // }
  // else {
  // DHOut_reqRobotAngle = requiredRobotAngle -180;
  // }
  // if (Shoot && DHIn_InAllianceZone){
  // SetShooterSpeed(shooterRPM);
  // if (ShooterAtSpeed()){
  // Feed = true;
  // }

  
  // }
  // else if (Shoot && DHIn_InNeutralZone){
  // CornerShoot();
  // if (ShooterAtSpeed()){
  // Feed = true;
  // }
  // }

  // else {
  //   RollerSpeed = 0;
  //   FeederSpeed = 0;
  //   SetShooterSpeeds(0);
  //   SetRollerSpeed(RollerSpeed);
  // SetFeederSpeed(FeederSpeed);
  // }
}

  public void SetHoodSpeed(double speed){
    hoodMotor.set(speed);
  }

  public void SetShooterSpeed(double speed)
  {
    m_shooterSpeed = speed; 
  }
  public void SetRollerSpeed (double speed)
  {
    rollerMotor.set(speed);
  }
  public void SetFeederSpeed (double speed)
  {
    feederMotor.set(speed);
  }

  public void EnableShooter(){
    shooterEnabled = true;
  }
public void DisableShooter(){
    shooterEnabled = false;
   
  }

  // public void SOTFShoot(){
  //   SetShooterSpeeds(shooterRPM);
  // }

// public void AutoShoot(){
  

// if (DHIn_ShotDistance >= 0 && DHIn_ShotDistance <0.5){

//  SetShooterSpeeds(1000);


// }

// else if (DHIn_ShotDistance >= 0.5 && DHIn_ShotDistance <1){

//  SetShooterSpeeds(1500);

// }  




// else if (DHIn_ShotDistance >= 1 && DHIn_ShotDistance <1.5){

//  SetShooterSpeeds(2000);

// }  



// else if (DHIn_ShotDistance >= 1.5 && DHIn_ShotDistance <2){

//  SetShooterSpeeds(2500);

 
// }


// else if (DHIn_ShotDistance >= 2 && DHIn_ShotDistance <2.5){

//  SetShooterSpeeds(3000);

 
// }

// else if (DHIn_ShotDistance >= 2.5 && DHIn_ShotDistance <3){

//  SetShooterSpeeds(3500);

// }
// else if (DHIn_ShotDistance >= 3 && DHIn_ShotDistance <3.5){

//  SetShooterSpeeds(4000);
// }
// else if (DHIn_ShotDistance >= 3.5 && DHIn_ShotDistance <4){

//  SetShooterSpeeds(4500);
// }
// else if (DHIn_ShotDistance >= 4 && DHIn_ShotDistance <4.5){

//  SetShooterSpeeds(5000);
// }
// else if (DHIn_ShotDistance >= 4.5 ){

//  SetShooterSpeeds(5500);
// }
//   else {
//    SetShooterSpeeds(0);
//   }
// }

public double getShooterSpeedForDistance(double Distance){
return 430 * Distance + 2420;
//430*distance + 2420 is good
}

public void CornerShoot(){
 
if (DHIn_CornerDistance >=4 && DHIn_CornerDistance < 5.5 ){

 SetShooterSpeed(4000);
 RollerSpeed=1;
 FeederSpeed=1;
 SetRollerSpeed(RollerSpeed);
 SetFeederSpeed(FeederSpeed);
 

}
else if (DHIn_CornerDistance >=5.5 && DHIn_CornerDistance < 6.5 ){
SetShooterSpeed(4500);
RollerSpeed=1;
  FeederSpeed=1;
 SetRollerSpeed(RollerSpeed);
 SetFeederSpeed(FeederSpeed);

}

else if (DHIn_CornerDistance >=6.5 && DHIn_CornerDistance < 10 ){
SetShooterSpeed(5000);
RollerSpeed=1;
  FeederSpeed=1;
 SetRollerSpeed(RollerSpeed);
 SetFeederSpeed(FeederSpeed);

}

else {
  RollerSpeed=0;
  FeederSpeed=0;
  SetShooterSpeed(0);
  SetRollerSpeed(RollerSpeed);
  SetFeederSpeed(FeederSpeed);

}

}






  @Override
  public void periodic() {

    updateLogs();
    ControlShooter();

    
    // DogLog.log("Shooter/Shooter1Speed",enc_Shooter1.getVelocity(),"rpm");
    // DogLog.log("Shooter/Shooter2Speed",enc_Shooter2.getVelocity(),"rpm");
    // DogLog.log("Shooter/RollerSpeed",enc_Roller.getVelocity(),"rpm");
    // DogLog.log("Shooter/FeederSpeed",enc_Feeder.getVelocity(),"rpm");


   

  }

  private void updateLogs(){

    DogLog.log("Shooter/Shooter1Amps", Shooter1Current,"amps");
    DogLog.log("Shooter/Shooter2Amps", Shooter2Current,"amps");
    DogLog.log("Shooter/FeederAmps", FeederCurrent,"amps");
    DogLog.log("Shooter/RollerAmps", RollerCurrent,"amps");
    DogLog.log("Shooter/Enabled",shooterEnabled);
    DogLog.log("Fieldinfo/reqRobotAngle",DHOut_reqRobotAngle);
    DogLog.log("Shooter/SavedShooterSpeedRPM", m_shooterSpeed / 60.0);
    DogLog.log("Shooter/ActualShooterSpeedRPM", shooterMotor1.getVelocity().getValueAsDouble() * 60.0);

  }

  private void ControlShooter(){
    if (shooterEnabled)
    shooterMotor1.setControl(m_shooterMotorVelocityRequest.withVelocity(m_shooterSpeed / 60.0));
  else
    shooterMotor1.setControl(m_shooterMotorVelocityRequest.withVelocity(0.0 / 60.0));
  }

  public void ChangeShooterSpeed(double delta){
        SetShooterSpeed(m_shooterSpeed + delta);
  }

}
