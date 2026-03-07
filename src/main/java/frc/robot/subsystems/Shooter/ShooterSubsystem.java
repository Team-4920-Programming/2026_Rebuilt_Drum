// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Newton;

import java.util.function.DoubleSupplier;

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
    RelativeEncoder enc_Auger = Mtr_Auger.getEncoder();
  RelativeEncoder enc_Feeder = Mtr_Feeder.getEncoder();

  //Other Speeds
  double AugerSpeed = 0;
  double FeederSpeed = 0;
  

  //Critical Motor Currents
  double AugerCurrent =0;
  double FeederCurrent = 0;
  double Shooter1Current = 0;
  double Shooter2Current = 0;
  double AugerJammedCurrent = 20;
  double FeederJammedCurrent = 20;
  boolean unJamAuger = false;
  boolean unJamFeeder = false;
  double  JamAugerTime = 0;
  double JamFdderTimer = 0;


  public double DHIn_ShotDistance = 0;
  public boolean DHIn_AutoShoot = false;
  public double AngleToHub =0;
  public boolean DHIn_Aimed = false;
   private boolean Shoot = false;
   public boolean DHIn_InAllianceZone = false;
   public boolean DHIn_InNeutralZone = false;
   public double DHIn_AngleToOutpost = 0;
   public double DHIn_AngleToDepot = 0;
   public double DHIn_CornerDistance = 0;
   public ChassisSpeeds DHIn_FieldVelocity = new ChassisSpeeds(0,0,0);
   public double DHIn_HubPoseX;
   public double DHIn_HubPoseY;
   public double DHOut_reqRobotAngle =0;
   
  



  //Servo
  ServoHub Servos = new ServoHub(14);
  ServoHubConfig cfg_Servos = new ServoHubConfig();
  ServoChannel Hood1 = Servos.getServoChannel(ChannelId.kChannelId0);
  ServoChannel Hood2 = Servos.getServoChannel(ChannelId.kChannelId1);


  

  //PIDs
  PIDController PID_Shooter1 = new PIDController(.000167, 0, 0.0);
  PIDController PID_Shooter2 = new PIDController(.0, 0, 0);
  SimpleMotorFeedforward FF_Shooter1 = new SimpleMotorFeedforward(0, 0.00017);
    SimpleMotorFeedforward FF_Shooter2 = new SimpleMotorFeedforward(0, 0.00017);

  DoubleSupplier ShooterKP = DogLog.tunable("Shooter/kp", 0.000167);
  DoubleSupplier ShooterKD = DogLog.tunable("Shooter/kD", 0.000);
   DoubleSupplier ShooterKv = DogLog.tunable("Shooter/kv", 0.00017);
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    //Setup Motors
    mtrCfg_Shooter1.smartCurrentLimit(60);
    mtrCfg_Shooter1.idleMode(IdleMode.kCoast);
    Mtr_Shooter1.configure(mtrCfg_Shooter1,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    
    mtrCfg_Shooter2.smartCurrentLimit(60);
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

public void SOTFCalc(){

double IdealSpeed = getShooterSpeedForDistance(DHIn_ShotDistance);
  Translation2d targetPosition = new Translation2d(DHIn_HubPoseX, DHIn_HubPoseY);

  Translation2d targetVector = targetPosition.div(DHIn_ShotDistance).times(IdealSpeed);
  
  Translation2d robotVelocity = new Translation2d(DHIn_FieldVelocity.vxMetersPerSecond, DHIn_FieldVelocity.vyMetersPerSecond);
  Translation2d shotVector = targetVector.minus(robotVelocity);

  double requiredRobotAngle = shotVector.getAngle().getDegrees();
double requiredSpeed = shotVector.getNorm();
double shooterRPM = (requiredSpeed);
  DHOut_reqRobotAngle = requiredRobotAngle -180;
if (Shoot){
 SetShooterSpeeds(shooterRPM);
}
else {
  SetShooterSpeeds(0);
}
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
    return PID_Shooter1.atSetpoint() || enc_Shooter1.getVelocity()>3000 ;
    //return (enc_Shooter1.getVelocity()>3000);
  }

  public boolean Shooter2AtSpeed ()
  {
    return PID_Shooter2.atSetpoint() || enc_Shooter1.getVelocity()>3000;
    //return (enc_Shooter2.getVelocity()>3000);
  }
  public void EnableShooter(){
    Shoot = true;
    
  }
public void DisableShooter(){
    Shoot = false;
   
  }

  // public void SOTFShoot(){
  //   SetShooterSpeeds(shooterRPM);
  // }

public void AutoShoot(){
  

if (DHIn_ShotDistance >= 0 && DHIn_ShotDistance <0.5){

 SetShooterSpeeds(1000);


}

else if (DHIn_ShotDistance >= 0.5 && DHIn_ShotDistance <1){

 SetShooterSpeeds(1500);

}  




else if (DHIn_ShotDistance >= 1 && DHIn_ShotDistance <1.5){

 SetShooterSpeeds(2000);

}  



else if (DHIn_ShotDistance >= 1.5 && DHIn_ShotDistance <2){

 SetShooterSpeeds(2500);

 
}


else if (DHIn_ShotDistance >= 2 && DHIn_ShotDistance <2.5){

 SetShooterSpeeds(3000);

 
}

else if (DHIn_ShotDistance >= 2.5 && DHIn_ShotDistance <3){

 SetShooterSpeeds(3500);

}
else if (DHIn_ShotDistance >= 3 && DHIn_ShotDistance <3.5){

 SetShooterSpeeds(4000);
}
else if (DHIn_ShotDistance >= 3.5 && DHIn_ShotDistance <4){

 SetShooterSpeeds(4500);
}
else if (DHIn_ShotDistance >= 4 && DHIn_ShotDistance <4.5){

 SetShooterSpeeds(5000);
}
else if (DHIn_ShotDistance >= 4.5 ){

 SetShooterSpeeds(5500);
}
  else {
   SetShooterSpeeds(0);
  }
}

public double getShooterSpeedForDistance(double Distance){
return 750 * Distance + 1000;
}

public void CornerShoot(){
if (DHIn_CornerDistance >=4 && DHIn_CornerDistance < 5.5 ){

 SetShooterSpeeds(4000);


}
else if (DHIn_CornerDistance >=5.5 && DHIn_CornerDistance < 6.5 ){
SetShooterSpeeds(4500);

}

else if (DHIn_CornerDistance >=6.5 && DHIn_CornerDistance < 10 ){
SetShooterSpeeds(5000);

}

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
    PID_Shooter1.setD(ShooterKD.getAsDouble());
    PID_Shooter2.setD(ShooterKD.getAsDouble());
    PID_Shooter1.setP(ShooterKP.getAsDouble());
    PID_Shooter2.setP(ShooterKP.getAsDouble());
    FF_Shooter1.setKv(ShooterKv.getAsDouble());
    FF_Shooter2.setKv(ShooterKv.getAsDouble());
    AugerCurrent = Mtr_Auger.getOutputCurrent();
    FeederCurrent = Mtr_Feeder.getOutputCurrent();
    Shooter1Current = Mtr_Shooter1.getOutputCurrent();
    Shooter2Current = Mtr_Shooter2.getOutputCurrent();
    if (DHIn_InAllianceZone){
    
    // AutoShoot();
    //SOTFShoot();
    SOTFCalc();
    
    
  }
else {
  if((DHIn_AngleToOutpost <20 || DHIn_AngleToDepot <20 )&& Shoot){
CornerShoot();
  }
  else {
SetShooterSpeeds(0);
  }
}
 

  
    DogLog.log("Shooter/Shooter1Speed",enc_Shooter1.getVelocity(),"rpm");
    DogLog.log("Shooter/Shooter2Speed",enc_Shooter2.getVelocity(),"rpm");
    DogLog.log("Shooter/AugerSpeed",enc_Auger.getVelocity(),"rpm");
    DogLog.log("Shooter/FeederSpeed",enc_Feeder.getVelocity(),"rpm");

    DogLog.log("Shooter/Shooter1Amps", Shooter1Current,"amps");
    DogLog.log("Shooter/Shooter2Amps", Shooter2Current,"amps");
    DogLog.log("Shooter/FeederAmps", FeederCurrent,"amps");
    DogLog.log("Shooter/AugerAmps", AugerCurrent,"amps");
    DogLog.log("Shooter/Shooter1Setpoint", PID_Shooter1.getSetpoint());
    DogLog.log("Shooter/Shooter2Setpoint", PID_Shooter2.getSetpoint());
    DogLog.log("Shooter/Shoot",Shoot);
DogLog.log("Fieldinfo/reqRobotAngle",DHOut_reqRobotAngle);


    if (PID_Shooter1.getSetpoint() > 500)
    {
      double mtrSpeed = PID_Shooter1.calculate(enc_Shooter1.getVelocity())+ FF_Shooter1.calculate(PID_Shooter1.getSetpoint());
      Mtr_Shooter1.set(mtrSpeed);
      DogLog.log("Shooter/MotorCmd",mtrSpeed);
    }
    else 
      Mtr_Shooter1.set(0);
    if (PID_Shooter2.getSetpoint() > 500)
        Mtr_Shooter2.set(PID_Shooter1.calculate(enc_Shooter2.getVelocity())+ FF_Shooter2.calculate(PID_Shooter2.getSetpoint()));
    else 
      Mtr_Shooter2.set(0);


    Mtr_Auger.set(AugerSpeed);
    Mtr_Feeder.set(FeederSpeed);




    
    // This method will be called once per scheduler run
    UpdateDataHighway();
  }
    public void UpdateDataHighway()
  {
    //Set Variables from Datahighway

    //Set Variable to DataHighway
  }
}
