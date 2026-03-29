// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.Newton;

import java.util.Map;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.fasterxml.jackson.databind.node.POJONode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.encoder.SplineEncoder;
import com.revrobotics.encoder.config.DetachedEncoderConfig;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.config.ServoChannelConfig.BehaviorWhenDisabled;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.VelocityUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.DataHighway.ShooterLookupTable;
import frc.robot.subsystems.DataHighway.ShooterLookupTable.ShooterParams;

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


  //Other Speeds
  double RollerSpeed = -0.6;// was .8
  double FeederSpeed = 0.6;//was .8 
  double m_shooterSpeed = 2000.0;
  double HoodSpeed = 0.3;
  double shooterTolerance = 50;
  double m_hoodAngle = 0;
  double hoodOutput = 0;

  //Shooter Velocity Control
  private final VelocityVoltage m_shooterMotorVelocityRequest  = new VelocityVoltage(0).withSlot(0);

  AbsoluteEncoder hoodEncoder = hoodMotor.getAbsoluteEncoder();
   PIDController hoodPID = new PIDController(Constants.Hood.HoodKp, Constants.Hood.HoodKi, Constants.Hood.HoodKd);
  //Critical Motor Currents
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

  public double calculatedShooterSpeed = 0.0;
  public double calculatedHoodAngle = 0.0;
  public double DHIn_ShotDistance = 0;
  public boolean DHIn_AutoShoot = false;
  public double AngleToHub =0;
  public boolean DHIn_Aimed = false;
  private double shooterCoastSpeed = 1500;
  public boolean DHIn_InAllianceZone = false;
  public boolean DHIn_InNeutralZone = false;
  public double DHIn_AngleToOutpost = 0;
  public double DHIn_AngleToDepot = 0;
  public double DHIn_CornerDistance = 0;
  public ChassisSpeeds DHIn_FieldVelocity = new ChassisSpeeds(0,0,0);
  public Pose2d DHIn_RobotPose2d = new Pose2d().kZero;
  public double DHIn_HubPoseX = 0;
  public double DHIn_HubPoseY =0;
  public double DHOut_reqRobotAngle =0;
  public Rotation2d DHOut_SOTFTargetAngle = new Rotation2d().kZero;
  public boolean DHOut_SOTF = false;
  public ShooterLookupTable DHIn_ShooterLookupTable;
  public Pose2d DHIN_HubPose = new Pose2d().kZero;
  DoubleSupplier HoodKpTunable = DogLog.tunable("Shooter/HoodAngle_kp", 0.003);
  DoubleSupplier HoodKdTunable = DogLog.tunable("Shooter/HoodAngle_kd", 0.0);
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    //Setup Motors

    var shooter1Config = new TalonFXConfiguration();
    shooter1Config.Slot0.kP = 0.3; // Example P gain //0.33
    shooter1Config.Slot0.kI = 0.0;
    shooter1Config.Slot0.kD = 0.0;
    shooter1Config.Slot0.kV = 0.12; // Example Velocity Feedforward V/rps
    shooter1Config.Slot0.kA = 0.0;
    shooter1Config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    shooter1Config.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(80));
    shooterMotor1.setNeutralMode(NeutralModeValue.Coast);
    shooterMotor1.getConfigurator().apply(shooter1Config);

    var shooter2Config = new TalonFXConfiguration();
    shooter2Config.withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(80));

    shooterMotor2.setNeutralMode(NeutralModeValue.Coast);
    shooterMotor2.setControl(new Follower(shooterMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
    shooterMotor2.getConfigurator().apply(shooter2Config);

    SparkMaxConfig hoodConfig =  new SparkMaxConfig();
    hoodConfig.inverted(false);
    hoodConfig.smartCurrentLimit(40);

    AbsoluteEncoderConfig hoodencoderConfig = new AbsoluteEncoderConfig();
    hoodencoderConfig.positionConversionFactor(180);
    hoodencoderConfig.zeroCentered(true);
    //hoodencoderConfig.zeroOffset(0.111133136);
    hoodConfig.apply(hoodencoderConfig);
    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    var FeederConfig = new TalonFXConfiguration();
    
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
  public void reverseFeeder (){
    feederMotor.set(0.1);
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
    updateShotParamsFromCalculations();
    ControlShooter();
    // ControlHood();
    // TuneHoodPID();
      hoodOutput = hoodPID.calculate(getHoodAngle(),m_hoodAngle);
      hoodMotor.set(hoodOutput);
     DogLog.log("Shooter/Shooter1Current",shooterMotor1.getStatorCurrent().getValue());
     DogLog.log("Shooter/Shooter2Current",shooterMotor2.getStatorCurrent().getValue());
     //DogLog.log("Shooter/Shooter2Speed",enc_Shooter2.getVelocity(),"rpm");
    // DogLog.log("Shooter/RollerSpeed",enc_Roller.getVelocity(),"rpm");
    // DogLog.log("Shooter/FeederSpeed",enc_Feeder.getVelocity(),"rpm");


   

  }

  private void updateLogs(){

    DogLog.log("Shooter/Shooter1Amps", shooterMotor1.getStatorCurrent().getValueAsDouble());
    DogLog.log("Shooter/Shooter2Amps", shooterMotor2.getStatorCurrent().getValueAsDouble());
    DogLog.log("Shooter/FeederAmps", feederMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Shooter/RollerAmps", rollerMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Shooter/Enabled",shooterEnabled);
    DogLog.log("Fieldinfo/reqRobotAngle",DHOut_reqRobotAngle);
    DogLog.log("Shooter/SavedShooterSpeedRPM", m_shooterSpeed);
    DogLog.log("Shooter/ActualShooterSpeedRPM", shooterMotor1.getVelocity().getValueAsDouble() * 60.0);
    DogLog.log("Shooter/HoodAngle",getHoodAngle());
    DogLog.log("Shooter/HoodSetpoint",hoodPID.getSetpoint());
    DogLog.log("Shooter/HoodAtSetpoint",hoodPID.atSetpoint());
    DogLog.log("Shooter/FeederSpeed",feederMotor.getVelocity().getValueAsDouble()*60);
    DogLog.log("Shooter/SOTF", DHOut_SOTF);
  }

  public void ToggleSOTF(boolean b){
    DHOut_SOTF = b;
  }

  private void updateShotParamsFromCalculations(){

    if (DHIn_ShooterLookupTable != null){
      if (DHOut_SOTF){
        ShootOnTheFlyCalculation();
      }
      else{
        ShooterParams shooterCal = DHIn_ShooterLookupTable.shooterTable.get(DHIn_ShotDistance);
        m_shooterSpeed = shooterCal.rpm();
        m_hoodAngle = shooterCal.hoodAngle();
        
        

        DogLog.log("Shooter/calculatedShooterSpeed",shooterCal.rpm());
        DogLog.log("Shooter/calculatedHoodAngle",shooterCal.hoodAngle());
        DogLog.log("Shooter/hoodOutput",hoodOutput);
      }
    }
  }



  public void ShootOnTheFlyCalculation(){
    double latencyCompensation = 0.2;
    Translation2d robotPosition = new Translation2d(DHIn_RobotPose2d.getTranslation().getX(),DHIn_RobotPose2d.getY());
    Translation2d robotVelocity = new Translation2d(DHIn_FieldVelocity.vxMetersPerSecond, DHIn_FieldVelocity.vyMetersPerSecond);
    Translation2d goalPose = new Translation2d(DHIN_HubPose.getX(),DHIN_HubPose.getY());
    // 1. Project future position
            Translation2d futurePos = robotPosition.plus(
                robotVelocity.times(latencyCompensation)
            );

            // 2. Get target vector
            Translation2d toGoal = goalPose.minus(futurePos);
            double distance = toGoal.getNorm();
            Translation2d targetDirection = toGoal.div(distance);

            // 3. Look up baseline velocity from table
            ShooterParams baseline = DHIn_ShooterLookupTable.shooterTable.get(distance);
            double baselineVelocity = distance / baseline.timeOfFlight();

            // 4. Build target velocity vector
            Translation2d targetVelocity = targetDirection.times(baselineVelocity);

            // 5. THE MAGIC: subtract robot velocity
            Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

            // 6. Extract results
            DHOut_SOTFTargetAngle = shotVelocity.getAngle();
            double requiredVelocity = shotVelocity.getNorm();

            // 7. Use table in reverse: velocity → effective distance → RPM
            double effectiveDistance = DHIn_ShooterLookupTable.inverseShooterTable.get(requiredVelocity);
            m_shooterSpeed = DHIn_ShooterLookupTable.shooterTable.get(effectiveDistance).rpm();
            m_hoodAngle = DHIn_ShooterLookupTable.shooterTable.get(effectiveDistance).hoodAngle();

  }

  public boolean isShooterAtSpeed(){
    return Math.abs((shooterMotor1.getVelocity().getValueAsDouble() * 60) - m_shooterSpeed) <= shooterTolerance;
  }

  private void ControlShooter(){
    if (shooterEnabled)
    shooterMotor1.setControl(m_shooterMotorVelocityRequest.withVelocity(m_shooterSpeed / 60.0));
  else
    shooterMotor1.setControl(m_shooterMotorVelocityRequest.withVelocity(shooterCoastSpeed / 60.0));
  }

  public void ChangeShooterSpeed(double delta){
        SetShooterSpeed(m_shooterSpeed + delta);
  }

  public void ChangeHoodAngle(double delta){
        SetHoodAngle(m_hoodAngle + delta);
  }

  public void SetHoodAngle (double angle){
    m_hoodAngle = angle; 
  }

    private void TuneHoodPID(){
    // hoodPID.setP(HoodKpTunable.getAsDouble());
    // hoodPID.setD(HoodKdTunable.getAsDouble());
  }

  public double getHoodAngle(){
    return hoodEncoder.getPosition();
  }

}
