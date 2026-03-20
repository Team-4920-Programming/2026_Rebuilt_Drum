// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DataHighway;

import java.util.Optional;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.*;
import frc.robot.subsystems.swervedrive.*;
import frc.robot.subsystems.Climber.*;
import frc.robot.subsystems.Intake.*;



public class DataHighway extends SubsystemBase {

  public enum AllianceColor{
    RED,
    BLUE,
    UNKNOWN
  }

  public enum AssignedShift{
    A,
    B,
    UNKNOWN
  }

  public enum MatchPhase{
    AUTO,
    TRANSITION,
    SHIFT_A,
    SHIFT_B,
    ENDGAME,
    UNKNOWN
  }

  public enum ZONE {
    ALLIANCESCORINGZONE,
    ALLIANCENEUTRAL,
    OPPONENTNEUTRAL,
    OPPONENTSCORINGZONE
  }

  private static enum ShiftSchedule{
    MATCH_START(160.0),
    AUTO_END(140.0),
    TRANSITION_END(130.0),
    SHIFTA_1_END(105.0),
    SHIFTB_1_END(80.0),
    SHIFTA_2_END(55.0),
    SHIFTB_2_END(30.0),
    END(0.0);


    private final double value;

    // Constructor
    ShiftSchedule(double value) {
        this.value = value;
    }

    // Getter
    public double getValue() {
        return value;
    }


  }




  //Initialization
  AllianceColor allianceColor = AllianceColor.UNKNOWN;
  AssignedShift assignedShift = AssignedShift.UNKNOWN;
  MatchPhase currentMatchPhase = MatchPhase.UNKNOWN;
  boolean gameDataUpdated = false;
  String gameData;


  //Subsystems
  ShooterSubsystem SS_Shooter;
  SwerveSubsystem SS_Swerve;
  IntakeSubsystem SS_Intake;
  ClimberSubsystem SS_Climber;

  //DH Data from Subsystems
  double DH_ShotDistance = 0;
  boolean DH_AutoAim = false;
  double DH_AngleToHub = 0;
  boolean DH_Aimed = false;
  boolean DH_InNeutralZone = false;
  boolean DH_InAllianceZone = false;
  ChassisSpeeds DH_FieldVelocity;
  double DH_HubPoseX =0;
  double DH_HubPoseY =0;
  double DH_reqRobotAngle =0;
  boolean DH_isHubActive = true;
  double DH_AngleToOutpost = 0;
  double DH_AngleToDepot = 0;
  double DH_CornerDistance = 0;

  double DH_matchTime = 0;
  boolean DH_safeToShoot = false;
  Pose2d DH_robotPose = new Pose2d().kZero;
  ShooterLookupTable DH_ShooterLookupTable = new ShooterLookupTable();

  /** Creates a new DataHighway. */
    public DataHighway(SwerveSubsystem SwerveSS, ShooterSubsystem ShooterSS, ClimberSubsystem ClimberSS, IntakeSubsystem IntakeSS) {
      SS_Shooter = ShooterSS;
      SS_Swerve = SwerveSS;
      SS_Intake = IntakeSS;
      SS_Climber = ClimberSS;

    }

  @Override
  public void periodic() {

    updateMatchTime();
    updateGameData();
    updateMatchPhase();
    updateActiveHub();
    updateCurrentZone();
    
    DH_safeToShoot = CalculateSafeToShoot();

    GetDHData();
    DogLogData();
    SetDHData();
    // This method will be called once per scheduler run
  }

  private void updateMatchTime(){
    if (DriverStation.isFMSAttached())
    {
      DH_matchTime = DriverStation.getMatchTime();
    }
    else{
      DH_matchTime = 160.0 - DriverStation.getMatchTime();
    }
  }

  private void updateMatchPhase(){
    if (DH_matchTime > ShiftSchedule.AUTO_END.getValue()){
      currentMatchPhase = MatchPhase.AUTO;
    }
    else if (DH_matchTime > ShiftSchedule.TRANSITION_END.getValue()){
      currentMatchPhase = MatchPhase.TRANSITION;
    }
    else if (DH_matchTime > ShiftSchedule.SHIFTA_1_END.getValue()){
      currentMatchPhase =MatchPhase.SHIFT_A;
    }
    else if (DH_matchTime > ShiftSchedule.SHIFTB_1_END.getValue()){
      currentMatchPhase =MatchPhase.SHIFT_B;
    }
    else if (DH_matchTime > ShiftSchedule.SHIFTA_2_END.getValue()){
      currentMatchPhase =MatchPhase.SHIFT_A;
    }
    else if (DH_matchTime > ShiftSchedule.SHIFTB_2_END.getValue()){
      currentMatchPhase =MatchPhase.SHIFT_B;
    }
    else if (DH_matchTime > ShiftSchedule.END.getValue()){
      currentMatchPhase =MatchPhase.ENDGAME;
    }
  }

  private void updateCurrentZone(){

  }

  private void updateGameData(){
    if (!gameDataUpdated){
      gameData = DriverStation.getGameSpecificMessage();
      if(gameData.length() > 0)
      {
        switch (gameData.charAt(0))
        {
          case 'B' :
            DogLog.log("Data/GameDataStatus", "Game Data Received");
            if (allianceColor == AllianceColor.BLUE){
              assignedShift = AssignedShift.B;
            }
            else{
              assignedShift = AssignedShift.A;
            }
            gameDataUpdated = true;
            break;
          case 'R' :
            DogLog.log("Data/GameDataStatus", "Game Data Received");
            if (allianceColor == AllianceColor.RED){
              assignedShift = AssignedShift.B;
            }
            else{
              assignedShift = AssignedShift.A;
            }
            gameDataUpdated = true;
            break;
          default :
            DogLog.log("Data/GameDataStatus", "Game Data Corrupt");
            break;
        }
      } else {
        DogLog.log("Data/GameDataStatus", "Game Data Not Received");
      }
    }
  }

  private void updateActiveHub(){
    if (currentMatchPhase == MatchPhase.AUTO || currentMatchPhase == MatchPhase.ENDGAME || currentMatchPhase == MatchPhase.TRANSITION){
      DH_isHubActive = true;
    }
    else if ((currentMatchPhase == MatchPhase.SHIFT_A && assignedShift == AssignedShift.A) || (currentMatchPhase == MatchPhase.SHIFT_B && assignedShift == AssignedShift.B) ){
      DH_isHubActive = true;
    }
    else{
      DH_isHubActive = false;
    }
  }

  private boolean CalculateSafeToShoot(){
    if (DH_isHubActive == true || CalculateTOFShiftChange())
    {
    return true;
    }
    else return false;
  }

  private boolean CalculateTOFShiftChange(){
    double timeBeforeShiftChange = 200.0;
    if (DH_isHubActive == false){
      if (assignedShift == AssignedShift.A){
        timeBeforeShiftChange = Math.min(Math.abs(DH_matchTime - ShiftSchedule.SHIFTB_1_END.getValue()),Math.abs(DH_matchTime - ShiftSchedule.SHIFTB_2_END.getValue()));
      }
      if (assignedShift == AssignedShift.B){
        timeBeforeShiftChange = Math.min(Math.abs(DH_matchTime - ShiftSchedule.SHIFTA_1_END.getValue()),Math.abs(DH_matchTime - ShiftSchedule.SHIFTA_2_END.getValue()));
      }

      ShooterLookupTable.ShooterParams currentParams =  DH_ShooterLookupTable.shooterTable.get(DH_ShotDistance);
      // if (currentParams.)


    }

    
    return false;
  }

  private void DogLogData()
  {
    DogLog.forceNt.log("ForcedNT/Match/MatchTime",DriverStation.getMatchTime(),"sec");
    DogLog.forceNt.log("ForcedNT/Match/GameDateMsg",DriverStation.getGameSpecificMessage());
    
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
            allianceColor = AllianceColor.RED;
        }
        if (ally.get() == Alliance.Blue) {
            allianceColor = AllianceColor.BLUE;
        }
    }
    else {
        allianceColor = AllianceColor.UNKNOWN;
    }

    DogLog.forceNt.log("ForcedNT/Match/Alliance", allianceColor);
  }
  
  private void GetDHData()
  {
    DH_ShotDistance = SS_Swerve.DHOut_HubDistance;
    DH_AutoAim = SS_Swerve.isAutoAim();
    DH_AngleToHub = SS_Swerve.DHOut_AngleToHub;
    DH_Aimed = SS_Swerve.DHOut_Aimed;
    DH_InAllianceZone = SS_Swerve.DHOut_InAllianceZone;
    DH_InNeutralZone = SS_Swerve.DHOut_InNeutralZone;
    DH_AngleToOutpost = SS_Swerve.DHOut_AngleToOutpost;
    DH_AngleToDepot = SS_Swerve.DHOut_AngleToDepot;
    DH_CornerDistance = SS_Swerve.DHOut_CornerDistance;
    DH_FieldVelocity = SS_Swerve.getFieldVelocity();
    DH_HubPoseX = SS_Swerve.DHOut_HubPoseX;
    DH_HubPoseY = SS_Swerve.DHOut_HubPoseY;
    DH_reqRobotAngle = SS_Shooter.DHOut_reqRobotAngle;
    DH_robotPose = SS_Swerve.DHOUT_RobotPose;
  }
  private void SetDHData()
  {
    SS_Shooter.DHIn_ShotDistance = DH_ShotDistance;
    SS_Shooter.DHIn_AutoShoot = DH_AutoAim;
    SS_Shooter.AngleToHub = DH_AngleToHub;
    SS_Shooter.DHIn_Aimed = DH_Aimed;
    SS_Shooter.DHIn_InAllianceZone = DH_InAllianceZone;
    SS_Shooter.DHIn_InNeutralZone = DH_InNeutralZone;
    SS_Shooter.DHIn_AngleToOutpost = DH_AngleToOutpost;
    SS_Shooter.DHIn_AngleToDepot = DH_AngleToDepot;
    SS_Shooter.DHIn_CornerDistance = DH_CornerDistance;
    SS_Shooter.DHIn_FieldVelocity = DH_FieldVelocity;
    SS_Shooter.DHIn_HubPoseX = DH_HubPoseX;
    SS_Shooter.DHIn_HubPoseY = DH_HubPoseY;
    SS_Swerve.DHIn_reqRobotAngle = DH_reqRobotAngle;
    SS_Shooter.DHIn_ShooterLookupTable = DH_ShooterLookupTable;
    SS_Swerve.DHIn_ShooterLookupTable = DH_ShooterLookupTable;
  }

  
}
