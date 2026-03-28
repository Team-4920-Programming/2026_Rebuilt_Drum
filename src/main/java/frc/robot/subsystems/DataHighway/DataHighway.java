// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DataHighway;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import dev.doglog.DogLog;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADXL345_I2C.AllAxes;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
    NEUTRAL,
    OPPONENTSCORINGZONE,
    UNKNOWN
  }

  public final AprilTagFieldLayout fieldLayout = AprilTagFields.k2026RebuiltWelded.loadAprilTagLayoutField();

  public enum TargetPoses {
    BLUEALLIANCEHUB(Pose2d.kZero),
    BLUEALLIANCEPASSDRIVERLEFT(Pose2d.kZero),
    BLUEALLIANCEPASSDRIVERRIGHT(Pose2d.kZero),
    REDALLIANCEHUB(Pose2d.kZero),
    REDALLIANCEPASSDRIVERLEFT(Pose2d.kZero),
    REDALLIANCEPASSDRIVERRIGHT(Pose2d.kZero);
  
    private Pose2d targetPose;

    // Constructor
    TargetPoses(Pose2d pose) {
        this.targetPose = pose;
    }
    // Getter
    public Pose2d getValue() {
        return this.targetPose;
    }
    public void setTargetPose(Pose2d pose) {
        this.targetPose = pose;
    }
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
  ZONE currentZone = ZONE.UNKNOWN;
  boolean gameDataUpdated = false;
  String gameData;
  boolean targetSetup = false;
  boolean validTargetSetup = false;

  public Timer DH_matchTimer = new Timer();

  //Subsystems
  ShooterSubsystem SS_Shooter;
  SwerveSubsystem SS_Swerve;
  IntakeSubsystem SS_Intake;
  ClimberSubsystem SS_Climber;

  //DH Data from Subsystems
  double DH_ShotDistance = 0;
  double DH_PassingDistance = 0;
  boolean DH_AutoAim = false;
  double DH_AngleToHub = 0;
  boolean DH_Aimed = false;
  boolean DH_InNeutralZone = false;
  boolean DH_InAllianceZone = false;
  boolean DH_InOpposingZone = false;
  ChassisSpeeds DH_FieldVelocity;
  double DH_HubPoseX =0;
  double DH_HubPoseY =0;
  double DH_reqRobotAngle =0;
  double DH_nextPhaseCountDown = 0;
  boolean DH_isHubActive = true;
  double DH_AngleToOutpost = 0;
  double DH_AngleToDepot = 0;
  double DH_CornerDistance = 0;
  boolean DH_SOTF = false;
  Rotation2d DH_SOTFTargetAngle = Rotation2d.kZero;
  double DH_matchTime = 0;
  boolean DH_safeToShoot = false;
  Pose2d DH_robotPose = new Pose2d().kZero;
  List<Pose2d> passTargetList = new ArrayList<>();
  Pose2d DH_HubPose = new Pose2d().kZero;
  ShooterLookupTable DH_ShooterLookupTable = new ShooterLookupTable();
  private Pose2d DH_passingTargetPose = new Pose2d().kZero;
  private Pose2d aimTarget = new Pose2d().kZero;
  CommandJoystick OperatorJoystick = new CommandJoystick(1);
  /** Creates a new DataHighway. */
    public DataHighway(SwerveSubsystem SwerveSS, ShooterSubsystem ShooterSS, ClimberSubsystem ClimberSS, IntakeSubsystem IntakeSS) {
      SS_Shooter = ShooterSS;
      SS_Swerve = SwerveSS;
      SS_Intake = IntakeSS;
      SS_Climber = ClimberSS;

    }

  @Override
  public void periodic() {


    GetDHData();
    DogLogData();
    SetDHData();
    updateTargetData();
    updateValidTargetData();
    updateMatchTime();
    updateGameData();
    processMatchPhase();
    updateActiveHub();
    updateCurrentZone();
    calculateAutoAim();
    calculateShotDistance();
    calculatePassingDistance();
    updateLogs();
    manualAssignShift();
    DH_safeToShoot = CalculateSafeToShoot();
    // This method will be called once per scheduler run

if (DH_InAllianceZone){
  currentZone = ZONE.ALLIANCESCORINGZONE;
}

else if (DH_InNeutralZone){
  currentZone = ZONE.NEUTRAL;
}
else if (DH_InOpposingZone){
  currentZone = ZONE.OPPONENTSCORINGZONE;
}



  }

  private void updateMatchTime(){
    if (DH_matchTimer.isRunning()){
      DH_matchTime = 160.0 - DH_matchTimer.get();
    }
  }

  private void calculateShotDistance(){
    DH_ShotDistance = DH_HubPose.getTranslation().getDistance(DH_robotPose.getTranslation());

  }
  private void calculatePassingDistance(){
    DH_PassingDistance = DH_robotPose.nearest(passTargetList).getTranslation().getDistance(DH_robotPose.getTranslation());

  }

  private void updateLogs(){
    DogLog.log("Data/Targets/BLUEALLIANCEHUB", TargetPoses.BLUEALLIANCEHUB.getValue());
    DogLog.log("Data/Targets/REDALLIANCEHUB", TargetPoses.REDALLIANCEHUB.getValue());

    DogLog.log("Data/Targets/BLUEALLIANCEPASSDRIVERLEFT", TargetPoses.BLUEALLIANCEPASSDRIVERLEFT.getValue());
    DogLog.log("Data/Targets/BLUEALLIANCEPASSDRIVERRIGHT", TargetPoses.BLUEALLIANCEPASSDRIVERRIGHT.getValue());

    DogLog.log("Data/Targets/REDALLIANCEPASSDRIVERLEFT", TargetPoses.REDALLIANCEPASSDRIVERLEFT.getValue());
    DogLog.log("Data/Targets/REDALLIANCEPASSDRIVERRIGHT", TargetPoses.REDALLIANCEPASSDRIVERRIGHT.getValue());

    DogLog.log("Data/Targets/HUBPOSE", DH_HubPose);

    for (int i = 0 ; i < passTargetList.size(); i++){
      String id = String.format("Data/Targets/PassTarget_%d",i);
      DogLog.log(id, passTargetList.get(i));
    }

    DogLog.log("Data/ShotData/DistancefromHub", DH_ShotDistance);
    DogLog.log("Data/ShotData/PassingDistance", DH_PassingDistance);

    DogLog.log("Data/ShotData/RobotIsAimed", DH_Aimed);

    DogLog.log("Data/MatchTimerReading", DH_matchTimer.get());
    DogLog.log("Data/MatchTimerStarted", DH_matchTimer.isRunning());

    DogLog.log("Data/MatchPhase", currentMatchPhase);
    DogLog.log("Data/AllianceColor", allianceColor);
    DogLog.log("Data/AssignedShift", assignedShift);
    DogLog.log("Data/CurrentZone", currentZone);


  }

  private void updateTargetData(){

    if (targetSetup == false){
      TargetPoses.BLUEALLIANCEHUB.setTargetPose(new Pose2d(4.6316, 4.035, Rotation2d.k180deg));
      TargetPoses.REDALLIANCEHUB.setTargetPose(new Pose2d(11.9094, 4.035, Rotation2d.kZero));

      
      TargetPoses.BLUEALLIANCEPASSDRIVERLEFT.setTargetPose(new Pose2d(2.4124, 6.1686, Rotation2d.k180deg));
      TargetPoses.BLUEALLIANCEPASSDRIVERRIGHT.setTargetPose(new Pose2d(2.4124, 1.9014, Rotation2d.k180deg));
      
      TargetPoses.REDALLIANCEPASSDRIVERLEFT.setTargetPose(new Pose2d(14.1276, 1.9014, Rotation2d.kZero));
      TargetPoses.REDALLIANCEPASSDRIVERRIGHT.setTargetPose(new Pose2d(14.1276, 6.1686, Rotation2d.kZero));
      targetSetup = true;
  
    }
  }


  private void updateValidTargetData(){

    if (targetSetup && !validTargetSetup && allianceColor != AllianceColor.UNKNOWN){
      if (allianceColor == AllianceColor.BLUE){
        passTargetList.add(TargetPoses.BLUEALLIANCEPASSDRIVERLEFT.getValue());
        passTargetList.add(TargetPoses.BLUEALLIANCEPASSDRIVERRIGHT.getValue());
        DH_HubPose = TargetPoses.BLUEALLIANCEHUB.getValue();
      }
      else{
        passTargetList.add(TargetPoses.REDALLIANCEPASSDRIVERLEFT.getValue());
        passTargetList.add(TargetPoses.REDALLIANCEPASSDRIVERRIGHT.getValue());
        DH_HubPose = TargetPoses.REDALLIANCEHUB.getValue();
      }
      validTargetSetup = true;
    }
    DH_passingTargetPose = DH_robotPose.nearest(passTargetList);
  }

  private void calculateAutoAim(){
    if (DH_InAllianceZone){
      aimTarget = DH_HubPose;
    }
    else{
      aimTarget = DH_passingTargetPose;
    }
  }
  public void updateMatchPhase(MatchPhase mp){
    currentMatchPhase = mp;
  }

  private void processMatchPhase(){
    if (DH_matchTime > ShiftSchedule.AUTO_END.getValue()){
      currentMatchPhase = MatchPhase.AUTO;
      DH_nextPhaseCountDown = DH_matchTime - ShiftSchedule.AUTO_END.getValue();
    }
    else if (DH_matchTime > ShiftSchedule.TRANSITION_END.getValue()){
      currentMatchPhase = MatchPhase.TRANSITION;
      DH_nextPhaseCountDown = DH_matchTime - ShiftSchedule.TRANSITION_END.getValue();
    }
    else if (DH_matchTime > ShiftSchedule.SHIFTA_1_END.getValue()){
      currentMatchPhase =MatchPhase.SHIFT_A;
      DH_nextPhaseCountDown = DH_matchTime - ShiftSchedule.SHIFTA_1_END.getValue();
    }
    else if (DH_matchTime > ShiftSchedule.SHIFTB_1_END.getValue()){
      currentMatchPhase =MatchPhase.SHIFT_B;
      DH_nextPhaseCountDown = DH_matchTime - ShiftSchedule.SHIFTB_1_END.getValue();
    }
    else if (DH_matchTime > ShiftSchedule.SHIFTA_2_END.getValue()){
      currentMatchPhase =MatchPhase.SHIFT_A;
      DH_nextPhaseCountDown = DH_matchTime - ShiftSchedule.SHIFTA_2_END.getValue();
    }
    else if (DH_matchTime > ShiftSchedule.SHIFTB_2_END.getValue()){
      currentMatchPhase =MatchPhase.SHIFT_B;
      DH_nextPhaseCountDown = DH_matchTime - ShiftSchedule.SHIFTB_2_END.getValue();
    }
    else if (DH_matchTime > ShiftSchedule.END.getValue()){
      currentMatchPhase =MatchPhase.ENDGAME;
      DH_nextPhaseCountDown = DH_matchTime - ShiftSchedule.END.getValue();
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
            DogLog.log("Data/GameDataReceived",true);
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
            DogLog.log("Data/GameDataReceived",true);
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
            DogLog.log("Data/GameDataReceived",false);
            break;
        }
      } else {
        DogLog.log("Data/GameDataStatus", "Game Data Not Received");
        DogLog.log("Data/GameDataReceived",false);
      }
    }
  }

  private void manualAssignShift(){
    if (!gameDataUpdated && DriverStation.isTeleopEnabled()){
      if (OperatorJoystick.button(1).getAsBoolean()){
        assignedShift = AssignedShift.B;
        gameDataUpdated = true;
      }
      if (OperatorJoystick.button(4).getAsBoolean()){
        assignedShift= AssignedShift.A;
        gameDataUpdated = true;
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
    DogLog.log("Data/hubActive",DH_isHubActive);
    DogLog.log("Data/AssignedShift", assignedShift.toString());
    DogLog.log("Data/matchTimeRemaining", DH_matchTime);
    DogLog.log("Data/NextPhaseCountdown", DH_nextPhaseCountDown);

    
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
    DH_InOpposingZone = SS_Swerve.DHOut_InOpposingZone;
    DH_SOTF = SS_Shooter.DHOut_SOTF;
    DH_SOTFTargetAngle = SS_Shooter.DHOut_SOTFTargetAngle;
    
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
    SS_Swerve.DHIn_ShotDistance = DH_ShotDistance;
    SS_Swerve.DHIn_aimTarget = aimTarget;
    SS_Shooter.DHIn_RobotPose2d = DH_robotPose;
    SS_Shooter.DHIN_HubPose = DH_HubPose;
    SS_Swerve.DHIn_SOTFTargetAngle = DH_SOTFTargetAngle;
    SS_Swerve.DHIn_SOTF = DH_SOTF;
  }

  
}
