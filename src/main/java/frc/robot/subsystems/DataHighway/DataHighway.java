// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DataHighway;

import java.util.Optional;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.*;
import frc.robot.subsystems.swervedrive.*;
import frc.robot.subsystems.Climber.*;
import frc.robot.subsystems.Intake.*;

public class DataHighway extends SubsystemBase {
  String AllianceColor = "None";

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
  
  double DH_AngleToOutpost = 0;
  double DH_AngleToDepot = 0;
  double DH_CornerDistance = 0;

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
    // This method will be called once per scheduler run
  }

  private void DogLogData()
  {
  DogLog.forceNt.log("ForcedNT/Match/MatchTime",DriverStation.getMatchTime(),"sec");
    DogLog.forceNt.log("ForcedNT/Match/GameDateMsg",DriverStation.getGameSpecificMessage());
    
  Optional<Alliance> ally = DriverStation.getAlliance();
  if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
          AllianceColor = "Red";
      }
      if (ally.get() == Alliance.Blue) {
          AllianceColor = "Blue";
      }
  }
  else {
      AllianceColor = "None";
  }

    DogLog.forceNt.log("ForcedNT/Match/Allicance",AllianceColor);
  }
  private void GetDHData(){
DH_ShotDistance = SS_Swerve.DHOut_HubDistance;
DH_AutoAim = SS_Swerve.isAutoAim();
DH_AngleToHub = SS_Swerve.DHOut_AngleToHub;
DH_Aimed = SS_Swerve.DHOut_Aimed;
DH_InAllianceZone = SS_Swerve.DHOut_InAllianceZone;
DH_InNeutralZone = SS_Swerve.DHOut_InNeutralZone;
DH_AngleToOutpost = SS_Swerve.DHOut_AngleToOutpost;
DH_AngleToDepot = SS_Swerve.DHOut_AngleToDepot;
DH_CornerDistance = SS_Swerve.DHOut_CornerDistance;

  }
  private void SetDHData(){
SS_Shooter.DHIn_ShotDistance = DH_ShotDistance;
SS_Shooter.DHIn_AutoShoot = DH_AutoAim;
SS_Shooter.AngleToHub = DH_AngleToHub;
SS_Shooter.DHIn_Aimed = DH_Aimed;
SS_Shooter.DHIn_InAllianceZone = DH_InAllianceZone;
SS_Shooter.DHIn_InNeutralZone = DH_InNeutralZone;
SS_Shooter.DHIn_AngleToOutpost = DH_AngleToOutpost;
SS_Shooter.DHIn_AngleToDepot = DH_AngleToDepot;
SS_Shooter.DHIn_CornerDistance = DH_CornerDistance;

  }
}
