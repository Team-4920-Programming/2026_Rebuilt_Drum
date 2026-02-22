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

  }
  private void SetDHData(){

  }
}
