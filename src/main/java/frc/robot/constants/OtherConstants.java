// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Use this constants class to keep it seperated from the generated constants
 * class from pheonix tuner x
 */
public final class OtherConstants {

  public static class IntakeConstants {
    public static final int kPivotID = 11;
    public static final int kIntakeID = 12;

    public static final double kPivotP = 0.1;
    public static final double kPivotI = 0;
    public static final double kPivotD = 0;
    public static final double kIntakeP = 0.2;
    public static final double kIntakeI = 0;
    public static final double kIntakeD = 0;

    public static final double kL1ScoringAngle = -10.29;
    public static final double kScoringAngle = -15.28;
    public static final double kL3ScoringAngle = -14.857;
    public static final double kAlgaeGroundIntakeAngle = -35;
    public static final double kAlgaeHighIntakeAngle = -27.1289;
    public static final double kAlgaeLowIntakeAngle = -35.29;
    public static final double kCoralIntakeAngle = -5;
    public static final double kCoralIntakeSpeed = -.5;

    public static final double kLowCoralOuttakeSpeed = .08;
    public static final double kCoralOutTakeSpeed = .18;
    public static final double kPassiveCoralIntakeSpeed = -0.08;

    public static final double kAlgaeIntakeSpeed = -.8;
    public static final double kAlgaeDescoreSpeed = -.6;
    public static final double kAlgaeOutTakeSpeed = .85;
    public static final double kAlgaeLowOutTakeSpeed = 0.20;
    public static final double kPassiveAlgaeIntakeSpeed = -0.1;
  }

  public static class ElevatorConstants {
    public static final int kElevatorID = 9;
    public static final double kElevatorP = 0.05;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;
    public static final double kElevatorSpeed = 0;

    public static final double kDefaultHeight = 0;

    // each of the coral levels on the reef
    public static final double kCoral1Height = 0;
    public static final double kCoralL2Height = -20.07;
    public static final double kCoralL3Height = -45.3;
    // true value: -79.880859375
    // each of the algae levels on reef
    public static final double kTopAlgae = -43;
    public static final double kBottomAlgae = -42;

    public static final double kIntakeHeight = -29
    ;
  }

  public static class ClimbConstants {
    public static final int kClimbID = 10;
    public static final double kClimbP = 0.175;
    public static final double kClimbI = 0.00;
    public static final double kClimbD = 0.00;
    public static final double kClimbSpeed = 0;
    public static final double kUpClimbAngle = 240;
    public static final double kDownClimbAngle = 59.5;

  }

  public static class VisionConstants {
    public static final String LIMELIGHT_NAME = "";
  }
}
