// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * Use this constants class to keep it seperated from the generated constants class from pheonix tuner x
 */
public final class OtherConstants {

  public static class IntakeConstants{
    public static final int kPivotID = 11;
    public static final int kIntakeID = 12;
    public static final double kPivotP = 0.1;
    public static final double kPivotI = 0;
    public static final double kPivotD = 0;
    public static final double kIntakeP = 0.2;
    public static final double kIntakeI = 0;
    public static final double kIntakeD = 0;
    public static final double kScoringAngle = -9.879;
    public static final double kDefaultAngle = -4.65;
    public static final double kIntakeSpeed = -.2;
    public static final double kOutTakeSpeed = .2;
    public static final double kPassiveIntakeSpeed = -0.1;
  }

  public static class ElevatorConstants{
    public static final int kElevatorID = 9;
    public static final double kElevatorP = 0.05;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;
    public static final double kElevatorSpeed = 0;
    
    public static final double kDefaultHeight = 0;

    //each of the coral levels on the reef
    public static final double kCoral1Height = -15;
    public static final double kCoralL2Height = -45;
    public static final double kCoralL3Height = -45;
// true value: -79.880859375
    //each of the algae levels on reef
    public static final double kTopAlgae = -35;
    public static final double kBottomAlgae = -25;
    public static double kElevatorMinimumLength;
  }

  public static class ClimbConstants{
    public static final int kClimbID = 10;
    public static final double kClimbP = 0.05;
    public static final double kClimbI = 0;
    public static final double kClimbD = 0;
    public static final double kClimbSpeed = 0;
    public static final double kClimbAngle = 0;
  }
}
