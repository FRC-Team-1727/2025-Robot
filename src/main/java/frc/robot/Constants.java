// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

  }
  public static class IntakeConstants{
    public static final int kPivotID = 1;
    public static final int kIntakeID = 2;
    public static final double kPivotP = 0.1;
    public static final double kPivotI = 0;
    public static final double kPivotD = 0;
    public static final double kIntakeP = 0.1;
    public static final double kIntakeI = 0;
    public static final double kIntakeD = 0;
    public static final double kPivotAngle = 0.5;
    public static final double kIntakeSpeed = 0.5;
    public static final double kOutTakeSpeed = 0.5;
  }

  public static class ElevatorConstants{
    public static final int kElevatorID = 3;
    public static final double kElevatorP = 0.1;
    public static final double kElevatorI = 0;
    public static final double kElevatorD = 0;
    public static final double kElevatorSpeed = 0;
    
    public static final double kDefaultHeight = 1;

    //each of the coral levels on the reef
    public static final double kCoral1Height = 1;
    public static final double kCoralL2Height = 2;
    public static final double kCoralL3Height = 3;

    //each of the algae levels on reef
    public static final double kTopAlgae = 2.5;
    public static final double kBottomAlgae = 1.5;
  }
}
