// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.logging.Logger;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.FieldConstants;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      LimelightHelpers.setPipelineIndex("limelight-left", 0);
      LimelightHelpers.setPipelineIndex("limelight-right", 0);
    } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
      LimelightHelpers.setPipelineIndex("limelight-left", 1);
      LimelightHelpers.setPipelineIndex("limelight-right", 1);
    } else {
      LimelightHelpers.setPipelineIndex("limelight-left", 1);
      LimelightHelpers.setPipelineIndex("limelight-right", 1);
    }
    m_robotContainer = new RobotContainer();
    FieldConstants.configureReefPositions(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue));
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // System.out.println(RobotContainer.getMode());
    SmartDashboard.putString("Mode", RobotContainer.getMode().toString());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
