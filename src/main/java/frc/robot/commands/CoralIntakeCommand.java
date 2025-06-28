// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.constants.OtherConstants.ElevatorConstants;
import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralIntakeCommand extends Command {

  private final IntakeSubsystem m_IntakeSubsystem;
  private final ElevatorSubsystem m_ElevatorSubsystem;

  public CoralIntakeCommand(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
    m_IntakeSubsystem = intakeSubsystem;
    m_ElevatorSubsystem = elevatorSubsystem;
    addRequirements(m_IntakeSubsystem, m_ElevatorSubsystem);
  }

  @Override
  public void initialize() {
    m_IntakeSubsystem.intakeCoastMode();
    m_IntakeSubsystem.setPivot(IntakeConstants.kCoralIntakeAngle);
    m_ElevatorSubsystem.setHeight(ElevatorConstants.kIntakeHeight);
    // m_IntakeSubsystem.clearThreshold(); // for testing purposes
  }

  @Override
  public void execute() {
    m_IntakeSubsystem.intakeSpeed();
    // m_IntakeSubsystem.autoIntakeSpeed(); //for testing purposes
  }

  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setSpeed(IntakeConstants.kPassiveCoralIntakeSpeed);
    m_IntakeSubsystem.intakeBrakeMode();
    m_IntakeSubsystem.setPivot(IntakeConstants.kScoringAngle);
    m_ElevatorSubsystem.moveZeroPosition();
    m_ElevatorSubsystem.resetLevels();
  }

  @Override
  public boolean isFinished() {
    return false;
    // return m_IntakeSubsystem.getIntakeVelocity() > -42 && m_IntakeSubsystem.getThreshold(); //for testing purposes

  }

}
