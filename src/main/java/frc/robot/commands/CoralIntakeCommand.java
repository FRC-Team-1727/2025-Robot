// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeSubsystem, m_ElevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSubsystem.intakeCoastMode();
    m_IntakeSubsystem.setPivot(-2.2);
    m_ElevatorSubsystem.setHeight(-53.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.setSpeed(IntakeConstants.kCoralIntakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setSpeed(IntakeConstants.kPassiveIntakeSpeed);
    m_IntakeSubsystem.intakeBrakeMode();
    m_IntakeSubsystem.setPivot(IntakeConstants.kScoringAngle);
    m_ElevatorSubsystem.moveZeroPosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
