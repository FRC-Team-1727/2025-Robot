// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeOuttakeCommand extends Command {

  private final IntakeSubsystem m_IntakeSubsystem;
  private final ElevatorSubsystem m_ElevatorSubsystem;

  public AlgaeOuttakeCommand(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
    m_IntakeSubsystem = intakeSubsystem;
    m_ElevatorSubsystem = elevatorSubsystem;
    addRequirements(m_IntakeSubsystem, m_ElevatorSubsystem);
  }

  @Override
  public void initialize() {
    m_IntakeSubsystem.intakeCoastMode();
    if(m_ElevatorSubsystem.getAlgaeLevel() == 1){
      m_IntakeSubsystem.setPivot(-28);
    }
  }

  @Override
  public void execute() {
    switch (m_ElevatorSubsystem.getAlgaeLevel()) {
      case 1:
        m_IntakeSubsystem.setSpeed(IntakeConstants.kAlgaeLowOutTakeSpeed);
        break;
      case 2:
        m_IntakeSubsystem.setSpeed(IntakeConstants.kAlgaeDescoreSpeed);
        break;
      case 3:
        m_IntakeSubsystem.setSpeed(IntakeConstants.kAlgaeOutTakeSpeed);
        break;
      default:
        System.out.println("it didnt work");
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setSpeed(IntakeConstants.kPassiveIntakeSpeed);
    if(m_ElevatorSubsystem.getAlgaeLevel() == 2 || m_ElevatorSubsystem.getAlgaeLevel() == 3){
      m_ElevatorSubsystem.moveZeroPosition();
      m_ElevatorSubsystem.resetLevels();
    }
    m_IntakeSubsystem.intakeBrakeMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
