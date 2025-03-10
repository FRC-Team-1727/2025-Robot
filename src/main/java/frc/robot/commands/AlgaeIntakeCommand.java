// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeIntakeCommand extends Command {
  private final IntakeSubsystem m_IntakeSubsystem;

  public AlgaeIntakeCommand(IntakeSubsystem intakeSubsystem) {
    m_IntakeSubsystem = intakeSubsystem;
    addRequirements(m_IntakeSubsystem);
  }

  @Override
  public void initialize() {
    m_IntakeSubsystem.intakeCoastMode();
    if(ElevatorSubsystem.curAlgaeLevel == 3) {
      m_IntakeSubsystem.setPivot(IntakeConstants.kAlgaeHighIntakeAngle);
    } else if(ElevatorSubsystem.curAlgaeLevel == 2) {
      m_IntakeSubsystem.setPivot(IntakeConstants.kAlgaeLowIntakeAngle);
    } else {
      m_IntakeSubsystem.setPivot(IntakeConstants.kAlgaeGroundIntakeAngle);

    }
  }

  @Override
  public void execute() {    
    m_IntakeSubsystem.setSpeed(IntakeConstants.kAlgaeIntakeSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setSpeed(IntakeConstants.kPassiveIntakeSpeed);
    m_IntakeSubsystem.intakeBrakeMode();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
