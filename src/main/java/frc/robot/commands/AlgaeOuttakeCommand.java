// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeOuttakeCommand extends Command {

  private final IntakeSubsystem m_IntakeSubsystem;

  public AlgaeOuttakeCommand(IntakeSubsystem intakeSubsystem) {
    m_IntakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSubsystem.intakeCoastMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.setSpeed(IntakeConstants.kOutTakeSpeed);
    // m_IntakeSubsystem.setPivot(IntakeConstants.kPivotAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_IntakeSubsystem.setSpeed(IntakeConstants.kPassiveIntakeSpeed);
    m_IntakeSubsystem.intakeBrakeMode();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
