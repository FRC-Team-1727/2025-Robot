// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class AlgaeIntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final IntakeSubsystem m_IntakeSubsystem;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AlgaeIntakeCommand(IntakeSubsystem intakeSubsystem) {
    m_IntakeSubsystem = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSubsystem.intakeCoastMode();
    if(ElevatorSubsystem.curAlgaeLevel > 1) {
      m_IntakeSubsystem.setPivot(IntakeConstants.kAlgaeHighIntakeAngle);
    } else {
      m_IntakeSubsystem.setPivot(IntakeConstants.kAlgaeGroundIntakeAngle);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_IntakeSubsystem.setPivot(IntakeConstants.kPivotAngle);
    
    m_IntakeSubsystem.setSpeed(IntakeConstants.kAlgaeIntakeSpeed);
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
