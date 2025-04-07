// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ExampleSubsystem;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDMode;
import frc.robot.subsystems.LEDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralOuttakeCommand extends Command {

  private final IntakeSubsystem m_IntakeSubsystem;
  private final ElevatorSubsystem m_ElevatorSubsystem;
  private final LEDSubsystem m_LedSubsystem;

  public CoralOuttakeCommand(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem, LEDSubsystem ledSubsystem) {
    m_IntakeSubsystem = intakeSubsystem;
    m_ElevatorSubsystem = elevatorSubsystem;
    m_LedSubsystem = ledSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeSubsystem, m_ElevatorSubsystem, m_LedSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeSubsystem.intakeCoastMode();
    m_LedSubsystem.setMode(LEDMode.kDefault);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if (m_ElevatorSubsystem.getCoralLevel() > 3) {
    //   m_IntakeSubsystem.setPivot(IntakeConstants.kL3ScoringAngle);
    //   if (m_IntakeSubsystem.getPivot().getPosition().getValueAsDouble() > -4) {
    //     m_IntakeSubsystem.setSpeed(IntakeConstants.kCoralOutTakeSpeed);
    //   }
    // } else {
      if(m_ElevatorSubsystem.getCoralLevel() == 2){
        m_IntakeSubsystem.setSpeed(IntakeConstants.kLowCoralOuttakeSpeed);
      }else{
        m_IntakeSubsystem.setSpeed(IntakeConstants.kCoralOutTakeSpeed);

      }
    //}

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_IntakeSubsystem.setSpeed(0);
    m_IntakeSubsystem.intakeBrakeMode();

    if(m_ElevatorSubsystem.getCoralLevel() == 4){
        m_IntakeSubsystem.setPivot(-23);

    }
    m_ElevatorSubsystem.moveZeroPosition();
    if(ElevatorSubsystem.getElevatorHeight() > -40) {
      m_IntakeSubsystem.setPivot(0);
    }
    m_ElevatorSubsystem.resetLevels();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
