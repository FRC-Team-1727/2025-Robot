// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.constants.OtherConstants.ClimbConstants;
import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbResetCommand extends Command {
  private final ClimbSubsystem m_climbSubsystem;

  public ClimbResetCommand(ClimbSubsystem climbSubsystem) {
    m_climbSubsystem = climbSubsystem;
    addRequirements(m_climbSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
      m_climbSubsystem.setClimb(0);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climbSubsystem.setBrakeMode();
   // System.out.println("ended RAAERE");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_climbSubsystem.atPosition();
  }
}
