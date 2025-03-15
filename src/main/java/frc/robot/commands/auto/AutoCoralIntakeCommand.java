package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.OtherConstants.ElevatorConstants;
import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoCoralIntakeCommand extends Command{
    private ElevatorSubsystem m_ElevatorSubsystem;
    private IntakeSubsystem m_IntakeSubsystem;
    public AutoCoralIntakeCommand(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem){
        m_ElevatorSubsystem = elevatorSubsystem;
        m_IntakeSubsystem = intakeSubsystem;
        addRequirements(m_ElevatorSubsystem, m_IntakeSubsystem);
    }
    public void initialize(){
        m_ElevatorSubsystem.setHeight(ElevatorConstants.kIntakeHeight);
        m_IntakeSubsystem.setPivot(IntakeConstants.kCoralIntakeAngle);
    }
    public void execute(){
        m_IntakeSubsystem.setSpeed(IntakeConstants.kCoralIntakeSpeed);
    }
    public void end(){
        m_IntakeSubsystem.setSpeed(IntakeConstants.kPassiveIntakeSpeed);
    }
    public boolean isFinished(){
        return false;
    }
    
}