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
        m_IntakeSubsystem.clearThreshold();
    }
    public void execute(){
        m_IntakeSubsystem.autoIntakeSpeed();
    }
    public void end(boolean interrupted){
        m_IntakeSubsystem.setSpeed(IntakeConstants.kPassiveCoralIntakeSpeed);
        System.out.println("ended");
    }
    public boolean isFinished(){
        return m_IntakeSubsystem.getIntakeVelocity() > -42 && m_IntakeSubsystem.getThreshold();
    }
    
}