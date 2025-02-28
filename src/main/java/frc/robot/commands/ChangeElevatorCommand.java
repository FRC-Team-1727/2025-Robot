package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Mode;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ChangeElevatorCommand extends Command {
    private IntakeSubsystem m_IntakeSubsystem;
    private ElevatorSubsystem m_ElevatorSubsystem;

    public ChangeElevatorCommand(IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem) {
        m_IntakeSubsystem = intakeSubsystem;
        m_ElevatorSubsystem = elevatorSubsystem;
    }

    public void initialize() {
        // if(RobotContainer.getMode() == Mode.CORALMODE){
        // switch (m_ElevatorSubsystem.getCoralLevel()) {
        // case 1:
        // m_ElevatorSubsystem.setCoralL1();
        // break;
        // case 2:
        // m_ElevatorSubsystem.setCoralL2();
        // break;
        // case 3:
        // m_ElevatorSubsystem.setCoralL3();
        // break;
        // default:
        // m_ElevatorSubsystem.setDefaultHeight();
        // break;
        // }}
        if(RobotContainer.getMode() == Mode.CORALMODE){
            m_ElevatorSubsystem.incrementCoralLevel();
        }else{
            m_ElevatorSubsystem.incrementAlgaeLevel();
        }
        // }else if(RobotContainer.getMode() == Mode.ALGAEMODE){
        // switch (m_ElevatorSubsystem.getAlgaeLevel()) {
        // case 1:
        // m_ElevatorSubsystem.setAlgaeBottom();
        // break;
        // case 2:
        // m_ElevatorSubsystem.setAlgaeTop();
        // break;
        // default:
        // m_ElevatorSubsystem.setDefaultHeight();
        // break;
        // }
        // m_ElevatorSubsystem.incrementCoralLevel();
        // }else{
        // m_ElevatorSubsystem.setDefaultHeight();
        // }
    }

    @Override
    public void execute() {
        if (RobotContainer.getMode() == Mode.CORALMODE) {
            switch (m_ElevatorSubsystem.getCoralLevel()) {
                case 1:
                 //   m_IntakeSubsystem.setPivot(0);
                    m_ElevatorSubsystem.moveZeroPosition();
                    break;
                case 2:
                    m_IntakeSubsystem.setPivot(IntakeConstants.kScoringAngle);
                    m_ElevatorSubsystem.setCoralL1();
                    m_IntakeSubsystem.setPivot(IntakeConstants.kScoringAngle);
                    break;
                case 3:
                    m_IntakeSubsystem.setPivot(IntakeConstants.kScoringAngle);
                    m_ElevatorSubsystem.setCoralL2();
                    break;
                case 4:
                    m_IntakeSubsystem.setPivot(IntakeConstants.kL3ScoringAngle);
                    m_ElevatorSubsystem.setCoralL3();
                    break;
                default:
                    m_ElevatorSubsystem.setDefaultHeight();
                    break;
            }
        } else if (RobotContainer.getMode() == Mode.ALGAEMODE) {
            switch (m_ElevatorSubsystem.getAlgaeLevel()) {
                case 1:
                m_ElevatorSubsystem.moveZeroPosition();
                    break;
                case 2:
              
                 m_ElevatorSubsystem.setAlgaeBottom();                    
                    break;
                case 3:
                m_IntakeSubsystem.setPivot(-30);
                m_ElevatorSubsystem.setAlgaeTop();
                    break;
                default:
                    m_ElevatorSubsystem.setDefaultHeight();
                    break;
            }
        } else {
            m_ElevatorSubsystem.setDefaultHeight();
        }

    }

    @Override
    public void end(boolean interrupted) {
     //   m_IntakeSubsystem.setPivot(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}