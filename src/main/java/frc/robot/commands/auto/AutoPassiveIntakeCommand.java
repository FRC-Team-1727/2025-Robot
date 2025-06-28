package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoPassiveIntakeCommand extends Command{
    private IntakeSubsystem m_IntakeSubsystem;
    public AutoPassiveIntakeCommand(IntakeSubsystem intakeSubsystem){
        m_IntakeSubsystem = intakeSubsystem;
        addRequirements(m_IntakeSubsystem);
    }
    public void initialize(){
        m_IntakeSubsystem.setSpeed(IntakeConstants.kPassiveCoralIntakeSpeed);
    }
    public void execute(){
        m_IntakeSubsystem.setSpeed(IntakeConstants.kPassiveCoralIntakeSpeed);
    }
    public void end(){
        m_IntakeSubsystem.setSpeed(IntakeConstants.kPassiveCoralIntakeSpeed);
    }
    public boolean isFinished(){
        return false;
    }
    
}
