package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoAlgaeOuttakeCommand extends Command{
    private IntakeSubsystem m_IntakeSubsystem;
    public AutoAlgaeOuttakeCommand(IntakeSubsystem intakeSubsystem){
        m_IntakeSubsystem = intakeSubsystem;
        addRequirements(m_IntakeSubsystem);
    }
    public void initialize(){
        m_IntakeSubsystem.setPivot(-27);
    }
    public void execute(){
        m_IntakeSubsystem.setSpeed(IntakeConstants.kAlgaeLowOutTakeSpeed);
    }
    public void end(){
        m_IntakeSubsystem.setSpeed(IntakeConstants.kPassiveAlgaeIntakeSpeed);
    }
    public boolean isFinished(boolean interrupted){
        return false;
    }
}
