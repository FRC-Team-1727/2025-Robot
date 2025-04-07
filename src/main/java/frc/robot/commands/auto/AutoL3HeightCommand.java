package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.OtherConstants.ElevatorConstants;
import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoL3HeightCommand extends Command {
    private ElevatorSubsystem m_ElevatorSubsystem;
    private IntakeSubsystem m_IntakeSubsystem;

    public AutoL3HeightCommand(ElevatorSubsystem elevatorSubsystem, IntakeSubsystem intakeSubsystem) {
        m_ElevatorSubsystem = elevatorSubsystem;
        m_IntakeSubsystem = intakeSubsystem;
        addRequirements(m_ElevatorSubsystem, m_IntakeSubsystem);
    }

    public void initialize() {
        m_IntakeSubsystem.setPivot(-35);
    }

    public void execute() {
        m_IntakeSubsystem.setPivot(-35);
        if (m_IntakeSubsystem.getPivot().getPosition().getValueAsDouble() > -34.3) {
            m_ElevatorSubsystem.setCoralL3();
        }
    }

    public void end(boolean interrupted) {
        System.out.println("ended");
        m_IntakeSubsystem.setPivot(IntakeConstants.kL3ScoringAngle);
    }

    public boolean isFinished() {
        return m_ElevatorSubsystem.getElevator().getPosition().getValueAsDouble() > (ElevatorConstants.kCoralL3Height + 0.3);
    }

}