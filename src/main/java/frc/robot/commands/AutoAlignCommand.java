package frc.robot.commands;

import java.util.logging.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAlignCommand extends Command {

    //not done
    private CommandSwerveDrivetrain m_CommandSwerveDrivetrain;
    private Pose3d targetPose;
    private Pose2d curPose;

    public AutoAlignCommand(CommandSwerveDrivetrain swerve) {
        m_CommandSwerveDrivetrain = swerve;
    }

    public void initialize() {
        curPose = m_CommandSwerveDrivetrain.getPose();
        targetPose = LimelightHelpers.getTargetPose3d_RobotSpace("");
    }

    public void execute() {
    }

    public void end() {

    }

    public boolean isFinished() {
        return false;
    }
}
