package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotStateEstimator extends SubsystemBase{
    private CommandSwerveDrivetrain m_CommandSwerveDrivetrain;
    private boolean doRejectUpdate = false;
    public RobotStateEstimator(CommandSwerveDrivetrain swerve){
        m_CommandSwerveDrivetrain = swerve;
    }
    @Override
    public void periodic() {
        LimelightHelpers.SetRobotOrientation("limelight", m_CommandSwerveDrivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(Math.abs(m_CommandSwerveDrivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        m_CommandSwerveDrivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_CommandSwerveDrivetrain.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
}
