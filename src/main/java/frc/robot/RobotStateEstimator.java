package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotStateEstimator extends SubsystemBase{
    private CommandSwerveDrivetrain m_CommandSwerveDrivetrain;
    private boolean doRejectLeftUpdate = false;
    private boolean doRejectRightUpdate = false;
    public RobotStateEstimator(CommandSwerveDrivetrain swerve){
        m_CommandSwerveDrivetrain = swerve;
    }
    @Override
    public void periodic() {
      LimelightHelpers.SetRobotOrientation("limelight-right", m_CommandSwerveDrivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
      LimelightHelpers.SetRobotOrientation("limelight-left", m_CommandSwerveDrivetrain.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2Left = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-left");
      LimelightHelpers.PoseEstimate mt2Right = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-right");

      if(Math.abs(m_CommandSwerveDrivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectLeftUpdate = true;
        doRejectRightUpdate = true;
      }

      if(mt2Left.tagCount == 0)
      {
        doRejectLeftUpdate = true;
      }
      if(mt2Right.tagCount == 0){
        doRejectRightUpdate = true;
      }
      if(!doRejectLeftUpdate)
      {
        m_CommandSwerveDrivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_CommandSwerveDrivetrain.addVisionMeasurement(
            mt2Left.pose,
            mt2Left.timestampSeconds);
      }
      if(!doRejectRightUpdate){
        
        m_CommandSwerveDrivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        m_CommandSwerveDrivetrain.addVisionMeasurement(
            mt2Right.pose,
            mt2Right.timestampSeconds);
      }
    }
}
