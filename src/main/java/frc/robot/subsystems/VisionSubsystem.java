package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.OtherConstants.VisionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;


public class VisionSubsystem extends SubsystemBase {
  private static RawFiducial[] fiducials;
  private NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight-left");
  private NetworkTableEntry botPoseYawEntry = limelightTable.getEntry("botpose_targetspace_yaw");
  private double robotYaw;
  private LimelightHelpers.PoseEstimate mt1Left = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");



  public VisionSubsystem() {
    config();
  }

  public static class NoSuchTargetException extends RuntimeException {
    public NoSuchTargetException(String message) {
      super(message);
    }
  }

  public void config() {

    // LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);
    LimelightHelpers.SetFiducialIDFiltersOverride("", new int[] {0,1,5,8,9,10,11,12});
    robotYaw = 0;
  }

  @Override
  public void periodic() {
    fiducials = LimelightHelpers.getRawFiducials("limelight-left");
    robotYaw = botPoseYawEntry.getDouble(0);
    mt1Left = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");
    SmartDashboard.putNumber("Robot YAw", robotYaw);

  }
  public static RawFiducial getClosestFiducial() {
    if (fiducials == null || fiducials.length == 0) {
        throw new NoSuchTargetException("No fiducials found.");
    }

    RawFiducial closest = fiducials[0];
    double minDistance = closest.ta;

    for (RawFiducial fiducial : fiducials) {
        if (fiducial.ta > minDistance) {
            closest = fiducial;
            minDistance = fiducial.ta;
        }
    }

    return closest;
  }

  public static RawFiducial getFiducialWithId(int id) {
  
    for (RawFiducial fiducial : fiducials) {
        if (fiducial.id == id) {
            return fiducial;
        }
    }
    throw new NoSuchTargetException("Can't find ID: " + id);
  }

public RawFiducial getFiducialWithId(int id, boolean verbose) {
  StringBuilder availableIds = new StringBuilder();

  for (RawFiducial fiducial : fiducials) {
      if (availableIds.length() > 0) {
          availableIds.append(", ");
      } //Error reporting
      availableIds.append(fiducial.id);
      
      if (fiducial.id == id) {
          return fiducial;
      }
  }
  throw new NoSuchTargetException("Cannot find: " + id + ". IN view:: " + availableIds.toString());
  }

  public double getTX(){
    return LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME);
  }
  public double getTY(){
    return LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME);
  }
  public double getTA(){
    return LimelightHelpers.getTA(VisionConstants.LIMELIGHT_NAME);
  }
  public boolean getTV(){
    return LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);
  }

  public double getClosestTX(){
    return getClosestFiducial().txnc;
  }
  public double getClosestTY(){
    return getClosestFiducial().tync;
  }
  public double getClosestTA(){
    return getClosestFiducial().ta;
  }
  public Pose2d getLimelightPose(){
    return mt1Left.pose;
  }
}