package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
    public static final double fieldWidth = 8.05;
    public static final double fieldLength = 17.55;
    public static ArrayList<Pose2d> REEF_LOCATIONS = new ArrayList<>();
    public static ArrayList<Pose2d> REEF_CENTER_LOCATIONS = new ArrayList<>();

    public static void configureReefPositions(Alliance alliance){
        boolean isRed = (alliance == Alliance.Red);

        Pose2d REEF_A = findPose(3.201, 4.186, 0, isRed);
        Pose2d REEF_B = findPose(3.219, 3.876, 0, isRed);
        Pose2d REEF_C = findPose(3.684, 2.988, Math.PI / 3, isRed);
        Pose2d REEF_D = findPose(3.976, 2.821, Math.PI / 3, isRed);
        Pose2d REEF_E = findPose(4.972, 2.851, 2 * Math.PI / 3, isRed);
        Pose2d REEF_F = findPose(5.294, 2.988, 2 * Math.PI / 3, isRed);
        Pose2d REEF_G = findPose(5.776, 3.840, Math.PI, isRed);
        Pose2d REEF_H = findPose(5.776, 4.192, Math.PI, isRed);
        Pose2d REEF_I = findPose(5.276, 5.026, -2 * Math.PI / 3, isRed);
        Pose2d REEF_J = findPose(4.958, 5.164, -2 * Math.PI / 3, isRed);
        Pose2d REEF_K = findPose(4, 5.164, -Math.PI / 3, isRed);
        Pose2d REEF_L = findPose(3.732, 5.062, -Math.PI/3, isRed);

        REEF_LOCATIONS.addAll(List.of(REEF_A, REEF_B, REEF_C, REEF_D, REEF_E, REEF_F, REEF_G, REEF_H, REEF_I, REEF_J, REEF_K, REEF_L));
    }
    public static Pose2d findPose(double x, double y, double rotation, boolean isRed){
        if(isRed){
            return new Pose2d(fieldLength - x, fieldWidth - y,  new Rotation2d(rotation + Math.PI));
        }
        else{
            return new Pose2d(x, y, new Rotation2d(rotation));
        }
    }
    
}
