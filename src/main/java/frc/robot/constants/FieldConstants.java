package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotation;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.HashMap;
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
    public static HashMap<Integer, Double> REEF_ANGLES = new HashMap<>();

    public static void configureReefPositions(Alliance alliance) {
        boolean isRed = (alliance == Alliance.Red);
        System.out.println(isRed);
        Pose2d REEF_A = findPose(3.201, 4.186, 0, isRed);
        Pose2d REEF_B = findPose(3.219, 3.876, 0, isRed);
        Pose2d REEF_C = findPose(3.684, 2.988, Math.PI / 3, isRed);
        Pose2d REEF_D = findPose(3.976, 2.821, Math.PI / 3, isRed);
        Pose2d REEF_E = findPose(4.972, 2.851, 2 * Math.PI / 3, isRed);
        Pose2d REEF_F = findPose(4.46112279, 3.14384523, 2 * Math.PI / 3, isRed);
        Pose2d REEF_G = findPose(5.776, 3.840, Math.PI, isRed);
        Pose2d REEF_H = findPose(5.776, 4.192, Math.PI, isRed);
        Pose2d REEF_I = findPose(5.276, 5.026, -2 * Math.PI / 3, isRed);
        Pose2d REEF_J = findPose(4.958, 5.164, -2 * Math.PI / 3, isRed);
        Pose2d REEF_K = findPose(4, 5.164, -Math.PI / 3, isRed);
        Pose2d REEF_L = findPose(3.732, 5.062, -Math.PI / 3, isRed);

        // Pose2d REEF_A = !isRed ? findPose(3.201377639, 4.208064793, 0) 
        //         : findPose(13.463156503, 4.180335526, Math.PI);
        // Pose2d REEF_B = !isRed ? findPose(3.227556986, 3.877701648, 0) 
        //         : findPose(13.457477850, 3.873110631, Math.PI);
        // Pose2d REEF_C = !isRed ? findPose(3.637024206, 3.486474973, Math.PI / 3)
        //         : findPose(13.026179117, 4.930651885, -2 * Math.PI / 3);
        // Pose2d REEF_D = !isRed ? findPose(3.635719579, 3.139646389, Math.PI / 3)
        //         : findPose(13.040941042, 4.604011991, -2 * Math.PI / 3);
        // Pose2d REEF_E = !isRed ? findPose(4.415836137, 3.487810176, 2 * Math.PI / 3)
        //         : findPose(12.206028265, 4.914124590, -Math.PI / 3);
        // Pose2d REEF_F = !isRed ? findPose(4.431206506, 3.156545677, 2 * Math.PI / 3)
        //         : findPose(12.206151527, 4.583430626, -Math.PI / 3);
        // Pose2d REEF_G = !isRed ? findPose(4.891986650, 4.192000225, Math.PI) 
        //         : findPose(11.795546335, 4.199969054, 0);
        // Pose2d REEF_H = !isRed ? findPose(4.883310836, 3.864837636, Math.PI) 
        //         : findPose(11.796662849, 3.884231930, 0);
        // Pose2d REEF_I = !isRed ? findPose(4.452373785, 3.032684325, -2 * Math.PI / 3)
        //         : findPose(12.207996887, 3.487440986, Math.PI / 3);
        // Pose2d REEF_J = !isRed ? findPose(4.458719057, 4.487131658, -2 * Math.PI / 3)
        //         : findPose(12.202502054, 3.159981766, Math.PI / 3);
        // Pose2d REEF_K = !isRed ? findPose(3.632789415, 4.930041057, -Math.PI / 3)
        //         : findPose(13.040006441, 3.485546452, 2 * Math.PI / 3);
        // Pose2d REEF_L = !isRed ? findPose(3.632590574, 4.605809369, -Math.PI / 3)
        //         : findPose(13.034827989, 3.144272366, 2 * Math.PI / 3);

        REEF_LOCATIONS.addAll(List.of(REEF_A, REEF_B, REEF_C, REEF_D, REEF_E, REEF_F, REEF_G, REEF_H, REEF_I, REEF_J,
                REEF_K, REEF_L));

        Pose2d REEF_1 = findPose(3.225, 4.013, 0, isRed);
        Pose2d REEF_2 = findPose(3.893, 2.928, Math.PI / 3, isRed);
        Pose2d REEF_3 = findPose(5.151, 2.934, 2 * Math.PI / 3, isRed);
        Pose2d REEF_4 = findPose(5.753, 4.003, Math.PI, isRed);
        Pose2d REEF_5 = findPose(5.127, 5.164, -2 * Math.PI / 3, isRed);
        Pose2d REEF_6 = findPose(3.851, 5.062, -Math.PI / 3, isRed);

        REEF_CENTER_LOCATIONS.addAll(List.of(REEF_1, REEF_2, REEF_3, REEF_4, REEF_5, REEF_6));

        REEF_ANGLES.put(17, Math.PI/3);
        REEF_ANGLES.put(18, 0.0);
        REEF_ANGLES.put(19, -Math.PI/3);
        REEF_ANGLES.put(20, -2 * Math.PI/3);
        REEF_ANGLES.put(21, Math.PI);
        REEF_ANGLES.put(22, 2 * Math.PI/3);
        REEF_ANGLES.put(6, 2 * Math.PI/3);
        REEF_ANGLES.put(7, Math.PI);
        REEF_ANGLES.put(8, -2 * Math.PI/3);
        REEF_ANGLES.put(9, -Math.PI / 3);
        REEF_ANGLES.put(10, 0.0);
        REEF_ANGLES.put(11, Math.PI / 3);
    }

    public static Pose2d findPose(double x, double y, double rotation, boolean isRed) {
        if (isRed) {
            return new Pose2d(fieldLength - x, fieldWidth - y, new Rotation2d(rotation + Math.PI));
        } else {
            return new Pose2d(x, y, new Rotation2d(rotation));
        }
    }

    public static Pose2d findPose(double x, double y, double rotation) {

        return new Pose2d(x, y, new Rotation2d(rotation));

    }
    public static Double tagAngle(int tagID){
        return REEF_ANGLES.get(tagID);
    }
}
