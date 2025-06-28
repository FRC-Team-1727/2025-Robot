package frc.robot.constants;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
    public static final double fieldWidth = 8.05;
    public static final double fieldLength = 17.55;
    public static ArrayList<Pose2d> REEF_LOCATIONS = new ArrayList<>();
    public static ArrayList<Pose2d> REEF_CENTER_LOCATIONS = new ArrayList<>();
    public static HashMap<Integer, Rotation2d> REEF_ANGLES = new HashMap<>();
    public static HashMap<Integer, Pose2d> REEF_ID_CENTER_LOCATIONS = new HashMap<>();
    public static HashMap<Integer, Pose2d> REEF_ID_LEFT_LOCATIONS = new HashMap<>();
    public static HashMap<Integer, Pose2d> REEF_ID_RIGHT_LOCATIONS = new HashMap<>();

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

        REEF_ANGLES.put(17, new Rotation2d(Math.PI/3));
        REEF_ANGLES.put(18, new Rotation2d(0.0));
        REEF_ANGLES.put(19, new Rotation2d(-Math.PI/3));
        REEF_ANGLES.put(20, new Rotation2d(-2 * Math.PI/3));
        REEF_ANGLES.put(21, new Rotation2d(Math.PI));
        REEF_ANGLES.put(22, new Rotation2d(2 * Math.PI/3));
        REEF_ANGLES.put(6, new Rotation2d(2 * Math.PI/3));
        REEF_ANGLES.put(7, new Rotation2d(Math.PI));
        REEF_ANGLES.put(8, new Rotation2d(-2 * Math.PI/3));
        REEF_ANGLES.put(9, new Rotation2d(-Math.PI / 3));
        REEF_ANGLES.put(10, new Rotation2d(0.0));
        REEF_ANGLES.put(11, new Rotation2d(Math.PI / 3));


        Pose2d REEF_22_LEFT = new Pose2d(-0.453806810, -0.185988737, new Rotation2d(0));
        Pose2d REEF_21_LEFT = new Pose2d(-0.414475637, -0.155664759, new Rotation2d(0));
        Pose2d REEF_20_LEFT = new Pose2d(-0.414211147, -0.157637442, new Rotation2d(0));
        Pose2d REEF_19_LEFT = new Pose2d(-0.415829251, -0.164104276, new Rotation2d(0));
        Pose2d REEF_18_LEFT = new Pose2d(-0.413190868, -0.183519771, new Rotation2d(0));
        Pose2d REEF_17_LEFT = new Pose2d(-0.414095375, -0.171540969, new Rotation2d(0));
        Pose2d REEF_11_LEFT = new Pose2d(-0.415022686, -0.177844488, new Rotation2d(0));
        Pose2d REEF_10_LEFT = new Pose2d(-0.415828777, -0.1610273677, new Rotation2d(0));
        Pose2d REEF_9_LEFT = new Pose2d(-0.4137795535, -0.1685568259, new Rotation2d(0));
        Pose2d REEF_8_LEFT = new Pose2d(-0.4144963204, -0.1795942466, new Rotation2d(0));
        Pose2d REEF_7_LEFT = new Pose2d(-0.412848306, -0.1631717659, new Rotation2d(0));
        Pose2d REEF_6_LEFT = new Pose2d(-0.413975518, -0.1774865167, new Rotation2d(0));

        REEF_ID_LEFT_LOCATIONS.put(22, REEF_22_LEFT);
        REEF_ID_LEFT_LOCATIONS.put(21, REEF_21_LEFT);
        REEF_ID_LEFT_LOCATIONS.put(20, REEF_20_LEFT);
        REEF_ID_LEFT_LOCATIONS.put(19, REEF_19_LEFT);
        REEF_ID_LEFT_LOCATIONS.put(18, REEF_18_LEFT);
        REEF_ID_LEFT_LOCATIONS.put(17, REEF_17_LEFT);
        REEF_ID_LEFT_LOCATIONS.put(11, REEF_11_LEFT);
        REEF_ID_LEFT_LOCATIONS.put(10, REEF_10_LEFT);
        REEF_ID_LEFT_LOCATIONS.put(9, REEF_9_LEFT);
        REEF_ID_LEFT_LOCATIONS.put(8, REEF_8_LEFT);
        REEF_ID_LEFT_LOCATIONS.put(7, REEF_7_LEFT);
        REEF_ID_LEFT_LOCATIONS.put(6, REEF_6_LEFT);

        Pose2d REEF_22_RIGHT = new Pose2d(-0.424301707, 0.179621061, new Rotation2d(0));
        Pose2d REEF_21_RIGHT = new Pose2d(-0.389637075, 0.175958616, new Rotation2d(0));
        Pose2d REEF_20_RIGHT = new Pose2d(-0.390062313, 0.167234918, new Rotation2d(0));
        Pose2d REEF_19_RIGHT = new Pose2d(-0.391715733, 0.1667086624, new Rotation2d(0));
        Pose2d REEF_18_RIGHT = new Pose2d(-0.389061295, 0.1613482324, new Rotation2d(0));
        Pose2d REEF_17_RIGHT = new Pose2d(-0.389322961, 0.1692195865, new Rotation2d(0));
        Pose2d REEF_11_RIGHT = new Pose2d(-0.391037851, 0.1682654144, new Rotation2d(0));
        Pose2d REEF_10_RIGHT = new Pose2d(-0.390484055, 0.1774308478, new Rotation2d(0));
        Pose2d REEF_9_RIGHT = new Pose2d(-0.390419736, 0.166420666, new Rotation2d(0));
        Pose2d REEF_8_RIGHT = new Pose2d(-0.389664948, 0.17719904407, new Rotation2d(0));
        Pose2d REEF_7_RIGHT = new Pose2d(-0.389618757, 0.1747919262, new Rotation2d(0));
        Pose2d REEF_6_RIGHT = new Pose2d(-0.390316848, 0.1740693366, new Rotation2d(0));
        
        REEF_ID_RIGHT_LOCATIONS.put(22, REEF_22_RIGHT);
        REEF_ID_RIGHT_LOCATIONS.put(21, REEF_21_RIGHT);
        REEF_ID_RIGHT_LOCATIONS.put(20, REEF_20_RIGHT);
        REEF_ID_RIGHT_LOCATIONS.put(19, REEF_19_RIGHT);
        REEF_ID_RIGHT_LOCATIONS.put(18, REEF_18_RIGHT);
        REEF_ID_RIGHT_LOCATIONS.put(17, REEF_17_RIGHT);
        REEF_ID_RIGHT_LOCATIONS.put(11, REEF_11_RIGHT);
        REEF_ID_RIGHT_LOCATIONS.put(10, REEF_10_RIGHT);
        REEF_ID_RIGHT_LOCATIONS.put(9, REEF_9_RIGHT);
        REEF_ID_RIGHT_LOCATIONS.put(8, REEF_8_RIGHT);
        REEF_ID_RIGHT_LOCATIONS.put(7, REEF_7_RIGHT);
        REEF_ID_RIGHT_LOCATIONS.put(6, REEF_6_RIGHT);

        Pose2d REEF_22_CENTER = new Pose2d(-0.367106994, 0, new Rotation2d(0));
        Pose2d REEF_21_CENTER = new Pose2d(-0.367106994, 0, new Rotation2d(0));
        Pose2d REEF_20_CENTER = new Pose2d(-0.367106994, 0, new Rotation2d(0));
        Pose2d REEF_19_CENTER = new Pose2d(-0.367106994, 0, new Rotation2d(0));
        Pose2d REEF_18_CENTER = new Pose2d(-0.367106994, 0, new Rotation2d(0));
        Pose2d REEF_17_CENTER = new Pose2d(-0.367106994, 0, new Rotation2d(0));
        Pose2d REEF_11_CENTER = new Pose2d(-0.367106994, 0, new Rotation2d(0));
        Pose2d REEF_10_CENTER = new Pose2d(-0.367106994, 0, new Rotation2d(0));
        Pose2d REEF_9_CENTER = new Pose2d(-0.367106994, 0, new Rotation2d(0));
        Pose2d REEF_8_CENTER = new Pose2d(-0.367106994, 0, new Rotation2d(0));
        Pose2d REEF_7_CENTER = new Pose2d(-0.367106994, 0, new Rotation2d(0));
        Pose2d REEF_6_CENTER = new Pose2d(-0.367106994, 0, new Rotation2d(0));

        REEF_ID_CENTER_LOCATIONS.put(22, REEF_22_CENTER);
        REEF_ID_CENTER_LOCATIONS.put(21, REEF_21_CENTER);
        REEF_ID_CENTER_LOCATIONS.put(20, REEF_20_CENTER);
        REEF_ID_CENTER_LOCATIONS.put(19, REEF_19_CENTER);
        REEF_ID_CENTER_LOCATIONS.put(18, REEF_18_CENTER);
        REEF_ID_CENTER_LOCATIONS.put(17, REEF_17_CENTER);
        REEF_ID_CENTER_LOCATIONS.put(11, REEF_11_CENTER);
        REEF_ID_CENTER_LOCATIONS.put(10, REEF_10_CENTER);
        REEF_ID_CENTER_LOCATIONS.put(9, REEF_9_CENTER);
        REEF_ID_CENTER_LOCATIONS.put(8, REEF_8_CENTER);
        REEF_ID_CENTER_LOCATIONS.put(7, REEF_7_CENTER);
        REEF_ID_CENTER_LOCATIONS.put(6, REEF_6_CENTER);
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
    public static Rotation2d tagAngle(int tagID){
        return REEF_ANGLES.get(tagID);
    }
}
