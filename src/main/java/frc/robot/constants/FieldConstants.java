package frc.robot.constants;

import static edu.wpi.first.units.Units.Rotation;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
    public static final double fieldWidth = 8.05;
    public static final double fieldLength = 17.55;
    public static ArrayList<Pose2d> REEF_LOCATIONS = new ArrayList<>();
    public static ArrayList<Pose2d> REEF_CENTER_LOCATIONS = new ArrayList<>();

    public void configureReefPositions(Alliance alliance){
        boolean isRed = (alliance == Alliance.Red);

        

    }
    public Pose2d findPose(double x, double y, double rotation, boolean isRed){
        if(isRed){
            return new Pose2d(fieldLength - x, fieldWidth - 7,  new Rotation2d(rotation + Math.PI));
        }
        else{
            return new Pose2d(x, y, new Rotation2d(rotation));
        }
    }
    
}
