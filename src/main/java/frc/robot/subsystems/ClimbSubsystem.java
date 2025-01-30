package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ClimbSubsystem extends SubsystemBase{
        // private SparkFlex elevator = new SparkFlex(ElevatorConstants.kElevatorID, MotorType.kBrushless);

    public ClimbSubsystem(){
        // SparkFlexConfig config = new SparkFlexConfig();

        // config
        //         .inverted(false)
        //         .idleMode(IdleMode.kBrake);
        // config.closedLoop
        //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //         .pid(ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorP);
    }
}
