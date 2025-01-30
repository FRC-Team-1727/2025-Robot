package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private SparkFlex intake = new SparkFlex(IntakeConstants.kIntakeID, MotorType.kBrushless);
    private SparkFlex pivot = new SparkFlex(IntakeConstants.kPivotID, MotorType.kBrushless);

    public IntakeSubsystem() {
        SparkFlexConfig config = new SparkFlexConfig();
        config
                .inverted(false)
                .idleMode(IdleMode.kBrake);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(IntakeConstants.kPivotP, IntakeConstants.kPivotI,
                        IntakeConstants.kPivotD);

        pivot.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        config
                .inverted(false)
                .idleMode(IdleMode.kBrake);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(IntakeConstants.kIntakeP, IntakeConstants.kIntakeI, IntakeConstants.kIntakeD);
        intake.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void setSpeed(double speed) {
        intake.set(speed); // test set reference with control type velocity
    }

    public void setPivot(double angle) {
        pivot.getClosedLoopController().setReference(angle, ControlType.kPosition);
    }

    public Command coralOutakeCommand(){
        return this.runOnce(()-> setSpeed(IntakeConstants.kOutTakeSpeed));
    }

    public Command algeaOutakeCommand(){
        return this.runOnce(()-> setSpeed(-IntakeConstants.kOutTakeSpeed));
    }

}