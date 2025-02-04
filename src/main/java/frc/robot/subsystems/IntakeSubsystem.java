package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.OtherConstants.ElevatorConstants;
import frc.robot.constants.OtherConstants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX intake = new TalonFX(IntakeConstants.kIntakeID);
    private TalonFX pivot = new TalonFX(IntakeConstants.kIntakeID);

    public IntakeSubsystem() {
        Slot0Configs configs = new Slot0Configs();
        
        configs.kP = IntakeConstants.kIntakeP;
        configs.kI = IntakeConstants.kIntakeI;
        configs.kD = IntakeConstants.kIntakeD;

        intake.getConfigurator().apply(configs);

        configs.kP = IntakeConstants.kPivotP;
        configs.kI = IntakeConstants.kPivotI;
        configs.kD = IntakeConstants.kPivotD;  
    
        pivot.getConfigurator().apply(configs);
    }

    public void setSpeed(double speed) {
        intake.set(speed); 
    }

    public void setPivot(double angle) {
        pivot.setPosition(angle);
    }

    public Command coralOutakeCommand(){
        return this.runOnce(()-> setSpeed(IntakeConstants.kOutTakeSpeed));
    }

    public Command algeaOutakeCommand(){
        return this.runOnce(()-> setSpeed(-IntakeConstants.kOutTakeSpeed));
    }

}