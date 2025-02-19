package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DifferentialPositionDutyCycle;
import com.ctre.phoenix6.controls.DifferentialVelocityDutyCycle;
import com.ctre.phoenix6.controls.DifferentialVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
    private TalonFX pivot = new TalonFX(IntakeConstants.kPivotID);

    public IntakeSubsystem() {
        Slot0Configs configs = new Slot0Configs();
        
        configs.kP = IntakeConstants.kIntakeP;
        configs.kI = IntakeConstants.kIntakeI;
        configs.kD = IntakeConstants.kIntakeD;

        intake.getConfigurator().apply(configs);

        intake.setNeutralMode(NeutralModeValue.Coast);

        configs.kP = IntakeConstants.kPivotP;
        configs.kI = IntakeConstants.kPivotI;
        configs.kD = IntakeConstants.kPivotD;

        pivot.getConfigurator().apply(configs);
        pivot.setPosition(0);
    }

    public void setSpeed(double speed) {
        intake.setControl(new DutyCycleOut(speed)); 
    }

    public void setPivot(double angle) {
        pivot.setControl(new DifferentialPositionDutyCycle(angle, 0));
    }

    public Command coralOutakeCommand(){
        return this.runOnce(()-> setSpeed(IntakeConstants.kOutTakeSpeed));
    }

    public Command algeaOutakeCommand(){
        return this.runOnce(()-> setSpeed(-IntakeConstants.kOutTakeSpeed));
    }
    public void periodic()
    {
        // System.out.println(pivot.getPosition());
    }
    public void intakeBrakeMode(){
        intake.setNeutralMode(NeutralModeValue.Brake);
    }

    public void intakeCoastMode(){
        intake.setNeutralMode(NeutralModeValue.Coast);
    }
    public TalonFX getPivot(){
        return pivot;
    }
}