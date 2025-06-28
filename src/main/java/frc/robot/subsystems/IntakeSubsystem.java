package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.OtherConstants.ElevatorConstants;
import frc.robot.constants.OtherConstants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private TalonFX intake = new TalonFX(IntakeConstants.kIntakeID);
    private TalonFX pivot = new TalonFX(IntakeConstants.kPivotID);
    private boolean thresholdReached = false;

    public IntakeSubsystem() {
        Slot0Configs configs = new Slot0Configs();
        
        configs.kP = IntakeConstants.kIntakeP;
        configs.kI = IntakeConstants.kIntakeI;
        configs.kD = IntakeConstants.kIntakeD;

        intake.getConfigurator().apply(configs);
        CurrentLimitsConfigs configLimit = new CurrentLimitsConfigs();

        configLimit.StatorCurrentLimit = 80;
        configLimit.SupplyCurrentLimit = 60;
        
        intake.setNeutralMode(NeutralModeValue.Coast);

        configs.kP = IntakeConstants.kPivotP;
        configs.kI = IntakeConstants.kPivotI;
        configs.kD = IntakeConstants.kPivotD;

        pivot.getConfigurator().apply(configs);
        pivot.setPosition(0);

        pivot.setNeutralMode(NeutralModeValue.Coast);

        intake.getConfigurator().apply(configLimit);
        pivot.getConfigurator().apply(configLimit);
    }

    public void setSpeed(double speed) {
        intake.setControl(new DutyCycleOut(speed)); 
    }
    public void intakeSpeed(){
        setSpeed(IntakeConstants.kCoralIntakeSpeed);
    }
    public void autoIntakeSpeed(){
        setSpeed(IntakeConstants.kCoralIntakeSpeed);
        if(getIntakeVelocity() < -45){
            thresholdReached = true;
        }
    }

    public void setPivot(double angle) {
        pivot.setControl(new DifferentialPositionDutyCycle(angle, 0));
    }
    public Command setPivotCommand(double angle){
        return runOnce(() -> setPivot(angle));

    }
    public Command setPivotCommand(){
        return runOnce(() -> setPivot(IntakeConstants.kScoringAngle));
    }
    public Command setZeroPivotCommand(){
        return runOnce(() -> setPivot(0));
    }

    public Command coralOutakeCommand(){
        return this.run(()-> setSpeed(IntakeConstants.kCoralOutTakeSpeed));
    }
    public Command coralIntakeCommand(){
        return this.runOnce(()-> setSpeed(-IntakeConstants.kCoralOutTakeSpeed));
    }

    public Command algaeHighOutakeCommand(){
        return this.run(()-> setSpeed(IntakeConstants.kAlgaeOutTakeSpeed));
    }
    public Command algaeLowOutakeCommand(){
        return this.run(()-> setSpeed(IntakeConstants.kAlgaeDescoreSpeed));
    }
    public Command passiveIntakeCommand(){
        return this.runOnce(() -> setSpeed(IntakeConstants.kPassiveCoralIntakeSpeed));
    }
    public void periodic()
    {
        SmartDashboard.putNumber("Pivot Angle", pivot.getPosition().getValueAsDouble());
        //   System.out.println("Pivot angle : " + pivot.getPosition().getValueAsDouble());
        // System.out.println(intake.getVelocity().getValueAsDouble());
    }
    public void intakeBrakeMode(){
        intake.setNeutralMode(NeutralModeValue.Brake);
    }

    public void intakeCoastMode(){
        intake.setNeutralMode(NeutralModeValue.Coast);
    }
    public void pivotCoastMode(){
        pivot.setNeutralMode(NeutralModeValue.Coast);
    }
    public void pivotBrakeMode(){
        pivot.setNeutralMode(NeutralModeValue.Brake);
    }
    public TalonFX getPivot(){
        return pivot;
    }
    public double getPivotVelocity(){
        return pivot.getVelocity().getValueAsDouble();
    }
    public double getIntakeVelocity(){
        return intake.getVelocity().getValueAsDouble();
    }
    public boolean getThreshold(){
        return thresholdReached;
    }
    public void clearThreshold(){
        thresholdReached = false;
    }
}