package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.DifferentialPositionDutyCycle;
import com.ctre.phoenix6.controls.DifferentialVelocityDutyCycle;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.OtherConstants.ClimbConstants;
import frc.robot.constants.OtherConstants.ElevatorConstants;
import frc.robot.constants.OtherConstants.IntakeConstants;

public class ClimbSubsystem extends SubsystemBase{
private static TalonFX climb = new TalonFX(ClimbConstants.kClimbID);
    private boolean deployed;
    private double curAngle;
    public ClimbSubsystem(){
        
        Slot0Configs configs = new Slot0Configs();
        CurrentLimitsConfigs configLimit = new CurrentLimitsConfigs();

        configLimit.StatorCurrentLimit = 80;
        configLimit.SupplyCurrentLimit = 60;
        
        configs.kP = ClimbConstants.kClimbP;
        configs.kI = ClimbConstants.kClimbI;
        configs.kD = ClimbConstants.kClimbD;

        climb.getConfigurator().apply(configs);
        climb.setNeutralMode(NeutralModeValue.Brake);
        climb.setPosition(0);

        climb.getConfigurator().apply(configLimit);
        deployed = false;
    }

    public void setClimb(double angle) {
        curAngle = angle;
        climb.setNeutralMode(NeutralModeValue.Coast);
        climb.setControl(new DifferentialPositionDutyCycle(curAngle, 0));
    }
    public void climbUp(){
        if(climb.getPosition().getValueAsDouble() > 0.5){
            climb.setControl(new DutyCycleOut(0.5));
        }
        
    }
    public void climbDown(){
        climb.setControl(new DutyCycleOut(-.5));
    }

    public Command setClimbCommand(double angle){
        return this.runOnce(()-> setClimb(angle));
    }

    public Command climbDownCommand(){
        return this.runOnce(()-> setClimb(0));
    }
    public Command manualClimbCommand(boolean up){
        if(up){
            return this.runOnce(() -> climbUp());
        }else{
            return this.runOnce(() -> climbDown());
        }
    }
    public void periodic(){
        SmartDashboard.putNumber("Climb Position", climb.getPosition().getValueAsDouble());
        //  System.out.println("Climb Position" + climb.getPosition().getValueAsDouble());
    }
    public void switchClimbStatus(){
        deployed = !deployed; //changes the status of the climb from deployed and retracted
    }
   
    public boolean getClimbStatus(){
        return deployed;
    }

    // public void setClimbStatus(boolean b) {
    //     deployed = b;
    // }
    public boolean atPosition()
    {
        if(deployed){
            return Math.abs(climb.getPosition().getValueAsDouble() - ClimbConstants.kUpClimbAngle) < 1;
        }else if(!deployed){
            return Math.abs(climb.getPosition().getValueAsDouble() - ClimbConstants.kDownClimbAngle) < 1;
        }
        return false;
    }
    public void setBrakeMode(){
        climb.setNeutralMode(NeutralModeValue.Brake);
        // System.out.println(climb.getPosition().getValueAsDouble() + " - BRAKED");
    }
    //118.596
}
