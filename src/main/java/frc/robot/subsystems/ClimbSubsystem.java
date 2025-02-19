package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DifferentialPositionDutyCycle;
import com.ctre.phoenix6.controls.DifferentialVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

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
        
        configs.kP = ClimbConstants.kClimbP;
        configs.kI = ClimbConstants.kClimbP;
        configs.kD = ClimbConstants.kClimbP;

        climb.getConfigurator().apply(configs);
        climb.setNeutralMode(NeutralModeValue.Brake);
        climb.setPosition(0);
        deployed = false;
    }


    public void setClimb(double angle) {
        curAngle = angle;
        climb.setNeutralMode(NeutralModeValue.Coast);
        climb.setControl(new DifferentialPositionDutyCycle(curAngle, 0));
        if(climb.getPosition().getValueAsDouble() - angle < 0.5){
            climb.setNeutralMode(NeutralModeValue.Brake);
        }
    }
    public void climbUp(){
        double decreaseValue = 1;
        if(climb.getPosition().getValueAsDouble() - decreaseValue > 0){
            curAngle -= decreaseValue;
            setClimb(curAngle);
        }else{
            curAngle = 0;
            setClimb(curAngle);
        }
        
    }
    public void climbDown(){
        curAngle++;
        setClimb(curAngle);
    }

    public Command setClimbCommand(double angle){
        return this.runOnce(()-> setClimb(angle));
    }

    public Command climbDownCommand(){
        return this.runOnce(()-> setClimb(0));
    }

    public Command climbUpCommand(){
        return this.runOnce(()-> setClimb(118.596));
    }
    public void periodic(){
        // System.out.println(climb.getPosition().getValueAsDouble());
    }
    //118.596
}
