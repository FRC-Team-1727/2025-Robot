package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
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
    public ClimbSubsystem(){
        Slot0Configs configs = new Slot0Configs();
        
        configs.kP = ClimbConstants.kClimbP;
        configs.kI = ClimbConstants.kClimbP;
        configs.kD = ClimbConstants.kClimbP;

        climb.getConfigurator().apply(configs);
    }


    public void setClimb(double angle) {
        climb.setPosition(angle);
    }
    public void climbUp(){
        climb.set(0.25f);
    }
    public void climbDown(){
        climb.set(-0.25f);
    }

       public Command setClimbCommand(double angle){
        return this.runOnce(()-> setClimb(angle));
    }

    public Command climbDownCommand(){
        return this.runOnce(()-> climbDown());
    }

    public Command climbUpCommand(){
        return this.run(()-> climbUp());
    }
}
