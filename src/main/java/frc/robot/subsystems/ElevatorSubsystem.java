package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DifferentialPositionDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Mode;
import frc.robot.RobotContainer;
import frc.robot.constants.OtherConstants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    public static TalonFX elevator = new TalonFX(ElevatorConstants.kElevatorID);
    public static int curCoralLevel;
    public static int curAlgaeLevel;

    public ElevatorSubsystem() {
        curCoralLevel = 0;
        curAlgaeLevel = 0;
        
        Slot0Configs configs = new Slot0Configs();
        
        configs.kP = ElevatorConstants.kElevatorP;
        configs.kI = ElevatorConstants.kElevatorI;
        configs.kD = ElevatorConstants.kElevatorD;

        elevator.getConfigurator().apply(configs);
        elevator.setPosition(0);
    }

    public void setHeight(double height) {
        elevator.setControl(new DifferentialPositionDutyCycle(height, 0));
        
    }

    public void incrementCoralLevel() {
        if (curCoralLevel < 4)
            curCoralLevel++;
        else
            curCoralLevel = 1;
    }

    public void incrementAlgaeLevel() {
        if (curAlgaeLevel < 3)
            curAlgaeLevel++;
        else
            curAlgaeLevel = 1;
    }

   
    public void resetLevels(){
        curAlgaeLevel = 1;
        curCoralLevel = 1;
    }

    /**
     * @return returns the current coral level on the reef that you are aiming for
     */
    public int getCoralLevel() {
        return curCoralLevel;
    }

    /**
     * @return returns the current algae level on the reef that you are aiming for
     */
    public int getAlgaeLevel() {
        return curAlgaeLevel;
    }

    /// change the height of the elevator to the designated heights each Coral
    /// levels on the reef
    public void moveZeroPosition(){
        setHeight(0);
    }
    public void setCoralL1() {
        setHeight(ElevatorConstants.kCoral1Height);
    }

    public void setCoralL2() {
        setHeight(ElevatorConstants.kCoralL2Height);
    }

    public void setCoralL3() {
        setHeight(ElevatorConstants.kCoralL3Height);
    }
    
   
    /// change the height of the elevator to the designated heights each Coral
    /// levels on the reef
    public void setAlgaeBottom(){
        setHeight(ElevatorConstants.kBottomAlgae);
    }
    public void setAlgaeTop(){
        setHeight(ElevatorConstants.kTopAlgae);
    }

    public void setDefaultHeight(){
        setHeight(ElevatorConstants.kDefaultHeight);
    }
    
    public Command resetHeightCommand(){
        return this.runOnce(() -> setDefaultHeight());
    }

    public Command switchModeCommand() {
        return this.runOnce(() -> switchMode());
    }
    public void switchMode() {
        if (RobotContainer.getMode() == Mode.ALGAEMODE)
            RobotContainer.setMode(Mode.CORALMODE);
        else
            RobotContainer.setMode(Mode.ALGAEMODE);
    }

    public void periodic(){
        System.out.println("Elevator height : " + elevator.getPosition().getValueAsDouble());
        // System.out.println(RobotContainer.getMode());
       
        // System.out.println(curCoralLevel);
     
    }

    public static double getElevatorHeight() {
        return elevator.getPosition().getValueAsDouble();
    }
    public void setZeroPositon(){
        elevator.setPosition(0);
    }
    public Command setZeroPositionCommand(){
        return runOnce(() -> setZeroPositon()).onlyIf(() -> elevator.getPosition().getValueAsDouble() < 0.075);
    }
    public Command setCoralHeightCommand(int height){
        if(height == 0){
            return runOnce(() -> moveZeroPosition());
        }else if(height == 1){
            return runOnce(() -> setCoralL1());

        }else if(height == 2){
            return runOnce(() -> setCoralL2());
        }else if(height == 3){
            return runOnce(() -> setCoralL3());
        }
        return runOnce(() -> moveZeroPosition());
    }
}