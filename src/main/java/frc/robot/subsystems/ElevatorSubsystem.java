package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DifferentialPositionDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Mode;
import frc.robot.RobotContainer;
import frc.robot.constants.OtherConstants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    public static TalonFX elevator = new TalonFX(ElevatorConstants.kElevatorID);
    public static int curCoralLevel;
    public static int curAlgaeLevel;
    public static CommandXboxController joystick;

    public ElevatorSubsystem() {
        curCoralLevel = 0;
        curAlgaeLevel = 0;
        
        Slot0Configs configs = new Slot0Configs();
        CurrentLimitsConfigs configLimit = new CurrentLimitsConfigs();

        configLimit.StatorCurrentLimit = 80;
        configLimit.SupplyCurrentLimit = 60;

        configs.kP = ElevatorConstants.kElevatorP;
        configs.kI = ElevatorConstants.kElevatorI;
        configs.kD = ElevatorConstants.kElevatorD;

        elevator.getConfigurator().apply(configs);
        elevator.setPosition(0);
        elevator.setNeutralMode(NeutralModeValue.Brake);

        elevator.getConfigurator().apply(configLimit);
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
        joystick.setRumble(RumbleType.kBothRumble, 0.2);

    }

    public void setCoralL2() {
        setHeight(ElevatorConstants.kCoralL2Height);
        joystick.setRumble(RumbleType.kBothRumble, 0.2);

    }

    public void setCoralL3() {
        setHeight(ElevatorConstants.kCoralL3Height);
        joystick.setRumble(RumbleType.kBothRumble, 0.2);

    }
    
   
    /// change the height of the elevator to the designated heights each Coral
    /// levels on the reef
    public void setAlgaeBottom(){
        setHeight(ElevatorConstants.kBottomAlgae);
        joystick.setRumble(RumbleType.kBothRumble, 0.2);

        
    }
    public void setAlgaeTop(){
        setHeight(ElevatorConstants.kTopAlgae);
        joystick.setRumble(RumbleType.kBothRumble, 0.2);

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
        if (RobotContainer.getMode() == Mode.ALGAEMODE){
            RobotContainer.setMode(Mode.CORALMODE);
            curCoralLevel = 0;
        }
        else{
            RobotContainer.setMode(Mode.ALGAEMODE);
            curAlgaeLevel = 0;
        }
    }

    public void periodic(){
        SmartDashboard.putNumber("Elevator Height", elevator.getPosition().getValueAsDouble());
        // System.out.println("Elevator height : " + elevator.getPosition().getValueAsDouble());
        // System.out.println(RobotContainer.getMode());
        // System.out.println(curCoralLevel);
     
    }

    public static double getElevatorHeight() {
        return elevator.getPosition().getValueAsDouble();
    }
    public void setZeroPosition(){
        elevator.setPosition(0);
    }
    public Command setZeroPositionCommand(){
        return runOnce(() -> setZeroPosition()).onlyIf(() -> elevator.getPosition().getValueAsDouble() < 0.075);
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
    public Command setIntakeHeightCommand()
    {
        return runOnce(() -> setHeight(ElevatorConstants.kIntakeHeight));
    }
    public Command setDescoreHighHeightCommand(){
        return runOnce(() -> setHeight(ElevatorConstants.kTopAlgae));
    }
    public Command setDescoreLowHeightCommand(){
        return runOnce(() -> setHeight(ElevatorConstants.kBottomAlgae));
    }
    public void initializeJoystick(CommandXboxController joystick){
        ElevatorSubsystem.joystick = joystick;
    }
    public TalonFX getElevator(){
        return elevator;
    }
}