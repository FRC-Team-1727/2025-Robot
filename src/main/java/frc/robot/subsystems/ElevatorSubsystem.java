package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Mode;
import frc.robot.RobotContainer;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    public static SparkFlex elevator = new SparkFlex(ElevatorConstants.kElevatorID, MotorType.kBrushless);
    private int curCoralLevel;
    private int curAlgaeLevel;

    public ElevatorSubsystem() {
        curCoralLevel = 1;
        curAlgaeLevel = 1;
        SparkFlexConfig config = new SparkFlexConfig();

        config
                .inverted(false)
                .idleMode(IdleMode.kBrake);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorP, ElevatorConstants.kElevatorP);
    }

    public void setHeight(double height) {
        elevator.getClosedLoopController().setReference(height, ControlType.kPosition);
    }

    public void incrementCoralLevel() {
        if (curCoralLevel < 3)
            curCoralLevel++;
        else
            curCoralLevel = 1;
    }

    public void incrementAlgaeLevel() {
        if (curAlgaeLevel < 2)
            curAlgaeLevel++;
        else
            curAlgaeLevel = 1;
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

    public void setdefaultHeight(){
        setHeight(ElevatorConstants.kDefaultHeight);
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
}