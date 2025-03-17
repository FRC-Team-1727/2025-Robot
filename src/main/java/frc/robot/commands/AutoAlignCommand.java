package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import org.w3c.dom.views.DocumentView;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.LimelightHelpers;

import edu.wpi.first.math.controller.PIDController;
class PIDControllerConfigurable extends PIDController {
  public PIDControllerConfigurable(double kP, double kI, double kD) {
      super(kP, kI, kD);
  }
  
  public PIDControllerConfigurable(double kP, double kI, double kD, double tolerance) {
      super(kP, kI, kD);
      this.setTolerance(tolerance);
  }
}
public class AutoAlignCommand extends Command {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final VisionSubsystem m_Limelight;

    private static final PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(0.05000, 0.000000, 0.001000, 0.01);
    private static final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(0.400000, 0.000000, 0.000600, 0.1);  // Tighter tolerance
    private static final PIDControllerConfigurable yPidController = new PIDControllerConfigurable(0.3, 0, 0, 0.1);
    private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

    public double rotationalRate = 0;
    public double velocityX = 0;
    public double velocityY = 0;

    public CommandXboxController joystick;
    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, CommandXboxController joystick) {
        this.m_drivetrain = drivetrain;
        this.m_Limelight = limelight;
        this.joystick = joystick;
        addRequirements(m_Limelight);
    }

    @Override
    public void initialize() {
        // Initialization code, if needed
    }

    @Override
    public void execute() {
        RawFiducial fiducial;

        try {
            fiducial = m_Limelight.getFiducialWithId(22);  // Get the AprilTag
            double adjustedTxnc;
            double angleToTargetRad;
            // Apply the 14-degree offset to the horizontal angle (txnc) if needed
             // 14-degree offset adjustment (if required)
             if(fiducial.distToRobot < .1) {
                adjustedTxnc = fiducial.txnc - 14;
                rotationalRate = rotationalPidController.calculate(adjustedTxnc, 0.0) * 0.75 * 0.9; 
                angleToTargetRad = Math.toRadians(adjustedTxnc);
            } else {
                rotationalRate = 0;
                angleToTargetRad = 0;
            }
            // Calculate the rotational rate to adjust robot orientation based on adjusted horizontal angle to the tag
            // Keep the sign as it is for correct direction
            
            // Calculate the velocity in the X and Y directions to move towards the tag's center
            final double velocityX = xPidController.calculate(-fiducial.distToRobot, .1) * Math.cos(angleToTargetRad) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * .3;
            final double velocityY = yPidController.calculate(-fiducial.distToRobot, 0) * Math.sin(angleToTargetRad)* TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.1;
            
            

            // If the robot is aligned both rotationally and distance-wise, end the command
            if (rotationalPidController.atSetpoint() && xPidController.atSetpoint() && yPidController.atSetpoint()) {
                this.end(true);  // Centered on the tag, end the alignment process
            }

            // Output the fiducial data for debugging
            SmartDashboard.putNumber("txnc", fiducial.txnc);
            SmartDashboard.putNumber("distToRobot", fiducial.distToRobot);
            SmartDashboard.putNumber("rotationalPidController", rotationalRate);
            SmartDashboard.putNumber("xPidController", velocityX);
            SmartDashboard.putNumber("yPidController", velocityY);

            // Apply the swerve control to align robot towards the tag
            m_drivetrain.setControl(
                alignRequest.withRotationalRate(-rotationalRate)  // Reverse the rotational direction if needed
                            .withVelocityX(velocityX)
                            .withVelocityY(-velocityY));
                            System.out.println("X:" + velocityX);
                            System.out.println("Y: " + velocityY);
        } catch (VisionSubsystem.NoSuchTargetException nste) {
            // If no target is found, try to continue the last known movement
            System.out.println("No apriltag found");
            // if (rotationalRate != 0 && velocityX != 0) {
            //     m_drivetrain.setControl(
            //         alignRequest.withRotationalRate(0)
            //                     .withVelocityX(.5)
            //                     .withVelocityY(0));
            // }
        }
        
    }

    @Override
    public boolean isFinished() {
        return rotationalPidController.atSetpoint() && xPidController.atSetpoint() && yPidController.atSetpoint() || joystick.leftTrigger().getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.applyRequest(() -> idleRequest);
    }
}
