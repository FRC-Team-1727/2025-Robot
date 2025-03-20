package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import org.w3c.dom.views.DocumentView;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.LeftOrRight;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.LimelightHelpers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
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

    private ProfiledPIDController rotationalPID;
    private final double rotationalSpeedLim = Math.PI;
    private final double rotationalAccelLim = Math.PI * 4;
    private final double rotationTol = Units.degreesToRadians(0.675);
    private final double speedTolRot = Math.PI / 16;
    private Pose2d curPose;
    private Pose2d targetPose;

    public CommandXboxController joystick;

    public AutoAlignCommand(CommandSwerveDrivetrain drivetrain, VisionSubsystem limelight, CommandXboxController joystick) {
        this.m_drivetrain = drivetrain;
        this.m_Limelight = limelight;
        this.joystick = joystick;
        addRequirements(m_Limelight);
    }

    @Override
    public void initialize() {
        rotationalPID = new ProfiledPIDController(6, 0, 0,
                new TrapezoidProfile.Constraints(rotationalSpeedLim, rotationalAccelLim));
        
        curPose = m_drivetrain.getPose();
        rotationalPID.enableContinuousInput(-Math.PI, Math.PI);
        rotationalPID.setTolerance(rotationTol, speedTolRot);
        ChassisSpeeds fieldRelative = ChassisSpeeds.fromFieldRelativeSpeeds(m_drivetrain.getState().Speeds, m_drivetrain.getPose().getRotation());
        rotationalPID.reset(
                curPose.getRotation().getRadians(),
                fieldRelative.omegaRadiansPerSecond);
    }

    @Override
    public void execute() {
        RawFiducial fiducial;
        curPose = m_drivetrain.getPose();
        Double targetRotation = FieldConstants.tagAngle(22);

        double rotationError = curPose.getRotation().minus(targetPose.getRotation()).getRadians();
        double rotationVelocity = rotationalPID.getSetpoint().velocity + rotationalPID.calculate(curPose.getRotation().getRadians(), targetRotation);
        if(Math.abs(rotationError) < rotationTol){
            rotationVelocity = 0;
        }
        try {
            fiducial = m_Limelight.getFiducialWithId(22);  // Get the AprilTag
            double adjustedTxnc;
            double angleToTargetRad;
            // Apply the 14-degree offset to the horizontal angle (txnc) if needed
             // 14-degree offset adjustment (if required)
            //  if(fiducial.distToRobot < .1) {
                
            // } else {
            //     rotationalRate = 0;
            //     angleToTargetRad = 0;
            // }
            adjustedTxnc = m_Limelight.getTX();
            angleToTargetRad = Math.toRadians(adjustedTxnc);
            // Calculate the rotational rate to adjust robot orientation based on adjusted horizontal angle to the tag
            // Keep the sign as it is for correct direction
            
            // Calculate the velocity in the X and Y directions to move towards the tag's center
            final double velocityX = xPidController.calculate(-fiducial.distToRobot * Math.cos(angleToTargetRad), 0) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * .2;
            final double velocityY = yPidController.calculate(-fiducial.distToRobot * Math.sin(angleToTargetRad), 0)* TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.2;
            
            

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
                alignRequest.withRotationalRate(rotationVelocity)  // Reverse the rotational direction if needed
                            .withVelocityX(0)
                            .withVelocityY(0));
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
        return rotationalPID.atSetpoint() && xPidController.atSetpoint() && yPidController.atSetpoint() || joystick.leftTrigger().getAsBoolean();
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.applyRequest(() -> idleRequest);
    }
}
