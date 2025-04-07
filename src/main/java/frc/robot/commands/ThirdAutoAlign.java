package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.lang.reflect.Field;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.LeftOrRight;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LEDMode;
import frc.robot.subsystems.LEDSubsystem;

public class ThirdAutoAlign extends Command {
    private final CommandSwerveDrivetrain m_DriveTrain;
    private final LEDSubsystem m_LedSubsystem;
    private Pose2d targetPose;
    private Pose2d curPose;
    private ProfiledPIDController translationalPID;
    private ProfiledPIDController rotationalPID;
    private Optional<LeftOrRight> leftOrRight;
    private final double translationSpeedLim = .75 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double translationAccelLim = 2.3;
    private final double rotationalSpeedLim = Math.PI;
    private final double rotationalAccelLim = Math.PI * 4;
    private final double speedTolerance = 0.075;
    private final double translationTol = Units.inchesToMeters(2);
    private final double rotationTol = Units.degreesToRadians(0.675);
    private final double speedTolRot = Math.PI / 16;
    private final double ffMinRadius = 0.2;
    private final double ffMaxRadius = 0.6;
    private final CommandXboxController joystick;

    public ThirdAutoAlign(CommandSwerveDrivetrain drivetrain, Optional<LeftOrRight> leftOrRight,
            CommandXboxController joystick, LEDSubsystem ledSubsystem) {
        m_DriveTrain = drivetrain;
        this.leftOrRight = leftOrRight;
        this.joystick = joystick;
        m_LedSubsystem = ledSubsystem;
        addRequirements(m_DriveTrain, m_LedSubsystem);
    }

    public void initialize() {
        curPose = new Pose2d(0, 0, new Rotation2d(0));
        targetPose = new Pose2d(0, 0, new Rotation2d(0));
        translationalPID = new ProfiledPIDController(2, 0, 0,
                new TrapezoidProfile.Constraints(translationSpeedLim, translationAccelLim));
        rotationalPID = new ProfiledPIDController(6, 0, 0,
                new TrapezoidProfile.Constraints(rotationalSpeedLim, rotationalAccelLim));
        double[] poseLeft = LimelightHelpers.getBotPose_TargetSpace("limelight-left");
        double[] poseRight = LimelightHelpers.getBotPose_TargetSpace("limelight-right");
        if (LimelightHelpers.getTV("limelight-left") && LimelightHelpers.getTV("limelight-right")) {
            curPose = new Pose2d((poseLeft[2] + poseRight[2]) / 2, (poseLeft[0] + poseRight[0]) / 2,
                    Rotation2d.fromDegrees(-(poseLeft[4] + poseRight[4]) / 2));
        } else if (LimelightHelpers.getTV("limelight-left")) {
            curPose = new Pose2d(poseLeft[2], poseLeft[0], Rotation2d.fromDegrees(-poseLeft[4]));
        } else if (LimelightHelpers.getTV("limelight-right")) {
            curPose = new Pose2d(poseRight[2], poseRight[0], Rotation2d.fromDegrees(-poseRight[4]));
        } else {
            System.out.println("no LL detected");
            this.end(true);
        }
        int targetTagID;
        if (LimelightHelpers.getTV("limelight-left") && LimelightHelpers.getTV("limelight-right")) {
            if(LimelightHelpers.getFiducialID("limelight-left") == LimelightHelpers.getFiducialID("limelight-right")){
                targetTagID = (int) LimelightHelpers.getFiducialID("limelight-left");
            }else{
                targetTagID = -1;
                targetPose = new Pose2d(-0.25670297370299904,0,new Rotation2d(0));
            }
        } else if (LimelightHelpers.getTV("limelight-left")) {
            targetTagID = (int) LimelightHelpers.getFiducialID("limelight-left");
        } else if (LimelightHelpers.getTV("limelight-right")) {
            targetTagID = (int) LimelightHelpers.getFiducialID("limelight-right");
        } else {
            targetTagID = -1;
            System.out.println("no target");
            this.end(true);
        }
        if(targetTagID != -1){
            if(leftOrRight.isEmpty()){
                targetPose = FieldConstants.REEF_ID_CENTER_LOCATIONS.get(targetTagID);
            }
            else if (leftOrRight.get() == LeftOrRight.LEFT) {
                targetPose = FieldConstants.REEF_ID_LEFT_LOCATIONS.get(targetTagID);
            } else if(leftOrRight.get() == LeftOrRight.RIGHT){
                targetPose = FieldConstants.REEF_ID_RIGHT_LOCATIONS.get(targetTagID);
            }
        }
        else{System.out.println("broken");
        this.end(true);
    }
        rotationalPID.enableContinuousInput(-Math.PI, Math.PI);

        translationalPID.setTolerance(translationTol, speedTolerance);
        rotationalPID.setTolerance(rotationTol, speedTolRot);

        ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(m_DriveTrain.getState().Speeds,
                new Rotation2d(-curPose.getRotation().getRadians()));
        translationalPID.reset(
                curPose.getTranslation().getDistance(targetPose.getTranslation()),
                Math.min(0, -new Translation2d(fieldRelative.vxMetersPerSecond, fieldRelative.vyMetersPerSecond)
                        .rotateBy(targetPose.getTranslation()
                                .minus(curPose.getTranslation())
                                .getAngle()
                                .unaryMinus())
                        .getX()));
        rotationalPID.reset(
                curPose.getRotation().getRadians(),
                fieldRelative.omegaRadiansPerSecond);
        m_LedSubsystem.setMode(LEDMode.kAligning);
        System.out.println(targetTagID);
    }

    public void execute() {
        double[] poseLeft = LimelightHelpers.getBotPose_TargetSpace("limelight-left");
        double[] poseRight = LimelightHelpers.getBotPose_TargetSpace("limelight-right");
        if (LimelightHelpers.getTV("limelight-left") && LimelightHelpers.getTV("limelight-right")) {
            curPose = new Pose2d((poseLeft[2] + poseRight[2]) / 2, (poseLeft[0] + poseRight[0]) / 2,
                    Rotation2d.fromDegrees(-(poseLeft[4] + poseRight[4]) / 2));
        } else if (LimelightHelpers.getTV("limelight-left")) {
            curPose = new Pose2d(poseLeft[2], poseLeft[0], Rotation2d.fromDegrees(-poseLeft[4]));
        } else if (LimelightHelpers.getTV("limelight-right")) {
            curPose = new Pose2d(poseRight[2], poseRight[0], Rotation2d.fromDegrees(-poseRight[4]));
        } else {
            System.out.println("no LL detected");
            this.end(true);
        }
        double curDist = curPose.getTranslation().getDistance(targetPose.getTranslation());
        double ffScalar = MathUtil.clamp((curDist - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.1, 1);

        double driveVelocityScalar = translationalPID.getSetpoint().velocity * ffScalar
                + translationalPID.calculate(curDist, 0);
        if (curDist < translationalPID.getPositionTolerance()) {
            driveVelocityScalar = 0;
        }
        double rotationError = curPose.getRotation().minus(targetPose.getRotation()).getRadians();
        double rotationVelocity = rotationalPID.getSetpoint().velocity
                * ffScalar
                + rotationalPID.calculate(curPose.getRotation().getRadians(),
                        targetPose.getRotation().getRadians());
        if (Math.abs(rotationError) < rotationTol) {
            rotationVelocity = 0;
        }
        Pose2d driveVelocity = new Pose2d(
                0,
                0,
                curPose.getTranslation()
                        .minus(targetPose.getTranslation()).getAngle())
                .transformBy(new Transform2d(driveVelocityScalar, 0, new Rotation2d()));

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                driveVelocity.getX(),
                -driveVelocity.getY(),
                rotationVelocity,
                curPose.getRotation());
        SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

        m_DriveTrain.setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds));
        // double[] curPoseData = { curPose.getX(), curPose.getY(), curPose.getRotation().getRadians() };
        // double[] targetPoseData = { targetPose.getX(), targetPose.getY(), targetPose.getRotation().getRadians() };
        // SmartDashboard.putNumber("Vx", driveVelocity.getX());
        // SmartDashboard.putNumber("Vy", driveVelocity.getY());
        // SmartDashboard.putNumber("Vr", rotationalPID.getSetpoint().velocity);
        // SmartDashboard.putNumberArray("CurPose", curPoseData);
        // SmartDashboard.putNumberArray("TargetPose", targetPoseData);

    }

    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                0,
                0,
                0,
                curPose.getRotation());
        SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

        m_DriveTrain.setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds));
        m_LedSubsystem.setMode(LEDMode.kAligned);
    }

    public boolean isFinished() {
        return (rotationalPID.atGoal() && translationalPID.atGoal()) || joystick.leftTrigger().getAsBoolean()
                || (!LimelightHelpers.getTV("limelight-left") && !LimelightHelpers.getTV("limelight-right"));

    }
    // new LL position
    /*
     * up : .244826539
     * forward : 0.28432802
     * left/right : 0.259959
     * angle up : 20 degrees
     * angle left and right : 22.5 degrees
     */

}
