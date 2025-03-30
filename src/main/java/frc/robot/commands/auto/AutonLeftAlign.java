package frc.robot.commands.auto;

import static edu.wpi.first.units.Units.MetersPerSecond;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.LeftOrRight;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutonLeftAlign extends Command{

    private final CommandSwerveDrivetrain m_DriveTrain;
    private Pose2d targetPose;
    private Pose2d curPose;
    private ProfiledPIDController translationalPID;
    private ProfiledPIDController rotationalPID;
    private Optional<LeftOrRight> leftOrRight;
    private final double translationSpeedLim = .5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double translationAccelLim = 2.3;
    private final double rotationalSpeedLim = Math.PI;
    private final double rotationalAccelLim = Math.PI * 4;
    private final double speedTolerance = 0.075;
    private final double translationTol = Units.inchesToMeters(0.675);
    private final double rotationTol = Units.degreesToRadians(0.675);
    private final double speedTolRot = Math.PI / 16;
    private final double ffMinRadius = 0.2;
    private final double ffMaxRadius = 0.6;
    private final CommandXboxController joystick;

    public AutonLeftAlign(CommandSwerveDrivetrain drivetrain, Optional<LeftOrRight> leftOrRight, CommandXboxController joystick){
        m_DriveTrain = drivetrain;
        this.leftOrRight = leftOrRight;
        this.joystick = joystick;
        addRequirements(m_DriveTrain);
    }

    public void initialize() {
        translationalPID = new ProfiledPIDController(2, 0, 0,
                new TrapezoidProfile.Constraints(translationSpeedLim, translationAccelLim));
        rotationalPID = new ProfiledPIDController(6, 0, 0,
                new TrapezoidProfile.Constraints(rotationalSpeedLim, rotationalAccelLim));
        double[] pose = LimelightHelpers.getBotPose_TargetSpace("limelight-left");
        curPose = new Pose2d(pose[2], pose[0], Rotation2d.fromDegrees(-pose[4]));
        //Aligns Left of REef
            targetPose = new Pose2d(-0.30670297370299904, -0.18610406522285455, new Rotation2d(0));
       
        rotationalPID.enableContinuousInput(-Math.PI, Math.PI);

        translationalPID.setTolerance(translationTol, speedTolerance);
        rotationalPID.setTolerance(rotationTol, speedTolRot);

        ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(m_DriveTrain.getState().Speeds,
                new Rotation2d(pose[4]));
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
        System.out.println(curPose);
        System.out.println(targetPose);
    }

    public void execute() {
        //ill do ignore some tags tomorrow
        if(LimelightHelpers.getTV("limelight-left")){
            double[] pose = LimelightHelpers.getBotPose_TargetSpace("limelight-left");
            curPose = new Pose2d(pose[2], pose[0], Rotation2d.fromDegrees(-pose[4]));
    
            double curDist = curPose.getTranslation().getDistance(targetPose.getTranslation());
            double ffScalar = MathUtil.clamp((curDist - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0, 1);
    
            double driveVelocityScalar = 
                translationalPID.getSetpoint().velocity * ffScalar
                + translationalPID.calculate(curDist, 0);
            if (curDist < translationalPID.getPositionTolerance()) {
                driveVelocityScalar = 0;
            }
            double rotationError = curPose.getRotation().minus(targetPose.getRotation()).getRadians();
            double rotationVelocity = 
                rotationalPID.getSetpoint().velocity 
                * ffScalar 
                + rotationalPID.calculate(curPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
            if(Math.abs(rotationError) < rotationTol){
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
                curPose.getRotation()
                );
            SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    
            m_DriveTrain.setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds));
            double[] curPoseData = {curPose.getX(), curPose.getY(), curPose.getRotation().getRadians()};
            double[] targetPoseData = {targetPose.getX(), targetPose.getY(), targetPose.getRotation().getRadians()};
            SmartDashboard.putNumber("Vx", driveVelocity.getX());
            SmartDashboard.putNumber("Vy", driveVelocity.getY());
            SmartDashboard.putNumber("Vr", rotationalPID.getSetpoint().velocity);
            SmartDashboard.putNumberArray("CurPose", curPoseData);
            SmartDashboard.putNumberArray("TargetPose", targetPoseData);
        }else{
            this.end(true);
                      
                System.out.println("ended");
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            0, 
            0, 
            0, 
            curPose.getRotation()
            );
            SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

            m_DriveTrain.setControl(m_pathApplyRobotSpeeds.withSpeeds(speeds));      }

    public boolean isFinished() {
        return (rotationalPID.atGoal() && translationalPID.atGoal()) || joystick.leftTrigger().getAsBoolean();

    }


}
