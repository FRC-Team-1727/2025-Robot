package frc.robot.commands;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.LeftOrRight;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class BasicAutoAlign extends Command{
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
    private final double headingTol = Units.degreesToRadians(0.675);
    private final double speedTolRot = Math.PI/16;
    private final double ffMinRadius = 0.2;
    private final double ffMaxRadius = 0.6;
    public BasicAutoAlign(CommandSwerveDrivetrain drivetrain, Optional<LeftOrRight> leftOrRight){
        m_DriveTrain = drivetrain;
        this.leftOrRight = leftOrRight;
        addRequirements(m_DriveTrain);
    }
    public void initialize(){        
        translationalPID = new ProfiledPIDController(6, 0, 0, new TrapezoidProfile.Constraints(translationSpeedLim, rotationalAccelLim));
        rotationalPID = new ProfiledPIDController(6, 0, 0, new TrapezoidProfile.Constraints(rotationalSpeedLim, rotationalAccelLim));
        curPose = m_DriveTrain.getPose();
        if(leftOrRight.isEmpty()){
            if(!FieldConstants.REEF_LOCATIONS.isEmpty()) targetPose =  curPose.nearest(FieldConstants.REEF_CENTER_LOCATIONS);
            else new Pose2d();
        }
        rotationalPID.enableContinuousInput(-Math.PI, Math.PI);

        translationalPID.setTolerance(translationTol, speedTolerance);
        rotationalPID.setTolerance(headingTol, speedTolRot);

        curPose = m_DriveTrain.getPose();
        
        ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(m_DriveTrain.getState().Speeds, m_DriveTrain.getPose().getRotation());
        translationalPID.reset(
            curPose.getTranslation().getDistance(targetPose.getTranslation()),
            Math.min(0,-new Translation2d(fieldRelative.vxMetersPerSecond, fieldRelative.vyMetersPerSecond)
            .rotateBy(targetPose.getTranslation()
            .minus(curPose.getTranslation())
            .getAngle()
            .unaryMinus()).getX())
        );
        rotationalPID.reset(
            curPose.getRotation().getRadians(),
            fieldRelative.omegaRadiansPerSecond
        );
    }
    public void execute(){
        curPose = m_DriveTrain.getPose();

        double curDist = curPose.getTranslation().getDistance(targetPose.getTranslation());
        double ffScalar = MathUtil.clamp((curDist - ffMinRadius)/(ffMaxRadius - ffMinRadius), 0, 1);

        double driveVelocityScalar = translationalPID.getSetpoint().velocity * ffScalar + translationalPID.calculate(curDist, 0);
         if(curDist < translationalPID.getPositionTolerance()){
            driveVelocityScalar = 0;
         }

    }
    public void end(){

    }
    public boolean isFinished(){
        return false;
    }
    
}