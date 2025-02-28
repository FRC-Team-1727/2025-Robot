// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.AlgaeOuttakeCommand;
//import frc.robot.commands.ClimbCommand;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralOuttakeCommand;
import frc.robot.commands.SwitchClimbCommand;
import frc.robot.commands.ChangeElevatorCommand;
import frc.robot.commands.ClimbResetCommand;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.OtherConstants.ClimbConstants;
import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class RobotContainer {
    private static Mode curMode = Mode.CORALMODE;
    private static IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
    private static ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
    private static ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
    private final SendableChooser<Command> autoChooser;
    
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {

        configureNamedCommands();

        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("None", Commands.none());
        // autoChooser.addOption("TestMove", new PathPlannerAuto("TestMove"));

        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        //  joystick.b().whileTrue(drivetrain.applyRequest(
        //          () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
        configureButtons();

    }

    public void configureButtons() {
        System.out.println(curMode);
        joystick.rightBumper().and(() -> curMode == Mode.ALGAEMODE).whileTrue(new AlgaeIntakeCommand(m_IntakeSubsystem));
        joystick.rightBumper().and(() -> curMode == Mode.CORALMODE).whileTrue(new CoralIntakeCommand(m_IntakeSubsystem, m_ElevatorSubsystem));

        joystick.rightTrigger().and(() -> curMode == Mode.ALGAEMODE).whileTrue(new AlgaeOuttakeCommand(m_IntakeSubsystem));
        joystick.rightTrigger().and(() -> curMode == Mode.CORALMODE).whileTrue(new CoralOuttakeCommand(m_IntakeSubsystem, m_ElevatorSubsystem));

        joystick.y().onTrue(new SwitchClimbCommand(m_ClimbSubsystem));

        joystick.b().onTrue(m_ElevatorSubsystem.switchModeCommand());
        joystick.a().onTrue(new ClimbResetCommand(m_ClimbSubsystem));
        joystick.leftBumper().whileTrue(new ChangeElevatorCommand(m_IntakeSubsystem, m_ElevatorSubsystem));
//        joystick.leftTrigger().onFalse(m_ElevatorSubsystem.setZeroPositionCommand());






        // joystick.leftBumper().whileTrue(new AlgaeIntakeCommand(m_IntakeSubsystem));
        // joystick.b().onFalse(m_ElevatorSubsystem.switchModeCommand());
        // joystick.leftTrigger().whileTrue(m_IntakeSubsystem.algeaOutakeCommand());
        // joystick.rightTrigger().whileTrue(m_IntakeSubsystem.coralOutakeCommand());
        // joystick.y().onFalse(new ChangeElevatorCommand(m_IntakeSubsystem, m_ElevatorSubsystem)); // this could
                                                                                                           // potentially
                                                                                                           // maybe be a
                                                                                                           // problem
      //  joystick.rightBumper().whileTrue(new ClimbCommand(m_IntakeSubsystem));
    }
    public void configureNamedCommands(){
        NamedCommands.registerCommand("Score L2", 
                m_ElevatorSubsystem.setCoralHeightCommand(2)
                .alongWith(m_IntakeSubsystem.setPivotCommand())
                .andThen(new WaitCommand(1))
                .andThen(m_IntakeSubsystem.runOnce(() -> m_IntakeSubsystem.setSpeed(IntakeConstants.kCoralOutTakeSpeed
                ))));
        NamedCommands.registerCommand("Coral Intake", new CoralIntakeCommand(m_IntakeSubsystem, m_ElevatorSubsystem));

        
    }

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        return autoChooser.getSelected();
    }

    public static Mode getMode() {
        return curMode;
    }

    public static void setMode(Mode mode) {
        curMode = mode;
    }
}
