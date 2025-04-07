// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.ModuleLayer.Controller;
import java.util.Optional;
import java.util.logging.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.BasicAutoAlign;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.CoralOuttakeCommand;
import frc.robot.commands.SwitchClimbCommand;
import frc.robot.commands.ThirdAutoAlign;
import frc.robot.commands.auto.AutoCoralIntakeCommand;
import frc.robot.commands.auto.AutoL3HeightCommand;
import frc.robot.commands.auto.AutoPassiveIntakeCommand;
import frc.robot.commands.ChangeElevatorCommand;
import frc.robot.commands.ClimbResetCommand;
import frc.robot.constants.LeftOrRight;
import frc.robot.constants.TunerConstants;
import frc.robot.constants.OtherConstants.ClimbConstants;
import frc.robot.constants.OtherConstants.IntakeConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import pabeles.concurrency.IntOperatorTask.Max;

public class RobotContainer {
        private static Mode curMode = Mode.CORALMODE;
        private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
        private final ElevatorSubsystem m_ElevatorSubsystem = new ElevatorSubsystem();
        private final ClimbSubsystem m_ClimbSubsystem = new ClimbSubsystem();
        private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
        private final LEDSubsystem m_LedSubsystem = new LEDSubsystem();
        private final SendableChooser<Command> autoChooser;

        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController joystick = new CommandXboxController(0);
        private final CommandXboxController joystick2 = new CommandXboxController(1);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
        // private final RobotStateEstimator m_RobotStateEstimator = new
        // RobotStateEstimator(drivetrain);

        public RobotContainer() {
                m_ElevatorSubsystem.initializeJoystick(joystick);

                configureNamedCommands();

                autoChooser = new SendableChooser<>();

                autoChooser.setDefaultOption("None", Commands.none());
                // autoChooser.addOption("L2", new PathPlannerAuto("Test"));
                // autoChooser.addOption("Testing", new PathPlannerAuto("Testing"));
                // autoChooser.addOption("L1", new PathPlannerAuto("Blue L1"));
                // autoChooser.addOption("Move up L1 Red", new PathPlannerAuto("Red L3"));
                // autoChooser.addOption("Move up L1 Blue", new PathPlannerAuto("Blue L3"));
                // autoChooser.addOption("Field Clai", new PathPlannerAuto("HI"));
                autoChooser.addOption("3 Piece L2", new PathPlannerAuto("3P L2"));
                autoChooser.addOption("4 Piece L2 Right", new PathPlannerAuto("4P L2 Right"));
                autoChooser.addOption("4 Piece L2 Left", new PathPlannerAuto("4P L2 Left"));

                // autoChooser.addOption("autoalign test", new PathPlannerAuto("3P L2"));
                autoChooser.addOption("1PL2 + 2PL3 Right", new PathPlannerAuto("1PL2 + 2PL3 Right"));
                autoChooser.addOption("1PL2 + 2PL3 Left", new PathPlannerAuto("1PL2 + 2PL3 Left"));
                autoChooser.addOption("Center L2 descore algae", new PathPlannerAuto("1P L2 Descore Center"));
                autoChooser.addOption("Center L2 to the right", new PathPlannerAuto("1P L2 Center to Right"));

                // autoChooser.addOption("L3 test", new PathPlannerAuto("L3 test"));
                // autoChooser.addOption("TestMove", new PathPlannerAuto("TestMove"));

                SmartDashboard.putData("Auto Chooser", autoChooser);

                configureBindings();
        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.

                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive
                                                .withVelocityX(-joystick.getLeftY() * MaxSpeed
                                                                * (joystick.leftTrigger().getAsBoolean() ? .2 : 1)) // Drive
                                                // forward
                                                // with
                                                // negative Y
                                                // (forward)
                                                .withVelocityY(
                                                                -joystick.getLeftX() * MaxSpeed
                                                                                * (joystick.leftTrigger().getAsBoolean()
                                                                                                ? .2
                                                                                                : 1)) // Drive
                                                                                                      // left
                                                                                                      // with
                                                                                                      // negative
                                                                                                      // X
                                                                                                      // (left)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate
                                                                * (joystick.leftTrigger().getAsBoolean() ? .2 : 1)) // Drive
                                                                                                                    // counterclockwise
                                                                                                                    // with
                                // negative X (left)
                                ));

                // drivetrain.setDefaultCommand(
                // // Drivetrain will execute this command periodically
                // drivetrain.applyRequest(() -> drive.withVelocityX(-(joystick.getLeftY() < 0 ?
                // Math.pow(joystick.getLeftY(), 2) : -Math.pow(joystick.getLeftY(), 2)) *
                // MaxSpeed * (joystick.leftTrigger().getAsBoolean() ? .2 : 1)) // Drive forward
                // with
                // // negative Y
                // // (forward)
                // .withVelocityY(-(joystick.getLeftX() < 0 ? Math.pow(joystick.getLeftX(), 2) :
                // -Math.pow(joystick.getLeftX(), 2)) * MaxSpeed *
                // (joystick.leftTrigger().getAsBoolean() ? .2 : 1)) // Drive left with negative
                // X (left)
                // .withRotationalRate(-joystick.getRightX() * MaxAngularRate *
                // (joystick.leftTrigger().getAsBoolean() ? .2 : 1)) // Drive counterclockwise
                // with
                // // negative X (left)
                // ));

                joystick2.y().whileTrue(drivetrain.applyRequest(() -> brake));
                // joystick.b().whileTrue(drivetrain.applyRequest(
                // () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
                // -joystick.getLeftX()))));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // reset the field-centric heading on left bumper press
                joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

                drivetrain.registerTelemetry(logger::telemeterize);
                configureButtons();

        }

        public void configureButtons() {
                System.out.println(curMode);
                joystick.rightBumper().and(() -> curMode == Mode.ALGAEMODE)
                                .whileTrue(new AlgaeIntakeCommand(m_IntakeSubsystem));
                joystick.rightBumper().and(() -> curMode == Mode.CORALMODE)
                                .whileTrue(new CoralIntakeCommand(m_IntakeSubsystem, m_ElevatorSubsystem));

                joystick.rightTrigger().and(() -> curMode == Mode.ALGAEMODE)
                                .whileTrue(new AlgaeOuttakeCommand(m_IntakeSubsystem, m_ElevatorSubsystem));
                joystick.rightTrigger().and(() -> curMode == Mode.CORALMODE)
                                .whileTrue(new CoralOuttakeCommand(m_IntakeSubsystem, m_ElevatorSubsystem,
                                                m_LedSubsystem));
                joystick.b().onTrue(m_ElevatorSubsystem.switchModeCommand()
                                .andThen(new ChangeElevatorCommand(m_IntakeSubsystem, m_ElevatorSubsystem, joystick)));
                joystick.leftBumper()
                                .whileTrue(new ChangeElevatorCommand(m_IntakeSubsystem, m_ElevatorSubsystem, joystick));
                joystick.povCenter().onFalse(new SwitchClimbCommand(m_ClimbSubsystem));
                joystick.x().onFalse(new ThirdAutoAlign(drivetrain, Optional.of(LeftOrRight.LEFT), joystick,
                                m_LedSubsystem));
                joystick.a().onFalse(new ThirdAutoAlign(drivetrain, Optional.of(LeftOrRight.RIGHT), joystick,
                                m_LedSubsystem));

                joystick2.a().onTrue(new ClimbResetCommand(m_ClimbSubsystem));
                joystick2.leftBumper()
                                .whileTrue(new BasicAutoAlign(drivetrain, Optional.of(LeftOrRight.LEFT), joystick));
                joystick2.rightBumper()
                                .whileTrue(new BasicAutoAlign(drivetrain, Optional.of(LeftOrRight.RIGHT), joystick));
                // joystick2.rightTrigger().whileTrue(new AutoAlignCommand(drivetrain,
                // m_VisionSubsystem, joystick, Optional.of(LeftOrRight.RIGHT)));
                // joystick2.y().whileTrue(m_ClimbSubsystem.manualClimbCommand(true));
                // joystick2.b().whileTrue(m_ClimbSubsystem.manualClimbCommand(false));

                // joystick.a().whileTrue(new AutoAlignCommand(drivetrain, m_VisionSubsystem));
                // joystick.leftTrigger().onFalse(m_ElevatorSubsystem.setZeroPositionCommand());

                // joystick.leftBumper().whileTrue(new AlgaeIntakeCommand(m_IntakeSubsystem));
                // joystick.b().onFalse(m_ElevatorSubsystem.switchModeCommand());
                // joystick.leftTrigger().whileTrue(m_IntakeSubsystem.algeaOutakeCommand());
                // joystick.rightTrigger().whileTrue(m_IntakeSubsystem.coralOutakeCommand());
                // joystick.y().onFalse(new ChangeElevatorCommand(m_IntakeSubsystem,
                // m_ElevatorSubsystem)); // this could
                // potentially
                // maybe be a
                // problem
                // joystick.rightBumper().whileTrue(new ClimbCommand(m_IntakeSubsystem));
        }

        public void configureNamedCommands() {
                NamedCommands.registerCommand("L3 height",
                                new AutoL3HeightCommand(m_ElevatorSubsystem, m_IntakeSubsystem).withTimeout(0.3));
                NamedCommands.registerCommand("L2 height",
                                m_ElevatorSubsystem.setCoralHeightCommand(2)
                                                .alongWith(m_IntakeSubsystem.setPivotCommand()));
                NamedCommands.registerCommand("L1 height", m_ElevatorSubsystem.setCoralHeightCommand(1)
                                .alongWith(m_IntakeSubsystem.setPivotCommand(IntakeConstants.kL1ScoringAngle)));

                NamedCommands.registerCommand("Coral Outtake",
                                new CoralOuttakeCommand(m_IntakeSubsystem, m_ElevatorSubsystem, m_LedSubsystem)
                                                .withTimeout(.4));
                NamedCommands.registerCommand("Coral Intake",
                                new AutoCoralIntakeCommand(m_ElevatorSubsystem, m_IntakeSubsystem).withTimeout(.8));
                // NamedCommands.registerCommand("Intake Position",
                // m_ElevatorSubsystem.setIntakeHeightCommand().alongWith(m_IntakeSubsystem.setPivotCommand(IntakeConstants.kCoralIntakeAngle)));
                NamedCommands.registerCommand("Zero Elevator", m_ElevatorSubsystem.setCoralHeightCommand(0));
                NamedCommands.registerCommand("Passive Intake",
                                new AutoPassiveIntakeCommand(m_IntakeSubsystem).withTimeout(0.01));
                NamedCommands.registerCommand("Auto Align Left",
                                new ThirdAutoAlign(drivetrain, Optional.of(LeftOrRight.LEFT), joystick, m_LedSubsystem)
                                                .withTimeout(1.5));
                NamedCommands.registerCommand("Auto Align Right",
                                new ThirdAutoAlign(drivetrain, Optional.of(LeftOrRight.RIGHT), joystick, m_LedSubsystem)
                                                .withTimeout(1.5));
                NamedCommands.registerCommand("Auto Align Center",
                                new ThirdAutoAlign(drivetrain, Optional.empty(), joystick, m_LedSubsystem)
                                                .withTimeout(1.5));
                NamedCommands.registerCommand("Descore Algae High Height",
                                m_ElevatorSubsystem.setDescoreHighHeightCommand().alongWith(m_IntakeSubsystem
                                                .setPivotCommand(IntakeConstants.kAlgaeHighIntakeAngle)));
                NamedCommands.registerCommand("Descore Algae Low Height",
                                m_ElevatorSubsystem.setDescoreLowHeightCommand().alongWith(m_IntakeSubsystem
                                                .setPivotCommand(IntakeConstants.kAlgaeLowIntakeAngle)));
                NamedCommands.registerCommand("Descore High Algae",
                                m_IntakeSubsystem.algaeHighOutakeCommand().withTimeout(.01));
                NamedCommands.registerCommand("Descore Low Algae",
                                m_IntakeSubsystem.algaeLowOutakeCommand().withTimeout(.01));
                // NamedCommands.registerCommand("Intake",
                // m_IntakeSubsystem.coralIntakeCommand().withTimeout(1));
                // NamedCommands.registerCommand("Outtake",
                // m_IntakeSubsystem.coralOutakeCommand().withTimeout(.5));
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
