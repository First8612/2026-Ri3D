// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DixieHornCommand;
import frc.robot.commands.DriveAndFaceTargetCommand;
import frc.robot.commands.ShootSequence;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TestShooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.FakeShooter;
import frc.robot.subsystems.Storage;
import edu.wpi.first.cameraserver.*;

import frc.robot.utils.TargetTracker;

public class RobotContainer {
    public final static double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public final static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystickDrive = new CommandXboxController(0);
    private final CommandXboxController joystickOperate = new CommandXboxController(1);
    
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final TargetTracker targetTracker = new TargetTracker(drivetrain);
    private final FakeShooter shooter = new FakeShooter(targetTracker);
    private final Storage storage = new Storage();
    private final TestShooter testShooter = new TestShooter(targetTracker);
    private final Vision vision = new Vision(drivetrain);

    private final DriveAndFaceTargetCommand driveAndFaceTarget = new DriveAndFaceTargetCommand(joystickDrive, drivetrain, targetTracker);
    private final ShootSequence shoot = new ShootSequence(testShooter, storage, targetTracker, joystickDrive, drivetrain, false);
    private final ShootSequence shootSimple = new ShootSequence(testShooter, storage, targetTracker, joystickDrive, drivetrain, true);

    Intake intake = new Intake();

    SendableChooser<Command> autonChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("ShootSequence", shootSimple);

        CameraServer.startAutomaticCapture();
        configureBindings();
        drivetrain.configureAutoBuilder();
        autonChooser = AutoBuilder.buildAutoChooser("RI3D Auto");

        SmartDashboard.putData("Auto Path", autonChooser);
        RobotModeTriggers.autonomous().onTrue(testShooter.getZeroCommand());
        RobotModeTriggers.teleop().onTrue(testShooter.getZeroCommand());

        Field.writeOnceToNT();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystickDrive.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystickDrive.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystickDrive.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        Supplier<Double> getIntakeSpeed = () -> {
            var speed = joystickOperate.getRightTriggerAxis()-joystickOperate.getLeftTriggerAxis();
            System.out.println(speed);
            return speed;
        };
        //TODO: Make intake more intuitive
        joystickOperate.rightTrigger(0.1).or(joystickOperate.leftTrigger(0.1)).whileTrue(new RunCommand(() -> intake.setSpeedRaw(getIntakeSpeed.get()), intake));
        joystickOperate.rightTrigger(0.1).and(joystickOperate.leftTrigger(0.1)).whileFalse(new RunCommand(() -> intake.stop(), intake));
        //  joystick.b().onTrue(shoot);
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));
        joystickDrive.rightBumper().whileTrue(driveAndFaceTarget);

        joystickOperate.a().whileTrue(shoot);
        joystickOperate.b().whileTrue(shootSimple);
        joystickOperate.rightBumper().whileTrue(new DixieHornCommand());


        joystickOperate.y().whileTrue(new RunCommand(() -> testShooter.inFeed()));
        joystickOperate.x().whileTrue(Commands.startEnd(testShooter::enableAiming, testShooter::stop, testShooter));
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystickDrive.back().and(joystickDrive.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystickDrive.back().and(joystickDrive.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystickDrive.start().and(joystickDrive.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystickDrive.start().and(joystickDrive.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystickOperate.povLeft().onTrue(new InstantCommand(() -> testShooter.cycleHoodLeft()));
        joystickOperate.povRight().onTrue(new InstantCommand(() -> testShooter.cycleHoodRight()));
        // Reset the field-centric heading on left bumper press.
        joystickDrive.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        CameraServer.startAutomaticCapture();
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //     // Reset our field centric heading to match the robot
        //     // facing away from our alliance station wall (0 deg).
        //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        //     // Then slowly drive forward (away from us) for 5 seconds.
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(0.5)
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     )
        //     .withTimeout(5.0),
        //     // Finally idle for the rest of auton
        //     drivetrain.applyRequest(() -> idle)
        // );
        var auton = autonChooser.getSelected();
        auton.addRequirements(drivetrain);
        return auton;
    }

    public void robotPeriodic() {
        Target.periodic(drivetrain.getState().Pose);
        targetTracker.periodic();
        vision.periodic();
    }
}