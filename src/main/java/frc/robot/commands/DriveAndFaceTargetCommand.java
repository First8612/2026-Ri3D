package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.TargetTracker;

public class DriveAndFaceTargetCommand extends Command {
    private final double maxSpeed = RobotContainer.MaxSpeed;
    private final CommandXboxController driverController;
    private final CommandSwerveDrivetrain drivetrain;
    private final TargetTracker targetTracker;

    private final SwerveRequest.FieldCentricFacingAngle driveFacingTarget = new SwerveRequest.FieldCentricFacingAngle()
        .withHeadingPID(10,0,0)
        .withDeadband(RobotContainer.MaxSpeed * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
        
        
    public DriveAndFaceTargetCommand(CommandXboxController driverController, CommandSwerveDrivetrain drivetrain, TargetTracker targetTracker) {
        super();
        this.driverController = driverController;
        this.drivetrain = drivetrain;
        this.targetTracker = targetTracker;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        var drive = driveFacingTarget
                .withVelocityX(-driverController.getLeftY() * maxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-driverController.getLeftX() * maxSpeed) // Drive left with negative X (left)
                .withTargetDirection(targetTracker.getRobotToTargetRotation());

        drivetrain.setControl(drive);
    }
}
