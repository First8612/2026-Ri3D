package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Storage;
import frc.robot.subsystems.TestShooter;
import frc.robot.utils.TargetTracker;

// shoot sequence without controlling drivetrain.
public class ShootSequence extends ParallelCommandGroup {
    public ShootSequence(
        TestShooter shooter,
        Storage storage,
        TargetTracker targetTracker,
        CommandXboxController driverController,
        CommandSwerveDrivetrain drivetrain,
        boolean unsmart
    ) {
        super(
            // retain drive control during
            new DriveAndFaceTargetCommand(driverController, drivetrain, targetTracker)
        );


        var aimCommand = Commands.runOnce(shooter::enableAiming, shooter);
        if (unsmart) {
            aimCommand = Commands.runOnce(() -> shooter.enableAiming(2.7), shooter);
        }

        var finishShootingDeadline =                 
            Commands.sequence(
                new StorageConveyUntilEmpty(storage),
                Commands.waitSeconds(1)
            );
        if (unsmart) {
            finishShootingDeadline = Commands.waitSeconds(10);
        }

        var mainSequence = new SequentialCommandGroup();
        mainSequence.addCommands(
            // warm up the shooter, and enable auto-aiming.
            // driveAndFaceTargetCommand will continue to keep robot pointed at target
            Commands.runOnce(setStatus("Start")),
            Commands.runOnce(shooter::warmup, shooter),
            aimCommand,

            Commands.runOnce(setStatus("WaitingForReady")),
            new WaitForReadyToShoot(shooter, targetTracker, unsmart),

            // shoot fuel until storage is empty + 1sec.
            Commands.runOnce(setStatus("Shooting")),
            Commands.deadline(
                finishShootingDeadline,
                new ShootFuel(shooter)
            ),

            Commands.runOnce(setStatus("Finished"))
        );
        this.addCommands(mainSequence);
    }

    private static Runnable setStatus(String status) {
        return () -> {
            SmartDashboard.putString("ShootSequence", status);
        };
    }
}