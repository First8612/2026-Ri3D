package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Storage;
import frc.robot.utils.TargetTracker;

// shoot sequence without controlling drivetrain.
public class ShootSequence extends ParallelCommandGroup {
    public ShootSequence(
        Shooter shooter,
        Storage storage,
        TargetTracker targetTracker,
        CommandXboxController driverController,
        CommandSwerveDrivetrain drivetrain
    ) {
        super(
            // retain drive control during
            new DriveAndFaceTargetCommand(driverController, drivetrain, targetTracker),

            // main sequence
            Commands.sequence(
                // warm up the shooter, and enable auto-aiming.
                // driveAndFaceTargetCommand will continue to keep robot pointed at target
                Commands.runOnce(setStatus("Warmup")),
                Commands.runOnce(shooter::warmup, shooter),
                Commands.runOnce(shooter::enableAiming, shooter),

                Commands.runOnce(setStatus("WaitingForReady")),
                new WaitForReadyToShoot(shooter, targetTracker),

                // shoot fuel until storage is empty + 1sec.
                Commands.runOnce(setStatus("Shooting")),
                Commands.deadline(
                    Commands.sequence(
                        new StorageConveyUntilEmpty(storage),
                        Commands.waitSeconds(1)
                    ),
                    new ShootFuel(shooter)
                ),

                // stop everything
                Commands.runOnce(setStatus("Stopping")),
                Commands.runOnce(shooter::stop, shooter),
                Commands.runOnce(storage::conveyStop, storage),

                Commands.runOnce(setStatus("Finished"))
            )
        );
    }

    private static Runnable setStatus(String status) {
        return () -> {
            SmartDashboard.putString("ShootSequence", status);
        };
    }
}