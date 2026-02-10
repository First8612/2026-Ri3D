package frc.robot.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.TargetTracker;

public class WaitForReadyToShoot extends Command {
    private Debouncer readyDebounce = new Debouncer(.5, DebounceType.kRising);
    private Shooter shooter;
    private TargetTracker targetTracker;

    public WaitForReadyToShoot(Shooter shooter, TargetTracker targetTracker) {
        super();
        this.shooter = shooter;
        this.targetTracker = targetTracker;
    }

    @Override
    public boolean isFinished() {
        var yawIsReady = Math.abs(targetTracker.getRobotToTargetRotation().getDegrees()) < 5;
        var shooterIsReady = shooter.readyToShoot();

        return readyDebounce.calculate(
            yawIsReady && shooterIsReady
        );
    }
}
