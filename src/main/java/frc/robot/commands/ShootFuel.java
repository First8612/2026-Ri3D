package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestShooter;

public class ShootFuel extends Command {
    private TestShooter shooter;

    public ShootFuel(TestShooter shooter) {
        super();
        this.shooter = shooter;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.enableAiming();
        shooter.inFeed();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFeed();
    }
}
