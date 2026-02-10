package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShootFuel extends Command {
    private Shooter shooter;

    public ShootFuel(Shooter shooter) {
        super();
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        shooter.inFeed();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopFeed();
    }
}
