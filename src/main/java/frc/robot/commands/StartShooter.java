package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.FakeShooter;

public class StartShooter extends InstantCommand {
    public StartShooter(FakeShooter shooter) {
        super(shooter::warmup, shooter);
    }
}