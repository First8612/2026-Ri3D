package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Storage;

public class StorageConveyUntilEmpty extends Command {
    private Storage storage;

    public StorageConveyUntilEmpty(Storage storage) {
        super();
        this.storage = storage;
        addRequirements(storage);
    }

    @Override
    public void initialize() {
        storage.conveyIn();
    }

    @Override
    public void end(boolean interrupted) {
        storage.conveyStop();
    }

    @Override
    public boolean isFinished() {
        return !storage.hasFuel();
    }
}
