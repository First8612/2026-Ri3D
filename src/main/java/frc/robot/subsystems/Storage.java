package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PlaceholderCode.*;

public class Storage extends SubsystemBase{
    private FakeMotor conveyMotor = new FakeMotor(44);
    private FakeSensor ballSensor = new FakeSensor(60);
    

    public Storage() {
        super();
        //More Init

        setDefaultCommand(Commands.runOnce(this::conveyStop, this));
    }

    public void conveyIn() {
        conveyMotor.set(0.5);
    }

    public void conveyOut() {
        conveyMotor.set(-0.5);
    }

    public void conveyStop() {
        conveyMotor.set(0);
    }

    public double getConveySpeed() {
        return conveyMotor.getSpeed();
    }

    public boolean hasFuel() {
        return ballSensor.getValue();
    }
}
