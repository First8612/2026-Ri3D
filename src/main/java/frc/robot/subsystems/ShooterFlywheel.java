package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterFlywheel {
    static AngularVelocityUnit RPS = Units.RotationsPerSecond;
    private TalonFX motor;
    private AngularVelocity targetVelocity = RPS.of(0);
    private Debouncer isReadyDebouncer = new Debouncer(0.25, DebounceType.kRising);

    public ShooterFlywheel(TalonFX motor) {
        super();
        this.motor = motor;
    }

    public void setTargetVelocity(AngularVelocity velocity) {
        motor.getConfigurator().apply(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)
        );

        motor.getConfigurator().apply(
            new Slot1Configs()
                .withKV(.13)
                .withKP(0)
                .withKI(0.1)
        );

        SmartDashboard.putData("Shooter/flywheel/motor", motor);
    }

    public boolean getIsReady() {
        return getIsReady(true);
    }

    public boolean getIsReady(boolean refreshSignal) {
        return isReadyDebouncer.calculate(
            getIsReadyUnfiltered(refreshSignal)
        );
    }

    private boolean getIsReadyUnfiltered(boolean refreshSignal) {
        var tolerance = RPS.of(2.5);
        if (targetVelocity.magnitude() == 0) return false;

        return motor.getVelocity(refreshSignal).isNear(targetVelocity, tolerance);
    }

    public void periodic() {
        var currentVelocity = motor.getVelocity().getValue();
        var cruiseModeTolerance = targetVelocity.times(0.1);

        if (targetVelocity.magnitude() == 0) {
            // coast mode
            motor.setControl(new CoastOut());
            SmartDashboard.putString("Shooter/flywheel/mode", "Coast");
        }
        else if (targetVelocity.minus(cruiseModeTolerance).gt(currentVelocity))
        {
            // accelerate mode
            motor.setControl(new DutyCycleOut(1));
            SmartDashboard.putString("Shooter/flywheel/mode", "Accelerate");
        }
        else
        {
            // cruise mode
            motor.setControl(new VelocityVoltage(targetVelocity).withSlot(1));
            SmartDashboard.putString("Shooter/flywheel/mode", "Cruise");
        }
        
        SmartDashboard.putBoolean("Shooter/flywheel/readyUnfiltered", getIsReadyUnfiltered(false));
        SmartDashboard.putBoolean("Shooter/flywheel/ready", getIsReady(false));
        SmartDashboard.putNumber("Shooter/flywheel/speedTarget", targetVelocity.magnitude());
    }
}
