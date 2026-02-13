package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.TargetTracker;

public class TestShooter extends SubsystemBase{
    TalonFX shootMotor = new TalonFX(41);
    TalonFX hoodMotor = new TalonFX(42);
    TalonFX feedMotor = new TalonFX(43);
    Boolean isAiming = false;
    double flywheelSpeedGoal = 0;
    double hoodGoal = 0.6; //number to get to when pressing button
    int hoodGoalIdx = 3;
    double[] hoodPresets = {0, 0.2, 0.4, 0.6, 0.8};
    double currHoodGoal = 0; //number used w/ PID
    private final Debouncer flywheelReadyDebounce = new Debouncer(0.5, DebounceType.kRising);
    private TargetTracker targetTracker;

    public TestShooter(TargetTracker targetTracker) {
        super();
        this.targetTracker = targetTracker;
        var mConfig = new MotorOutputConfigs();
        mConfig.NeutralMode = NeutralModeValue.Brake;
        mConfig.Inverted = InvertedValue.Clockwise_Positive;
        hoodMotor.getConfigurator().apply(mConfig);
        hoodMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(12).withStatorCurrentLimitEnable(true));

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = 10;
        slot0Configs.kI = 0.1; // no output for integrated error
        slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output
        hoodMotor.getConfigurator().apply(slot0Configs);

        shootMotor.getConfigurator().apply(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast)
                .withPeakReverseDutyCycle(0)
        );
        shootMotor.getConfigurator().apply(
            new Slot0Configs()
                // TODO: still iterating on what these values should be
                .withKV(.12) // this seems right
                .withKP(0)
                .withKI(.1)
        );
    }
    
    public void warmup() {
    }
    
    public void inFeed() {
        feedMotor.set(0.5);
    }

    public void backFeed() {
        feedMotor.set(-0.5);
    }

    public void stopFeed() {
        feedMotor.set(0);
    }

    public void stop() {
        feedMotor.set(0);
        isAiming = false;
    }

    public void enableAiming() {
        isAiming = true;
    }

    public boolean readyToShoot() {
        return flywheelReady() && hoodReady();
    }

    private Boolean flywheelReady() {
        if (flywheelSpeedGoal == 0) return false;

        return flywheelReadyDebounce.calculate(
            Math.abs(Math.abs(flywheelSpeedGoal - shootMotor.getVelocity().getValueAsDouble())) < 5
        );
    }

    private Boolean hoodReady() {
        return true;
    }

    public void cycleHoodRight() {
        hoodGoalIdx = (hoodGoalIdx + hoodPresets.length + 1) % hoodPresets.length;
        hoodGoal = hoodPresets[hoodGoalIdx];
    }

    public void cycleHoodLeft() {
        hoodGoalIdx = (hoodGoalIdx + hoodPresets.length - 1) % hoodPresets.length;
        hoodGoal = hoodPresets[hoodGoalIdx];
    }

    public void spinUp(double speed) {
        flywheelSpeedGoal = speed;
    }

    public Command getZeroCommand() {
        var command = Commands.sequence(
            Commands.deadline(
                Commands.waitUntil(() -> Math.abs(hoodMotor.getStatorCurrent().getValueAsDouble()) > 6),
                Commands.run(() -> hoodMotor.setControl(new DutyCycleOut(-0.1)))
            ),
            Commands.runOnce(() -> {
                hoodMotor.setPosition(-0.01);
                hoodMotor.setControl(new DutyCycleOut(0));
                hoodMotor.getConfigurator().apply(new HardwareLimitSwitchConfigs()
                    .withForwardLimitAutosetPositionEnable(true)
                    .withForwardLimitAutosetPositionValue(1.18));
                System.out.println("reset finished");
            })
        );

        command.addRequirements(this);
            
        return command;
    }

    @Override
    public void periodic() {
        currHoodGoal = 0;
        flywheelSpeedGoal = 0;
        if (isAiming) {
            var distance = targetTracker.getRobotToTargetTranslation().getNorm();
            // TODO: fancy math/algorithm here

            currHoodGoal = hoodGoal;
            flywheelSpeedGoal = 50; // max 100rps
        }

        hoodMotor.setControl(new PositionVoltage(0).withSlot(0).withPosition(currHoodGoal));
        shootMotor.setControl(new VelocityVoltage(flywheelSpeedGoal).withSlot(0));

        SmartDashboard.putBoolean("Shooter/isAiming", isAiming);
        SmartDashboard.putNumber("Shooter/shootActual", shootMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("Shooter/hoodReady", hoodReady());
        SmartDashboard.putBoolean("Shooter/flywheelReady", flywheelReady());
        SmartDashboard.putBoolean("Shooter/readyToShoot", readyToShoot());
        
    }
}
