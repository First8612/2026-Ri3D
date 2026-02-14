package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.AimData;
import frc.robot.utils.TargetTracker;

public class TestShooter extends SubsystemBase{
    static AngularVelocityUnit RPS = Units.RotationsPerSecond;
    ShooterFlywheel flywheel = new ShooterFlywheel(new TalonFX(41));
    TalonFX hoodMotor = new TalonFX(42);
    TalonFX feedMotor = new TalonFX(43);
    Boolean isAiming = false;
    double feedDutyCycle = 0;
    double hoodGoal = 0.2; //number to get to when pressing button
    int hoodGoalIdx = 1;
    double[] hoodPresets = {0, 0.2, 0.4, 0.6, 0.8};
    //For aiming (pseudocode)
    //List of limelight distances + hood poses + shooter speed
    AimData[] shootCalc = { new AimData(0.0, 0.0, 49.0, 1.0),
                            new AimData(1.2, 0.0, 49.0, 1.0),
                            new AimData(4.2, 0.2, 52.5, 1.0)};

    double currHoodGoal = 0; //number used w/ PID
    private TargetTracker targetTracker;

    public TestShooter(TargetTracker targetTracker) {
        super();
        this.targetTracker = targetTracker;
        hoodMotor.getConfigurator().apply(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive));
        hoodMotor.getConfigurator().apply(new CurrentLimitsConfigs()
            .withStatorCurrentLimit(12)
            .withStatorCurrentLimitEnable(true));
        hoodMotor.getConfigurator().apply(new Slot0Configs()
            .withKP(10)
            .withKI(0.1)
            .withKD(0));

        SmartDashboard.putData("Shooter/hoodMotor", hoodMotor);
        SmartDashboard.putData("Shooter/feedMotor", feedMotor);
    }
    
    public void inFeed() {
        feedDutyCycle = 0.5;
    }

    public void backFeed() {
        feedDutyCycle = -0.5;
    }

    public void stopFeed() {
        feedDutyCycle = 0;
    }

    public void stop() {
        feedDutyCycle = 0;
        isAiming = false;
    }

    public void enableAiming() {
        isAiming = true;
    }

    public boolean readyToShoot() {
        return readyToShoot(true);
    }

    public boolean readyToShoot(boolean refreshSignal) {
        return flywheel.getIsReady(refreshSignal) && hoodReady(refreshSignal);
    }

    private Boolean hoodReady(boolean refreshSignal) {
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

    public void warmUp() {
        flywheel.setTargetVelocity(Units.RotationsPerSecond.of(50));
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
                System.out.println("Hood zero finished");
            })
        );

        command.addRequirements(this);
            
        return command;
    }

    @Override
    public void periodic() {
        currHoodGoal = 0;
        var flywheelSpeedGoal = RPS.of(0);
        if (isAiming) {
            var distance = targetTracker.getRobotToTargetTranslation().getNorm();
            var lerpAmount = 0.0;
            AimData setAmounts = shootCalc[0];
            boolean hasAimed = false;
            // TODO: fancy math/algorithm here
            for(var i = 1; i < shootCalc.length; i++) {
                if(distance < shootCalc[i].getDist() && !hasAimed) {
                    lerpAmount = (distance - shootCalc[i - 1].getDist()) / (shootCalc[i].getDist() - shootCalc[i - 1].getDist());
                    setAmounts = shootCalc[i - 1].lerp(shootCalc[i], lerpAmount);
                    hasAimed = true;
                }
            }
            // TODO: change aiming from hardcoded to calculated
            // currHoodGoal = setAmounts.getHood();
            // flywheelSpeedGoal = setAmounts.getSpeed();
            currHoodGoal = hoodGoal;
            flywheelSpeedGoal = RPS.of(52.5); // max 100rps
        }

        hoodMotor.setControl(new PositionVoltage(0).withSlot(0).withPosition(currHoodGoal));

        flywheel.setTargetVelocity(flywheelSpeedGoal);

        if (readyToShoot()) {
            feedMotor.set(feedDutyCycle);
        } else {
            feedMotor.set(0);
        }
        
        SmartDashboard.putNumber("Shooter/Distance", targetTracker.getRobotToTargetTranslation().getNorm());
        SmartDashboard.putNumber("Shooter/HoodPreset", hoodGoal);
        SmartDashboard.putBoolean("Shooter/isAiming", isAiming);
        SmartDashboard.putBoolean("Shooter/hoodReady", hoodReady(false));
        SmartDashboard.putBoolean("Shooter/readyToShoot", readyToShoot(false));

        flywheel.periodic();
    }
}
