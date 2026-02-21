package frc.robot.subsystems;

import java.lang.StackWalker.Option;
import java.util.Optional;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.controls.CoastOut;
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
import frc.robot.utils.AimData;
import frc.robot.utils.TargetTracker;

public class TestShooter extends SubsystemBase{
    TalonFX shootMotor = new TalonFX(41);
    TalonFX hoodMotor = new TalonFX(42);
    TalonFX feedMotor = new TalonFX(43);
    Boolean isAiming = false;
    Optional<Double> aimingDistOveride = Optional.empty();
    double feedDutyCycle = 0;
    double flywheelSpeedGoal = 0;
    double hoodGoal = 0.2; //number to get to when pressing button
    int hoodGoalIdx = 1;
    double[] hoodPresets = {0, 0.2, 0.4, 0.6, 0.8};
    //For aiming (pseudocode)
    //List of limelight distances + hood poses + shooter speed
    AimData[] shootCalc = { new AimData(0.0, 0.0, 49.0, 1.0),
                            new AimData(1.2, 0.0, 49.0, 1.0),
                            new AimData(2.7, 0.1, 46.0, 1.0),
                            new AimData(4.2, 0.2, 52.5, 1.0),
                            new AimData(5.6, 0.3, 58, 1.0),
                            new AimData(200, 0.3, 58, 1.0)};

    double currHoodGoal = 0; //number used w/ PID
    private final Debouncer flywheelReadyDebounce = new Debouncer(0.07, DebounceType.kRising);
    private TargetTracker targetTracker;

    public TestShooter(TargetTracker targetTracker) {
        super();
        this.targetTracker = targetTracker;
        var mConfig = new MotorOutputConfigs();
        mConfig.NeutralMode = NeutralModeValue.Brake;
        mConfig.Inverted = InvertedValue.Clockwise_Positive;
        hoodMotor.getConfigurator().apply(mConfig);
        hoodMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(12).withStatorCurrentLimitEnable(true));

        
        feedMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(40).withStatorCurrentLimitEnable(false));
        // shootMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(80).withStatorCurrentLimitEnable(true));

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
                .withKV(.24) // this seems right
                .withKP(0)
                .withKI(0)
                
        );
        shootMotor.getConfigurator().apply(
            new Slot1Configs()
                .withKV(.13)
                .withKP(0)
                .withKI(0.1)
        );

        setDefaultCommand(Commands.runOnce(this::stop, this));
        SmartDashboard.putData("Shooter/shootMotor", shootMotor);
        SmartDashboard.putData("Shooter/feedMotor", feedMotor);
    }
    
    public void warmup() {
    }
    
    public void inFeed() {
        feedDutyCycle = 0.25;
    }

    public void backFeed() {
        feedDutyCycle = -0.25;
    }

    public void stopFeed() {
        feedDutyCycle = 0;
    }

    public void stop() {
        feedDutyCycle = 0;
        isAiming = false;
        aimingDistOveride = Optional.empty();
    }

    public void enableAiming() {
        isAiming = true;
        aimingDistOveride = Optional.empty();
    }

    public void enableAiming(double distOverride) {
        isAiming = true;
        aimingDistOveride = Optional.of(distOverride);
    }


    public boolean readyToShoot() {
        return flywheelReady() && hoodReady();
    }

    private Boolean flywheelReady() {
        if (flywheelSpeedGoal == 0) return false;

        return flywheelReadyDebounce.calculate(
            Math.abs(Math.abs(flywheelSpeedGoal - shootMotor.getVelocity().getValueAsDouble())) < 2.5
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

            if (aimingDistOveride.isPresent()) {
                distance = aimingDistOveride.get();
            }


            var lerpAmount = 0.0;
            AimData setAmounts = shootCalc[0];
            boolean hasAimed = false;
            // TODO: fancy math/algorithm here (leading?)
            for(var i = 1; i < shootCalc.length; i++) {
                if(distance < shootCalc[i].getDist() && !hasAimed) {
                    lerpAmount = (distance - shootCalc[i - 1].getDist()) / (shootCalc[i].getDist() - shootCalc[i - 1].getDist());
                    setAmounts = shootCalc[i - 1].lerp(shootCalc[i], lerpAmount);
                    hasAimed = true;
                }
            }
            // TODO: change aiming from hardcoded to calculated
            currHoodGoal = setAmounts.getHood();
            flywheelSpeedGoal = setAmounts.getSpeed();
            SmartDashboard.putNumber("Shooter/speedGoal", flywheelSpeedGoal);
            // currHoodGoal = hoodGoal;
            // flywheelSpeedGoal = 52.5; // max 100rps
        }

        hoodMotor.setControl(new PositionVoltage(0).withSlot(0).withPosition(currHoodGoal));
        if(flywheelSpeedGoal == 0)
        {
            shootMotor.setControl(new CoastOut());
        }
        else if(flywheelSpeedGoal - shootMotor.getVelocity().getValueAsDouble() < flywheelSpeedGoal * 0.1) {
            //At speed
            shootMotor.setControl(new VelocityVoltage(flywheelSpeedGoal).withSlot(1));
            SmartDashboard.putNumber("Shooter/flywheelSlot", 1);
        }

        else {
            //Speeding up
            shootMotor.setControl(new DutyCycleOut(1).withEnableFOC(true));
            // shootMotor.setControl(new VelocityVoltage(flywheelSpeedGoal).withSlot(0));
            SmartDashboard.putNumber("Shooter/flywheelSlot", 0);
        }

        if (flywheelReady()) {
            feedMotor.set(feedDutyCycle);
        } else {
            feedMotor.set(0);
        }
        

        SmartDashboard.putNumber("Shooter/Distance", targetTracker.getRobotToTargetTranslation().getNorm());
        SmartDashboard.putNumber("Shooter/HoodPreset", hoodGoal);
        SmartDashboard.putNumber("Shooter/ShootSpeed", shootMotor.getVelocity().getValueAsDouble());
        SmartDashboard.putBoolean("Shooter/isAiming", isAiming);
        SmartDashboard.putNumber("Shooter/hoodActual", hoodMotor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Shooter/hoodReady", hoodReady());
        SmartDashboard.putBoolean("Shooter/flywheelReady", flywheelReady());
        SmartDashboard.putBoolean("Shooter/readyToShoot", readyToShoot());
        SmartDashboard.putNumber("Shooter/feedSet", feedMotor.get());
        
    }
}
