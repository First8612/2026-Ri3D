package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PlaceholderCode.*;
import frc.robot.utils.TargetTracker;


public class Shooter extends SubsystemBase{
    FakeMotor shootMotor = new FakeMotor(41);
    FakeMotor hoodMotor = new FakeMotor(42);
    FakeMotor feedMotor = new FakeMotor(43);
    FakeEncoder hoodEncoder = new FakeEncoder(50);
    Boolean isAiming = false;
    PIDController shooterPID = new PIDController(0, 0, 0);
    PIDController hoodPID = new PIDController(0, 0, 0);
    double motorSpeedGoal = 0;
    double hoodGoal = 0;
    private final Debouncer flywheelReadyDebounce = new Debouncer(0.5, DebounceType.kRising);
    private TargetTracker targetTracker;

    public Shooter(TargetTracker targetTracker) {
        super();
        this.targetTracker = targetTracker;

        SmartDashboard.putData("Shooter/shootController", shooterPID);
        SmartDashboard.putData("Shooter/hoodController", hoodPID);
    }
    
    public void warmup() {
        shooterPID.setSetpoint(100);
    }
    
    public void inFeed() {
        feedMotor.set(0.2);
    }

    public void backFeed() {
        feedMotor.set(-0.2);
    }

    public void stopFeed() {
        feedMotor.set(0);
    }

    public void stop() {
        feedMotor.set(0);
        shooterPID.setSetpoint(0);
        hoodPID.setSetpoint(0);
    }

    public void enableAiming() {
        isAiming = true;
        shooterPID.setSetpoint(100);
        hoodPID.setSetpoint(25);
    }

    public boolean readyToShoot() {
        return flywheelReady() && hoodReady();
    }

    private Boolean flywheelReady() {
        if (shooterPID.getSetpoint() == 0) return false;

        return flywheelReadyDebounce.calculate(
            Math.abs(shooterPID.getError()) < 5
        );
    }

    private Boolean hoodReady() {
        if (hoodPID.getSetpoint() == 0) return false;

        return Math.abs(hoodPID.getError()) < 1;
    }


    @Override
    public void periodic() {
        if (isAiming) {
            var distance = targetTracker.getRobotToTargetTranslation().getNorm();
            // TODO: fancy math/algorithm here

            shooterPID.setSetpoint(100);
            hoodPID.setSetpoint(25);
        }



        double motorSpeed = shooterPID.calculate(shootMotor.getSpeed());
        shootMotor.set(motorSpeed);
        double hoodSpeed = hoodPID.calculate(hoodEncoder.getPosition());
        hoodMotor.set(hoodSpeed);

        SmartDashboard.putNumber("Shooter/shootActual", shootMotor.getSpeed());
        SmartDashboard.putNumber("Shooter/hoodActual", hoodEncoder.getPosition());
        SmartDashboard.putBoolean("Shooter/hoodReady", hoodReady());
        SmartDashboard.putBoolean("Shooter/flywheelReady", flywheelReady());
        SmartDashboard.putBoolean("Shooter/readyToShoot", readyToShoot());
    }
}
