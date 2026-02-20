package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PlaceholderCode.FakeEncoder;
import frc.robot.PlaceholderCode.FakeMotor;

public class Intake extends SubsystemBase {
    TalonFX intakeMotor = new TalonFX(40);
    FakeMotor extendMotor = new FakeMotor(44);
    private FakeEncoder extendEncoder = new FakeEncoder(51);
    private double speed = 0;
    private String extendStatus = "Extended";
    private Timer extendTimer = new Timer();
    private PIDController extendPID = new PIDController(0, 0, 0);

    public Intake() {
        super();
        
        setDefaultCommand(Commands.runOnce(this::stop, this));
    }

    public void in() {
        speed = 0.75;
    }

    public void out() {
        speed = -0.75;
    }

    // intended for prototyping time
    public void setSpeedRaw(double speed) {
        this.speed = speed;
    }

    public void stop() {
        speed = 0;
    }

    public void extend() {
        extendStatus = "Extending";
        // extendPID.setSetpoint(0);
        extendTimer.restart();
    }

    public void retract() {
        extendStatus = "Retracting";
        // extendPID.setSetpoint(0);
        extendTimer.restart();
    }

    public Boolean isExtended() {
        return extendStatus == "Extended";
    }

    public Boolean isRetracted() {
        return extendStatus == "Retracted";
    }

    @Override
    public void periodic() {
        if (extendStatus == "Extending" && extendTimer.hasElapsed(2)) {
            extendStatus = "Extended";
            extendTimer.stop();
            extendTimer.reset();
        }
        else if (extendStatus == "Retracting" && extendTimer.hasElapsed(2)) {
            extendStatus = "Retracted";
            extendTimer.stop();
            extendTimer.reset();
        }

        if(isExtended()) {
            intakeMotor.set(speed);
        }else{
            intakeMotor.set(speed / 2);
        }

        double extendSpeed = extendPID.calculate(extendEncoder.getPosition());
        extendMotor.set(extendSpeed);

        SmartDashboard.putNumber("Intake/speed", speed);
        SmartDashboard.putString("Intake/extendStatus", extendStatus);
    }
}
