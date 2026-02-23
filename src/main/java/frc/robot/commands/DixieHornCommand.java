package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DixieHornCommand extends Command {
    private static final Note[] DIXIE_HORN_NOTES = {
        // gemini
        new Note(392.00, 1.0/4.0),   // G4 - Quarter
        new Note(329.63, 1.0/4.0),   // E4 - Quarter
        new Note(261.63, 1.0/8.0),   // C4 - Eighth
        new Note(261.63, 1.0/8.0),   // C4 - Eighth
        new Note(261.63, 1.0/4.0),   // C4 - Quarter
        new Note(293.66, 1.0/8.0),   // D4 - Eighth
        new Note(329.63, 1.0/8.0),   // E4 - Eighth
        new Note(349.23, 1.0/8.0),   // F4 - Eighth
        new Note(392.00, 1.0/8.0),   // G4 - Eighth
        new Note(392.00, 1.0/8.0),   // G4 - Eighth
        new Note(392.00, 1.0/8.0),   // G4 - Eighth
        new Note(329.63, 1.0/2.0)    // E4 - Half
    };
    private static final Time beatDuration = Units.Seconds.of(1.25);
    private static List<TalonFX> motorControllers = new ArrayList<TalonFX>();
    private static List<SubsystemBase> subsystems = new ArrayList<SubsystemBase>();

    public static void enrollSubsystemMotors(SubsystemBase subsystem, TalonFX... motors) {
        // Add motors to motorControllers list
        motorControllers.addAll(Arrays.asList(motors));

        // Add subsystem to subsystems list if not already present
        if (!subsystems.contains(subsystem)) {
            subsystems.add(subsystem);
        }
    }

    private final Timer noteTimer = new Timer();
    private int noteIndex = -1;

    @Override
    public void initialize() {
        noteTimer.restart();
        noteIndex = 1;
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("DixieHorn/noteIndex", noteIndex);
        
        // if current note is outside of array bounds, return
        if (noteIndex < 0 || noteIndex >= DIXIE_HORN_NOTES.length) {
            setControlOnAll(new MusicTone(0));
            return;
        }

        var currentNote = DIXIE_HORN_NOTES[noteIndex];
        setControlOnAll(new MusicTone(currentNote.note));

        if (noteTimer.hasElapsed(beatDuration.times(currentNote.beats))) {
            // advance to next note
            noteIndex++;
            noteTimer.restart();
        }

    }

    @Override
    public boolean isFinished() {
        return noteIndex >= DIXIE_HORN_NOTES.length;
    }

    @Override
    public void end(boolean interrupted) {
        setControlOnAll(new MusicTone(0));
        noteIndex = -1;
        noteTimer.stop();
        noteTimer.reset();
    }

    private void setControlOnAll(MusicTone tone) {
        for (TalonFX motor : motorControllers) {
            motor.setControl(tone);
        }
    }

    static class Note {
        public double note;
        public double beats;

        public Note(double note, double beats) {
            super();
            this.note = note;
            this.beats = beats;
        }
    }
}
