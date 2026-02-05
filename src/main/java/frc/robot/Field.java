package frc.robot;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.measure.Distance;

public class Field {
    // referencing page 3 of
    // https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf
    private static final Distance fieldLength = Inches.of(651.22);
    private static final Distance fieldWidth = Inches.of(317.69);
    private static final Distance allianceZoneDepth = Inches.of(158.6);

    private static final Pose2d blueOrigin = new Pose2d();
    private static final Pose2d redOrigin = new Pose2d(fieldLength, fieldWidth, new Rotation2d(Degree.of(180)));

    // public
    public static final ByAlliance blueAlliance = new ByAlliance("Blue", blueOrigin, new Rotation2d());
    public static final ByAlliance redAlliance = new ByAlliance("Red", redOrigin, new Rotation2d(Degree.of(180)));


    public static class ByAlliance {
        public ByAlliance(String name, Pose2d origin, Rotation2d perspectiveRotation) {
            super();

            var noRotation = new Rotation2d();

            var hubFromOrigin = new Translation2d(Inches.of(182.11), Inches.of(158.84));

            this.name = name;
            hub = origin.plus(new Transform2d(hubFromOrigin, perspectiveRotation));
            zone = new Rectangle2d(origin, allianceZoneDepth, fieldWidth)
                .transformBy(new Transform2d(
                    new Translation2d(allianceZoneDepth.div(2), fieldWidth.div(2)),
                    noRotation
                ));

            var passingTargetYOffset = Inches.of(24);
            var passingTargetXOffset = Inches.of(24);

            passingTargetRight = origin.plus(
                new Transform2d(passingTargetXOffset, passingTargetYOffset, noRotation)
            );

            passingTargetLeft = origin.plus(
                new Transform2d(passingTargetXOffset, fieldWidth.minus(passingTargetYOffset), noRotation)
            );
        }

        public final String name;
        public final Pose2d hub;
        public final Rectangle2d zone;
        public final Pose2d passingTargetRight;
        public final Pose2d passingTargetLeft;
    }

    public static void writeOnceToNT() {
        var nt = NetworkTableInstance.getDefault();
        nt.getStructTopic("Field/blue/hub", Pose2d.struct).publish().set(blueAlliance.hub);
        nt.getStructTopic("Field/blue/passingTargetRight", Pose2d.struct).publish().set(blueAlliance.passingTargetRight);
        nt.getStructTopic("Field/blue/passingTargetLeft", Pose2d.struct).publish().set(blueAlliance.passingTargetLeft);
        nt.getStructTopic("Field/red/hub", Pose2d.struct).publish().set(redAlliance.hub);
        nt.getStructTopic("Field/red/passingTargetRight", Pose2d.struct).publish().set(redAlliance.passingTargetRight);
        nt.getStructTopic("Field/red/passingTargetLeft", Pose2d.struct).publish().set(redAlliance.passingTargetLeft);
    }
}
