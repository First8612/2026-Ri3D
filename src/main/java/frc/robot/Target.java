package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Target {
    private static final StringPublisher targetNameNT = NetworkTableInstance.getDefault().getStringTopic("Target/target").publish();
    private static final StructPublisher<Pose2d> targetNT = NetworkTableInstance.getDefault().getStructTopic("Target/targetPose", Pose2d.struct).publish();
    private static final DoublePublisher targetDistNT = NetworkTableInstance.getDefault().getDoubleTopic("Target/distance").publish();
    private static final DoublePublisher targetAngleRelativeNT = NetworkTableInstance.getDefault().getDoubleTopic("Target/angleRelative").publish();

    private static Field.ByAlliance currentAllianceField = Field.blueAlliance;
    private static Pose2d currentTarget = currentAllianceField.hub;

    public static Pose2d getPose() {
        return currentTarget;
    }

    public static Translation2d getTranslationFrom(Pose2d pose) {
        return currentTarget.getTranslation().minus(pose.getTranslation());
    }

    public static Rotation2d getDirectionFrom(Pose2d pose) {
        return getTranslationFrom(pose).getAngle();
    }

    public static void periodic(Pose2d robot) {
        var robotToTargetTranslation = getTranslationFrom(robot);

        currentAllianceField = DriverStation.getAlliance()
                .map(color -> color == Alliance.Red ? Field.redAlliance : Field.blueAlliance)
                .orElse(Field.blueAlliance);

        if (currentAllianceField.zone.contains(robot.getTranslation()))
        {
            currentTarget = currentAllianceField.hub;
            targetNameNT.set(currentAllianceField.name + " Hub");
        }
        else {
            currentTarget = currentAllianceField.passingTargetRight;
            targetNameNT.set(currentAllianceField.name + " Passing Target Right");
        }

        targetNT.set(currentTarget);
        targetDistNT.set(robotToTargetTranslation.getNorm());
        targetAngleRelativeNT.set(robotToTargetTranslation.getAngle().minus(robot.getRotation()).getDegrees());
    }
}
