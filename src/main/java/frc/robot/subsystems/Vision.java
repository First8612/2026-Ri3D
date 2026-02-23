
package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.utils.LimelightHelpers;

public class Vision {

    private StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Poses/Pose", Pose2d.struct).publish();
    private StructPublisher<Pose2d> poseMT1Publisher = NetworkTableInstance.getDefault()
            .getStructTopic("Poses/Pose_WpiBlue", Pose2d.struct).publish();
    private StructPublisher<Pose2d> poseMT2Publisher = NetworkTableInstance.getDefault()
            .getStructTopic("Poses/Pose_MT2_WpiBlue", Pose2d.struct).publish();
    private CommandSwerveDrivetrain driveBase;

    public Vision(CommandSwerveDrivetrain drivebase) {

        super();

        driveBase = drivebase;
    }

    public void periodic() {
        var poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
        LimelightHelpers.SetRobotOrientation("limelight",
                driveBase.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        var poseEstimateMT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if(poseEstimateMT2.tagCount != 0){
            driveBase.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
            driveBase.addVisionMeasurement(poseEstimateMT2.pose, poseEstimateMT2.timestampSeconds);
        }
        

        var doRejectUpdate = false;
        if (poseEstimateMT2.tagCount == 0) {
            doRejectUpdate = true;
        }

        
        if (!doRejectUpdate) {
            driveBase.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
            driveBase.addVisionMeasurement(poseEstimateMT2.pose, poseEstimateMT2.timestampSeconds);
        }

        posePublisher.set(driveBase.getState().Pose);
        poseMT1Publisher.set(poseEstimate.pose);
        poseMT2Publisher.set(poseEstimateMT2.pose);
    }
}