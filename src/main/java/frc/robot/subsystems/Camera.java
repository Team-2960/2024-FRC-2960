package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Camera extends SubsystemBase {
    private static Camera vision = null;

    PhotonCamera camera = new PhotonCamera("Camera_Module_v1");

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    Transform3d robotToCamera = new Transform3d(
            new Translation3d(Constants.cameraToRobotX, Constants.cameraToRobotY, Constants.cameraToRobotHeight),
            new Rotation3d(Constants.cameraRoll, Constants.cameraPitch, Constants.cameraYaw));

    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCamera);
            
    double lastTimeStamp = 0;

    @Override
    public void periodic() {
        var estPoseUpdate = photonPoseEstimator.update();
        SmartDashboard.putBoolean("IsPresent", estPoseUpdate.isPresent());
        SmartDashboard.putNumber("thenumberzero", 0);

        if (estPoseUpdate.isPresent() && lastTimeStamp < estPoseUpdate.get().timestampSeconds){
                var poseUpdate = estPoseUpdate.get();
                double ts = poseUpdate.timestampSeconds;
                if (lastTimeStamp < ts) {
                Pose3d pose3d = poseUpdate.estimatedPose;
                Pose2d pose2d = pose3d.toPose2d();

                Drive.get_instance().setVisionPose(pose2d, ts);
        }
    }
    

    public static Camera get_instance() {
        if (vision == null) {
            vision = new Camera();
        }

        return vision;
    }

}
