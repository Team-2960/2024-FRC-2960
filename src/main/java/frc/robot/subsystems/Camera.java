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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Camera extends SubsystemBase {
    private static Camera vision = null;

    private PhotonCamera camera;

    private AprilTagFieldLayout aprilTagFieldLayout;

    private Transform3d robotToCamera;

    private PhotonPoseEstimator photonPoseEstimator;
            
    private double lastTimeStamp;

    private Transform2d fieldCenterOffset = new Transform2d(8.270875, 4.105275, new Rotation2d()); // TODO Move to constants

    /**
     * Constructor
     */
    private Camera() {
        camera = new PhotonCamera("Camera_Module_v1");

        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        
        // TODO Initialize transform 3d in constants
        robotToCamera = new Transform3d(
                new Translation3d(Constants.cameraToRobotX, Constants.cameraToRobotY, Constants.cameraToRobotHeight),
                new Rotation3d(Constants.cameraRoll, Constants.cameraPitch, Constants.cameraYaw));  

        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCamera);
                
        lastTimeStamp = 0;
    }

    /**
     * Periodically checks the camera for updates
     */
    @Override
    public void periodic() {
        var estPoseUpdate = photonPoseEstimator.update();

        // TODO move to shuffle board
        SmartDashboard.putBoolean("IsPresent", estPoseUpdate.isPresent());
        SmartDashboard.putNumber("thenumberzero", 0);

        // Check if an AprilTag is visible
        if (estPoseUpdate.isPresent()) {
            // Retrieve pose update
            var poseUpdate = estPoseUpdate.get();

            // Check if the camera has a new value
            double ts = poseUpdate.timestampSeconds;
            if (lastTimeStamp < ts) {
                Pose2d pose_est = poseUpdate.estimatedPose.toPose2d().transformBy(fieldCenterOffset);

                // Update drivetrain pose estimation
                Drive.getInstance().setVisionPose(pose_est, ts);

                // Update last timestamp
                lastTimeStamp = ts;
            }
        }
    }
    
    /**
     * Static initializer
     */
    public static Camera getInstance() {
        if (vision == null) {
            vision = new Camera();
        }

        return vision;
    }

}
