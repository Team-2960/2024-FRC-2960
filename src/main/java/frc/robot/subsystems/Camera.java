package frc.robot.subsystems;

import java.util.function.Predicate;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;

public class Camera extends SubsystemBase {
    private static Camera vision = null;

    private PhotonCamera camera;

    private AprilTagFieldLayout aprilTagFieldLayout;

    private PhotonPoseEstimator photonPoseEstimator;

    private Pose2d lastPose;
    private double lastTimeStamp;

    private GenericEntry sb_PoseX;
    private GenericEntry sb_PoseY;
    private GenericEntry sb_PoseR;
    private GenericEntry sb_lastTimestamp;
    private GenericEntry sb_lastUpdatePeriod;

    /**
     * Constructor
     */
    private Camera() {
        // Initialize Camera
        camera = new PhotonCamera("Camera_Module_v1");

        // Get apriltag layout
        aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

        // Initialize camera pose estimator
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, Constants.robotToCamera);

        // Initialize class variables
        lastPose = new Pose2d();
        lastTimeStamp = 0;

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status")
                .getLayout("Camera", BuiltInLayouts.kList)
                .withSize(1, 4);
        sb_PoseX = layout.add("Pose X", 0).getEntry();
        sb_PoseY = layout.add("Pose Y", 0).getEntry();
        sb_PoseR = layout.add("Pose R", 0).getEntry();
        sb_lastTimestamp = layout.add("Last Timestamp", lastTimeStamp).getEntry();
        sb_lastUpdatePeriod = layout.add("Time Since Last Update", 0).getEntry();

    }

    /**
     * Periodically checks the camera for updates
     */
    @Override
    public void periodic() {
        updatePose();
        updateUI();
    }

    /**
     * Retrieves camera pose estimation updates
     */
    private void updatePose() {
        var estPoseUpdate = photonPoseEstimator.update();
        var resultUpdate = camera.getLatestResult();
        // Check if an AprilTag is visible
        if (estPoseUpdate.isPresent()) {
            // Retrieve pose update
            var poseUpdate = estPoseUpdate.get();
            var result = resultUpdate.getBestTarget();

            if (resultUpdate.hasTargets() && Math.toDegrees(Math.abs(result.getBestCameraToTarget().getRotation().getZ())) > 172) {
                double ts = poseUpdate.timestampSeconds;
                //double angleFromAprilTag = photonPoseEstimator.get
                //if (resultUpdate.hasTargets() && Math.toDegrees((Math.abs(result.getBestCameraToTarget().getRotation().getZ()))) > 165) { TODO
                lastPose = poseUpdate.estimatedPose.toPose2d(); 
                    //.transformBy(Constants.fieldCenterOffset);
                    //if (poseUpdate.estimatedPose.toPose2d().getRotation().getDegrees() > 0)

                        lastPose = new Pose2d(new Translation2d(lastPose.getX(), lastPose.getY()),
                                lastPose.getRotation());

                    // Update drivetrain pose estimation
                    Drive.getInstance().setVisionPose(lastPose, ts);

                    // Update last timestamp
                    lastTimeStamp = ts;

                    // If angle to April Tag < 30 degrees, ignore April Tag

                }

            }

            // Check if the camera has a new value

        }
    

    /**
     * Updates Shuffleboard
     */
    private void updateUI() {
        sb_PoseX.setDouble(lastPose.getX());
        sb_PoseY.setDouble(lastPose.getY());
        sb_PoseR.setDouble(lastPose.getRotation().getDegrees());
        sb_lastTimestamp.setDouble(lastTimeStamp);
        sb_lastUpdatePeriod.setDouble(Timer.getFPGATimestamp() - lastTimeStamp);

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
