/**
 * Copyright 2024 Ryan Fitz-Gerald
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to 
 * deal in the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 */

 package frc.lib2960_photonvision;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Manages connection to a single PhotonVision AprilTag Pipeline
 */
public class AprilTagPipeline extends SubsystemBase {

    /**
     * Apriltag Pipeline Settings
     */
    public class Settings {
        public final String name;                       /**< Name of the pipeline */
        public final String camera_name;                /**< Camera name for the pipeline configured in PhotonVision */
        public final AprilTagFieldLayout field_layout;  /**< AprilTag Field Layout object */
        public final Transform3d robot_to_camera;       /**< Robot to Camera transformation */
        public final PoseStrategy pose_strategy;        /**< Pose Estimation Strategy */
        public final Rotation2d max_angle;              /**< Maximum angle between camera and target to accept */
        public final double max_dist;                   /**< Maximum acceptable target distance from camera in meters */
        public final Matrix<N3,N1> single_tag_std;      /**< Single tag standard deviation matrix */
        public final Matrix<N3,N1> multi_tag_std;       /**< Multi tag standard deviation matrix */

        // TODO Allow the standard deviation to increase based on distance

        /**
         * Constructor
         * @param   name                Name of the pipeline
         * @param   camera_name         Camera name for the pipeline as configured in PhotoVision
         * @param   field_layout        AprilTag Field Layout object
         * @param   robot_to_camera     Robot to Camera transformation
         * @param   pose_strategy       Pose Estimation Strategy
         * @param   max_dist            Maximum acceptable target distance from camera in meters
         * @param   single_tag_std      Single tag standard deviation matrix
         * @param   multi_tag_std       Multi tag standard deviation matrix
         */
        public Settings(
            String name, 
            String camera_name, 
            AprilTagFieldLayout field_layout, 
            Transform3d robot_to_camera, 
            PoseStrategy pose_strategy,
            double max_dist,
            Matrix<N3,N1> single_tag_std,
            Matrix<N3,N1> multi_tag_std
        ) {
            this.name = name;
            this.camera_name = camera_name;
            this.field_layout = field_layout;
            this.robot_to_camera = robot_to_camera;
            this.pose_strategy = pose_strategy;
            this.max_dist = max_dist;
            this.single_tag_std = single_tag_std;
            this.multi_tag_std = multi_tag_std;
        }

        /**
         * Constructor. 
         *      - name is set to camera_name
         *      - field_layout is set to AprilTagFields.kDefault (current season)
         *      - pose_strategy is set to PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
         * @param   camera_name         Camera name for the pipeline as configured in PhotoVision
         * @param   robot_to_camera     Robot to Camera transformation
         * @param   max_dist            Maximum acceptable target distance from camera in meters
         * @param   single_tag_std      Single tag standard deviation matrix
         * @param   multi_tag_std       Multi tag standard deviation matrix
         */
        public Settings(
            String camera_name, 
            Transform3d robot_to_camera, 
            double max_dist,
            Matrix<N3,N1> single_tag_std,
            Matrix<N3,N1> multi_tag_std
        ) {
            Settings(camera_name, camera_name, AprilTagFields.kDefault, 
                robot_to_camera, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                max_dist, single_tag_std, multi_tag_std);
        }

        /**
         * Constructor. 
         *      - name is set to camera_name
         *      - field_layout is set to AprilTagFields.kDefault (current season)
         *      - pose_strategy is set to PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
         *      - max_dist is set to 4 meters
         *      - single_tag_std is set to VecBuilder.fill(4, 4, 8)
         *      - multi_tag_std is set to VecBuilder.fill(0.5, 0.5, 1)
         * @param   camera_name         Camera name for the pipeline as configured in PhotoVision
         * @param   robot_to_camera     Robot to Camera transformation
         */
        public Settings(String camera_name, Transform3d robot_to_camera) {
            Settings(camera_name, camera_name, AprilTagFields.kDefault, 
                robot_to_camera, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
                4, VecBuilder.fill(4, 4, 8), VecBuilder.fill(0.5, 0.5, 1));
        }
    }

    private final Settings settings;                /**< Pipeline Settings */
    private final PhotonCamera camera;              /**< Camera object */
    private final PhotonPoseEstimator pose_est;     /**< Pose Estimator */

    private Pose2d last_pose;                       /**< Most recent estimated pose */
    private double last_timestamp;                  /**< Timestamp of the most recent pose estimation */

    // TODO Make DrivetrainBase class to as parent to SwerveDriveBase
    private Drivetrain dt;                     /**< Drivetrain object to update */

    // Shuffleboard 
    private GenericEntry sb_PoseX;
    private GenericEntry sb_PoseY;
    private GenericEntry sb_PoseR;
    private GenericEntry sb_lastTimestamp;
    private GenericEntry sb_lastUpdatePeriod;

    /**
     * Constructor
     * @param   settings    Pipeline settings
     * @param   dt          Drivetrain object to update
     */
    public AprilTagPipeline(Settings settings, Drivetrain dt) {
        this.settings = settings;
        this.dt = dt;

        camera = new PhotonCamera(settings.camera_name);
        pose_est = new PhotonPoseEstimator(settings.field_layout, settings.pose_strategy, 
            camera, settings.robot_to_camera);

        last_pose = new Pose2d();
        last_timestamp = 0;

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("AprilTags")
                .getLayout(settings.name, BuiltInLayouts.kList)
                .withSize(1, 4);
        sb_PoseX = layout.add("Pose X", 0).getEntry();
        sb_PoseY = layout.add("Pose Y", 0).getEntry();
        sb_PoseR = layout.add("Pose R", 0).getEntry();
        sb_lastTimestamp = layout.add("Last Timestamp", lastTimeStamp).getEntry();
        sb_lastUpdatePeriod = layout.add("Time Since Last Update", 0).getEntry();
    }

    /**
     * Period method. Updates UI.
     */
    @Override
    public void periodic() {
        
        updatePose();
        updateUI();
    }

    /**
     * Updates camera pose estimation
     */
    private void updatePose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for(var change : camera.getAllUnreadResults()) {
            // Get Estimated Position
            vision_est = pose_est.update(change);

            // Check if a pose was estimated
            if(!visionEst.IsEmpty()) {
                Pose2d est_pose = vision_est.get().toPose2d();
                double est_timestamp = vision_est.get().timestampSeconds;

                double avg_dist = 0;
                int tag_count = 0;

                // Get found tag count and average distance
                for(var tag : visionEst.targetsUsed) {
                    var tag_pose3d = pose_est.getFieldTags().getTagPose(tag.getFiducialId());
                    
                    if(!tag_pose3d.isEmpty()) {
                        Pose2d tag_pose = tag_pose3d.get().toPose2d();
                        
                        avg_dist += tag_pose.getTranslation().getDistance(est_pose.getTranslation());
                        tag_count++;
                    }
                }

                // Check if any targets were found
                if(tag_count > 0) {
                    var est_std = settings.single_tag_std;

                    // Calculate average target distance
                    avg_dist /= tag_count;

                    // Decrease standard deviation if multiple tags are found
                    if(tag_count > 1) est_std = settings.multi_tag_std;

                    // Increase standard deviation based on average distance and ignore single 
                    // target results over the settings.max_dist
                    if(tag_count > 1 || avg_dist < settings.max_dist) {
                        // TODO Add average distance scaler to settings
                        est_std = est_std.times(1 + (avg_dist * avg_dist / 30));

                        dt.addVisionPose(est_pose, est_timestamp, est_std);
                        
                        last_pose = est_pose;
                        last_timestamp = est_timestamp;
                    }
                }
            }
        }
    }

    /**
     * Updates Shuffleboard
     */
    private void updateUI() {
        sb_PoseX.setDouble(last_pose.getX());
        sb_PoseY.setDouble(last_pose.getY());
        sb_PoseR.setDouble(last_pose.getRotation().getDegrees());
        sb_lastTimestamp.setDouble(last_timestamp);
        sb_lastUpdatePeriod.setDouble(Timer.getFPGATimestamp() - last_timestamp);
    }
}