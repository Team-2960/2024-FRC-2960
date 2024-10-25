package frc.robot.subsystems;

import frc.lib2960_photonvision.AprilTagPipeline;
import frc.robot.Constants;

/**
 * Manages all vision subsystems
 */
public class Vision  {
    private static Vision vision = null;    /**< Singleton Instance of Vision */

    private AprilTagPipeline camera;        /**< AprilTag Camera Pipeline */

    /**
     * Constructor
     */
    private Vision() {
        camera = new AprilTagPipeline(
            new AprilTagPipeline.Settings("Camera_Module_v1", Constants.robotToCamera),
            Drive.getInstance()
        );
    }

    /**
     * Static initializer
     */
    public static Camera getInstance() {
        if (vision == null) {
            vision = new Vision();
        }

        return vision;
    }

}
