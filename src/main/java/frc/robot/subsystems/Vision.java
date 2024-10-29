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
        camera = new AprilTagPipeline(Constants.vision_settings, Drive.getInstance());
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
