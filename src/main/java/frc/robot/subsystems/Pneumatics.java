package frc.robot.subsystems;

import frc.lib2960.subsystems.PneumaticsRevPH;
import frc.lib2960.subsystems.PneumaticsRevPHSettings;

import frc.robot.Constants;

public class Pneumatics {
    private static Pneumatics instance = null;      /**< Static singleton instance */

    public PneumaticsRevPH ph;                     /**< Pneumatics Hub instance */

    /**
     * Constructor
     */
    private Pneumatics() {
        ph = new PneumaticsRevPH(
            new PneumaticsRevPHSettings(
                PneumaticsRevPHSettings.DEF_NAME,
                Constants.phCANID,
                PneumaticsRevPHSettings.ControlMode.ANALOG,
                Constants.minPressure,
                Constants.maxPressure
            ), 
            true
        );

        ph.enableCompressor(true);
    }

    /**
     * Retrieves the robot's instance of the Pneumatics class
     * @return  robot's instance of the Pneumatics class
     */
    public static Pneumatics getInstance() {
        if (instance == null) instance = new Pneumatics();

        return instance;
    }
}
