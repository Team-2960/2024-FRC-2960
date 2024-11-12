package frc.robot.subsystems;

import frc.robot.subsystems.Pneumatics_RevPH;

import frc.robot.Constants;

public class Pneumatics {
    private static Pneumatics instance = null;      /**< Static singleton instance */

    public Pneumatics_RevPH ph;                     /**< Pneumatics Hub instance */

    /**
     * Constructor
     */
    private Pneumatics() {
        ph = Pneumatics_RevPH(
            new Pneumatics_RevPH.Settings(
                Pneumatics_RevPH.def_name,
                Constants.phCANID,
                Pneumatics_RevPH.Settings.ANALOG,
                Constants.minPressure,
                Constants.maxPressure
            )
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
