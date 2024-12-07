package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import frc.lib2960.controllers.PositionControllerSettings;
import frc.lib2960.controllers.RateControllerSettings;
import frc.lib2960.subsystems.SwerveModuleBaseSettings;
import frc.lib2960.util.MotorSettings;

public class SwerveSettings  extends SwerveModuleBaseSettings {
        public final MotorSettings angle_motor;     /**< Angle motor settings */
        public final MotorSettings drive_motor;     /**< Drive motor settings */
        public final boolean invert_angle_enc;      /**< Invert angle encoder flag */
        
        /**
         * Constructor
         * @param name                  Module name
         * @param translation           Module Translation
         * @param drive_ratio           Drive gear ratio
         * @param wheel_radius          Drive wheel radius
         * @param anglePosCtrl          Module Angle Pos Controller Settings
         * @param angleRateCtrl         Module Angle Rate Controller Settings
         * @param driveCtrl             Module Drive Rate Controller Settings
         * @param angle_motor           Angle motor settings
         * @param drive_motor           Drive motor settings
         * @param invert_angle_enc      Invert angle encoder flag 
         */
        public SwerveSettings(
            String name, 
            Translation2d translation, 
            double drive_ratio,
            double wheel_radius,
            PositionControllerSettings anglePosCtrl, 
            RateControllerSettings angleRateCtrl, 
            RateControllerSettings driveRateCtrl,
            MotorSettings angle_motor,
            MotorSettings drive_motor,
            boolean invert_angle_enc
        ) {
            super(name, translation, drive_ratio, wheel_radius, anglePosCtrl, angleRateCtrl, driveRateCtrl);

            this.drive_motor = drive_motor;
            this.angle_motor = angle_motor;
            this.invert_angle_enc = invert_angle_enc;
        }
    
}
