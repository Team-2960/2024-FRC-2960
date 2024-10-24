package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.lib2960.subsystems.SwerveModuleBase;

/**
 * Defines the swerve module objects
 */
public class Swerve extends SwerveModuleBase {

    /**
     * Swerve module settings
     */
    public class Settings extends SwerveModuleBase.Settings {
        public final int angle_motor_id;            /**< CAN ID of the angle motor controller */
        public final int drive_motor_id;            /**< CAN ID of the drive motor controller */
        public final boolean invert_angle_motor;    /**< Invert angle motor flag */
        public final boolean invert_drive_motor;    /**< Invert drive motor flag */
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
         * @param angle_motor_id        CAN ID of the angle motor controller 
         * @param drive_motor_id        CAN ID of the drive motor controller 
         * @param invert_angle_motor    Invert angle motor flag 
         * @param invert_drive_motor    Invert drive motor flag 
         * @param invert_angle_enc      Invert angle encoder flag 
         */
        public Settings(
            String name, 
            Translation2d translation, 
            double drive_ratio,
            double wheel_radius,
            PositionController.Settings anglePosCtrl, 
            RateController.Settings angleRateCtrl, 
            RateController.Settings driveRateCtrl,
            int drive_motor_id,
            int angle_motor_id,
            boolean invert_drive_motor,
            boolean invert_angle_motor,
            boolean invert_angle_enc
        ) {
            super(name, translation, drive_ratio, wheel_radius, anglePosCtrl, angleRateCtrl, driveRateCtrl)
            this.drive_motor_id = drive_motor_id;
            this.angle_motor_id = angle_motor_id;
            this.invert_drive_motor = invert_drive_motor;
            this.invert_angle_motor = invert_angle_motor;
            this.invert_angle_enc = invert_angle_enc;
        }
    }


    private final CANSparkMax angle_motor;          /**< Angle motor controller */
    private final TalonFX drive_motor;              /**< Drive motor controller */

    private final SparkAbsoluteEncoder encAngle;    /**< Angle encoder */

    /**
     * Constructor
     * @param   settings    Swerve module settings
     */
    public Swerve(Settings settings) {
        // Initialize Angle Motor
        angle_motor = new CANSparkMax(settings.angle_motor_id, MotorType.kBrushless);
        angle_motor.setInverted(settings.invert_angle_motor);

        // Initialize Drive Motor
        drive_motor = new TalonFX(settings.drive_motor_id);
        drive_motor.setInverted(settings.invert_drive_motor);

        // Initialize Angle Sensor
        encAngle = angle_motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        encAngle.setInverted(settings.invert_angle_enc);

        // Initialize parent class
        super(settings)
    }

    /**
     * Gets the current angle
     * @return current angle
     */
    @Override
    public Rotation2d getAnglePos() {
        return Rotation2d.fromRotations(encAngle.getPosition());
    }

    /**
     * Get the current angle rate
     * @return current angle rate
     */
    @Override
    public double getAngleRate() {
        return encAngle.getVelocity();
    }

    /**
     * Get the current angle motor voltage
     * @return current angle motor voltage
     */
    @Override
    public double getAngleVolt() {
        return angle_motor.getBusVoltage() * angle_motor.getAppliedOutput();
    }

    /**
     * Get the current drive distance
     * @return current drive distance
     */
    @Override
    public double getDrivePos() {
        return drive_motor.getPosition().getValueAsDouble() * motorToDistRatio();
    }

    /**
     * Get the current drive speed
     * @return current drive speed
     */
    @Override
    public double getDriveRate() {
        return drive_motor.getVelocity().getValueAsDouble() * motorToDistRatio();
    }

    /**
     * Get the current drive motor voltage
     * @return current drive motor voltage
     */
    @Override
    public double getDriveVolt(){
        return drive_motor.getMotorVoltage().getValueAsDouble();
    }

    /**
     * Set the output drive motor voltage
     * @param   volt    output voltage
     */
    @Override
    public double setDriveVolt(double volt) {
        drive_motor.setVoltage(volt);
    }

    /**
     * Set the output angle motor voltage
     * @param   volt    output voltage
     */
    @Override
    public double setAngleVolt(double volt) {
        angle_motor.setVoltage(volt);
    }
}
