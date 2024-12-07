package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib2960.subsystems.SwerveModuleBase;

/**
 * Defines the swerve module objects
 */
public class Swerve extends SwerveModuleBase {
    public final SwerveSettings settings;

    private final SparkMax angle_motor;          /**< Angle motor controller */
    private final TalonFX drive_motor;              /**< Drive motor controller */

    private final SparkAbsoluteEncoder encAngle;    /**< Angle encoder */

    /**
     * Constructor
     * @param   settings    Swerve module settings
     */
    public Swerve(SwerveSettings settings) {
        // Initialize parent class
        super(settings);
        this.settings = settings;

        // Initialize Angle Motor
        angle_motor = new SparkMax(settings.angle_motor.id, MotorType.kBrushless);
        SparkMaxConfig angle_motor_config = new SparkMaxConfig();
        angle_motor_config.inverted(settings.angle_motor.inverted);
        
        AbsoluteEncoderConfig angle_enc_config = new AbsoluteEncoderConfig();
        angle_enc_config.inverted(settings.invert_angle_enc);

        angle_motor_config.apply(angle_enc_config);

        angle_motor.configure(angle_motor_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // Initialize Drive Motor
        drive_motor = new TalonFX(settings.drive_motor.id);

        TalonFXConfigurator configurator = drive_motor.getConfigurator();
        MotorOutputConfigs drive_motor_config = new MotorOutputConfigs();
        configurator.refresh(drive_motor_config);
        drive_motor_config.Inverted = settings.drive_motor.inverted ? 
            InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

        configurator.apply(drive_motor_config);

        // Initialize Angle Sensor
        encAngle = angle_motor.getAbsoluteEncoder();
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
    public void setDriveVolt(double volt) {
        drive_motor.setVoltage(volt);
    }

    /**
     * Set the output angle motor voltage
     * @param   volt    output voltage
     */
    @Override
    public void setAngleVolt(double volt) {
        angle_motor.setVoltage(volt);
    }
}
