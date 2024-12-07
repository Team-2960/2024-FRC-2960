package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;

import frc.lib2960.subsystems.SwerveDriveBase;

public class Drive extends SwerveDriveBase {
    private static Drive instance = null; // Statically initialized instance

    public final DriveSettings settings;

    private final AHRS navx;

    /**
     * Constructor
     */
    private Drive(DriveSettings settings) {
        super(
            settings.drive_settings,
            createModules(settings.module_settings)
        );

        this.settings = settings;

        // Initialize NavX
        navx = new AHRS(SPI.Port.kMXP);
        navx.reset(); 
    }

    /**
     * Gets the current robot angle relative to the field
     * @return  robot angle relative to the field
     */
    @Override
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(navx.getFusedHeading());
    }

    /**
     * Gets the current robot angle rate in degrees per second
     * @return  current robot angle rate in degrees per second
     */
    @Override
    public double getAngleRate() {
        // TODO Re-implement using Units library
        return navx.getRate();
    }

    /**
     * Creates swerve module objects
     * @param module_settings
     * @return
     */
    private static Swerve[] createModules(SwerveSettings[] module_settings) {
        Swerve[] modules = new Swerve[module_settings.length];

        for(int i = 0; i < module_settings.length; i++) modules[i] = new Swerve(module_settings[i]);

        return modules;
    }

    /**
     * Singleton Intiailizer
     * @return  Singleton instance
     */
    public static Drive getInstance() {
        if(instance == null) instance = new Drive(Constants.drive_settings);
        return instance;
    }
}