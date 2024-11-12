package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;

import frc.lib2960.subsystems.SwerveDriveBase;

public class Drive extends SwerveDriveBase {
    private static Drive instance = null; // Statically initialized instance

    private final AHRS navx;

    /**
     * Constructor
     */
    private Drive() {
        super(
            Constants.drive_base_settings, 
            new Swerve[]{
                new Swerve(Constants.fl_swerve_settings), 
                new Swerve(Constants.fr_swerve_settings), 
                new Swerve(Constants.rl_swerve_settings), 
                new Swerve(Constants.rr_swerve_settings)
            }
        );

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

    public static Drive getInstance() {
        if(instance == null) instance = new Drive();
        return instance;
    }
}