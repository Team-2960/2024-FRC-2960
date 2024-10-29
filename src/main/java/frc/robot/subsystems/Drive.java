package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;

import frc.lib2960.subsystem.SwerveDriveBase;

public class Drive extends SwerveDriveBase {
    private static Drive drive = null; // Statically initialized instance

    private final AHRS navx;

    /**
     * Constructor
     */
    private Drive() {

        // Initialize NavX
        navx = new AHRS(SPI.Port.kMXP);
        navx.reset(); 

        // Create Module List
        Swerve[] modules = {
            new Swerve(Constants.fl_swerve_settings), 
            new Swerve(Constants.fr_swerve_settings), 
            new Swerve(Constants.rl_swerve_settings), 
            new Swerve(Constants.rr_swerve_settings)
        };
                
        super(Constants.drive_base_settings, modules);
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
        return return navx.getRate();
    }