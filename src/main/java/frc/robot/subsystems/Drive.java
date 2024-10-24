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

        // Calculate swerve drive module offset from center of robot
        double x_pos = (Constants.robotLength / 2 - Constants.wheelInset);
        double y_pos = (Constants.robotWidth / 2 - Constants.wheelInset);

        // Setup Swerve Drive control settings
        // TODO Move to Constants
        PositionControl.Settings angle_pos_settings = new PositionControl.Settings(
            Constants.maxSwerveAngularSpeed * 10,                   // Maximum Acceleration
            math.pow(Constants.maxSwerveAngularSpeed,2) /           // Maximum Deceleration
                (2 * Constants.swerveAngleRampDist.getDegrees()),   
            Constants.maxSwerveAngularSpeed,                        // Maximum Rate
            0,                                                      // Minimum Angle
            360,                                                    // Maximum Angle
            true                                                    // Is Continuous
        )

        RateControl.Settings angle_rate_settings = new RateControl.Settings(
            Constants.driveAngPID,
            Constants.driveAngFF
        )

        RateControl.Settings drive_rate_settings = new RateControl.Settings(
            Constants.drivePID,
            Constants.driveFF
        )

        // TODO Move Swerve settings to constants
        // Front Left Swerve
        Swerve fl_swerve = new Swerve(
            new Swerve.Settings(
                "Front Left",                       // Module Name
                new Translation2d(x_pos, y_pos),    // Module Translation
                Constants.drive_ratio,              // Drive gear ratio
                Constants.wheel_radius,             // Drive wheel radius in meters
                angle_pos_settings,                 // Swerve angle position control settings
                angle_rate_settings,                // Swerve angle rate control settings
                drive_rate_settings,                // Swerve drive rate control settings
                Constants.frontLeftAngleM,          // Angle motor CAN ID
                Constants.frontLeftDriveM,          // Drive Motor CAN ID
                true,                               // Invert Angle Motor
                true,                               // Invert Drive Motor
                true                                // Invert Angle Encoder
            )
        );

        // Front Right Swerve
        Swerve fr_swerve = new Swerve(
            new Swerve.Settings(
                "Front Right",                      // Module Name
                new Translation2d(x_pos, -y_pos),   // Module Translation
                Constants.drive_ratio,              // Drive gear ratio
                Constants.wheel_radius,             // Drive wheel radius in meters
                angle_pos_settings,                 // Swerve angle position control settings
                angle_rate_settings,                // Swerve angle rate control settings
                drive_rate_settings,                // Swerve drive rate control settings
                Constants.frontRightAngleM,         // Angle motor CAN ID
                Constants.frontRightDriveM,         // Drive Motor CAN ID
                true,                               // Invert Angle Motor
                false,                              // Invert Drive Motor
                true                                // Invert Angle Encoder
            )
        );

        // Rear Left Swerve
        Swerve rl_swerve = new Swerve(
            new Swerve.Settings(
                "Rear Left",                        // Module Name
                new Translation2d(-x_pos, y_pos),   // Module Translation
                Constants.drive_ratio,              // Drive gear ratio
                Constants.wheel_radius,             // Drive wheel radius in meters
                angle_pos_settings,                 // Swerve angle position control settings
                angle_rate_settings,                // Swerve angle rate control settings
                drive_rate_settings,                // Swerve drive rate control settings
                Constants.backLeftAngleM,           // Angle motor CAN ID
                Constants.backLeftDriveM,           // Drive Motor CAN ID
                true,                               // Invert Angle Motor
                true,                               // Invert Drive Motor
                true                                // Invert Angle Encoder
            )
        );

        // Rear Right Swerve
        Swerve rr_swerve = new Swerve(
            new Swerve.Settings(
                "Rear Right",                       // Module Name
                new Translation2d(-x_pos, -y_pos),  // Module Translation
                Constants.drive_ratio,              // Drive gear ratio
                Constants.wheel_radius,             // Drive wheel radius in meters
                angle_pos_settings,                 // Swerve angle position control settings
                angle_rate_settings,                // Swerve angle rate control settings
                drive_rate_settings,                // Swerve drive rate control settings
                Constants.frontRightAngleM,         // Angle motor CAN ID
                Constants.frontRightDriveM,         // Drive Motor CAN ID
                true,                               // Invert Angle Motor
                false,                              // Invert Drive Motor
                true                                // Invert Angle Encoder
            )
        );

        // Create Module List
        Swerve[] modules = {fl_swerve, fr_swerve, rl_swerve, rr_swerve}

        // Initialize parent class
        ServeDriveBase.Settings settings = new SwerveDriveBase.Settings(
            Constants.maxSpeed,
            Constants.maxAutoAngularSpeed,
            Constants.maxAutoAngularSpeed,
            Constants.maxAutoAngularSpeed
        )

        
        super(settings, modules)
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