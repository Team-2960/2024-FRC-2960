package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.SPI;

import java.util.Optional;
import java.util.function.Supplier;

import javax.swing.text.html.Option;

import java.lang.reflect.Field;
import java.sql.Driver;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

        // Front Left Swerve
        Swerve fl_swerve = new Swerve(
            new Swerve.Settings(
                "Front Left",
                new Translation2d(x_pos, y_pos),
                Constants.drive_ratio,
                Constants.wheel_radius,
                Constants.swerveAnglePosSettings,
                Constants.swerveAngleRateSettings,
                Constants.swerveDriveRateSettings,
                Constants.frontLeftAngleM,
                Constants.frontLeftDriveM,
                true, true, true
            )
        );

        // Front Right Swerve
        Swerve fr_swerve = new Swerve(
            new Swerve.Settings(
                "Front Right",
                new Translation2d(x_pos, -y_pos),
                Constants.drive_ratio,
                Constants.wheel_radius,
                Constants.swerveAnglePosSettings,
                Constants.swerveAngleRateSettings,
                Constants.swerveDriveRateSettings,
                Constants.frontRightAngleM,
                Constants.frontRightDriveM,
                true, false, true
            )
        );

        // Rear Left Swerve
        Swerve rl_swerve = new Swerve(
            new Swerve.Settings(
                "Rear Left",
                new Translation2d(-x_pos, y_pos),
                Constants.drive_ratio,
                Constants.wheel_radius,
                Constants.swerveAnglePosSettings,
                Constants.swerveAngleRateSettings,
                Constants.swerveDriveRateSettings,
                Constants.backLeftAngleM,
                Constants.backLeftDriveM,
                true, true, true
            )
        );

        // Rear Right Swerve
        Swerve rr_swerve = new Swerve(
            new Swerve.Settings(
                "Rear Right",
                new Translation2d(-x_pos, -y_pos),
                Constants.drive_ratio,
                Constants.wheel_radius,
                Constants.swerveAnglePosSettings,
                Constants.swerveAngleRateSettings,
                Constants.swerveDriveRateSettings,
                Constants.backRightAngleM,
                Constants.backRightDriveM,
                true, false, true
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