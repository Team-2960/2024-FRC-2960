package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

public class Drive extends SubsystemBase {
    private static Drive drive = null; // Statically initialized instance

    private final Translation2d frontLeftLocation;
    private final Translation2d frontRightLocation;
    private final Translation2d backLeftLocation;
    private final Translation2d backRightLocation;

    private final Swerve frontLeft;
    private final Swerve frontRight;
    private final Swerve backLeft;
    private final Swerve backRight;

    private final AHRS navx;

    private final SwerveDriveOdometry swerveDriveOdometry;

    private final SwerveDriveKinematics kinematics;
    private Pose2d getPosition;

    /*
     * private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
     * kinematics,
     * null,
     * new SwerveModulePosition[] {
     * frontLeft.getPosition(),
     * frontRight.getPosition(),
     * backLeft.getPosition(),
     * backRight.getPosition()
     * });
     */

    private Drive() {
        frontLeftLocation = new Translation2d(-0.307975, 0.27305);
        frontRightLocation = new Translation2d(0.307975, 0.27305);
        backLeftLocation = new Translation2d(-0.307975, -0.27305);
        backRightLocation = new Translation2d(0.307975, -0.27305);

        kinematics = new SwerveDriveKinematics(
                frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

        frontLeft = new Swerve(Constants.frontLeftDriveM, Constants.frontLeftAngleM, Constants.frontLeftAngleENC,
                "FrontLeft");
        frontRight = new Swerve(Constants.frontRightDriveM, Constants.frontRightAngleM, Constants.frontRightAngleENC,
                "FrontRight");
        backLeft = new Swerve(Constants.backLeftDriveM, Constants.backLeftAngleM, Constants.backLeftAngleENC,
                "BackLeft");
        backRight = new Swerve(Constants.backRightDriveM, Constants.backRightAngleM, Constants.backRightAngleENC,
                "BackRight");

        navx = new AHRS(SPI.Port.kMXP);
        navx.reset();

        swerveDriveOdometry = new SwerveDriveOdometry(kinematics, navx.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });
    }

    public void set_speed(double xSpeed, double ySpeed, double rSpeed, boolean fieldRelative) {
        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rSpeed, navx.getRotation2d());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rSpeed);
        }

        speeds = ChassisSpeeds.discretize(speeds, Constants.updatePeriod);

        var swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxSpeed);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]); 


    }

    public void updateOdometry() {
        getPosition = swerveDriveOdometry.update(navx.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });

                
    }

    @Override
    public void periodic() {
        updateOdometry();
        SmartDashboard.putNumber("Current X", getPosition.getX());
        SmartDashboard.putNumber("Current Y", getPosition.getY() );
        SmartDashboard.putNumber("Current Rotation", getPosition.getRotation().getDegrees());
       // System.out.println("I'm Working");
    }

    /**
     * Static Initializer
     * 
     * @return Common object of Drive
     */
    public static Drive get_instance() {
        if (drive == null) {
            drive = new Drive();
        }

        return drive;
    }
}
