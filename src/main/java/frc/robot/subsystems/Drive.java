package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import java.util.Map;


public class Drive extends SubsystemBase {
    public enum AngleControlMode {
        AngleRate,
        Angle,
        LookAtPoint
    } 
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

    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    private final SwerveDriveKinematics kinematics;
    private Pose2d getPosition;
    private Pose2d initialPosition;

    private boolean fieldRelative = true;
    private double xSpeed = 0;
    private double ySpeed = 0;
    private double rSpeed = 0;
    private Rotation2d targetAngle;
    private Translation2d targetPoint = new Translation2d();
    private AngleControlMode angleMode = AngleControlMode.AngleRate;

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
        frontLeftLocation = new Translation2d(-0.3302, 0.3302);
        frontRightLocation = new Translation2d(0.3302, 0.3302);
        backLeftLocation = new Translation2d(-0.3302, -0.3302);
        backRightLocation = new Translation2d(0.3302, -0.3302);

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

        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                kinematics,
                navx.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));
    }

    public void setSpeed(double xSpeed, double ySpeed){
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
    }

    public void setAngleRate(double rSpeed){
        this.rSpeed = rSpeed;
        this.angleMode = AngleControlMode.AngleRate;
    }

    public void setFieldRelative(boolean fieldRelative){
        this.fieldRelative = fieldRelative;
    }

    public void setTargetAngle(Rotation2d angle){
        this.targetAngle = angle;
        this.angleMode = AngleControlMode.Angle;
    }

    public void setTargetPoint(Translation2d point, Rotation2d offset) {
        this.targetPoint = point;
        this.targetAngle = offset;
        this.angleMode = AngleControlMode.LookAtPoint;
    }

    public void updateKinematics() {
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

    public void setVector(double speed, Rotation2d heading, double rSpeed) {
        double xSpeed = Math.cos(heading.getRadians()) * speed;
        double ySpeed = Math.sin(heading.getRadians()) * speed;
        setSpeed(xSpeed, ySpeed);
        setAngleRate(rSpeed);
        setFieldRelative(false);
    }

    public Pose2d getEstimatedPos() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void updateOdometry(){
         swerveDrivePoseEstimator.update(navx.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });
    }

    public void setVisionPose(Pose2d pose, double timeStamp) {
        swerveDrivePoseEstimator.addVisionMeasurement(pose, timeStamp);
        

        SmartDashboard.putNumber("Current X", getPosition.getX());
        SmartDashboard.putNumber("Current Y", getPosition.getY());
        SmartDashboard.putNumber("Current Rotation", getPosition.getRotation().getDegrees());
        SmartDashboard.putNumber("Estimated X", swerveDrivePoseEstimator.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Estimated Y", swerveDrivePoseEstimator.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Estimated Rotation", swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees());

    }
    private double getAngleRate() {
        switch (angleMode) {
            case LookAtPoint:
                return calcRateToPoint(targetPoint, targetAngle);
            case Angle:
                return calcRateToAngle(targetAngle);
            case AngleRate:
            default:
                return rSpeed;    
        }
    }

    private double calcRateToAngle(Rotation2d targetAngle) { 
        // Get current angle position
        Pose2d pose = getEstimatedPos();
        Rotation2d currentAngle = pose.getRotation();

        return calcRateToAngle(targetAngle, currentAngle);
    }

    private double calcRateToAngle(Rotation2d targetAngle, Rotation2d currentAngle) {
        Rotation2d rampDistance = Rotation2d.fromRadians(0.5);    // TODO move to constants

        // Determine minimum error distance
        double error = currentAngle.minus(targetAngle).getRadians();
        double compError = 2 * Math.PI - Math.abs(error);
        double minError = Math.min(Math.abs(error), Math.abs(compError));

        // Calculate ramp down speed
        double speed = Math.min(minError * rampDistance.getRadians(), Constants.kMaxAngularSpeed);
        
        // Set direction
        double direction = error > 0 ? 1 : -1;
        if (minError == compError)  direction *= -1;
        speed *= direction;

        return speed;
    }

    /**
     * Calculates the angle rate to look at a target point
     * @param   point   target point
     * @param   offset  target orientation offset
     */
    private double calcRateToPoint(Translation2d point, Rotation2d offset) {
        Pose2d pose = getEstimatedPos();
        Translation2d targetOffset = targetPoint.minus(pose.getTranslation());
        Rotation2d targetAngle = targetOffset.getAngle().plus(offset);

        return calcRateToAngle(targetAngle, pose.getRotation());
    }

    @Override
    public void periodic() {
        // System.out.println("I'm Working");
        updateOdometry();
        updateKinematics();
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
