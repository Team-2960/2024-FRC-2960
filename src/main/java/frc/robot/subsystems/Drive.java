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
    public enum AngleControlMode = {
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

    private double xSpeed = 0;
    private double ySpeed = 0;
    private double rSpeed = 0;
    private Rotation2d targetAngle = new Rotation2d();
    private Translation2d targetPoint = new Translation2d();
    private AngleControlMode angleMode = AngleRate;
    private boolean fieldRelative = true;

    public class DriveDiag extends ComplexData<DriveDiag> {
        
        private Drive drive;

        public DriveDiag(Drive drive) {
            this.drive = drive;
        }

        @Override
        public Map<String, Object> asMap() {
            Pose2d pose = drive.getEstimatedPos();


            return Map.of(
                "x", pose.getX(),
                "y", pose.getY(),
                "r", pose.getRotation().asDegrees()
                )
        }

    }

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

        frontLeft = new Swerve(Constants.frontLeftDriveM, Constants.frontLeftAngleM, Constants.frontLeftAngleENC, "FrontLeft");
        frontRight = new Swerve(Constants.frontRightDriveM, Constants.frontRightAngleM, Constants.frontRightAngleENC, "FrontRight");
        backLeft = new Swerve(Constants.backLeftDriveM, Constants.backLeftAngleM, Constants.backLeftAngleENC, "BackLeft");
        backRight = new Swerve(Constants.backRightDriveM, Constants.backRightAngleM, Constants.backRightAngleENC, "BackRight");

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

        
        var tab = ShuffleBoard.getTab("Diag");
        var layout = tab.getLayout("Drive", BuiltInLayouts.kList);
        
        elevatorCommands.add("Estimated Position", new DriveDiag(this));
    }

    public void setfieldRelative(boolean enable) {
        this.fieldRelative = enable;
    }

    public void setSpeed(double xSpeed, double ySpeed) {
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.fieldRelative = fieldRelative;
    }

    public void setVector(double speed, Rotation2d heading) {
        double xSpeed = Math.cos(heading.getRadians()) * speed;
        double ySpeed = Math.sin(heading.getRadians()) * speed;
        
        set_speed(xSpeed, ySpeed, rSpeed, fieldRelative);
    }

    public void setAngleRate(double rSpeed) {
        this.rSpeed = rSpeed;
        this.angleMode = AngleRate;
    }

    public void setTargetAngle(Rotation2d angle) {
        this.targetAngle = angle;
        this.angleMode = Angle;
    }

    public void setTargetPoint(Translation2d point, Rotation2d offset) {
        this.targetPoint = point;
        this.targetAngle = offset;
    }

    public Pose2d getEstimatedPos() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void setVisionPose(Pose2d pose, double timeStamp) {
        swerveDrivePoseEstimator.addVisionMeasurement(pose, timeStamp);
    }

    @Override
    public void periodic() {
        update_kinematics();
        update_odometry();
    }

    private void update_kinematics() {
        ChassisSpeeds speeds;

        rSpeed = getAngleRate();

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

    private void update_odometry() {
        swerveDrivePoseEstimator.update(navx.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });
    }

    private double getAngleRate() {
        switch (angleMode) {
            case LookAtPoint:
                return calcRateToPoint(targetPoint, targetAngle);
            case Angle:
                return calcRateToAngle(TargetAngle);
            case AngleRate:
            default:
                return rSpeed;    
        }
    }

    private double calcRateToAngle(Rotation2d targetAngle) { 
        // Get current angle position
        Pose2d pose = getEstimatedPos()
        Rotation2d currentAngle = pose.getRotation();

        return calcRateToAngle(targetAngle, currentAngle);
    }

    private double calcRateToAngle(Rotation2d targetAngle, Rotation2d currentAngle) {
        Rotatation2d rampDistance = Rotation2d(0.5);    // TODO move to constants

        
        // Determine minimum error distance
        double error = currentAngle.minus(targetAngle).toRadians();
        double compError = 2 * Math.Pi - Math.abs(error);
        double minError = Math.min(Math.abs(error), math.abs(compError));

        // Calculate ramp down speed
        double speed = Math.min(minError * rampDistance , kMaxAngularSpeed);
        
        // Set direction
        double direction = error > 0 ? 1 : -1;
        if (minError == compError)  direction *= -1;
        speed *= direction;

        return speed;
    }

    private double calcRateToPoint(Translation2d point, Rotatation2d offset) {
        Pose2d pose = getEstimatedPos();
        Translation2d targetOffset = targetPoint.minus(pose.getTranslation());
        Rotation2d targetAngle = targetOffset.getAngle().add(offset);

        return calcRateToAngle(targetAngle, pose.getRotation());
    }

    /**
     * Static Initializer
     * 
     * @return Common object of Drive
     */
    public static Drive getInstance() {
        if (drive == null) {
            drive = new Drive();
        }

        return drive;
    }

}
