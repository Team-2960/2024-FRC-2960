package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

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

    private double xSpeed = 0;
    private double ySpeed = 0;
    private double rSpeed = 0;
    private Rotation2d targetAngle = new Rotation2d();
    private Translation2d targetPoint = new Translation2d();
    private AngleControlMode angleMode = AngleControlMode.AngleRate;
    private boolean fieldRelative = true;

    private GenericEntry sb_posEstX;
    private GenericEntry sb_posEstY;
    private GenericEntry sb_posEstR;

    private GenericEntry sb_speedX;
    private GenericEntry sb_speedY;
    private GenericEntry sb_speedR;

    private GenericEntry sb_speedTargetR;

    private ComplexWidget sb_field2d;

    private boolean targetSeen;
    private boolean ignoreCamera;

    private Field2d field2d;

    /**
     * Constructor
     */
    private Drive() {
        // Set swerve drive positions
        frontLeftLocation = new Translation2d((Constants.robotLength/2 - Constants.wheelInset), -(Constants.robotWidth/2 - Constants.wheelInset));
        frontRightLocation = new Translation2d((Constants.robotLength/2 - Constants.wheelInset), (Constants.robotWidth/2 - Constants.wheelInset));
        backLeftLocation = new Translation2d(-(Constants.robotLength/2 - Constants.wheelInset), -(Constants.robotWidth/2 - Constants.wheelInset));
        backRightLocation = new Translation2d(-(Constants.robotLength/2 - Constants.wheelInset), (Constants.robotWidth/2 - Constants.wheelInset));

        // Initialize Swerve Kinematics
        kinematics = new SwerveDriveKinematics(
                frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);
        // Create swerve drive module objects
        frontLeft = new Swerve(Constants.frontLeftDriveM, Constants.frontLeftAngleM, "FrontLeft", Rotation2d.fromDegrees(0), true);
        frontRight = new Swerve(Constants.frontRightDriveM, Constants.frontRightAngleM, "FrontRight", Rotation2d.fromDegrees(0), false);
        backLeft = new Swerve(Constants.backLeftDriveM, Constants.backLeftAngleM, "BackLeft", Rotation2d.fromDegrees(0), true);
        backRight = new Swerve(Constants.backRightDriveM, Constants.backRightAngleM, "BackRight", Rotation2d.fromDegrees(0), false);
        
        // Initialize NavX
        navx = new AHRS(SPI.Port.kMXP);
        navx.reset();

        targetSeen = false;

        field2d = new Field2d();

        // Initialize pose estimation
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

        // Setup Shuffleboard
        var pose_layout = Shuffleboard.getTab("Drive")
            .getLayout("Drive Pose", BuiltInLayouts.kList)
            .withSize(1,4);
        sb_posEstX = pose_layout.add("Pose X", swerveDrivePoseEstimator.getEstimatedPosition().getX()).getEntry();
        sb_posEstY = pose_layout.add("Pose Y", swerveDrivePoseEstimator.getEstimatedPosition().getY()).getEntry();
        sb_posEstR = pose_layout.add("Pose R", swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees()).getEntry();
        
        sb_speedX  = pose_layout.add("Speed X", 0).getEntry();
        sb_speedY  = pose_layout.add("Speed Y", 0).getEntry();
        sb_speedR  = pose_layout.add("Speed R", 0).getEntry();

        sb_speedTargetR  = pose_layout.add("Target Speed R", 0).getEntry();

        sb_field2d = Shuffleboard.getTab("Drive").add(field2d).withWidget("Field");

        
    }

    /**
     * Sets field relative mode
     * 
     * @param enable true to enable field relative mode, false to disable. (Default:
     *               true)
     */
    public void setfieldRelative(boolean enable) {
        this.fieldRelative = enable;
    }

    /**
     * Sets the drivetrain linear speeds in x and y parameters. Direction of
     * the axis used determine by field relative setting.
     * 
     * @param xSpeed speed of along the x-axis
     * @param ySpeed speed of along the y-axis
     */
    public void setSpeed(double xSpeed, double ySpeed) {
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
    }

    /**
     * Sets the drivetrain linear speeds by setting speed and a heading. Direction
     * of the axis used determine by field relative setting.
     * 
     * @param speed   linear speed of the robot
     * @param heading heading of the robot
     */
    public void setVector(double speed, Rotation2d heading) {
        double xSpeed = Math.cos(heading.getRadians()) * speed;
        double ySpeed = Math.sin(heading.getRadians()) * speed;

        setSpeed(xSpeed, ySpeed);
    }

    /**
     * Sets the angular rate of the robot and sets the robot to AngleRate
     * angle control mode.
     * 
     * @param rSpeed angle rate for the robot
     */
    public void setAngleRate(double rSpeed) {
        this.rSpeed = rSpeed;
        this.angleMode = AngleControlMode.AngleRate;
    }

    /**
     * Sets the target angle of the robot and sets the robot to Angle angle
     * control mode.
     * 
     * 
     * @param angle target angle for the robot.
     */
    public void setTargetAngle(Rotation2d angle) {
        this.targetAngle = angle;
        this.angleMode = AngleControlMode.Angle;
    }

    /**
     * Sets the target point of the robot and sets the robot to LookAtPoint
     * angle control mode.
     * 
     * @param point  point to look at
     * @param offset offset angle for the robot orientation
     */
    public void setTargetPoint(Translation2d point, Rotation2d offset) {
        this.targetPoint = point;
        this.targetAngle = offset;
        this.angleMode = AngleControlMode.LookAtPoint;
    }

    /**
     * Gets the robots estimiated pose
     * 
     * @return estimated robot pose
     */
    public Pose2d getEstimatedPos() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    /**
     * Sets a new vision pose update
     * 
     * @param pose      estimated pose from the vision
     * @param timestamp timestamp of when the pose was captured
     */
    public void setVisionPose(Pose2d pose, double timeStamp) {
        // TODO Adjust standard deviations based on distance from target
        if(!ignoreCamera){
        swerveDrivePoseEstimator.addVisionMeasurement(pose, timeStamp);
        targetSeen = true;
        }
    }

    public boolean getTargetSeen() {
        return targetSeen;
    }

    /**
     * Subsystem periodic method
     */
    @Override
    public void periodic() {
        update_kinematics();
        update_odometry();
        updateUI();
    }

    /**
     * Updates the robot swerve kinematics
     */
    private void update_kinematics() {
        ChassisSpeeds speeds;

        double xSpeed = this.xSpeed;
        double ySpeed = this.ySpeed;

        if(targetSeen && fieldRelative){
            xSpeed *= -1;
            ySpeed *= -1;
        }

        rSpeed = getAngleRate();

        if (fieldRelative) {
            Pose2d robot_pose = getEstimatedPos();
            Rotation2d fieldAngle = Rotation2d.fromDegrees(360).minus(robot_pose.getRotation());
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rSpeed, fieldAngle);
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rSpeed);
        }

        speeds = ChassisSpeeds.discretize(speeds, Constants.updatePeriod);

        var swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.maxSpeed);

        sb_speedTargetR.setDouble(rSpeed);

        frontLeft.setDesiredState(swerveModuleStates[0]);
        frontRight.setDesiredState(swerveModuleStates[1]);
        backLeft.setDesiredState(swerveModuleStates[2]);
        backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Updates the robot swerve odometry
     */
    private void update_odometry() {
        swerveDrivePoseEstimator.update(navx.getRotation2d(),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });
    }

    /**
     * Calculates the target angular rate for the robot
     * 
     * @return target angular rate for the robot based on current angle
     *         control mode and settings
     */
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

    /**
     * Calculates the angle rate to reach a target angle
     * 
     * @param targetAngle target robot angle
     */
    private double calcRateToAngle(Rotation2d targetAngle) {
        // Get current angle position
        Pose2d pose = getEstimatedPos();
        Rotation2d currentAngle = pose.getRotation();

        return calcRateToAngle(targetAngle, currentAngle);
    }

    /**
     * Calculates the angle rate to reach a target angle
     * 
     * @param targetAngle  target robot angle
     * @param currentAngle current robot angle
     */
    private double calcRateToAngle(Rotation2d targetAngle, Rotation2d currentAngle) {

        // Determine minimum error distance
        double error = currentAngle.minus(targetAngle).getRadians();
        double compError = 2 * Math.PI - Math.abs(error);
        double minError = Math.min(Math.abs(error), Math.abs(compError));

        // Calculate ramp down speed
        double speed = Math.min(minError / Constants.driveAngleRampDistance.getRadians(), 1) *Constants.maxAngularSpeed;

        // Set direction
        double direction = error > 0 ? 1 : -1;
        
        if (minError == compError)  direction *= -1;

        speed *= direction;

        return speed;
    }

    /**
     * Calculates the angle rate to look at a target point
     * 
     * @param point  target point
     * @param offset target orientation offset
     */
    private double calcRateToPoint(Translation2d point, Rotation2d offset) {
        Pose2d pose = getEstimatedPos();
        Translation2d targetOffset = point.minus(pose.getTranslation());
        Rotation2d targetAngle = targetOffset.getAngle().plus(offset);

        return calcRateToAngle(targetAngle, pose.getRotation());
    }


    private void updateUI() {
        Pose2d pose = getEstimatedPos();
        sb_posEstX.setDouble(pose.getX());
        sb_posEstY.setDouble(pose.getY());
        sb_posEstR.setDouble(pose.getRotation().getDegrees());

        sb_speedX.setDouble(xSpeed);
        sb_speedY.setDouble(ySpeed);
        sb_speedR.setDouble(rSpeed);

        field2d.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
    }

    public void ignoreCamera(boolean ignore){
        this.ignoreCamera = ignore;
    }

    public void presetPosition(Pose2d pose2d){
        swerveDrivePoseEstimator.resetPosition(
            navx.getRotation2d(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
                }, 
            pose2d
            );
        targetSeen = false;
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
