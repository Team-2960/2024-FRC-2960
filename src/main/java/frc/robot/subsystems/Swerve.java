package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {

    private String swerveName;

    private final TalonFX mDrive;

    private final CANSparkMax mAngle;

    private final SparkAbsoluteEncoder encAngle;

    private final PIDController drivePIDcontroller;
    private final SimpleMotorFeedforward driveFeedforward;
    
    private final PIDController anglePIDController;
    private final SimpleMotorFeedforward angleFeedforward;

    private SwerveModuleState desiredState;

    private GenericEntry sb_angleSetPoint;
    private GenericEntry sb_angleCurrent;
    private GenericEntry sb_driveSetPoint;
    private GenericEntry sb_driveCurrent;

    public final double driveRatio =  Constants.wheelCirc / Constants.driveGearRatio;   // TODO Move to constants

    public Swerve(int driveMotorID, int angleMotorID, int angleMotorEncID, String swerveName) {
        // Initialize Motors
        mDrive = new TalonFX(driveMotorID);
        mAngle = new CANSparkMax(angleMotorID, MotorType.kBrushless);

        // Initalize Angle Sensor
        encAngle = mAngle.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        // Set swerve name
        this.swerveName = swerveName;

        // Initialize Drive rate controllers 
        drivePIDcontroller = new PIDController(Constants.driveSwerveP, Constants.driveSwerveI,
                Constants.driveSwerveD);
        driveFeedforward = new SimpleMotorFeedforward(Constants.driveSwerveSFF, Constants.driveSwerveVFF,
                Constants.driveSwerveAFF);

        // Initalize angle position controllers
        AngleTrapezoidProfile = new TrapezoidProfile.Constraints(Constants.maxSwerveAngularSpeed,
                Constants.maxSwerveAngularAccel);

        anglePIDController = new PIDController(Constants.angleSwerveP, Constants.angleSwerveI, Constants.angleSwerveD);
        angleFeedforward = new SimpleMotorFeedforward(Constants.angleSwerveSFF, Constants.angleSwerveVFF,
                Constants.angleSwerveAFF);
        
        //Initialize desired state
        desiredState = new SwerveModuleState();

        // Setup Shuffleboard
        
    }

    /**
     * Gets the current swerve module angle
     * @return  current swerve module angle
     */
    public Rotation2d getAnglePos() {
        return Rotation2d.fromRotations(encAngle.getPosition());
    }

    /**
     * Get the current swerve module angle rate
     */
    public double getAngleRate() {
        return encAngle.GetVelocity()
    }

    /**
     * Gets the current swerve module drive distance
     * @return  current swerve module drive distance
     */
    public double gettDrivePos() {
        return mDrive.getPosition().getValueAsDouble() * driveRatio;
    }

    /**
     * Gets the current swerve module drive speed
     * @return  current swerve module drive speed
     */
    public double getDriveVelocity() {
        return mDrive.getVelocity().getValueAsDouble() * driveRatio;
    }

    /**
     * Gets the current swerve module positions
     * @return  current swerve module positions 
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePos(),getAnglePos());
    }

    /**
     * Gets the current swerve module state
     * @return  current swerve module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(),
                getCurrentAngle());
    }

    /**
     * Sets the desired module state
     * @param   desiredState    desired state of the swerve module
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        this.desiredState = desiredState;
    }

    /**
     * Subsystem period update
     */
    @Override
    public void periodic() {

        updateDrive();
        updateAngle();
        updateUI();
    }

    /**
     * Updates the drive rate controllers
     */
    private void updateDrive() {
        // Calculate the drive output from the drive PID controller.
        double pidOutput = drivePIDcontroller.calculate(getCurrentVelocity(),
                state.speedMetersPerSecond);

        double ffOutput = driveFeedforward.calculate(state.speedMetersPerSecond);

        mDrive.setVoltage(pidOutput + ffOutput);
    }

    /**
     * Updates the angle position and rate controllers
     */
    private void updateAngle() {
        // Get current module angle
        Rotation2d encoderRotation = getCurrentAngle();

        //Optimize desired state
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);
        
        // Calculate target rate
        double error = encoderRotation.getRadians() - state.angle.getRadians();
        double compError = 2 * Math.PI - (Math.abs(error));
        double compareError = Math.min(Math.abs(error), compError);
        double direction = error > 0 ? 1 : -1;
        double rampRate = 20; // TODO Move to constants
        
        if(compareError == compError) direction *= -1;
            
        double finalError = compareError * direction;
        double angleVelocity = finalError * rampRate;
        
        // Calcualate motor output
        double pidOutput = anglePIDController.calculate(getAngleRate(), angleVelocity);
                
        double ffOutput = angleFeedforward.calculate(angleVelocity);


        mAngle.setVoltage(pidOutput + ffOutput);
    }

    /**
     * Updated shuffleboard outputs
     */
    private void updateUI() {

    }
}
