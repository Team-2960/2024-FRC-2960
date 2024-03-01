package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {

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
    private GenericEntry sb_angleVolt;
    private GenericEntry sb_driveSetPoint;
    private GenericEntry sb_driveCurrent;
    private GenericEntry sb_driveVolt;


    public Swerve(int driveMotorID, int angleMotorID, String swerveName) {
        // Initialize Motors
        mDrive = new TalonFX(driveMotorID);
        mAngle = new CANSparkMax(angleMotorID, MotorType.kBrushless);

        // Initialize Angle Sensor
        encAngle = mAngle.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        // Initialize Drive rate controllers 
        drivePIDcontroller = new PIDController(Constants.drivePID.kP, Constants.drivePID.kI,
                Constants.drivePID.kD);
        driveFeedforward = new SimpleMotorFeedforward(Constants.driveFF.kS, Constants.driveFF.kV,
                Constants.driveFF.kA);

        // Initialize angle position controllers
        anglePIDController = new PIDController(Constants.driveAngPID.kP, Constants.driveAngPID.kI, Constants.driveAngPID.kD);
        angleFeedforward = new SimpleMotorFeedforward(Constants.driveAngFF.kS, Constants.driveAngFF.kV,
                Constants.driveAngFF.kA);
        
        //Initialize desired state
        desiredState = new SwerveModuleState();

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status").getLayout(swerveName + " Swerve");
        sb_angleSetPoint = layout.add("Angle Desired", 0).getEntry();
        sb_angleCurrent = layout.add("Angle Current", 0).getEntry();
        sb_angleVolt = layout.add("Angle Voltage", 0).getEntry();

        sb_driveSetPoint = layout.add("Drive Desired", 0).getEntry();
        sb_driveCurrent = layout.add("Drive Current", 0).getEntry();
        sb_driveVolt = layout.add("Drive Voltage", 0).getEntry();
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
        return encAngle.getVelocity();
    }

    /**
     * Gets the current swerve module drive distance
     * @return  current swerve module drive distance
     */
    public double getDrivePos() {
        return mDrive.getPosition().getValueAsDouble() * Constants.driveRatio;
    }

    /**
     * Gets the current swerve module drive speed
     * @return  current swerve module drive speed
     */
    public double getDriveVelocity() {
        return mDrive.getVelocity().getValueAsDouble() * Constants.driveRatio;
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
                getAnglePos());
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
        //Optimize desired state
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getAnglePos());

        updateDrive(state);
        updateAngle(state);
        updateUI();
    }

    /**
     * Updates the drive rate controllers
     */
    private void updateDrive(SwerveModuleState state) {
        // Calculate the drive output from the drive PID controller.
        double pidOutput = drivePIDcontroller.calculate(getDriveVelocity(),
                state.speedMetersPerSecond);

        double ffOutput = driveFeedforward.calculate(state.speedMetersPerSecond);

        mDrive.setVoltage(pidOutput + ffOutput);
    }

    /**
     * Updates the angle position and rate controllers
     */
    private void updateAngle(SwerveModuleState state) {
        // Get current module angle
        Rotation2d encoderRotation = getAnglePos();
        
        // Calculate target rate
        double error = encoderRotation.getRadians() - state.angle.getRadians();
        double compError = 2 * Math.PI - (Math.abs(error));
        double compareError = Math.min(Math.abs(error), compError);
        double direction = error > 0 ? 1 : -1;
        
        if(compareError == compError) direction *= -1;
            
        double finalError = compareError * direction;
        double angleVelocity = finalError * Constants.swerveAngleRampRate;
        
        // Calculate motor output
        double pidOutput = anglePIDController.calculate(getAngleRate(), angleVelocity);
                
        double ffOutput = angleFeedforward.calculate(angleVelocity);

        // Set Motor Output
        mAngle.setVoltage(pidOutput + ffOutput);
    }

    /**
     * Updated shuffleboard outputs
     */
    private void updateUI() {
        sb_angleSetPoint.setDouble(desiredState.angle.getDegrees());
        sb_angleCurrent.setDouble(getAnglePos().getDegrees());
        sb_angleVolt.setDouble(mAngle.getBusVoltage() * mAngle.getAppliedOutput());
        sb_driveSetPoint.setDouble(desiredState.speedMetersPerSecond);
        sb_driveCurrent.setDouble(getDriveVelocity());
        sb_driveVolt.setDouble(mDrive.getMotorVoltage().getValueAsDouble());
    }
}
