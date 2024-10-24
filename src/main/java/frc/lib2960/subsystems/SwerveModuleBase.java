package frc.lib2960.subsystems;

import frc.lib2960.util.*;
import frc.lib2960.controllers.*;

import edu.wpi.first.math.geometry.*;

import edu.wpi.first.wpilibj2.command.*;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Abstract class containing most of the control necessary for a swerve drive module. It is 
 * designed to be used as a parent class for a swerve drive module class that implements access 
 * to the motors and sensors of the swerve module.
 */
public abstract class SwerveDriveBase extends SubsystemBase {
    /**
     * Defines module settings
     */
    public class Settings {
        
        public String name;                 /**< Human friendly module name */
        public Translation2d translation;   /**< Module translation */

        PositionController.Settings anglePosCtrl;   /**< Module Angle Pos Controller */
        RateController.Settings angleRateCtrl;      /**< Module Angle Rate Controller */
        RateController.Settings driveRateCtrl;      /**< Module Drive Rate controller */

        /**
         * Constructor
         * @param   name            Module name
         * @param   translation     Module Translation
         * @param   anglePosCtrl    Module Angle Pos Controller Settings
         * @param   angleRateCtrl   Module Angle Rate Controller Settings
         * @param   driveCtrl       Module Drive Rate Controller Settings
         */
        public Settings(String name, Translation2d translation, 
                        PositionController.Settings anglePosCtrl, 
                        RateController.Settings angleRateCtrl, 
                        RateController.Settings driveRateCtrl) {

            this.name = name;
            this.translation = translation;
            this.anglePosCtrl = anglePosCtrl;
            this.angleRateCtrl = angleRateCtrl;
            this.driveRateCtrl = driveRateCtrl;
        }
    }

    /**
     * Automatic Swerve Drive Module Control Command
     */
    public class AutoCommand extends Command {
        private final SwerveDriveBase module;   /**< Module to control */
        
        /**
         * Constructor
         * @param   module  module for command to control
         */
        public AutoCommand(SwerveDriveBase module) {
            this.module = module;

            // Add module as required subsystem 
            addRequirements(module);
        }

        /**
         * Update module position from desired module state
         */
        @Override
        public void execute() {
            module.updateAutoControl();
        }

    }

    private final Settings settings;                /**< Module Settings */

    private final PositionController anglePosCtrl;  /**< Module Angle Position Controller */
    private final RateController angleRateCtrl;     /**< Module Angle Rate Controller */
    private final RateController driveRateCtrl;     /**< Module Drive Controller */

    private SwerveModuleState desiredState;         /**< Most recent Module Desired State */

    // Shuffleboard Elements
    private GenericEntry sb_anglePosTarget;
    private GenericEntry sb_anglePosCurrent;
    private GenericEntry sb_angleVoltCurrent;
    private GenericEntry sb_angleRateCurrent;
    private GenericEntry sb_angleError;

    private GenericEntry sb_driveRateTarget;
    private GenericEntry sb_driveRateCurrent;
    private GenericEntry sb_driveVolt;
    
    /**
     * Constructor
     * @param   settings    module settings
     */
    public SwerveDriveBase(Settings settings) {
        // Initialize variables
        this.settings = settings;

        this.anglePosCtrl = new PositionController(settings.anglPosCtrl);
        this.driveCtrl = new RateController(settings.driveCtrl);
        this.driveCtrl = new RateController(settings.driveCtrl);

        desiredState = new SwerveModuleState();

        // Set default command
        setDefaultCommand(new AutoCommand(this));

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Drive")
            .getLayout(settings.name + " Swerve", BuiltInLayouts.kList)
            .withSize(1, 4);

        sb_anglePosTarget = layout.add("Angle Target", 0).getEntry();
        sb_anglePosCurrent = layout.add("Angle Current", 0).getEntry();
        sb_angleVoltCurrent = layout.add("Angle Voltage", 0).getEntry();
        sb_angleRateCurrent = layout.add("Angle Rate", 0).getEntry();
        sb_angleError = layout.add("Angle Error", 0).getEntry();

        sb_driveRateTarget = layout.add("Drive Target", 0).getEntry();
        sb_driveRateCurrent = layout.add("Drive Current", 0).getEntry();
        sb_driveVolt = layout.add("Drive Voltage", 0).getEntry();
    }
    
    /**
     * Gets the current swerve module positions
     * 
     * @return current swerve module positions
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePos(), getAnglePos());
    }

    /**
     * Gets the current swerve module state
     * 
     * @return current swerve module state
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(),
                getAnglePos());
    }

    /**
     * Sets the desired module state
     * 
     * @param desiredState desired state of the swerve module
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        this.desiredState = desiredState;
    }

    /**
     * Periodic Method
     */
    @Override
    public void periodic() {
        updateUI();
    }

    /**
     * Updates module based on desired state
     */
    private void updateAutoControl() {
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, getAnglePos());

        updateAngle(state.angle);
        updateDrive(state.speedMetersPerSecond);
    }

    /**
     * Updates the angle position and rate controllers
     */
    private void updateAngle(Rotation2d target_angle) {
        double current_pos = getAnglePos().asDegrees();
        double current_rate = getAngleRate();
        double target_pos = target_angle.asDegrees();
        double target_rate = anglePosCtrl.update(current_pos, current_rate, target_pos);
        double angle_volt = angleRateCtrl.update(current_pos, current_rate, target_rate);
        
        setAngleVolt(angle_volt);
    }
    
    /**
     * Updates the drive rate controllers
     */
    private void updateDrive(double target_rate) {
        // Calculate the drive output from the drive PID controller.
        setDriveVolt(driveCtrl.update(0, getDriveRate(), target_rate));  
    }

    /**
     * Updates shuffleboard
     */
    private void updateUI() {
        sb_anglePosTarget.setDouble(desiredState.angle.getDegrees());
        sb_anglePosCurrent.setDouble(getAnglePos().getDegrees());
        sb_angleVoltCurrent.setDouble(getAngleVolt());
        sb_angleRateCurrent.setDouble(getAngleRate());
        sb_angleError.setDouble(desiredState.angle.getDegrees() - getAnglePos().getDegrees());

        sb_driveTarget.setDouble(desiredState.speedMetersPerSecond);
        sb_driveCurrent.setDouble(getDriveVelocity());
        sb_driveVolt.setDouble(mDrive.getMotorVoltage().getValueAsDouble());
    }

    

    public abstract double getAnglePos();
    public abstract double getAngleRate();
    public abstract double getAngleVolt();
    public abstract double getDrivePos();
    public abstract double getDriveRate();
    public abstract double getDriveVolt();
    
    public abstract void setAngleVolt(double volt);
    public abstract void setDriveVolt(double volt);
}
