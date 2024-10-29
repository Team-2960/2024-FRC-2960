/**
 * Copyright 2024 Ryan Fitz-Gerald
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to 
 * deal in the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 */

package frc.lib2960.subsystems;

import frc.lib2960.util.*;
import frc.lib2960.controllers.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.*;

/**
 * Base class for motorized mechanisms such as Arm joints, Elevators, Turrets, and Angle 
 * adjustments for Shooters.
 */
public abstract class MotorMechanismBase extends SubsystemBase {

    /**
     * Motor Mechanism Settings
     */
    public class Settings {
        // TODO Allow RateControl Feed Forward settings to change for dynamic loads
        // TODO allow soft limits to change
        public final String name;                           /**< Mechanism Name */
        public final String tab_name;                       /**< ShuffleBoard tab name */
        public final PositionController.Settings pos_ctrl;  /**< Position Controller settings */
        public final RateController.Settings[] rate_ctrls;  /**< List of Rate Controller settings */
        public final Limits[] soft_limits;                  /**< List of soft limits */
        public final Limits def_tol;                        /**< Default acceptable distance range 
                                                                 around target to be considered 
                                                                 "at target" */

        /**
         * Constructor
         * @param   name        Name of the mechanism
         * @param   tab_name    ShuffleBoard tab name
         * @param   pos_ctrl    Position Controller settings
         * @param   rate_ctrls  List of Rate Controller settings
         * @param   soft_limits List of soft limits
         * @param   def_tol     Default acceptable distance range around target to be considered 
         *                          "at target"
         */
        public Settings(String name, String tab_name, PositionController.Settings pos_ctrl, 
                        RateController.Settings[] rate_ctrls, Limits[] soft_limits, Limit def_tol) {
            this.name = name;
            this.tab_name = tab_name;
            this.pos_ctrl = pos_ctrl;
            this.rate_ctrls = rate_ctrls;
            this.soft_limits = soft_limits;
            this.def_tol = def_tol;
        }

        // TODO add different default permutations of constructor
    }

    /**
     * Hold Position Command
     */
    public class HoldPositionCommand extends Command {
        private final MotorMechanismBase mechanism;     /**< Mechanism object reference */
        private double target;                          /**< Target Position to hold */

        /**
         * Constructor
         * @param   mechanism   Mechanism object reference
         */
        public HoldPositionCommand(MotorMechanismBase mechanism) {
            this.mechanism = mechanism;
            target = mechanism.getPosition();

            addRequirements(mechanism);
        }

        /**
         * Initialize command. Record current position
         */
        @Override
        public void initialize() {
            target = mechanism.getPosition();
        }

        /**
         * Execute Command. Hold position when command was started
         */
        @Override
        public void execute() {
            mechanism.getPosTrackingRate(target)
        }
    }

    /**
     * Set Voltage Command
     */
    public class SetVoltageCommand extends Command {
        private final MotorMechanismBase mechanism;     /**< Mechanism object reference */
        private double target_voltage;                  /**< Target voltage for the mechanism */

        /**
         * Constructor
         * @param   mechanism       Mechanism object reference
         * @param   target_voltage  Target voltage for the mechanism
         */
        public SetVoltageCommand(MotorMechanismBase mechanism, double voltage) {
            this.mechanism = mechanism;
            this.target_voltage = target_voltage;
        }

        /**
         * Execute Method. Update mechanism voltage
         */
        @Override
        public void execute() {
            mechanism.updateVoltage(target_voltage);
        }

        /**
         * Sets the target voltage
         * @param   target_voltage     Target voltage for the mechanism
         */
        public void setVoltage(double target_voltage) {
            this.target_voltage = target_voltage;
        }
    }

    /**
     * Set Rate Command
     */
    public class SetRateCommand extends Command {
        private final MotorMechanismBase mechanism;     /**< Mechanism object reference */
        private double target_rate;                     /**< Target rate for the mechanism */

        /**
         * Constructor
         * @param   mechanism       Mechanism object reference
         * @param   target_rate     Target rate for the mechanism
         */
        public SetRateCommand(MotorMechanismBase mechanism, double target_rate) {
            this.mechanism = mechanism;
            this.target_rate = target_rate;
        }

        /**
         * Execute Method. Update mechanism rate
         */
        @Override
        public void execute() {
            mechanism.updateRate(target_rate);
        }

        /**
         * IsFinished method. End if target rate is 0
         */
        @Override
        public boolean isFinished() {
            return target_rate == 0 || mechanism.atLimit();
        }

        /**
         * Sets the target rate
         * @param   target_rate     Target rate for the mechanism
         */
        public void setRate(double target_rate) {
            this.target_rate = target_rate;
        }
    }

    /**
     * Set Position Command
     */
    public class SetPositionCommand extends Command {
        private final MotorMechanismBase mechanism;     /**< Mechanism object reference */
        private double target;                          /**< Target position for the mechanism */
        private final Limits tol;                       /**< Acceptable distance from target 
                                                             position to consider mechanism "at target" */
        private final boolean auto_complete;            /**< Command automatically finishes when 
                                                             "at target" */
        /**
         * Constructor.
         *      - tol is set to def_tol of the mechanism
         *      - auto_complete is set to true
         * @param   mechanism       Mechanism object reference
         * @param   target          Target position for the mechanism
         */
        public SetPositionCommand(MotorMechanismBase mechanism, double target) {
            this.mechanism = mechanism;
            this.target = target;
            this.tol = mechanism.settings.def_tol;
            this.auto_complete = true;
        }

        /**
         * Constructor
         *      - auto_complete is set to true
         * @param   mechanism       Mechanism object reference
         * @param   target          Target position for the mechanism
         * @param   tol             Acceptable distance from target position to consider mechanism
         *                              "at target"
         */
        public SetPositionCommand(MotorMechanismBase mechanism, double target, double tol) {
            this.mechanism = mechanism;
            this.target = target;
            this.tol = tol;
            this.auto_complete = true;
        }
        /**
         * Constructor
         *      - tol is set to def_tol of the mechanism
         * @param   mechanism       Mechanism object reference
         * @param   target          Target position for the mechanism
         * @param   auto_complete   Command automatically finishes when "at target" if true. 
         *                              Continues indefinitely if false.  
         */
        public SetPositionCommand(MotorMechanismBase mechanism, double target, boolean auto_complete) {
            this.mechanism = mechanism;
            this.target = target;
            this.tol = mechanism.settings.def_tol;
            this.auto_complete = auto_complete;
        }

        /**
         * Constructor
         * @param   mechanism       Mechanism object reference
         * @param   target          Target position for the mechanism
         * @param   tol             Acceptable distance from target position to consider mechanism
         *                              "at target"
         * @param   auto_complete   Command automatically finishes when "at target" if true. 
         *                              Continues indefinitely if false.  
         */
        public SetPositionCommand(MotorMechanismBase mechanism, double target, double tol, boolean auto_complete) {
            this.mechanism = mechanism;
            this.target = target;
            this.tol = tol;
            this.auto_complete = auto_complete;
        }

        /**
         * Execute Method. Update mechanism rate
         */
        @Override
        public void execute() {
            double target_rate = mechanism.getPosTrackingRate(target);
            mechanism.setRate(target_rate);
        }

        /**
         * IsFinished method. End if target rate is 0
         */
        @Override
        public boolean isFinished() {
            double current_pos = mechanism.getPosition();
            double error = current_pos - target;
            
            return auto_complete && atTarget() && mechanism.atLimit();
        }

        /**
         * Sets the target rate
         * @param   target          Target position for the mechanism
         */
        public void setTarget(double target) {
            this.target = target;
        }

        /**
         * Checks if the mechanism is at target
         * @return  true if mechanism is at the target position
         */
        public boolean atTarget() {
            return tol.inRange(error);
        }
    }
    
    public final Settings settings;                 /**< Mechanism settings */
    public final int motor_count;                   /**< Number of motors in the list */

    public final PositionController pos_ctrl;       /**< Position Controller */
    public final RateController[] rate_ctrls;       /**< Rate Controller */
    public int cur_rate_ctrl;                       /**< Current rate controller index */

    public int cur_soft_limits;                     /**< Current soft limit index */

    public final HoldPositionCommand hold_pos_cmd;  /**< Internal hold position command */
    public final SetVoltageCommand set_voltage_cmd; /**< Internal set voltage command */
    public final SetRateCommand set_rate_cmd;       /**< Internal set rate command */
    public final SetPositionCommand set_pos_cmd;    /**< Internal set position command */

    // ShuffleBoard
    protected final ShuffleboardLayout sb_layout;

    private GenericEntry sb_position;
    private GenericEntry sb_rate;
    
    private GenericEntry[] sb_voltages;
    private GenericEntry[] sb_currents;

    private GenericEntry sb_atLowerLimitSensor;
    private GenericEntry sb_atLowerSoftLimit;
    private GenericEntry sb_atUpperLimitSensor;
    private GenericEntry sb_atUpperSoftLimit;

    /**
     * Constructor
     * @param   settings    Mechanism Settings
     */
    public MotorMechanismBase(Settings settings, int motor_count) {
        this.settings = settings;
        this.motor_count = motor_count;

        // Initialize Controllers
        pos_ctrl = new PositionController(settings.pos_ctrl);

        int rate_ctrl_count = settings.rate_ctrls.size();
        rate_ctrls = new RateController[rate_ctrl_count];

        for(int i = 0; i < rate_ctrl_count; i++) {
            rate_ctrls[i] = new RateController(settings.rate_ctrls[i]);
        }

        cur_rate_ctrl = 0;

        // Initialize Soft Limits
        cur_soft_limits = 0;

        // Initialize ShuffleBoard
        sb_layout = Shuffleboard.getTab(settings.tab_name)
            .getLayout(settings.name, BuiltInLayouts.kList)
            .withSize(1, 4);

        sb_position = layout.add("Position", getPosition()).getEntry();
        sb_rate = layout.add("Rate", getRate()).getEntry();

        sb_voltages = new GenericEntry[motor_count];
        sb_currents = new GenericEntry[motor_count];

        for(i = 0; i < motor_count; i++) {
            sb_voltages[i] = layout.add("Motor " + i + " Voltage", getVoltage(i)).getEntry();
            sb_currents[i] = layout.add("Motor " + i + " Current", getCurrent(i)).getEntry();
        }

        sb_atLowerLimitSensor = layout.add("Position", atLowerLimitSensor()).getEntry();
        sb_atLowerSoftLimit = layout.add("Position", atLowerSoftLimit()).getEntry();
        sb_atUpperLimitSensor = layout.add("Position", atUpperLimitSensor()).getEntry();
        sb_atUpperSoftLimit = layout.add("Position", atUpperSoftLimit()).getEntry();

        // Initialize commands
        hold_pos_cmd = new HoldPositionCommand(this);
        set_voltage_cmd = new SetVoltageCommand(this);
        set_rate_cmd = new SetRateCommand(this, 0);
        set_pos_cmd = new SetPositionCommand(this, getPosition(), false);

        // Set Default Command
        setDefaultCommand(hold_pos_cmd);
    }


    /**************************/
    /* Public Access Methods */
    /**************************/

    /**
     * Get the current rate controller index
     * @return current rate controller index
     */
    public int getRateCtrlIndex() {
        return cur_rate_ctrl;
    }

    /**
     * Get the current soft limit index
     * @return current soft limit index
     */
    public int getSoftLimitsIndex() {
        return cur_soft_limits;
    }

    /**
     * Get the current soft limits
     * @return  current soft limits
     */
    public Limits getSoftLimits() {
        return settings.soft_limits[cur_soft_limits];
    }

    /**
     * Checks if mechanism is at its lower soft limit
     * @return  true if at limit
     */
    public boolean atLowerSoftLimit() {
        
        return !settings.is_cont && getSoftLimits().inLower(getPosition());
    }

    /**
     * Checks if mechanism is at its upper soft limit
     * @return  true if at limit
     */
    public boolean atUpperSoftLimit() {
        return !settings.is_cont && getSoftLimits().inUpper(getPosition());;
    }

    /**
     * Checks if mechanism is at its any soft limit
     * @return  true if at limit
     */
    public boolean atSoftLimit() {
        return getSoftLimits().inRange(getPosition());
    }

    /**
     * Checks if mechanism is at its any limit sensor
     * @return  true if at limit
     */
    public boolean atLimitSensor() {
        return atLowerLimitSensor() || atUpperLimitSensor();
    }

    /**
     * Checks if mechanism is at its any lower limit
     * @return  true if at limit
     */
    public boolean atLowerLimit() {
        return atLowerSoftLimit() || atLowerLimitSensor();
    }

    /**
     * Checks if mechanism is at its any upper limit
     * @return  true if at limit
     */
    public boolean atUpperLimit() {
        return atUpperSoftLimit() || atUpperLimitSensor();
    }

    /**
     * Checks if mechanism is at its any limit
     * @return  true if at limit
     */
    public boolean atLimit() {
        return atLowerLimitSwitch()
    }

    /**
     * Checks if the mechanism is at the target position. Will always return false if in not 
     * running internal PositionTracking or HoldPosition commands. Always True if HoldPosition
     * is running. Checks atTarget in internal PositionTracking if it is running.
     * @return  True if at target, false otherwise.
     */
    public boolean atTarget() {
        boolean result = false;

        if(getCurrentCommand() == hold_pos_cmd) result = true;
        if (getCurrentCommand() == set_pos_cmd) result = set_pos_cmd.atTarget();

        return result;
    }
    

    /**************************/
    /* Public Control Methods */
    /**************************/

    /**
     * Selects the active rate controller. If selected rate controller does not exist, nothing 
     * changes.
     * @param   index   new rate controller index
     */
    public void setRateCtrlIndex(int index) {
        if(0 <= index && index < rate_ctrls.size()) cur_rate_ctrl = index;
    }

    /**
     * Select active soft limits. If selected soft limits do not exist, nothing changes.
     * @param   index   new soft limit index index
     */
    public void setSoftLimitIndex(int index) {
        if(0 <= index && index < soft_limits.size()) cur_soft_limits = index;
    }

    /**
     * Sets the voltage of the mechanism
     * @param   voltage    target voltage
     */
    public void setVoltage(double voltage) {
        set_voltage_cmd.setVoltage(voltage);
        if(getCurrentCommand() != set_voltage_cmd) set_voltage_cmd.schedule();
    }

    /**
     * Sets the rate of the mechanism
     * @param   rate    target rate
     */
    public void setRate(double rate) {
        if(rate != 0) {
            set_rate_cmd.setRate(rate);
            if(getCurrentCommand() != set_rate_cmd) set_rate_cmd.schedule();
        } else {
            holdPosition();
        }
    }

    /**
     * Sets the target position of the mechanism
     * @param   position    target position
     */
    public void setPosition(double position) {
        //TODO:Allow tolerance to be updated
        set_pos_cmd.setTarget(position);
        if(getCurrentCommand() != set_pos_cmd) set_pos_cmd.schedule();
    }

    /**
     * Sets the mechanism to hold its current position
     */
    public void holdPosition() {
        Command current_cmd = getCurrentCommand();
        if(current_cmd != getDefaultCommand()) cur_command.cancel();
    }

    
    /******************************/
    /* Command Generation Methods */
    /******************************/

    /**
     * Generate a set position command. Mechanism default tolerances used.
     * @param   target  target position
     * @return  set position command
     */
    public SetPositionCommand getSetPositionCommand(double target) {
        return new SetPositionCommand(this, target);
    }

    /**
     * Generate a set position command
     * @param   target      target position
     * @param   tolerance   distance from the target position that is considered "at target"
     * @return  set position command
     */
    public SetPositionCommand getSetPositionCommand(double target, Limit tolerance) {
        return new SetPositionCommand(this, target, tolerance);
    }   

    /**
     * Generate a set position command
     * @param   target          target position
     * @param   auto_complete   If true, command automatically finishes when atTarget is true.
     * @return  set position command
     */
    public SetPositionCommand getSetPositionCommand(double target, boolean auto_complete) {
        return new SetPositionCommand(this, target, auto_complete);
    }   

    /**
     * Generate a set position command
     * @param   target      target position
     * @param   tolerance   distance from the target position that is considered "at target"
     * @param   auto_complete   If true, command automatically finishes when atTarget is true.
     * @return  set position command
     */
    public SetPositionCommand getSetPositionCommand(double target, Limit tolerance, boolean auto_complete) {
        return new SetPositionCommand(this, target, tolerance, auto_complete);
    }   

    /*****************************/
    /* Protected Control Methods */
    /*****************************/

    /**
     * Calculates the rate to reach the target position
     * @param   target_pos  target position
     * @return  rate to reach the desired position
     */
    protected double getPosTrackingRate(double target_pos) {
        double current_pos = getPosition();
        double current_rate = getRate();

        return pos_ctrl.update(current_pos, current_rate, target_pos);
    }

    /**
     * Updates the motor output to reach the target rate
     * @param   target_rate     target rate
     */
    protected void updateRate(double target_rate) {
        double current_pos = getPosition();
        double current_rate = getRate();

        // Set target rate to 0 if it would cause the mechanism to go out of range
        if(!pos_ctrl.settings.is_cont) {
            if(atLowerLimit() && target_rate < 0) target_rate = 0;
            if(atUpperLimit() && target_rate > 0) target_rate = 0;
        }
        
        updateVoltage(rate_ctrls[cur_rate_ctrl].update(current_pos, current_rate, target_rate));
    }

    protected void updateVoltage(double voltage) {
        // Set voltage to zero if it would cause the mechanism to go out of range
        if(!pos_ctrl.settings.is_cont) {
            if(atLowerLimit() && voltage < 0) voltage = 0;
            if(atUpperLimit() && voltage > 0) voltage = 0;
        }

        setMotorVoltage(voltage);
    }

    /****************************/
    /* Protected Access Methods */
    /****************************/

    /**
     * Get shuffleboard layout for the mechanism
     * @return  Shuffleboard layout for the mechanism
     */
    protected ShuffleboardLayout getSBLayout() {
        return sb_layout;
    }

    /*********************/
    /* Subsystem Methods */
    /*********************/

    public void periodic() {
        updateUI();
    }

    /***************************/
    /* Periodic Helper Methods */
    /***************************/
    public void updateUI() {
        sb_position.setDouble(getPosition());
        sb_rate.setDouble(getRate());

        for(int i = 0; i < motor_count; i++) {
            sb_voltages[i].setDouble(getMotorVoltage(i));
            sb_currents[i].setDouble(getMotorCurrent(i));
        }
        
        sb_atLowerLimitSensor.setBoolean(atLowerLimitSensor());
        sb_atLowerSoftLimit.setBoolean(atLowerSoftLimit());
        sb_atUpperLimitSensor.setBoolean(atUpperLimitSensor());
        sb_atUpperSoftLimit.setBoolean(atUpperSoftLimit());
    }

    /********************/
    /* Abstract Methods */
    /********************/
    public abstract double getPosition();
    public abstract double getRate();
    public abstract double getMotorVoltage();
    public abstract double getMotorCurrent();

    public boolean atLowerLimitSensor() { return false }
    public boolean atUpperLimitSensor() { return false }

    public abstract void setMotorVoltage(double voltage);
}