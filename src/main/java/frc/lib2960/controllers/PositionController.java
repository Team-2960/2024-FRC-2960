package frc.lib2960.controllers;

import frc.lib2960.util.*;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * Position Controller Class
 */
public class PositionController {
    /**
     * Position Controller Settings
     */
    public class Settings {
        public final double ramp_rate;       /**< Maximum rate change per second */
        public final double max_rate;        /**< Maximum rate */

        public final double min_pos;        /**< Minimum allowed position */
        public final double max_pos;        /**< Maximum allowed position */
        public final boolean is_cont;       /**< Continuous position range  */

        // TODO Move to global settings
        public final double period;         /**< Update period */
        
        /**
         * Constructor
         * @param   max_accel   Maximum acceleration rate
         * @param   max_decel   Maximum deceleration rate
         * @param   max_rate    Maximum rate
         * @param   min_pos     Minimum position for continuous roll over
         * @param   max_pos     Maximum position for continuous roll over
         * @param   is_cont     Continuous position roll over flag. Set to true to allow roll over 
         *                          at Minimum amd Maximum positions. Limits ignored if False.
         * @param   Param
         */
        public Settings(double max_accel, double max_decel, double max_rate, double min_pos, double max_pos, boolean is_cont, double period) {
            this.max_accel = max_accel;
            this.max_decel = max_decel;
            this.max_rate = maxRate;
            this.min_pos = min_pos;
            this.max_pos = max_pos;
            this.is_cont = is_cont;
            this.period = period;
        }
        
        /**
         * Constructor. Period set to TimeRobot.kDefaultPeriod.
         * @param   max_accel   Maximum acceleration rate
         * @param   max_decel   Maximum deceleration rate
         * @param   max_rate    Maximum rate
         * @param   min_pos     Minimum position for continuous roll over
         * @param   max_pos     Maximum position for continuous roll over
         * @param   is_cont     Continuous position roll over flag. Set to true to allow roll over 
         *                          at Minimum amd Maximum positions. Limits ignored if False.
         */
        public Settings(double max_accel, double max_decel, double max_rate, double min_pos, double max_pos, boolean is_cont) {
            Settings(max_accel, max_decel, max_rate, min_pos, max_pos, is_cont, TimeRobot.kDefaultPeriod)
        }
    }

    private final Settings settings;    /**< Position Controller Settings */

    /**
     * Constructor
     * @param   settings    Position Controller Settings
     */
    public PositionController(Settings settings) {
        this.settings = settings;
    }

    /**
     * Updates the position controller
     * @param   current_pos     current mechanism position
     * @param   current_rate    current mechanism rate
     * @param   target_pos      target mechanism position
     * @return  target rate for the position controller
     */
    public double update(double current_pos, double current_rate, double target_pos) {
        // Calculate error and direction to target
        double error = target_pos - current_pos;

        if(settings.is_cont) {
            double range = settings.max_pos - settings.min_pos;
            double error_low = target_pos - range - current_pos;
            double error_high = target_pos + range - current_pos;
            
            if(Math.abs(error) > abs(error_low)) error = error_low;
            if(Math.abs(error) > abs(error_high)) error = error_high;
        }

        double dir = error > 0 ? 1 : -1;

        // Calculate target Rate
        double rate = dir * settings.max_rate;
        double accel_rate = dir * settings.max_accel * settings.period + current_rate;
        
        double decel_dist = Math.pow(max_speed , 2) / (2 * settings.max_decel);
        double decel_rate = error / decel_dist  * settings.max_decel;
        
        if(Math.abs(rate - accel_rate) > settings.max_rate) rate = accel_rate;
        if(Math.abs(rate) > Math.abs(accel_rate)) rate = accel_rate;
        if(Math.abs(rate) > Math.abs(decel_rate)) rate = decel_rate;

        return rate;
    }
}