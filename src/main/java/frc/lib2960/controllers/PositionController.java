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

package frc.lib2960.controllers;

import frc.lib2960.util.*;

import edu.wpi.first.wpilibj.TimedRobot;

/**
 * Position Controller Class
 */
public class PositionController {
    /**
     * Position Controller Settings
     */
    public class Settings {
        public final double max_accel;      /**< Maximum acceleration rate */
        public final double max_decel;      /**< Maximum deceleration rate */
        public final double max_rate;       /**< Maximum rate */

        public final boolean is_cont;       /**< Continuous position range  */
        public final Limits cont_range;     /**< Range of a continuous position controller */

        // TODO Move to global settings
        public final double period;         /**< Update period */
        
        /**
         * Constructor
         * @param   max_accel   Maximum acceleration rate
         * @param   max_decel   Maximum deceleration rate
         * @param   max_rate    Maximum rate
         * @param   is_cont     Continuous position roll over flag. Set to true to allow roll over 
         *                          at lower and upper limits set in cont_range. Limits ignored if
         *                          False.
         * @param   cont_range  Range of a continuous position controller
         * @param   period      Update period of the controller
         */
        public Settings(double max_accel, double max_decel, double max_rate, boolean is_cont, Limits cont_range, double period) {
            this.max_accel = max_accel;
            this.max_decel = max_decel;
            this.max_rate = max_rate;
            this.is_cont = is_cont;
            this.cont_range = cont_range;
            this.period = period;
        }
        
        /**
         * Constructor. Period set to TimeRobot.kDefaultPeriod.
         * @param   max_accel   Maximum acceleration rate
         * @param   max_decel   Maximum deceleration rate
         * @param   max_rate    Maximum rate
         * @param   is_cont     Continuous position roll over flag. Set to true to allow roll over 
         *                          at Minimum amd Maximum positions. Limits ignored if False.
         * @param   cont_range  Range of a continuous position controller
         */
        public Settings(double max_accel, double max_decel, double max_rate, boolean is_cont, Limits cont_range) {
            this.max_accel = max_accel;
            this.max_decel = max_decel;
            this.max_rate = max_rate;
            this.is_cont = is_cont;
            this.cont_range = cont_range;
            this.period = TimedRobot.kDefaultPeriod;
        }
        
        /**
         * Constructor. 
         *      - cont_range is set to (0, 0)
         *      - is_cont is set to False
         *      - period set to TimeRobot.kDefaultPeriod.
         * @param   max_accel   Maximum acceleration rate
         * @param   max_decel   Maximum deceleration rate
         * @param   max_rate    Maximum rate
         */
        public Settings(double max_accel, double max_decel, double max_rate) {
            this.max_accel = max_accel;
            this.max_decel = max_decel;
            this.max_rate = max_rate;
            this.is_cont = false;
            this.cont_range = new Limits(0,0);
            this.period = TimedRobot.kDefaultPeriod;
        }
    }

    public final Settings settings;    /**< Position Controller Settings */

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
            double range = this.settings.cont_range.getRange();
            double error_low = target_pos - range - current_pos;
            double error_high = target_pos + range - current_pos;
            
            if(Math.abs(error) > Math.abs(error_low)) error = error_low;
            if(Math.abs(error) > Math.abs(error_high)) error = error_high;
        }

        double dir = error > 0 ? 1 : -1;

        // Calculate target Rate
        double rate = dir * settings.max_rate;
        double accel_rate = dir * settings.max_accel * settings.period + current_rate;
        
        double decel_dist = Math.pow(settings.max_rate, 2) / (2 * settings.max_decel);
        double decel_rate = error / decel_dist  * settings.max_decel;
        
        if(Math.abs(rate - accel_rate) > settings.max_rate) rate = accel_rate;
        if(Math.abs(rate) > Math.abs(accel_rate)) rate = accel_rate;
        if(Math.abs(rate) > Math.abs(decel_rate)) rate = decel_rate;

        return rate;
    }
}