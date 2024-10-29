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

import edu.wpi.first.math.controller.*;

/**
 * Implements rate control for mechanisms
 */
public abstract class RateController {
    /**
     * Rate Controller Settings
     */
    public class Settings {
        public final FFParam ff;     /**< Feed Forward controller parameters */
        public final PIDParam pid;   /**< PID controller parameters */

        public Settings(FFParam ff, PIDParam pid) {
            this.ff = ff;
            this.pid = pid;
        }
    }

    /**
     * Generalization of the Feed Forward class to allow overloading
     */
    public abstract class FFControlBase{
        public final FFParam param; /**< Feedforward Parameters */
        
        /**
         * Constructor
         * @param   param   Feedforward Parameters
         */
        public FFControlBase(FFParam param){
            this.settings = settings;
        }

        /**
         * Updates the feedforward value
         * @param   current_pos     current mechanism position
         * @param   target_rate     target mechanism rate
         */
        public double update(double current_pos, double target_rate) {
            update(current_pos, target_rate, 0);
        }

        /**
         * Updates the feedforward value
         * @param   current_pos     current mechanism position
         * @param   target_rate     target mechanism rate
         * @param   target_accel    target mechanism acceleration
         */
        public abstract double update(double current_pos, double target_rate, double target_accel);
    }

    /**
     * Generalized version of the SimpleMotorFeedforward controller
     */
    public class FFControlSimple extends FFControlBase{
        private final SimpleMotorFeedforward controller;    /**< Feedforward controller */

        
        /**
         * Constructor
         * @param   param   Feedforward Parameters
         */
        public FFControlSimple(FFParam param){
            super(param);

            // Initialize controller
            controller = new SimpleMotorFeedforward(param.kS, param.kV, param.kA);
        }

        /**
         * Updates the feedforward value
         * @param   current_pos     current mechanism position
         * @param   target_rate     target mechanism rate
         * @param   target_accel    target mechanism acceleration
         */
        public double update(double current_pos, double target_rate, double target_accel) {
            return controller.calculate(target_rate, target_accel);
        }
    }


    /**
     * Generalized version of the ArmFeedforward controller
     */
    public class FFControlArm extends FFControlBase{
        private final ArmFeedforward controller;    /**< Feedforward controller */
        
        /**
         * Constructor
         * @param   param   Feedforward Parameters
         */
        public FFControlArm(FFParam param){
            super(param);

            controller = new ArmFeedforward(param.kS, param.kG, param.kV, param.kA);
        }

        /**
         * Updates the feedforward value
         * @param   current_pos     current mechanism position
         * @param   target_rate     target mechanism rate
         * @param   target_accel    target mechanism acceleration
         */
        public double update(double current_pos, double target_rate, double target_accel) {
            controller.calcuate(current_pos, target_rate, target_accel);
        }
    }


    /**
     * Generalized version of the ElevatorFeedforward controller
     */
    public class FFControlElevator extends FFControlBase{
        private final ElevatorFeedforward controller;    /**< Feedforward controller */
        
        /**
         * Constructor
         * @param   param   Feedforward Parameters
         */
        public FFControlElevator(FFParam param){
            super(param);

            controller = new ElevatorFeedforward(param.kS, param.kG, param.kV, param.kA);
        }

        /**
         * Updates the feedforward value
         * @param   current_pos     current mechanism position
         * @param   target_rate     target mechanism rate
         * @param   target_accel    target mechanism acceleration
         */
        public double update(double current_pos, double target_rate, double target_accel) {
            controller.calcuate(current_pos, target_rate, target_accel);
        }
    }

    protected final Settings settings;      /**< Controller settings */
    
    private final PIDController pidControl; /**< PID Controller object */
    private final FFControlBase ffControl;  /**< Feed Forward controller object */
    
    /**
     * Constructor
     * @param   settings    Rate Controller Settings
     */
    public RateController (Settings settings) {
        self.settings = settings;

        // Initialize PID Controller
        pidControl = PIDController(settings.pid.kP, settings.pid.kI, settings.pid.kD)
        
        // Initialize Feedforward controller
        if(settings.ff.type == FFParam.FFType.SIMPLE) {
            ffControl = new FFControlSimple(settings.ff);
        } else if(settings.ff.type == FFParam.FFType.ARM) {
            ffControl = new FFControlArm(settings.ff);
        } else if(settings.ff.type == FFParam.FFType.ELEVATOR) {
            ffControl = new FFControlElevator(settings.ff);
        }
    }

    /**
     * Update the controller output value
     */
    public double update(double current_pos, double current_rate, double target_rate) {
        return update_ff(current_pos, target_rate) + update_pid(current_rate, target_rate); 
    }

    /**
     * Update PID controller output
     * @param   current_rate    current rate of the mechanism
     * @param   target_rate     target rate of the mechanism
     * @return  output for the mechanism
     */
    private double update_pid(double current_rate, double target_rate) {
        return pidControl.calculate(current_rate, target_rate);
    }

    /**
     * Update PID controller output
     * @param   current_pos     current position of the mechanism
     * @param   target_rate     target rate of the mechanism
     * @return  output for the mechanism
     */
    private double update_ff(double current_pos, double target_rate) {
        return ffControl.update(current_pos, target_pos);
    }
}