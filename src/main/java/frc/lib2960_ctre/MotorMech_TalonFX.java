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

package frc.lib2960_ctre;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;


import frc.lib2960.MotorMechanismBase;

/**
 * Motor Mechanism to control the arm shoulder joint
 */
class MotorMech_TalonFX extends MotorMechanismBase {
    class Settings extends MotorMechanismBase.Settings {
        public final int[] motor_ids;           /**< List of Motor CAN IDs */
        public final boolean[] invert_motors;   /**< List of inverted flags for each motor */
        public final int quad_enc_a_port;       /**< Quadrature Encoder Digital Input Port A  */
        public final int quad_enc_b_port;       /**< Quadrature Encoder Digital Input Port B  */
        public final boolean invert_quad_enc;   /**< Invert the quadrature encoder */
        public final int abs_enc_port;          /**< Absolute Encoder Digital Input Port  */
        public final boolean invert_abs_enc;    /**< Invert the absolute encoder */
        public final int enc_dpp;               /**< Quadrature Encoder Pulse per revolution */
        public final double zero_offset;        /**< Zero position offset */
        
        /**
         * Constructor. All values should be in degrees.
         * @param   name                Name of the mechanism
         * @param   tab_name            ShuffleBoard tab name
         * @param   pos_ctrl            Position Controller settings
         * @param   rate_ctrls          List of Rate Controller settings
         * @param   soft_limits         List of soft limits
         * @param   def_tol             Default position tolerances
         * @param   motor_ids           List of Motor CAN IDs
         * @param   invert_motors       List of inverted flags for each motor
         * @param   quad_enc_a_port     Quadrature Encoder Digital Input Port A
         * @param   quad_enc_b_port     Quadrature Encoder Digital Input Port B
         * @param   invert_quad_enc     Invert the quadrature encoder
         * @param   abs_enc_port        Absolute Encoder Digital Input Port
         * @param   invert_abs_enc      Invert the absolute Encoder
         * @param   enc_dpp             Quadrature Encoder distance per pulse
         * @param   zero_offset         Zero position offset;
         */
        public Settings(String name, String tab_name, PositionController.Settings pos_ctrl, 
                    RateController.Settings[] rate_ctrls, Limits[] soft_limits, Limits def_tol,
                    int[] motor_ids, boolean invert_motors[],
                    int quad_enc_a_port, int quad_enc_b_port, boolean invert_quad_enc,
                    int abs_enc_port, boolean invert_abs_enc,
                    int enc_dpp, double zero_offset) {
            
            super(name, tab_name, pos_ctrl, rate_ctrls, soft_limits, def_tol);

            this.motor_ids = motor_ids;
            this.invert_motors = invert_motors;
            this.quad_enc_a_port = quad_enc_a_port;
            this.quad_enc_b_port = quad_enc_b_port;
            this.invert_quad_enc = invert_quad_enc;
            this.abs_enc_port = abs_enc_port;
            this.invert_abs_enc = invert_abs_enc;
            this.enc_dpp = enc_dpp;
            this.zero_offset = zero_offset;

        }
    }
    
    private TalonFX[] motors;               /**< List of motors */

    private Encoder quad_encoder;           /**< Quadrature Encoder */
    private DutyCycleEncoder abs_encoder;   /**< Absolute Encoder */

    /**
     * Constructor
     * @param   settings    Settings object
     */
    public MotorMech_TalonFX(Settings settings) {
        super(settings, settings.motor_ids.length);

        // Initialize Motors
        int motor_count = settings.motor_ids.length;
        motors = new TalonFX[motor_count];

        for(int i = 0; i < motor_count; i++) {
            motors[i] = new TalonFX(settings.motor_ids[i]);
            motors[i].setInverted(settings.invert_motors[i]);
        }
        
        // Initialize Encoders
        abs_encoder = new DutyCycleEncoder(settings.abs_enc_port);

        quad_encoder = new Encoder(settings.quad_enc_a_port, 
                                    settings.quad_enc_b_port, 
                                    settings.invert_quad_enc);
        quadArmEncoder.setDistancePerPulse(settings.enc_dpp);

        // Initialize parent class
    }

    /**
     * Sets brake mode on the motors
     * @param   enable  true to enable brake mode. False to disable brake mode.
     */
    public void setBrakeMode(boolean enabled) {
        var motor_configs = new MotorOutputConfigs();

        motorConfigs.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;

        for(var motor : motors) motor.getConfigurator().apply(motor_configs);
    }

    /**
     * Get the current position
     * @return  current position
     */
    @Override
    public double getPosition() {
        double current_pos = Rotation2d.fromRotations(abs_encoder.get).getDegrees();

        // Apply zero offset and invert value if necessary
        if(settings.invert_abs_enc) {
            current_pos = settings.zero_offset - current_pos();
        } else {
            current_pos = current_pos - settings.zero_offset;
        }

        return current_pos;
    }

    /**
     * Get the current rate
     * @return  current rate
     */
    @Override
    public double getRate() {
        return quad_encoder.getRate();
    }

    /**
     * Get the current motor voltage.
     * @param   motor_index     Index of the motor to get the current voltage from.
     * @return  current motor voltage
     */
    @Override
    public double getVoltage(int motor_index) {
        double voltage = 0;

        if(motor_index < motors.size()){
            voltage = motor[motor_index].getMotorVoltage().getValueAsDouble();
        } 

        return voltage;
    }

    /**
     * Get the current motor current.
     * @param   motor_index     Index of the motor to get the current current from.
     * @return  current motor current
     */
    @Override
    public double getCurrent(int motor_index) {
        double current = 0;

        if(motor_index < motors.size()) {
            current = motor[motor_index].getTorqueCurrent().getValueAsDouble();
        }

        return current;
    }


    /**
     * Sets the motor voltages
     * @param   voltage     motor voltage to set
     */
    @Override
    public void setVoltage(double voltage) {
        for(int i = 0; i < motors.size(); i++) motors[i].setVoltage(voltage);
    }
}