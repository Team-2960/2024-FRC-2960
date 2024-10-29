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

 package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsHub;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;

import frc.robot.Constants;

/**
 * Manages a REV Pneumatics Hub
 */
public class Pneumatics_RevPH extends SubsystemBase {

    /**
     * REV Pneumatics Hub settings
     */
    public class Settings {
        public static final String DEF_NAME = "Pneumatics Hub";
        public static final int DEF_CAN_ID = 1;

        public enum ControlMode {DIGITAL, ANALOG, HYBRID};

        public final String name; 
        public final int can_id;
        public final ControlMode control_mode;
        public final double min_pressure;
        public final double max_pressure;

        /**
         * Constructor
         * @param   name            name of the module
         * @param   can_id          CAN ID for the module
         * @param   control_mode    Module Control Mode
         * @param   min_pressure    Minimum allowed pressure in PSI
         * @param   max_pressure    Maximum allowed pressure in PSI
         */
        public Settings(String name, int can_id, ControlMode control_mode, double min_pressure, double max_pressure) {
            this.name = name;
            this.can_id = can_id;
            this.control_mode = control_mode;
            this.min_pressure = min_pressure;
            this.max_pressure = max_pressure;
        }

        /**
         * Constructor
         *      - name set to DEF_NAME
         *      - can_id set to DEF_CAN_ID
         *      - control_mode set to DIGITAL
         *      - min_pressure set to 0
         *      - max_pressure set to 120 
         */
        public Settings() {
            Settings(def_name, def_can_id, ControlMode.DIGITAL, 0, 120);
        }
    }


    public final Settings settings;     /**< Module Settings */
    
    private final PneumaticsHub ph;      /**< Module objet reference*/

    // ShuffleBoard
    private GenericEntry sb_pressure;   
    private GenericEntry sb_current;

    /**
     * Constructor. Settings set to default values;
     * @param   enabled     determines if the compressor is enabled when object is created
     */
    private Pneumatics_RevPH(boolean enabled) {
        Pneumatics_RevPH(new Settings());
    }

    /**
     * Constructor
     * @param   settings    Pneumatics Hub settings
     * @param   enabled     determines if the compressor is enabled when object is created
     */
    private Pneumatics_RevPH(Settings settings, boolean enabled) {
        this.settings = settings;

        // Create PneumaticsHub
        ph = PneumaticsHub(settings.can_id);

        // Setup ShuffleBoard
        var layout = Shuffleboard.getTab("Status")
                .getLayout(settings.name, BuiltInLayouts.kList)
                .withSize(1, 2);
        sb_pressure = layout.add("Pressure", 0).getEntry();
        sb_current = layout.add("Current", 0).getEntry();

        // Set Compressor Enabled
        steEnabled(enabled);
    }

    /**
     * Enables/Disables the compressor
     * @param   enabled     true to enable the compressor, false to disable.
     */
    public void enableCompressor(boolean enabled) {
        if(enabled) {
            switch(settings.control_mode) {
                case ControlMode.DIGITAL:
                    ph.enableCompressorDigital();
                    break;
                case ControlMode.ANALOG:
                    ph.enableCompressorAnalog(settings.min_pressure, settings.max_pressure);
                    break;
                case ControlMode.HYBRID:
                    ph.enableCompressorHybrid(settings.min_pressure, settings.max_pressure);
                    break;
            }
        }else{
            ph.disableCompressor();
        }
    }

    /**
     * Periodic method. Updates UI.
     */
    @Override
    public void periodic() {
        updateUI();
    }

    /**
     * Updates ShuffleBoard
     */
    public void updateUI() {
        sb_pressure.setDouble(compressor.getPressure(0));
        sb_current.setDouble(ph.getCompressorCurrent());
    }
}
