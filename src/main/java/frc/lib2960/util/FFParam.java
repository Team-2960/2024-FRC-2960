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

package frc.lib2960.util;

/**
 * Feed forward controller settings object
 */
public class FFParam {
    /**
     * Feed Forward Type Enum
     */
    public enum FFType {SIMPLE, ARM, ELEVATOR};

    public FFType type; /**< Feed Forward Type */

    public double kS;   /**< Static gain in Volts */
    public double kV;   /**< Velocity Gain in Volts * seconds / distance */
    public double kG;   /**< Gravity Gain in Volts * seconds / distance */
    public double kA;   /**< Acceleration Gain in Volts * seconds ^ 2 / distance */

    private FFParam(FFType type, double kS, double kV, double kG, double kA) {
        this.type = type;
        this.kS = kS;
        this.kV = kV;
        this.kG = kG;
        this.kA = kA;
    }

    public static FFParam simpleMotor(double kS, double kV){
        return new FFParam(SIMPLE, kS, kV, 0, 0);
    }

    public static FFParam simpleMotor(double kS, double kV, double kA){
        return new FFParam(SIMPLE, kS, kV, 0, kA);
    }

    public static FFParam arm(double kS, double kV, double kG){
        return new FFParam(ARM, kS, kV, kG, 0);
    }

    public static FFParam arm(double kS, double kV, double kG, double kA){
        return new FFParam(ARM, kS, kV, kG, kA);
    }

    public static FFParam elevator(double kS, double kV, double kG){
        return new FFParam(ELEVATOR, kS, kV, kG, 0);
    }

    public static FFParam elevator(double kS, double kV, double kG, double kA){
        return new FFParam(ELEVATOR, kS, kV, kG, kA);
    }
}