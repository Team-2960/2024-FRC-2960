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

package frc.lib2960.oi;

import edu.wpi.first.wpilibj.Joystick;

import frc.lib2960.util.Limits;

/**
 * Manages a Joystick treating a joystick axis as a button
*/
public class JoystickAxisToButton extends ButtonBase {
    private Joystick joystick;      /**< Joystick the axis is on */
    private int axis;               /**< Index of the axis on the joystick */
    private Limits pressed_range;   /**< Range of values considered pressed */
    private boolean invert;         /**< Inverts the axis direction */

    /**
     * Constructor
     * @param   joystick        Joystick the axis is on
     * @param   axis            Index of the axis on the joystick
     * @param   pressed_range   Range of values considered pressed
     * @param   invert          Inverts the axis direction
     */
    public JoystickAxisToButton(Joystick joystick, int axis, Limits pressed_range, boolean invert) {
        this.joystick = joystick;
        this.axis = axis;
        this.pressed_range = pressed_range
    }

    /**
     * Constructor
     * @param   joystick        Joystick the axis is on
     * @param   axis            Index of the axis on the joystick
     * @param   threshold       Minimum value considered pressed
     * @param   invert          Inverts the axis direction
     */
    public JoystickAxisToButton(Joystick joystick, int axis, double threshold, boolean invert) {
        JoystickAxisToButton(joystick, axis, new Limits(threshold, Double.MAX_VALUE), invert);
    }

    /**
     * Constructor
     *      - Limits are set to (.5, Double.MAX_VALUE)
     * @param   joystick        Joystick the axis is on
     * @param   axis            Index of the axis on the joystick
     * @param   invert          Inverts the axis direction
     */
    public JoystickAxisToButton(Joystick joystick, int axis, boolean invert) {
        JoystickAxisToButton(joystick, axis, 0.5, invert);
    }

    /**
     * Constructor
     *      - invert set to false
     * @param   joystick        Joystick the axis is on
     * @param   axis            Index of the axis on the joystick
     * @param   pressed_range   Range of values considered pressed
     */
    public JoystickAxisToButton(Joystick joystick, int axis, Limits pressed_range) {
        JoystickAxisToButton(joystick, axis, pressed_range, false);
    }

    /**
     * Constructor
     *      - invert set to false
     * @param   joystick        Joystick the axis is on
     * @param   axis            Index of the axis on the joystick
     * @param   threshold       Minimum value considered pressed
     */
    public JoystickAxisToButton(Joystick joystick, int axis, double threshold) {
        JoystickAxisToButton(joystick, axis, new Limits(threshold, Double.MAX_VALUE), false);
    }

    /**
     * Constructor
     *      - Limits are set to (.5, Double.MAX_VALUE)
     *      - invert set to false
     * @param   joystick        Joystick the axis is on
     * @param   axis            Index of the axis on the joystick
     */
    public JoystickAxisToButton(Joystick joystick, int axis) {
        JoystickAxisToButton(joystick, axis, 0.5, false);
    }


    /**
     * Checks if the button is pressed.
    * @return true if the button is considered pressed
    */
    @Override
    public boolean pressed() {
        return pressed_range.inRange(joystick.getRawAxis(button));
    }
}