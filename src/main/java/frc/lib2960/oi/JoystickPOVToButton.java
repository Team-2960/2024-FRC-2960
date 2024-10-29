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
 * Manages a Joystick POV as a button
*/
public class JoystickAxisToButton extends ButtonBase {
    
    public enum Direction{
        UP(0),
        UP_RIGHT(45),
        RIGHT(90),
        DOWN_RIGHT(135),
        DOWN(180),
        DOWN_LEFT(225),
        LEFT(270),
        UP_LEFT(315);

        public final int value;

        public Direction(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    private Joystick joystick;      /**< Joystick the POV is on */
    private Direction direction;    /**< POV direction */

    /**
     * Constructor
    * @param   joystick        Joystick the button is on
    * @param   direction       POV direction
    */
    public JoystickAxisToButton(Joystick joystick, Direction direction) {
        this.joystick = joystick;
        this.axis = direction;
    }

    /**
     * Checks if the button is pressed.
    * @return true if the button is considered pressed
    */
    @Override
    public boolean pressed() {
        return joystick.getPOV() == direction.value;
    }
}