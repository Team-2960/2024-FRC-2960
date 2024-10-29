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

/**
 * Base class for button object
 */
public abstract class ButtonBase {
    // TODO find improved way to update rising and falling edge checks

    /**
     * Button Pressed Edge type enumeration
     */
    public enum EdgeType {NONE, RISING, FALLING};

    private boolean last_state;     /**< Button state the last time an edge check was preformed */

    public ButtonBase() {
        last_state = pressed();
    }

    /**
     * Check an edge was detected. This will reset the last found state to the current state so 
     * all further calls this cycle will return None.
     * @return type of edge detected
     */
    public EdgeType checkEdge() {
        EdgeType result = EdgeType.NONE;
        boolean cur_state = pressed();

        if(cur_state && !last_state) result = EdgeType.RISING;
        if(!cur_state && last_state) result = EdgeType.FALLING;

        last_state = cur_state;

        return result;
    }
    
    /**
     * Checks if a Rising edge is detected. This will reset the last found state to the current 
     * state so all further calls this cycle will return False. 
     * @return true if a rising edge was detected since the last time an edge check was performed
     */
    public boolean risingEdge() {
        return checkEdge() == EdgeType.RISING;
    }
    
    /**
     * Checks if a Falling edge is detected. This will reset the last found state to the current 
     * state so all further calls this cycle will return False. 
     * @return true if a falling edge was detected since the last time an edge check was performed
     */
    public boolean fallingEdge() {
        return checkEdge() == EdgeType.falling;
    }

    /**
     * Checks if the button is pressed.
     * @return true if the button is considered pressed
     */
    protected abstract boolean pressed();
}