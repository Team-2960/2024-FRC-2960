package frc.lib2960.util;

/**
 * Handles limit checks
 */
public class Limits {
    public final double lower;  /**< Lower Limit Value */
    public final double upper;  /**< Upper Limit Value */

    /**
     * Constructor
     * @param   lower       Lower Limit Value
     * @param   upper       Upper Limit Value
     */
    public Limits(double lower, double upper) {
        this.lower = lower;
        this.upper = upper;
    }
    
    /**
     * Constructor
     * @param   nominal         Nominal value
     * @param   lower_bound     Acceptable distance below nominal value
     * @param   upper_bound     Acceptable distance above nominal value
     */
    public Limits(double nominal, double lower_bound, double upper_bound) {
        Limit(nominal - lower_bound, nominal + upper_bound);
    }
    
    /**
     * Constructor
     * @param   nominal         Nominal value
     * @param   tolerance       Acceptable range around nominal value
     */
    public Limits(double nominal, double tolerance) {
        Limits(nominal, tolerance, tolerance);
    }

    /**
     * Retrieves the range of the between the limits
     * @return  the range between the limits
     */
    public double getRange() {
        return upper - lower;
    }

    /**
     * Checks if value is above lower limit
     * @param   value
     * @return  true if value is above lower limit
     */
    public boolean inLower(double value) {
        return value >= lower;
    }

    /**
     * Checks if value is below upper limit
     * @param   value
     * @return  true if value is below upper limit
     */
    public boolean inUpper(double value) {
        return value <= upper;
    }

    /**
     * Checks if value is between the limits
     * @param   value
     * @return  true if value is between the limits
     */
    public boolean inRange(double value){
        return inLower(value) && inUpper(value);
    }
}