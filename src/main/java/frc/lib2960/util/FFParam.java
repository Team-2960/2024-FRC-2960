package frc.lib2960.Util;

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