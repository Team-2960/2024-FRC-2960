package frc.lib2960.Util;


public class FFParam {
    public double kS;   /**< Static gain in Volts */
    public double kV;   /**< Velocity Gain in Volts * seconds / distance */
    public double kG;   /**< Gravity Gain in Volts * seconds / distance */
    public double kA;   /**< Acceleration Gain in Volts * seconds ^ 2 / distance */

    private FFParam(double kS, double kV, double kG, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kG = kG;
        this.kA = kA;
    }

    public static FFParam simpleMotor(double kS, double kV){
        return new FFParam(kS, kV, 0, 0);
    }

    public static FFParam simpleMotor(double kS, double kV, double kA){
        return new FFParam(kS, kV, 0, kA);
    }

    public static FFParam arm(double kS, double kV, double kG){
        return new FFParam(kS, kV, kG, 0);
    }

    public static FFParam arm(double kS, double kV, double kG, double kA){
        return new FFParam(kS, kV, kG, kA);
    }

    public static FFParam elevator(double kS, double kV, double kG){
        return new FFParam(kS, kV, kG, 0);
    }

    public static FFParam elevator(double kS, double kV, double kG, double kA){
        return new FFParam(kS, kV, kG, kA);
    }
}