package frc.robot.subsystems;

import frc.lib2960.util.MotorSettings;

public class IntakePizzaboxSettings {
    public final MotorSettings intake_settings;
    public final MotorSettings[] shooter_settings;
    public final int shooter_pe_port;
    public final int intake_pe_port;
    public final double min_slow_shoot_speed;
    public final double min_fast_shoot_speed;
    public final double intake_slow_volt;
    public final double intake_in_volt;
    public final double intake_out_volt;
    public final double shooter_prep_volt;
    public final double shooter_shoot_volt;
    public final double shooter_rev_volt;

    public IntakePizzaboxSettings( 
        MotorSettings intake_settings,
        MotorSettings[] shooter_settings,
        int shooter_pe_port,
        int intake_pe_port,
        double min_slow_shoot_speed,
        double min_fast_shoot_speed,
        double intake_slow_volt,
        double intake_in_volt,
        double intake_out_volt,
        double shooter_prep_volt,
        double shooter_shoot_volt,
        double shooter_rev_volt
    ) {
        this.intake_settings = intake_settings;
        this.shooter_settings = shooter_settings;
        this.shooter_pe_port = shooter_pe_port;
        this.intake_pe_port = intake_pe_port;
        this.min_slow_shoot_speed = min_slow_shoot_speed;
        this.min_fast_shoot_speed = min_fast_shoot_speed;
        this.intake_slow_volt = intake_slow_volt;
        this.intake_in_volt = intake_in_volt;
        this.intake_out_volt = intake_out_volt;
        this.shooter_prep_volt = shooter_prep_volt;
        this.shooter_shoot_volt = shooter_shoot_volt;
        this.shooter_rev_volt = shooter_rev_volt;
        
    }
}
