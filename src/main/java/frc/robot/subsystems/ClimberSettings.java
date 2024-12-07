

package frc.robot.subsystems;

import frc.lib2960.util.DoubleSolinoidSettings;
import frc.lib2960.util.MotorSettings;

public class ClimberSettings {
    public final MotorSettings[] motor_settings;
    public final DoubleSolinoidSettings latch_settings;
    public final double winch_circumfrance;
    public final int limit_index;
    public final int encoder_index;
    public final boolean encoder_invert;
    public final double max_extension;
    public final double ratchet_delay;

    public ClimberSettings(
        MotorSettings[] motor_settings, 
        DoubleSolinoidSettings latch_settings, 
        double winch_circumfrance, 
        int limit_index,
        int encoder_index,
        boolean encoder_invert,
        double max_extension,
        double ratchet_delay
    ) {

        // Check if limit_index is in range
        if(limit_index > motor_settings.length) 
            throw new RuntimeException("limit_index is out of range for Climber");

        // Check if encoder_index is in range
        if(encoder_index > motor_settings.length) 
            throw new RuntimeException("encoder_index is out of range for Climber");
        
        this.motor_settings = motor_settings;
        this.latch_settings = latch_settings;
        this.winch_circumfrance = winch_circumfrance;
        this.limit_index = limit_index;
        this.encoder_index = encoder_index;
        this.encoder_invert = encoder_invert;
        this.max_extension = max_extension;
        this.ratchet_delay = ratchet_delay;
    }
}
