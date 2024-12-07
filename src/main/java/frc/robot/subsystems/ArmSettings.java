package frc.robot.subsystems;

import frc.lib2960.util.DoubleSolinoidSettings;
import frc.lib2960_ctre.MotorMechTalonFXSettings;

public class ArmSettings {
    public final MotorMechTalonFXSettings joint_settings;
    public final DoubleSolinoidSettings ext_1_settings;
    public final DoubleSolinoidSettings ext_2_settings;
    public final int brake_btn_port;

    public ArmSettings(
        MotorMechTalonFXSettings joint_settings, 
        DoubleSolinoidSettings ext_1_settings, 
        DoubleSolinoidSettings ext_2_settings, 
        int brake_btn_port
    ) {
        this.joint_settings = joint_settings;
        this.ext_1_settings = ext_1_settings;
        this.ext_2_settings = ext_2_settings;
        this.brake_btn_port = brake_btn_port;
    }
}
