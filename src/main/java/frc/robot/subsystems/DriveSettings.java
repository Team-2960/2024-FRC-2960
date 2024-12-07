package frc.robot.subsystems;

import frc.lib2960.subsystems.SwerveDriveBaseSettings;

public class DriveSettings {
    public final SwerveDriveBaseSettings drive_settings;
    public final SwerveSettings[] module_settings;

    public DriveSettings(SwerveDriveBaseSettings drive_settings, SwerveSettings[] module_settings) {
        this.drive_settings = drive_settings;
        this.module_settings = module_settings;
    }
}
