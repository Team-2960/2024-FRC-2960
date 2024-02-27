package frc.robot.Auton.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;


public class setFieldRelative extends Commands {
    boolean isFieldRelative;

    Drive drive;

    public setFieldRelative(boolean isFieldRelative) {
        this.isFieldRelative = isFieldRelative;
        this.drive = Drive.getInstance();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void execute() {
        drive.setFieldRelative(isFieldRelative);
    }
}