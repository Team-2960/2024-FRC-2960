package frc.robot.Auton.Commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;


public class setEnableCamera extends Command {
    boolean enable;

    public setEnableCamera(boolean enable) {
        this.enable = enable;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void execute() {
        Drive.getInstance().ignoreCamera(!enable);    }
}