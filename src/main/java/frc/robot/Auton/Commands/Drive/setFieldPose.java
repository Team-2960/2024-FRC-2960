package frc.robot.Auton.Commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;


public class setFieldPose extends Command {
    Pose2d field_pose;

    public setFieldPose(Pose2d field_pose) {
        this.field_pose = field_pose;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void execute() {
        Drive.getInstance().presetPosition(field_pose);
    }
}