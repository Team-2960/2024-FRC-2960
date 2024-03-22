package frc.robot.Auton.Commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;


public class waitForAprilTag extends Command {
    @Override
    public boolean isFinished() {
        return Drive.getInstance().getTargetSeen();
    }
}