package frc.robot.Auton.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class goToAngle extends Command {
    Rotation2d targetAngle;
    Rotation2d tolerance;

    Drive drive;

    public goToAngle(Rotation2d targetAngle, Rotation2d tolerance) {
        this.targetAngle = targetAngle;
        this.tolerance = tolerance;

        drive = Drive.getInstance();
    }

    public Rotation2d getTargetError() {
        return targetAngle.minus(drive.getEstimatedPos().getRotation());
    }

    @Override
    public void initialize() {
        drive.setTargetAngle(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(getTargetError().getRadians()) < tolerance.getRadians();
    }
}