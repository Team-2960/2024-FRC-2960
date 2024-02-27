package frc.robot.Auton.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class alignToPoint extends Command {
    Translation2d targetPoint;
    Rotation2d offsetAngle;
    double speed;
    Rotation2d tolerance;

    Drive drive;

    public alignToPoint(Translation2d targetPoint, double speed, Rotation2d tolerance) {
        this.alignToPoint(targetPoint, Rotaiton2d(), speed, tolerance);
    }

    public alignToPoint(Translation2d targetPoint, Rotaiton2d offsetAngle, double speed, Rotation2d tolerance) {
        this.targetPoint = targetPoint;
        this.offsetAngle = offsetAngle;
        this.speed = speed;
        this.tolerance = tolerance;

        drive = Drive.get_instance();
    }

    public Rotation2d getTargetAngle() {
        return targetPoint.minus(drive.getEstimatedPos().getTranslation()).getAngle();
    }

    public Rotation2d getTargetError() {
        return getTargetAngle().minus(drive.getEstimatedPos().getRotation());
    }

    @Override
    public void initiaize() {
        drive.setTargetPoint(targetPoint, offsetAngle);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(getTargetError().toRadians()) < tolerance.toRadians();
    }