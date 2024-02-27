package frc.robot.Auton.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

/**
 * Sets the robot to track a point on the field
 */
public class alignToPoint extends Command {
    Translation2d targetPoint;
    Rotation2d offsetAngle;
    Rotation2d tolerance;

    Drive drive;

    /**
     * Constructor
     * @param   targetPoint     Point for the robot to track
     * @param   tolerance       Angle tolerance on tracking the target
     */
    public alignToPoint(Translation2d targetPoint, Rotation2d tolerance) {
        this.alignToPoint(targetPoint, Rotaiton2d(), speed, tolerance);
    }

    /**
     * Constructor
     * @param   targetPoint     Point for the robot to track
     * @param   offsetAngle     Angle offset from target point
     * @param   tolerance       Angle tolerance on tracking the target
     */
    public alignToPoint(Translation2d targetPoint, Rotation2d offsetAngle, Rotation2d tolerance) {
        this.targetPoint = targetPoint;
        this.offsetAngle = offsetAngle;
        this.tolerance = tolerance;

        drive = Drive.get_instance();
    }

    /**
     * Gets the current target angle
     * @return  current target angle
     */
    public Rotation2d getTargetAngle() {
        return targetPoint.minus(drive.getEstimatedPos().getTranslation()).getAngle().plus(offsetAngle);
    }

    /**
     * gets the current target angle error
     * @return  current target angle error
     */
    public Rotation2d getTargetError() {
        return getTargetAngle().minus(drive.getEstimatedPos().getRotation());
    }

    /**
     * Initialize command
     */
    @Override
    public void initiaize() {
        // Set robot to target point tracking mode
        drive.setTargetPoint(targetPoint, offsetAngle);
    }

    /**
     * Command is finished
     */
    @Override
    public boolean isFinished() {
        // Check if the robot is at the target angle
        return Math.abs(getTargetError().toRadians()) < tolerance.toRadians();
    }