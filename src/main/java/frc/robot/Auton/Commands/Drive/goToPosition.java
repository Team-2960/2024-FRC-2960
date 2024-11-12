package frc.robot.Auton.Commands.Drive;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class goToPosition extends Command {
    Translation2d targetPos;
    double speed;
    double tolerance;

    Drive drive;

    public goToPosition(double x, double y, double speed, double tolerance) {
        this(new Translation2d(x, y), speed, tolerance);
    }

    public goToPosition(Translation2d targetPos, double speed, double tolerance) {
        this.targetPos = targetPos;
        this.speed = speed;
        this.tolerance = tolerance;

        drive = Drive.getInstance();
    }

    public Translation2d getTargetError() {
        return targetPos.minus(drive.getEstimatedPos().getTranslation());
    }

    @Override
    public boolean isFinished() {
        return new Translation2d().getDistance(getTargetError()) < tolerance;
    }

    @Override
    public void execute() {
        Translation2d error = getTargetError();
        double rampDistance = .5; // TODO Move to constants

        double speed = Math.min(new Translation2d().getDistance(error) / rampDistance, 1) * this.speed;
        speed = Math.max(speed, Constants.minSpeed);
        
        SmartDashboard.putNumber("autonSpeed", speed);
        drive.setVector(speed, error.getAngle());
    }

    @Override
    public void end(boolean interrupt) {
        //if robot does not stop, set all to 0
        drive.setSpeed(0, 0);
    }
}
