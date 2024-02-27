package frc.robot.Auton.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class goToPosition extends Command {
    Translation2d targetPos;
    double tolerance;

    Drive drive;

    public goToPosition(Translation2d targetPos, double tolerance) {
        this.targetPos = targetPos;
        this.tolerance = tolerance;

        drive = Drive.get_instance();
    }

    public Translation2d getTargetError() {
        return targetPos.minus(drive.getEstimatedPos().getTranslation());
    }

    @Override
    public boolean isFinished() {
        return getTargetError().getDistance() < tolerance;
    }

    @Override
    public void execute() {
        Translation2d error = getTargetError();

        double speed = Math.min(error.getDistance() / rampDistance, 1) * Constants.kMaxSpeed ;
        speed = Math.max(speed, Constants.minSpeed);
        
        SmartDashboard.putNumber("autonSpeed", speed);
        drive.setVector(speed, error.GetAngle());
    }

    @Override
    public void end(boolean interrupt) {
        //if robot does not stop, set all to 0
        drive.setSpeed(0, 0);
    }
}
