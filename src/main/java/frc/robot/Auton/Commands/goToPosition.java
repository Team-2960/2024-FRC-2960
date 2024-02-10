package frc.robot.Auton.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class goToPosition extends Command {
    boolean isFinished;
    double xTarget;
    double yTarget;
    Rotation2d thetaTarget;
    double speed;
    double tolerance;
    Drive drive;
    Pose2d currentPos;

    

    public goToPosition(double xPos, double yPos, Rotation2d T, double speed, double tolerance) {
        this.xTarget = xPos;
        this.yTarget = yPos;
        this.thetaTarget = T;
        this.speed = speed;
        this.tolerance = tolerance;

        drive = Drive.get_instance();
    }

    @Override
    public void initialize() {
        currentPos = drive.getOdomXYPos();
    }

    @Override
    public boolean isFinished() {
        //Pose2d currentPos = drive.getOdomXYPos();
        double tError = thetaTarget.getRadians() - currentPos.getRotation().getRadians();
        return (getDistToTarget()<tolerance) && (Math.abs(tError)<tolerance);
    }

    public double getDistToTarget(){
        //Pose2d currentPos = drive.getOdomXYPos();
        double xError = xTarget - currentPos.getX();
        double yError = yTarget - currentPos.getY();
        return Math.sqrt(Math.pow(xError, 2) + Math.pow(yError, 2));
        
    }

    @Override
    public void execute() {
        currentPos = drive.getOdomXYPos();
        double xError = xTarget - currentPos.getX();
        double yError = yTarget - currentPos.getY();
        double tError = currentPos.getRotation().getRadians() - thetaTarget.getRadians();

        Rotation2d headingAngle = new Rotation2d(Math.atan2(yError, xError));

        // Most efficient way to get to target theta
        double complementaryTheta = 2 * Math.PI - (Math.abs(tError));
        double compareThetaError = Math.min(Math.abs(tError), complementaryTheta);
        double direction;
        double rampDistance = 1;

        if (tError < 0) {
            direction = -1;
        } else {
            direction = 1;
        }

        if (compareThetaError == complementaryTheta) {
            direction *= -1;
        }
        double finalError = compareThetaError * direction;
        double thetaVelocity = finalError * (getDistToTarget()/rampDistance);

        double rampDownSpeed = (getDistToTarget() / Constants.autonRampDownSpeed)  * speed;
        speed = Math.min(speed, rampDownSpeed);
        speed = Math.max(speed, Constants.minSpeed);
        SmartDashboard.putNumber("autonSpeed", speed);
        drive.setVector(speed, headingAngle, thetaVelocity);
    }

    @Override
    public void end(boolean interrupt) {
        //if robot does not stop, set all to 0
        drive.set_speed(0, 0, 0, true);
    }
}
