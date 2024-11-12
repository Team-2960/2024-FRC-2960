ackage frc.robot.Auton.Commands.Drive;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

/**
 * Sets the robot to track a point on the field
 */
public class driveToTime extends Command {
    double speed;
    Rotation2d angle;
    double time;

    Timer driveTimer = new Timer();

    Drive drive;

    public driveToTime(double speed, Rotation2d angle, double time) {
        this.speed = speed;
        this.angle = angle;
        this.time = time;
    }

    /**
     * Initialize command
     */
    @Override
    public void initialize() {
        driveTimer.restart();
    }

    /**
     * Command is finished
     */
    @Override
    public boolean isFinished() {
        // Check if the robot is at the target angle
        return driveTimer.get() >= time;
    }

    @Override
    public void execute() {
        Drive.getInstance().setVector(speed, angle);
    }

    @Override
    public void end(boolean interrupt) {
        Drive.getInstance().setSpeed(0, 0);
    }
}