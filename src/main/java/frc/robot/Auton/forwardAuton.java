package frc.robot.Auton;

import java.io.IOException;

import com.ctre.phoenix6.configs.GyroTrimConfigs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Auton.Commands.goToPosition;
import frc.robot.Auton.Commands.goToAngle;

public class forwardAuton {
    public Command getCommand() {
        Command command = new SequentialCommandGroup(
            new ParallelCommandGroup(
                new goToPosition(new Translation2d(0, 5), 10, .5),
                new goToAngle(new Rotation2d(), new Rotation2d(.5))
            ),
            new ParallelCommandGroup(
                new goToPosition(new Translation2d(5.4, 5), 10, .5),
                new goToAngle(new Rotation2d(), new Rotation2d(.5))
            ),
            new ParallelCommandGroup(
                new goToPosition(new Translation2d(5, 0), 10, .5),
                new goToAngle(new Rotation2d(), new Rotation2d(.5))
            ),
            new ParallelCommandGroup(
                new goToPosition(new Translation2d(0, 0), 10, .5),
                new goToAngle(new Rotation2d(), new Rotation2d(.5))
            )
        );

        return command;
    }
}
