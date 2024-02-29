package frc.robot.Auton;

import java.io.IOException;

import com.ctre.phoenix6.configs.GyroTrimConfigs;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Auton.Commands.Drive.*;
import frc.robot.Auton.Commands.Arm.*;

public class forwardAuton {
    public static Command getCommand() {
        Command command = new SequentialCommandGroup(
            new setFieldRelative(true),
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
