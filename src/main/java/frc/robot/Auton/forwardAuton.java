package frc.robot.Auton;

import java.io.IOException;

import com.ctre.phoenix6.configs.GyroTrimConfigs;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auton.Commands.goToPosition;

public class forwardAuton extends SequentialCommandGroup {
    public forwardAuton(String url) throws IOException{
        addCommands(
            new goToPosition(0, 5, new Rotation2d(0), 10, .5),
            new goToPosition(5.4, 5, new Rotation2d(), 10, .5),
            new goToPosition(5, 0, new Rotation2d(), 10, 0.5),
            new goToPosition(0, 0, new Rotation2d(), 10, 0.1)
          /*  new goToPosition(-1.09, 1.8, new Rotation2d(0), 10, 0.2),
           new goToPosition(-1.09, 5.28, new Rotation2d(), 10, 0.2),
           new goToPosition(5.2, 5.28, new Rotation2d(), 10, .2)*/

           //new goToPosition(0, 0, new Rotation2d(), 5, .1)
        );

    }
}
