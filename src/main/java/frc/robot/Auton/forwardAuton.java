package frc.robot.Auton;

import java.io.IOException;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auton.Commands.goToPosition;

public class forwardAuton extends SequentialCommandGroup {
    public forwardAuton(String url) throws IOException{
        addCommands(
            new goToPosition(0, 5, new Rotation2d(0), 4, .1),
            new goToPosition(3, 5, new Rotation2d(), 4, .1)
        );
    }
}
