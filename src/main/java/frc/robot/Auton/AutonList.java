package frc.robot.Auton;

import java.io.IOException;
import java.sql.Driver;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.Auton.Commands.Drive.*;
import frc.robot.Auton.Commands.Arm.*;
import frc.robot.Auton.Commands.Pizzabox.*;
import frc.robot.Util.FieldLayout;

public class AutonList {
    public static final Command shootAndDrive = new SequentialCommandGroup(
        new setFieldRelative(true),
        new ParallelCommandGroup(
            new armToPreset("Speaker"),
            new prepShootNote()
        ),
        new shootNote(),
        new ParallelCommandGroup(
            new goToPosition(FieldLayout.getAutoClearX(), -2, 3, .1),
            new goToAngle(FieldLayout.getForwardAngle(), Rotation2d.fromDegrees(2)),
            new armToPreset("Home")
        )
    );
    

    
    public static final Map<String, Map<Alliance, Command>> auton_list = Map.of(
        "Shoot and Drive", Map.of(
            Alliance.Red, shootAndDrive, 
            Alliance.Blue, shootAndDrive
        )
    );

    public static Optional<Command> getCommands(String name) {
        Map<Alliance, Command> named_auton = auton_list.get(name);
        Command auton = null;
        var alliance = DriverStation.getAlliance();

        if(named_auton != null && alliance.isPresent()) auton = named_auton.get(alliance.get());

        return Optional.ofNullable(auton);
    }
    
    public static Optional<Command> getDefaultCommands() {
        return Optional.ofNullable(shootAndDrive);
    }
}
