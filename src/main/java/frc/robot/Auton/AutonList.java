package frc.robot.Auton;

import java.io.IOException;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.Auton.Commands.Drive.*;
import frc.robot.Auton.Commands.Arm.*;
import frc.robot.Auton.Commands.Pizzabox.*;
import frc.robot.Util.FieldLayout;

public class AutonList {
    public static final Commands shootAndDrive = new SequentialCommandGroup(
        new setFieldRelative(true),
        new ParallelCommandGroup(
            new armToPreset("Speaker"),
            new prepShootNote()
        ),
        new shootNote(),
        new ParallelCommandGroup(
            new goToPosition(FieldLayout.getAutoClearX(), -2, 3, .1),
            new goToAngle(FieldLayout.getForwardAngle()),
            new armToPreset("Home")
        )
    );


    public static final Map<String, Map<Alliance, Commands>> auton_list = Map.of(
        "Shoot and Drive", Map.of(
            Alliance.Red, shootAndDrive, 
            Alliance.Blue, shootAndDrive
        )
    );

    public static Optional<Comamnds> getCommands(String name) {
        Map<Alliance, Commands> named_auton = auton_list.get(name);
        Comamnds auton = null;

        if(named_auton != null) auton = named_auton.get(DriverStation.getAlliance());


        return Optional<Comamnds>.ofNullable(auton);
    }
}
