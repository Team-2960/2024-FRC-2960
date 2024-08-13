package frc.robot.Auton;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auton.Commands.Arm.armToPreset;
import frc.robot.Auton.Commands.Pizzabox.intakeNote;
import frc.robot.Auton.Commands.Pizzabox.prepShootNote;
import frc.robot.Auton.Commands.Pizzabox.shootNote;
import frc.robot.subsystems.Drive;
public class RobotContainer {  
  Drive drive = Drive.getInstance();
  public Command getAutonomousCommand() {
    NamedCommands.registerCommand("armIntake", new armToPreset("Intake"));
    NamedCommands.registerCommand("armHome", new armToPreset("home"));
    NamedCommands.registerCommand("armSpeaker", new armToPreset("Speaker"));
    NamedCommands.registerCommand("intakeNote", new intakeNote());
    NamedCommands.registerCommand("shootNote", new shootNote());
    NamedCommands.registerCommand("prepShoot", new prepShootNote());
    
    // Load the path you want to follow using its name in the GUI
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return Drive.getInstance().autoBuilder.buildAuto("AS 4 Note");
    
  }

  
}
