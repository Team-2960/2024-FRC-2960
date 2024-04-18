package frc.robot.Auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
public class RobotContainer {  
  public Command getAutonomousCommand() {
    // Load the path you want to follow using its name in the GUI
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return Drive.getInstance().autoBuilder.buildAuto("New Auto");
  }

  
}
