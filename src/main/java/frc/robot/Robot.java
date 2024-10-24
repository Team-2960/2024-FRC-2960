// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.*;
import frc.robot.auton.commands.arm.*;
import frc.robot.auton.commands.pizzabox.*;

import frc.lib2960.commands.PPCommandGen;
import frc.lib2960.util.PIDParam;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    private Optional<Command> autonCommand;

    private Drive drive;
    private OperatorInterface oi;
    private Camera camera;
    private Arm arm;
    private Climber climber;
    private IntakePizzaBox intake;
    private Pneumatics pneumatics;
    private RobotContainer robotContainer;
    private Command autonomousCommand;

    @Override
    public void robotInit() {
        // Get instance of all core subsystems
        drive = Drive.getInstance();
        oi = OperatorInterface.getInstance();
        //camera = Camera.getInstance();
        arm = Arm.getInstance();
        climber = Climber.getInstance();
        intake = IntakePizzaBox.getInstance();
        pneumatics = Pneumatics.getInstance();
        robotContainer = new RobotContainer();

        // Start Camera Feedback
        CameraServer.startAutomaticCapture();
        
        // Initialize Path Planner
        PathPlanner.init(
            drive, 
            new PathPlanner.Settings(           // TODO Move to Constants
                new PIDParam(5.0, 0.0, 0.0),
                new PIDParam(5.0, 0.0, 0.0)
            )
        )

        // Add named commands
        PathPlanner.registerCommand("armIntake", new armToPreset("Intake"));        // TODO Register in Arm class
        PathPlanner.registerCommand("armHome", new armToPreset("home"));            // TODO Register in Arm class
        PathPlanner.registerCommand("armSpeaker", new armToPreset("Speaker"));      // TODO Register in Arm class
        PathPlanner.registerCommand("intakeNote", new intakeNote());                // TODO Register in Pizzabox Class
        PathPlanner.registerCommand("shootNote", new shootNote());                  // TODO Register in Pizzabox Class
        PathPlanner.registerCommand("prepShoot", new prepShootNote());              // TODO Register in Pizzabox Class
        PathPlanner.registerCommand("armAutoAlign", new autoAlignArm());            // TODO Register in Pizzabox Class
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        //if (autonCommand.isPresent()) autonCommand.get().schedule();
        autonomousCommand = PathPlanner.getSelectedAuto();

        if(autonomousCommand != null) autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        drive.ignoreCamera(false);
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {

    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }

}
