// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.subsystems.*;

import frc.lib2960.pathplanner.PathPlanner;
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

    private Drive drive;
    private OperatorInterface oi;
    private Vision vision;
    private Arm arm;
    private Climber climber;
    private IntakePizzaBox intake;
    private Pneumatics pneumatics;

    private Command autonomousCommand;

    @Override
    public void robotInit() {
        // Get instance of all core subsystems
        drive = Drive.getInstance();
        oi = OperatorInterface.getInstance();
        vision = Vision.getInstance();
        arm = Arm.getInstance();
        climber = Climber.getInstance();
        intake = IntakePizzaBox.getInstance();
        pneumatics = Pneumatics.getInstance();
        
        // Initialize Path Planner
        PathPlanner.init(
            drive, 
            new PIDParam(5.0, 0.0, 0.0), 
            new PIDParam(5.0, 0.0, 0.0)
        );

        // Start Camera Feedback
        CameraServer.startAutomaticCapture();
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
