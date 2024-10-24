package frc.lib2960.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.commands.PathPLannerAuto;
import com.pathplanner.lib.util.*;

import frc.lib2960.subsystems.SwerveDriveBase;
import frc.lib2960.util.*;

/**
 * Initializes the PathPlanner system and 
 */
public class PathPlanner {

    /**
     * PathPlanner settings
     */
    public class Settings {
        public final PIDParam translation;  /**< Translation PID parameters */
        public final PIDParam rotation;     /**< Rotation PID Parameters */
    }

    private static SendableChooser<Command> autoChooser = null; /**< Auton Selector */

    /**
     * Constructor
     * @param   dt          drivetrain object reference
     * @param   settings    PPCommandGen settings
     */
    public static init(SwerveDriveBase dt, Settings settings) {
        // Initialize robot config from GUI Settings
        RobotConfig config;
        try{
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          // Handle exception as needed
          e.printStackTrace();
        }

        // Initialize Autobuilder
        AutoBuilder.configure(
            dt::getEstimatedPos,
            dt::resetPoseEst,
            dt::getRobotRelativeSpeeds,
            dt::setRobotRelativeSpeeds,
            new PPHolonomicDriveController(
                toPIDConstants(settings.translation),
                toPIDConstants(settings.rotation)
            ),
            config,
            this::isRedAlliance,
            dt
        )

        // Initialize Shuffleboard
        // TODO Allow a default auton to be set
        autoChooser = AutoBuilder.buildAutoChooser();

        var layout = Shuffleboard.getTab("Main")
            .getLayout("Auton", BuiltInLayouts.kList)
            .withSize(1, 4);
        layout.add("Auton Selector", autoChooser)
    }

    /**
     * Returns the selected auton command
     * @return the selected auton command
     */
    public static Command getSelectedAuto() {
        return autoChooser.getSelected();
    }

    /**
     * Registers commands with the PathPlanner system
     * @param   name    Name of the command to register
     * @param   command Command object to register
     */
    public static registerCommand(String name, Command command) {
         NamedCommands.registerCommand(name, command);
    }
    
    /**
     * Converts PIDParam to PIDConstants values
     * @param   pid_param   PIDParam values
     * @return  PIDConstants values
     */
    public static PIDConstants toPIDConstants(PIDParam pid_param) {
        return new PIDConstants(pid_param.kP, pid_param.kI, pid_param.kD);
    }

    /**
     * Checks if the current alliance is red
     * @return  true of current alliance is red, false otherwise
     */
    public static boolean isRedAlliance() {
        boolean result = false;
        var alliance = DriverStation.getAlliance

        if(alliance.is_present()) result = alliance.get() == DriverStation.Alliance.Red;

        return result;
    }
}