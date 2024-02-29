package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OperatorInterface extends SubsystemBase {
    // INSTANCE
    public static OperatorInterface oi = null;

    // JOYSTICKS
    private Joystick driverController;
    private Joystick operatorController;

    // Shuffleboard Entries
    private GenericEntry sb_driveX;
    private GenericEntry sb_driveY;
    private GenericEntry sb_driveR;
    private GenericEntry sb_driveFR;

    /**
     * Constructor
     */
    private OperatorInterface() {
        // Create Joysticks
        driverController = new Joystick(0);
        operatorController = new Joystick(1);

        // Setup Shuffleboard
        var drive_layout = Shuffleboard.getTab("OI").getLayout("Drive");
        sb_driveX = drive_layout.add("X Speed", 0);
        sb_driveY = drive_layout.add("Y Speed", 0);
        sb_driveR = drive_layout.add("R Speed", 0);
        sb_driveFR = drive_layout.add("Field Relative", true);
    }

    /**
     * Subsystem Period Method
     */
    @Override
    public void periodic() {
        if (DriverStation.isTeleop()) {
            updateDrive();
            updateArm();
            updatePizzabox();
            updateClimber();
        }
    }
    
    /**
     * Updates the controls for the drivetrain
     */
    private void updateDrive() {
        boolean fieldRelative = !driverController.getRawButton(1);
        
        double alliance_dir = DriverStation.getAlliance() == Alliance.Red ? 1 : -1;

        double xSpeed = MathUtil.applyDeadband(driverController.getRawAxis(0), 0.1) * Constants.kMaxSpeed * alliance_dir;
        double ySpeed = -MathUtil.applyDeadband(driverController.getRawAxis(1), 0.1) * Constants.kMaxSpeed * alliance_dir;
        double rSpeed =  MathUtil.applyDeadband(driverController.getRawAxis(4), 0.1) * Constants.kMaxAngularSpeed;

        Drive drive = Drive.getInstance();
        
        drive.setfieldRelative(fieldRelative);
        drive.setSpeed(xSpeed, ySpeed);    
        drive.setAngleRate(rSpeed);

        // Update Shuffleboard
        sb_driveX.setDouble(xSpeed);
        sb_driveY.setDouble(ySpeed);
        sb_driveR.setDouble(rSpeed);
        sb_driveFR.setBoolean(fieldRelative);
    }

    /**
     * Updates the controls for the arm
     */
    private void updateArm() {
        // TODO Implement arm controls
    }

    /**
     * Updates the controls for the Pizzabox
     */
    private void updatePizzabox() {
        // TODO implement pizzabox controls
    }

    /**
     * Updates the controls for the climber
     */
    private void updateClimber() {
        // TODO implement climber controls
    }

    /**
     * Static Initializer
     * @return  common instance of the OperatorInterface class
     */
    public static OperatorInterface getInstance() {
        if (oi == null) {
            oi = new OperatorInterface();
        }

        return oi;
    }
}
