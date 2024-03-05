package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.IntakePizzaBox;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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

    private GenericEntry sb_armRate;

    /**
     * Constructor
     */
    private OperatorInterface() {
        // Create Joysticks
        driverController = new Joystick(0);
        operatorController = new Joystick(1);

        // Setup Shuffleboard
        var drive_layout = Shuffleboard.getTab("OI")
            .getLayout("Drive", BuiltInLayouts.kList)
            .withSize(2,4);
        sb_driveX = drive_layout.add("X Speed", 0).getEntry();
        sb_driveY = drive_layout.add("Y Speed", 0).getEntry();
        sb_driveR = drive_layout.add("R Speed", 0).getEntry();
        sb_driveFR = drive_layout.add("Field Relative", true).getEntry();

        var arm_layout = Shuffleboard.getTab("OI")
            .getLayout("Arm", BuiltInLayouts.kList)
            .withSize(2,4);
        sb_armRate = arm_layout.add("Arm Manual Rate", 0).getEntry();
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
        Drive drive = Drive.getInstance();

        boolean fieldRelative = !driverController.getRawButton(1);
        var alliance = DriverStation.getAlliance();
        double alliance_dir = alliance.isPresent() && alliance.get()== Alliance.Red ? 1 : -1;

        double xSpeed = -MathUtil.applyDeadband(driverController.getRawAxis(1), 0.1) * Constants.maxSpeed;
        double ySpeed = MathUtil.applyDeadband(driverController.getRawAxis(0), 0.1) * Constants.maxSpeed;
        double rSpeed = MathUtil.applyDeadband(driverController.getRawAxis(4), 0.1) * Constants.maxAngularSpeed;
        
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
        Arm arm = Arm.getInstance();

        // Arm Angle Control
        double armManual = -operatorController.getRawAxis(1);
        
        if(Math.abs(armManual) > .05){
            double armManualRate = armManual * PowerDistribution.getVoltage();
            arm.setArmVoltage(armManualRate);
            sb_armRate.setDouble(armManualRate);
        }

        // Arm Extension control
        int opPOVAngle = operatorController.getPOV();
        if(opPOVAngle == 0) arm.stepExtOut();
        if(opPOVAngle == 180) arm.stepExtOut(); 
    }

    /**
     * Updates the controls for the Pizzabox
     */
    private void updatePizzabox() {
        if(driverController.getRawButton(1)){
            IntakePizzaBox.getInstance().setState(IntakePizzaBox.PizzaboxState.INTAKE);
        }else if(driverController.getRawButton(2)){
            IntakePizzaBox.getInstance().setState(IntakePizzaBox.PizzaboxState.SHOOT);
        }else if(driverController.getRawButton(3)){
            IntakePizzaBox.getInstance().setState(IntakePizzaBox.PizzaboxState.SHOOT_PREP);
        }else if(driverController.getRawButton(4)){
            IntakePizzaBox.getInstance().setState(IntakePizzaBox.PizzaboxState.REVERSE);
        }else{
            IntakePizzaBox.getInstance().setState(IntakePizzaBox.PizzaboxState.IDLE);
        }
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
