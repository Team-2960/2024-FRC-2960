package frc.robot;

import frc.robot.Auton.Commands.Drive.alignToPoint;
import frc.robot.Auton.Commands.Drive.goToAngle;
import frc.robot.Util.FieldLayout;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.IntakePizzaBox;
import frc.robot.subsystems.Arm.ArmControlMode;
import frc.robot.subsystems.Climber.ClimberStates;

import java.text.FieldPosition;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class OperatorInterface extends SubsystemBase {
    // INSTANCE
    public static OperatorInterface oi = null;

    // JOYSTICKS
    private Joystick driverController;
    private Joystick operatorController;

    private int lastOpPOV;

    private Timer rumbleTimer;
    private boolean lastIsNotePresent = true;
    private boolean lastIsEndGame = false;

    // Shuffleboard Entries
     sb_driveX;
     sb_driveY;
     sb_driveR;
     sb_driveFR;

     sb_armRate;
     sb_armExtendManual;

     sb_rumblePower;
     sb_rumbleTimer;
     sb_isEndGame;

    /**
     * Constructor
     */
    private OperatorInterface() {
        // Create Joysticks
        driverController = new Joystick(0);
        operatorController = new Joystick(1);

        lastOpPOV = -1;

        // Setup Shuffleboard
        var drive_layout = Shuffleboard.getTab("OI")
                .getLayout("Drive", BuiltInLayouts.kList)
                .withSize(2, 4);
        sb_driveX = drive_layout.add("X Speed", 0).getEntry();
        sb_driveY = drive_layout.add("Y Speed", 0).getEntry();
        sb_driveR = drive_layout.add("R Speed", 0).getEntry();
        sb_driveFR = drive_layout.add("Field Relative", true).getEntry();

        var arm_layout = Shuffleboard.getTab("OI")
                .getLayout("Arm", BuiltInLayouts.kList)
                .withSize(2, 4);
        sb_armRate = arm_layout.add("Arm Manual Rate", 0).getEntry();
        sb_armExtendManual = arm_layout.add("Arm Extend", 0).getEntry();

        var rumble_layout = Shuffleboard.getTab("OI")
                .getLayout("Rumble", BuiltInLayouts.kList)
                .withSize(2, 4);
        

         sb_rumblePower = rumble_layout.add("Rumble Power", 0).getEntry();
         sb_rumbleTimer = rumble_layout.add("Rumble Timer", 0).getEntry();
         sb_isEndGame = rumble_layout.add("Is End Game", false).getEntry();
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
            updateRumble();
        }
    }

    /**
     * Updates the controls for the drivetrain
     */
    private void updateDrive() {
        Drive drive = Drive.getInstance();

        boolean slowSpeed = driverController.getRawButton(5);
        double maxSpeed = (slowSpeed ? .5 : 1) * Constants.maxSpeed;
        double maxAngleRate = (slowSpeed ? .5 : 1) * Constants.maxAngularSpeed;

        boolean fieldRelative = true;//!driverController.getRawButton(1);
        var alliance = DriverStation.getAlliance();
        double alliance_dir = alliance.isPresent() && alliance.get() == Alliance.Red ? 1 : -1;

        double xSpeed = -MathUtil.applyDeadband(driverController.getRawAxis(1), 0.1) * maxSpeed * alliance_dir;
        double ySpeed = MathUtil.applyDeadband(driverController.getRawAxis(0), 0.1) * maxSpeed * alliance_dir;
        double rSpeed = MathUtil.applyDeadband(driverController.getRawAxis(4), 0.1) * maxAngleRate;

        if (driverController.getRawButton(1)){
            drive.setTargetAngle(Rotation2d.fromDegrees(90));
        }else if(driverController.getRawButton(2)){
            drive.setTargetPoint(FieldLayout.getSpeakerPose().getTranslation(), new Rotation2d());
        }else{
            drive.setAngleRate(rSpeed);
        }

        drive.setfieldRelative(fieldRelative);
        drive.setSpeed(xSpeed, ySpeed);
        

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

        // Set Arm Presets
        if (operatorController.getRawButton(1)) {
            arm.setState("Speaker");
        } else if (operatorController.getRawButton(2)) {
            arm.setState("Amp");
        }else if(operatorController.getRawButton(3)){
            arm.setState("Climb");
        }else if(operatorController.getRawButton(4)){
            arm.setState("Intake");
        }

        // TODO Map home preset
        // TODO Map Podium shot preset 

        // Manual Arm Angle Control
        double armManual = operatorController.getRawAxis(1);
        double armManualRate = armManual * Constants.maxArmSpeed;

        if (Math.abs(armManual) > .1) {
            arm.setArmRate(armManualRate);
        } else if (arm.getControlMode() == Arm.ArmControlMode.MANUAL_RATE) {
            arm.setArmRate(0);
        } else if (arm.getControlMode() == Arm.ArmControlMode.MANUAL_VOLT) {
            arm.setArmVolt(0);
        }
        
        sb_armRate.setDouble(armManualRate);

        // Manual Arm Extension control
        int opPOVAngle = operatorController.getPOV();
        if (opPOVAngle == 0 && lastOpPOV != 0)
            arm.stepExtOut();
        if (opPOVAngle == 180 && lastOpPOV != 180)
            arm.stepExtIn();

        lastOpPOV = opPOVAngle;

        // Update shuffleboard
        sb_armExtendManual.setInteger(opPOVAngle);
    }

    /**
     * Updates the controls for the Pizzabox
     */
    private void updatePizzabox() {
        IntakePizzaBox intakePB = IntakePizzaBox.getInstance();

        if (operatorController.getRawButton(6)) {
            intakePB.setState(IntakePizzaBox.PizzaboxState.SHOOT_PREP);
        } else if (operatorController.getRawAxis(3) > .1) {
            intakePB.setState(IntakePizzaBox.PizzaboxState.SHOOT);
        } else if (operatorController.getRawButton(5)){
            intakePB.setState(IntakePizzaBox.PizzaboxState.INTAKE);
        } else if(operatorController.getRawAxis(2) > .1){
            intakePB.setState(IntakePizzaBox.PizzaboxState.REVERSE);
        } else {
            intakePB.setState(IntakePizzaBox.PizzaboxState.IDLE);
        }
    }

    /**
     * Updates the controls for the climber
     */
    private void updateClimber() {
        Climber climber = Climber.getInstance();
        // TODO implement climber controls
        if (operatorController.getRawButton(7) || driverController.getRawButton(7)) {
            climber.setClimbState(ClimberStates.CLIMB_START);
        } else if (operatorController.getRawButton(8) || driverController.getRawButton(8)) {
            climber.setClimbState(ClimberStates.CLIMB);
        } else {
            climber.setClimbState(ClimberStates.IDLE);
        }

    }

    /**
     * Updates the controller rumble state
     */
    private void updateRumble() {
        double rumblePower = 0;
        IntakePizzaBox intakePB = IntakePizzaBox.getInstance();
        DriverStation ds = DriverStation.getInstance();
        
        boolean isEndGame = ds.isTeleop() && ds.getMatchTime() >= 50;

        // Rumble the controllers at half power for .5 seconds when a note is in the intake 
        if (intakePB.isNotePresent()) {
            if(!lastIsNotePresent) rumbleTimer.reset();

            if(rumbleTimer.get() < .5) rumblePower = .5;

            // TODO update LEDs
        } 
        
        // Rumble the controllers at full power for 1 second when the end game is about to start
        if(isEndGame) {
            if(!lastIsEndGame) rumbleTimer.reset();
            if(rumbleTimer.get() < 1) rumblePower = 1;

            // TODO update LEDs
        } 

        // Update controller rumble
        driverController.setRumble(RumbleType.kBothRumble, rumblePower);
        operatorController.setRumble(RumbleType.kBothRumble, rumblePower);

        // Update state transition checks
        lastIsNotePresent = intakePB.isNotePresent();
        lastIsEndGame = isEndGame;

        // Update UI
         sb_rumblePower.setDouble(rumblePower);
         sb_rumbleTimer.setDouble(rumbleTimer.get());
         sb_isEndGame.setBoolean(isEndGame);
    }

    /**
     * Static Initializer
     * 
     * @return common instance of the OperatorInterface class
     */
    public static OperatorInterface getInstance() {
        if (oi == null) {
            oi = new OperatorInterface();
        }

        return oi;
    }
}
