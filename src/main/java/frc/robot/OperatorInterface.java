package frc.robot;

import frc.robot.Util.FieldLayout;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.IntakePizzaBox;
import frc.robot.subsystems.Climber.ClimberStates;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import frc.lib2960.oi.*;

public class OperatorInterface extends SubsystemBase {
    // INSTANCE
    public static OperatorInterface oi = null;

    // JOYSTICKS
    private Joystick driver_ctrl;
    private Joystick op_ctrl;

    private int lastOpPOV;

    // Joystick Button Objects
    private ButtonBase slow_speed_btn = new JoystickButton(driver_ctrl, 5);

    private ButtonBase track_speaker_btn = new JoystickButton(driver_ctrl, 1);
    private ButtonBase align_speaker_btn = new JoystickButton(driver_ctrl, 2);
    private ButtonBase center_robot_btn = new JoystickButton(driver_ctrl, 3);

    private ButtonBase zero_pose_btn = new JoystickPOVToButton(driver_ctrl, JoystickPOVToButton.UP);

    private ButtonBase speaker_preset_btn = new JoystickButton(op_ctrl, 1);
    private ButtonBase line_speaker_preset_btn = new JoystickButton(op_ctrl, 2);
    private ButtonBase amp_preset_btn = new JoystickButton(op_ctrl, 3);
    private ButtonBase intake_preset_btn = new JoystickButton(op_ctrl, 4);
    private ButtonBase arm_auto_align_btn = new JoystickPOVToButton(op_ctrl, JoystickPOVToButton.RIGHT);
    private ButtonBase home_preset_btn = new JoystickPOVToButton(op_ctrl, JoystickPOVToButton.LEFT);

    private ButtonBase manual_extend_btn = new JoystickPOVToButton(op_ctrl, JoystickPOVToButton.UP);
    private ButtonBase manual_retract_btn = new JoystickPOVToButton(op_ctrl, JoystickPOVToButton.DOWN);

    private ButtonBase fast_shoot_btn = new JoystickButton(driver_ctrl, 6);
    private ButtonBase shoot_btn = new ButtonOrGroup(
        new JoystickButton(driver_ctrl, 6),
        new JoystickAxisToButton(op_ctrl, 3, .1)
    );
    private ButtonBase intake_btn = new ButtonOrGroup(
        new JoystickAxisToButton(driver_ctrl, 3, .1),
        new JoystickButton(op_ctrl, 5)
    );
    private ButtonBase intake_rev_btn = new JoystickAxisToButton(op_ctrl, 2, .2);
    private ButtonBase shoot_prep_btn = new JoystickAxisToButton(op_ctrl, 5, .1);

    private ButtonBase climb_start_btn = new ButtonOrGroup(
        new JoystickButton(driver_ctrl, 7),
        new JoystickButton(op_ctrl, 7)
    );
    private ButtonBase climb_btn = new ButtonOrGroup(
        new JoystickButton(driver_ctrl, 8),
        new JoystickButton(op_ctrl, 8)
    );

    // LED Control
    int led_count = 69;
    AddressableLED leds;
    AddressableLEDBuffer led_idle;
    AddressableLEDBuffer led_note;
    AddressableLEDBuffer led_endgame1;
    AddressableLEDBuffer led_endgame2;
    Timer ledTimer = new Timer();

    // Robot State Tracking
    private Timer rumbleTimer = new Timer();
    private boolean lastIsNotePresent = true;
    private boolean lastIsEndGame = false;

    // Shuffleboard Entries
    private GenericEntry sb_driveX;
    private GenericEntry sb_driveY;
    private GenericEntry sb_driveR;
    private GenericEntry sb_driveFR;

    private GenericEntry sb_armRate;
    private GenericEntry sb_armExtendManual;

    private GenericEntry sb_rumblePower;
    private GenericEntry sb_rumbleTimer;
    private GenericEntry sb_isEndGame;

    /**
     * Constructor
     */
    private OperatorInterface() {
        // Create Joysticks
        driver_ctrl = new Joystick(0);
        op_ctrl = new Joystick(1);

        lastOpPOV = -1;

        // Setup LEDs
        leds = new AddressableLED(0);
        leds.setLength(led_count);

        led_idle = new AddressableLEDBuffer(led_count);
        led_note = new AddressableLEDBuffer(led_count);
        led_endgame1 = new AddressableLEDBuffer(led_count);
        led_endgame2 = new AddressableLEDBuffer(led_count);

        for (int i = 0; i < led_count; i++) {
            led_idle.setRGB(i, 148, 148, 148);
            led_note.setRGB(i, 0, 0, 127);

            if (i % 2 == 0) {
                led_endgame1.setRGB(i, 255, 255, 255);
                led_endgame2.setRGB(i, 0, 0, 127);
            } else {

                led_endgame1.setRGB(i, 0, 0, 127);
                led_endgame2.setRGB(i, 255, 255, 255);
            }
        }

        leds.setData(led_idle);
        leds.start();

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
            updateDriverFeedback();
        }
    }

    /**
     * Updates the controls for the drivetrain
     */
    private void updateDrive() {
        Drive drive = Drive.getInstance();

        double maxSpeed = (slow_speed_btn.pressed() ? .5 : 1) * Constants.maxSpeed;
        double maxAngleRate = (slow_speed_btn.pressed() ? .5 : 1) * Constants.maxAngularSpeed;

        boolean fieldRelative = true;// !driverController.getRawButton(1);
        var alliance = DriverStation.getAlliance();
        double alliance_dir = alliance.isPresent() && alliance.get() == Alliance.Red ? 1 : -1;

        double xSpeed = MathUtil.applyDeadband(driver_ctrl.getRawAxis(1), 0.05) * maxSpeed * alliance_dir;
        double ySpeed = MathUtil.applyDeadband(driver_ctrl.getRawAxis(0), 0.05) * maxSpeed * alliance_dir;
        double rSpeed = MathUtil.applyDeadband(driver_ctrl.getRawAxis(4), 0.05) * maxAngleRate * -1;
        
        if (track_speaker_btn.pressed()) {
            drive.setTargetAngle(Rotation2d.fromDegrees(-90));
        } else if (align_speaker_btn.pressed()) {
            drive.setTargetPoint(
                FieldLayout.getSpeakerPose().getTranslation(), 
                FieldLayout.getSpeakerPose().getRotation()
            );
        } else if (center_robot_btn.pressed()){
            drive.setTargetPoint(new Translation2d(0,0), Rotation2d.fromDegrees(180));
        } else {
            drive.setAngleRate(rSpeed);
        }


        if (zero_pose_btn.pressed()) {
            drive.presetPosition(new Pose2d(0.0, 0.0, new Rotation2d()));
        }
        
        climber.setRatchet(slow_speed_btn.pressed());

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
        if (speaker_preset_btn.risingEdge()) arm.setState("Speaker");
        if (line_speaker_preset_btn.risingEdge()) arm.setState("lineSpeaker");
        if (amp_preset_btn.risingEdge()) arm.setState("Amp");
        if (intake_preset_btn.risingEdge()) arm.setState("Intake");
        if (arm_auto_align_btn.risingEdge()) arm.armAutoAlign();
        if (home_preset_btn.risingEdge()) arm.setState("home");
        
        // Manual Arm Angle Control
        double armManual = MathUtil.applyDeadband(op_ctrl.getRawAxis(1), 0.1);
        double armManualRate = armManual * Constants.maxArmSpeed;
        
        arm.setArmRate(armManualRate);
        
        sb_armRate.setDouble(armManualRate);

        // Manual Arm Extension control
        if (manual_extend_btn.risingEdge()) arm.stepExtOut();
        if (manual_retract_btn.risingEdge()) arm.stepExtIn();


        // Update shuffleboard
        sb_armExtendManual.setInteger(opPOVAngle);
    }

    /**
     * Updates the controls for the Pizzabox
     */
    private void updatePizzabox() {
        IntakePizzaBox intakePB = IntakePizzaBox.getInstance();

        if (fast_shoot_btn.pressed()) {
            intakePB.setState(IntakePizzaBox.PizzaboxState.FAST_SHOOT);
        } else if (shoot_btn.pressed()) {
            intakePB.setState(IntakePizzaBox.PizzaboxState.SHOOT);
        } else if (intake_btn.pressed()) {
            intakePB.setState(IntakePizzaBox.PizzaboxState.INTAKE);
        } else if (intake_rev_btn.pressed()) {
            intakePB.setState(IntakePizzaBox.PizzaboxState.REVERSE);
        } else if (shoot_prep_btn.pressed()) {
            intakePB.setState(IntakePizzaBox.PizzaboxState.SHOOT_PREP);
        } else {
            intakePB.setState(IntakePizzaBox.PizzaboxState.IDLE);
        }
    }

    /**
     * Updates the controls for the climber
     */
    private void updateClimber() {
        Climber climber = Climber.getInstance();
        
        if (climb_start_btn.pressed()) {
            climber.setClimbState(ClimberStates.CLIMB_START);
        } else if (climb_button.pressed()) {
            climber.setClimbState(ClimberStates.CLIMB);
        } else {
            climber.setClimbState(ClimberStates.IDLE);
        }
    }

    /**
     * Updates the driver feedback state
     */
    private void updateDriverFeedback() {
        double rumblePower = 0;
        IntakePizzaBox intakePB = IntakePizzaBox.getInstance();
        AddressableLEDBuffer ledColor = led_idle;

        boolean isEndGame = DriverStation.isTeleop() && DriverStation.getMatchTime() <= 50 && DriverStation.getMatchType() != MatchType.None;

        // Rumble the controllers at half power for .5 seconds when a note is in the
        // intake
        if (intakePB.isNotePresent()) {
            if (!lastIsNotePresent) {
                rumbleTimer.restart();
                ledTimer.restart();
            }

            if (rumbleTimer.get() < .5)
                rumblePower = .5;

            ledColor = led_note;
        }

        // Rumble the controllers at full power for 1 second when the end game is about
        // to start
        if (isEndGame) {
            if (!lastIsEndGame) {
                rumbleTimer.restart();
                ledTimer.restart();
            }

            // if(rumbleTimer.get() < 1) rumblePower = 1;TODO add this back later

            double ledTime = ledTimer.get();
            if (ledTime < 1) {
                if (ledTime % .2 < .1) {
                    ledColor = led_endgame1;
                } else {
                    ledColor = led_endgame2;
                }
            }
        }

        // Update controller rumble
        driverController.setRumble(RumbleType.kBothRumble, rumblePower);
        operatorController.setRumble(RumbleType.kBothRumble, rumblePower);

        // Update LEDs
        leds.setData(ledColor);

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
