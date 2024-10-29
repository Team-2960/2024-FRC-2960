package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

import frc.lib2960_ctre.MotorMech_TalonFX;

import frc.lib2960_pathplanner.PathPlanner;

import frc.robot.Constants;
import frc.robot.Util.FieldLayout;

public class Arm extends SubsystemBase {
    private static Arm arm;

    /**
     * Defines an arm position state
     */
    public class ArmStateValues {
        public Rotation2d targetAngle;
        public Rotation2d angleTol;
        public int extState;

        public ArmStateValues(Rotation2d targetAngle, int extState) {
            this(targetAngle, Rotation2d.fromDegrees(2), extState);
        }

        public ArmStateValues(Rotation2d targetAngle, Rotation2d angleTol, int extState) {
            this.targetAngle = targetAngle;
            this.angleTol = angleTol;
            this.extState = Math.max(0, Math.min(2, extState));
        }
    }

    /**
     * Goto Named Preset command
     */
    public class GotoNamedPreset extends Command {
        private final Arm arm;                /**< Reference to the arm object */
        private final String preset_name;     /**< Name of the preset to goto */

        /**
         * Constructor
         * @param   arm             Reference to the arm object
         * @param   preset_name     Name of the preset to goto
         */
        public GotoNamedPreset(Arm arm, String preset_name) {
            this.arm = arm;
            this.preset_name = preset_name;
        }
        
        /**
         * initialize method. Set arm to preset state
         */
        @Override
        public void initialize() {
            arm.setState(preset_name);
        }

        /**
         * isFinished method. Check if arm is at the target position
         */
        @Override
        public boolean isFinished() {
            return arm.atTarget();
        }
    }

    /**
     * Goto Preset command
     */
    public class GotoPreset extends Command {
        private final Arm arm;                /**< Reference to the arm object */
        private final ArmStateValues preset;  /**< preset to goto */

        /**
         * Constructor
         * @param   arm             Reference to the arm object
         * @param   preset          preset to goto
         */
        public GotoNamedPreset(Arm arm, String preset) {
            this.arm = arm;
            this.preset = preset;
        }
        
        /**
         * initialize method. Set arm to preset state
         */
        @Override
        public void initialize() {
            arm.setState(preset);
        }

        /**
         * isFinished method. Check if arm is at the target position
         */
        @Override
        public boolean isFinished() {
            return arm.atTarget();
        }
    }
    
    /**
     * Auto Align command
     */
    public class AutoAlign extends Command {
        private final Arm arm;  /**< Reference to the arm object */

        /**
         * Constructor
         * @param   arm     Reference to the arm object
         */
        public AutoAlign(Arm arm){
            this.arm = arm;
        }

        /**
         * initialize method. Start arm auto alignment
         */
        @Override
        public void initialize(){
            arm.armAutoAlign();
        }
        
        /**
         * isFinished method. Checks if arm is at aligned to target.
         */
        @Override
        public boolean isFinished() {
            return arm.atTarget();
        }
    }


    public final MotorMech_TalonFX shoulder_joint;
    public final Limits climber_zone;
    
    // TODO Create subsystem to manage arm extension
    private final DoubleSolenoid armExtender1;
    private final DoubleSolenoid armExtender2;

    private final int target_ext;
    private final Timer extenderTimer;

    private final DigitalInput brakeModeDisableBtn;
    

    private Map<String, ArmStateValues> armStates = Map.of(
            "Match Start", new ArmStateValues(Rotation2d.fromDegrees(60), 0),
            "Home", defaultState,
            "Intake", new ArmStateValues(Rotation2d.fromDegrees(7), 1),
            "Speaker", new ArmStateValues(Rotation2d.fromDegrees(46), 0),
            "lineSpeaker", new ArmStateValues(Rotation2d.fromDegrees(56), 0),
            "longShot", new ArmStateValues(Rotation2d.fromDegrees(67.5), 0),
            "Amp", new ArmStateValues(Rotation2d.fromDegrees(102), 1),
            "Climb", new ArmStateValues(Rotation2d.fromDegrees(97.38), 0),
            "AmpSideShoot", new ArmStateValues(Rotation2d.fromDegrees(47), 0),
            "home", new ArmStateValues(Rotation2d.fromDegrees(23), 0)
            //"Climb Balance", new ArmStateValues(Rotation2d.fromDegrees(97.38), 0),
            //"Trap Score", new ArmStateValues(Rotation2d.fromDegrees(70), 2)
        );

    private GenericEntry sb_armMode;
    private GenericEntry sb_extStage1;
    private GenericEntry sb_extStage2;
    private GenericEntry sb_extState;
    private GenericEntry sb_brakeModeDisabled;
    private GenericEntry sb_armClearOfClimber;
    private GenericEntry sb_errorOverTime;
    private GenericEntry sb_atAngle;
    private GenericEntry sb_atExt;
    private GenericEntry sb_atTarget;

    /**
     * Constructor
     */
    private Arm() {
        // Initialize Shoulder Joint
        shoulder_joint = new MotorMech_TalonFX(
            new MotorMech_TalonFX.Settings(
                "Shoulder Joint",
                "Arm",
                new PositionController.Settings(    // Position Controller Settings
                    Constants.armRampDownDist.getDegrees() * Constants.maxArmAutoSpeed,
                    Constants.armRampDownDist.getDegrees() * Constants.maxArmAutoSpeed,
                    Constants.maxArmAutoSpeed,
                    true,
                    new Limits(0,360)
                ),
                {   // Rate Controller Settings
                    new RateController.Settings(Constants.armFFS0, Constants.armPIDS0),
                    new RateController.Settings(Constants.armFFS1, Constants.armPIDS0),
                    new RateController.Settings(Constants.armFFS2, Constants.armPIDS0)
                },
                { // Soft Limits
                    new Limits(Constants.LowerEncLimitS0, Constants.upperEncLimit),
                    new Limits(Constants.lowerEncLimit, Constants.upperEncLimit),
                    new Limits(Constants.lowerEncLimitS2, Constants.upperEncLimit)
                }
                new Limits(-1,1), // Default Position Tolerance
                {Constants.armMotor1, Constants.armMotor2},     // Motor CAN IDs
                {false, false},                                 // Invert Motors
                Constants.armQuadEncoderAPort,                  // Quadrature Encoder Digital Input A 
                Constants.armQuadEncoderBPort,                  // Quadrature Encoder Digital Input B
                false,                                          // Quadrature Encoder Digital Input Inverted
                Constants.armDCEncoderPort,                     // Absolute Encoder Digital Input
                true,                                           // Absolute Encoder Inverted
                Constants.armEncAnglePerRot.getDegrees() /      // Encoder Distance per Pulse
                    Constants.revTBEncCountPerRev,                  
                Constants.armEncAngleOffset.getDegrees()        // Absolute Encoder Offset
            )
        );
        
        climber_zone = new Limits(Constants.climberZoneLowerAngle.getDegrees(), Constants.climberZoneUpperAngle.getDegrees());

        
        // TODO Move initialization to Pneumatics class
        armExtender1 = new DoubleSolenoid(Constants.phCANID, PneumaticsModuleType.REVPH, Constants.armExt1Rev,
                Constants.armExt1For);
        armExtender2 = new DoubleSolenoid(Constants.phCANID, PneumaticsModuleType.REVPH, Constants.armExt2Rev,
                Constants.armExt2For);

        brakeModeDisableBtn = new DigitalInput(Constants.armBrakeModeBtn);

        //Auton Positions
        // TODO Set abs encoder offset

        // Set target state to current state
        targetState = new ArmStateValues(getArmAngle(), getArmExtension());

        // Initialize Timer
        extenderTimer = new Timer();

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Arm")
                .getLayout("Main Arm", BuiltInLayouts.kList)
                .withSize(2, 6);

        sb_armMode = layout.add("Arm Control Mode", control_mode.name()).getEntry();
        sb_extStage1 = layout.add("Ext Stage 1 State", armExtender1.get().name()).getEntry();
        sb_extStage2 = layout.add("Ext Stage 2 State", armExtender2.get().name()).getEntry();
        sb_extState = layout.add("Ext State", manual_ext).getEntry();
        sb_brakeModeDisabled = layout.add("Brake Mode Disabled", brakeModeDisableBtn.get()).getEntry();
        sb_armClearOfClimber = layout.add("Arm clear of climber", false).getEntry();
        sb_errorOverTime = layout.add("Error Over Time", 0).getEntry();
        
        sb_atAngle = layout.add("At Angle", false).getEntry();
        sb_atExt = layout.add("At Extension", false).getEntry();
        sb_atTarget = layout.add("At Target", false).getEntry();

        // Initialize PathPlanner named commands
        PathPlanner.registerCommand("Arm Intake Position", createGotoNamedPresetCmd("Intake"));
        PathPlanner.registerCommand("Arm Home Position", createGotoNamedPresetCmd("Home"));
        PathPlanner.registerCommand("Arm Speaker Position", createGotoNamedPresetCmd("Speaker"));
    }


    /*************************/
    /* Public Access Methods */
    /*************************/

    /**
     * Gets the current arm angle
     * 
     * @return current arm angle
     */
    public Rotation2d getArmAngle() {
        return Rotation2d.fromDegrees(shoulder_joint.getPosition());
    }

    /**
     * Gets the current arm angle rate
     * 
     * @return current arm angle rate in degrees per second
     */
    public double getArmVelocity() {
        return shoulder_joint.getRate();
    }

    /**
     * Checks the current extension state
     * 
     * @return current extension state
     */
    public int getArmExtension() {
        boolean isLowerExt = armExtender1.get() == DoubleSolenoid.Value.kForward;
        boolean isUpperExt = armExtender2.get() == DoubleSolenoid.Value.kForward;
        int state = 0;

        if (isLowerExt) {
            if (isUpperExt) {
                state = 2;
            } else {
                state = 1;
            }
        }

        return state;
    }

    /**
     * Check if the arm is at its target angle
     * 
     * @return true if the angle are at their target
     */
    public boolean atAngle() {
        shoulder_joint.atTarget();
    }

    /**
     * Check if the arm is at its target extension
     * 
     * @return true if the extension are at their target
     */
    public boolean atExtention() {
        return getArmExtension() == targetState.extState;
    }

    /**
     * Check if the arm is at its target angle and extension
     * 
     * @return true if the angle and extension are at their targets
     */
    public boolean atTarget() {
        return atAngle() && atExtention();
    }

    /**
     * Checks if the arm is an safe position to extend the climber
     * @return true if the arm is in a safe position to extend the climber
     */
    public boolean isInClimberZone() {
        return climber_zone.inRange(currentAngle.getDegrees());
    }


    /**************************/
    /* Public Control Methods */
    /**************************/

    /**
     * Sets the arm's output voltage to the motor. Puts the arm into manual
     * voltage mode. If the arm is not in a manual mode already, the extension
     * state is set to its current state.
     * 
     * @param voltage voltage to set to the motor
     */
    public void setArmVolt(double voltage) {
        shoulder_joint.setVoltage(voltage);
    }

    /**
     * Sets the arm's target rate. Puts the arm into manual mode. If the arm is
     * not in manual mode already, the extension state is set to its current
     * state.
     * 
     * @param rate new arm rate
     */
    public void setArmRate(double rate) {
        shoulder_joint.setRate(rate)
    }

    /**
     * Sets the arm's extension state. Puts the arm into manual mode. If the arm is
     * not in manual mode already, the arm rate is set to 0.
     * 
     * @param state extension state
     */
    public void setExtState(int state) {
        manual_ext = Math.max(0, Math.min(2, state));
        shoulder_joint.holdPosition();
    }

    /**
     * Steps the arm extension one stage out.
     */
    public void stepExtOut() {
        setExtState(getArmExtension() + 1);
    }

    /**
     * Steps the arm extension one stage out.
     */
    public void stepExtIn() {
        setExtState(getArmExtension() - 1);
    }

    /**
     * Looks up a standard target state
     * 
     * @param Name of the standard state. If an unknown name is supplied,
     *             the state will be set to the home position
     */
    public void setState(String name) {
        setState(getTargetValues(name));
    }

    /**
     * Sets the target state for the arm
     * 
     * @param targetState Current targetState value for the arm
     */
    public void setState(ArmStateValues targetState) {
        setExtState(targetState.extState);
        shoulder_joint.setPosition(targetAngle.getDegrees());
    }


    /******************************/
    /* Command Generation Methods */
    /******************************/
    
    /**
     * Generates a GotoNamedPreset command
     * @param   name    name of the preset to goto
     * @return  new GotoNamedPreset command
     */
    public Command getGotoNamedPresetCmd(String name) {
        return new GotoNamedPreset(this, name);
    }
    
    /**
     * Generates a GotoPreset command
     * @param   preset  preset to goto
     * @return  new GotoPreset command
     */
    public Command getGotoPresetCmd(ArmStateValues preset) {
        return new GotoPreset(this, preset);
    }
    
    /**
     * Generates a AutoAlign command
     * @return  new AutoAlign command
     */
    public Command getAutoAlignCmd() {
        return new AutoAlign(this);
    }

    /*********************/
    /* Subsystem Methods */
    /*********************/

    /**
     * Subsystem periodic method
     */
    @Override
    public void periodic() {
        updateBrakeMode();
        updateShoulderControl();
        updateExtension();

        updateUI(targetArmRate, voltage);
        SmartDashboard.putNumber("SpeakerPosition", FieldLayout.getSpeakerPose().getX());
    }
    

    /****************************/
    /* Subsystem Helper Methods */
    /****************************/

    // TODO update armAutoAlign to work with MotorMechanismBase
    /*
    public void armAutoAlign(){
        Drive drive = Drive.getInstance();
        double distance = Math.abs(FieldLayout.getShootSpeakerPose().getX() - drive.getEstimatedPos().getX()) + 
            ((Math.cos(getArmAngle().minus(Rotation2d.fromDegrees(11)).getRadians()) * Constants.armLength) + 0.2413);
        double height =  FieldLayout.stageHeight - Constants.armHeightOffset - (Math.sin(getArmAngle().minus(Rotation2d.fromDegrees(11)).getRadians()) * Constants.armLength);
        double desiredAngle = 90 - Math.toDegrees(Math.atan2(height, distance));
        control_mode = ArmControlMode.AUTOMATIC;
        if(desiredAngle < 23){
            desiredAngle = 23;
        }else if(desiredAngle > 100 ){
            desiredAngle = 100;
        }
        new Rotation2d();
        ArmStateValues targetState = new ArmStateValues(Rotation2d.fromDegrees(desiredAngle), 0);
        setState(targetState); 
    }
    */


    /**
     * Updates the brake mode control of the
     */
    private void updateBrakeMode() {
        shoulder_joint.setBrakeMode(!brakeModeDisableBtn.get());
    }

    /**
     * Updates the shoulder control parameters and soft limits
     */
    private void updateShoulderControl() {
        int current_ext = getArmExtension();

        // Update Shoulder Joint rate controller and limits
        shoulder_joint.setRateCtrlIndex(current_ext);
        shoulder_joint.setSoftLimitsIndex(current_ext);
    }

    /**
     * Updates the control of the arm extension
     */
    private void updateExtension() {
        int currentState = getArmExtension();
        int targetState = target_ext;

        boolean aboveState2Angle = getArmAngle().getDegrees() > Constants.armMinState2Angle.getDegrees();

        // Set target extension valve state
        if (targetState == 2 && getArmAngle().getDegrees() > Constants.minArmS2Angle.getDegrees()) {
            armExtender1.set(DoubleSolenoid.Value.kForward);
            armExtender2.set(DoubleSolenoid.Value.kForward);
        } else if (targetState == 1) {
            armExtender1.set(DoubleSolenoid.Value.kForward);
            armExtender2.set(DoubleSolenoid.Value.kReverse);
        } else {
            armExtender1.set(DoubleSolenoid.Value.kReverse);
            armExtender2.set(DoubleSolenoid.Value.kReverse);
        }

        // Reset extension timer of the extension state has chanced
        if (currentState != targetState)
            extenderTimer.restart();

    }

    /**
     * Updates shuffleboard
     */
    private void updateUI(double targetRate, double targetVolt) {
        sb_armMode.setString(control_mode.name());
        sb_extStage1.setString(armExtender1.get().name());
        sb_extStage2.setString(armExtender2.get().name());
        sb_extState.setInteger(target_ext);
        sb_brakeModeDisabled.setBoolean(!brakeModeDisableBtn.get());
        sb_armClearOfClimber.setBoolean(!isInClimberZone());
        sb_atAngle.setBoolean(atAngle());
        sb_atExt.setBoolean(atExtention());
        sb_atTarget.setBoolean(atTarget());
    }
    

    /****************************/
    /* Static Singleton Methods */
    /****************************/

    /**
     * Static initializer for the arm class
     */
    public static Arm getInstance() {
        if (arm == null) {
            arm = new Arm();
        }
        return arm;
    }

}
