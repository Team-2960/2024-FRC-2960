package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;

import frc.lib2960_ctre.MotorMechTalonFX;

import frc.lib2960.pathplanner.PathPlanner;

import frc.robot.Constants;
import frc.robot.Util.FieldLayout;

public class Arm extends SubsystemBase {
    private static Arm arm;     /**< Singalton Instance */

    /**
     * Goto an arm state command
     */
    public class GotoArmStateCommand extends Command {
        private final Rotation2d angle; /**< Target arm angle */
        private final int ext_state;    /**< Target extension state */

        /**
         * Constructor
         * @param   angle       Target arm angle
         * @param   ext_state   Target extension state 
         */
        public GotoArmStateCommand(Rotation2d angle, int ext_state) {
            this.angle = angle;
            this.ext_state = Math.max(0, Math.min(2, ext_state));

            addRequirements(Arm.this);
        }
        
        /**
         * initialize method. Set arm to preset state
         */
        @Override
        public void initialize() {
            setState(angle, ext_state);
        }

        /**
         * isFinished method. Check if arm is at the target position
         */
        @Override
        public boolean isFinished() {
            return atTarget();
        }
    }
    
    /**
     * Auto Align command
     */
    public class AutoAlignCommand extends Command {
        /**
         * Constructor
         */
        public AutoAlignCommand(){
            addRequirements(Arm.this);
        }

        /**
         * initialize method. Start arm auto alignment
         */
        @Override
        public void initialize(){
            updateAutoAlignCommand();
        }
    }
    
    public final ArmSettings settings;

    public final MotorMechTalonFX shoulder_joint;
    
    private final DoubleSolenoid arm_ext_1;
    private final DoubleSolenoid arm_ext_2;

    private int target_ext;
    private final Timer extenderTimer;

    private final DigitalInput brakeModeDisableBtn;
    private boolean last_brake_pressed;

    private final HashMap<String, GotoArmStateCommand> preset_list;
    private final AutoAlignCommand auto_align_cmd;

    // Shuffleboard
    private GenericEntry sb_armMode;
    private GenericEntry sb_extStage1;
    private GenericEntry sb_extStage2;
    private GenericEntry sb_extState;
    private GenericEntry sb_brakeModeDisabled;
    private GenericEntry sb_armClearOfClimber;
    private GenericEntry sb_atAngle;
    private GenericEntry sb_atExt;
    private GenericEntry sb_atTarget;

    /**
     * Constructor
     */
    private Arm(ArmSettings settings) {
        this.settings = settings;

        // Initialize Shoulder Joint
        shoulder_joint = new MotorMechTalonFX(settings.joint_settings);
        
        // TODO Move initialization to Pneumatics class
        arm_ext_1 = new DoubleSolenoid(
            settings.ext_1_settings.ph_can_id, 
            settings.ext_1_settings.module_type, 
            settings.ext_1_settings.fwd_port,
            settings.ext_1_settings.rev_port
        );

        arm_ext_2 = new DoubleSolenoid(
            settings.ext_2_settings.ph_can_id, 
            settings.ext_2_settings.module_type, 
            settings.ext_2_settings.fwd_port,
            settings.ext_2_settings.rev_port
        );

        brakeModeDisableBtn = new DigitalInput(Constants.armBrakeModeBtn);
        last_brake_pressed = !brakeModeDisableBtn.get();
        updateBrakeMode();

        // Initialize presets
        preset_list = new HashMap<String, GotoArmStateCommand>();

        preset_list.put("Match Start", new GotoArmStateCommand(Rotation2d.fromDegrees(60), 0));
        preset_list.put("Home", new GotoArmStateCommand(Rotation2d.fromDegrees(15), 0));
        preset_list.put("Intake", new GotoArmStateCommand(Rotation2d.fromDegrees(7), 1));
        preset_list.put("Speaker", new GotoArmStateCommand(Rotation2d.fromDegrees(46), 0));
        preset_list.put("lineSpeaker", new GotoArmStateCommand(Rotation2d.fromDegrees(56), 0));
        preset_list.put("longShot", new GotoArmStateCommand(Rotation2d.fromDegrees(67.5), 0));
        preset_list.put("Amp", new GotoArmStateCommand(Rotation2d.fromDegrees(102), 1));
        preset_list.put("Climb", new GotoArmStateCommand(Rotation2d.fromDegrees(97.38), 0));
        preset_list.put("AmpSideShoot", new GotoArmStateCommand(Rotation2d.fromDegrees(47), 0));
        preset_list.put("home", new GotoArmStateCommand(Rotation2d.fromDegrees(23), 0));
        preset_list.put("Climb Balance", new GotoArmStateCommand(Rotation2d.fromDegrees(97.38), 0));
        preset_list.put("Trap Score", new GotoArmStateCommand(Rotation2d.fromDegrees(70), 2));

        // Initialize auto align command
        auto_align_cmd = new AutoAlignCommand();

        
        // TODO Set abs encoder offset

        // Initialize Timer
        extenderTimer = new Timer();

        // Setup Shuffleboard
        init_ui();

        // Initialize PathPlanner named commands
        for(var preset: preset_list.entrySet()) {
            PathPlanner.registerCommand("Arm Goto " + preset.getKey(), preset.getValue());
        } 
    }

    /**
     * Initialize Shuffleboard
     */
    private void init_ui() {
        var layout = Shuffleboard.getTab("Arm")
            .getLayout("Main Arm", BuiltInLayouts.kList)
            .withSize(2, 6);

        String cmd_name = "";
        Command cmd = getCurrentCommand();
        if(cmd != null) cmd_name = cmd.getName();

        sb_armMode = layout.add("Arm Control Mode", cmd_name).getEntry();
        sb_extStage1 = layout.add("Ext Stage 1 State", arm_ext_1.get().name()).getEntry();
        sb_extStage2 = layout.add("Ext Stage 2 State", arm_ext_2.get().name()).getEntry();
        sb_extState = layout.add("Ext State", getArmExtension()).getEntry();
        sb_brakeModeDisabled = layout.add("Brake Mode Disabled", brakeModeDisableBtn.get()).getEntry();
        sb_armClearOfClimber = layout.add("Arm clear of climber", false).getEntry();

        sb_atAngle = layout.add("At Angle", false).getEntry();
        sb_atExt = layout.add("At Extension", false).getEntry();
        sb_atTarget = layout.add("At Target", false).getEntry();
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
        boolean isLowerExt = arm_ext_1.get() == DoubleSolenoid.Value.kForward;
        boolean isUpperExt = arm_ext_2.get() == DoubleSolenoid.Value.kForward;
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
        return shoulder_joint.atTarget();
    }

    /**
     * Check if the arm is at its target extension
     * 
     * @return true if the extension are at their target
     */
    public boolean atExtention() {
        return getArmExtension() == target_ext;
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
        return Constants.climber_zone.inRange(shoulder_joint.getPosition());
    }


    /**************************/
    /* Public Control Methods */
    /**************************/

    /**
     * Sets the target arm angle
     * @param angle     target arm angle
     */
    public void setArmAngle(Rotation2d angle) {
        shoulder_joint.setPosition(angle.getDegrees());
    }

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
        shoulder_joint.setRate(rate);
    }

    /**
     * Sets the arm's extension state. Puts the arm into manual mode. If the arm is
     * not in manual mode already, the arm rate is set to 0.
     * 
     * @param state extension state
     */
    public void setExtState(int state) {
        target_ext = Math.max(0, Math.min(2, state));
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
     * Sets the arm target angle and extension state
     * @param angle         target arm angle
     * @param ext_state     target arm extension
     */
    public void setState(Rotation2d angle, int ext_state) {
        setArmAngle(angle);
        setExtState(ext_state);
    }

    /**
     * Executes a command to go to a named preset. If the named preset does not exist, no changes 
     * are made.
     * @param name  name of the preset
     */
    public void gotoPreset(String name) {
        if(preset_list.containsKey(name)){
            Command preset_cmd = preset_list.get(name);
            if(getCurrentCommand() != preset_cmd) preset_cmd.schedule();
        }
    }

    /**
     * Start the auto align command
     */
    public void startAutoAlign() {
        if(getCurrentCommand() != auto_align_cmd) auto_align_cmd.schedule();
    }
    
    /*********************/
    /* Subsystem Methods */
    /*********************/

    /**
     * Subsystem periodic method
     */
    @Override
    public void periodic() {
        double start_time = Timer.getFPGATimestamp();

        updateBrakeMode();

        System.out.print(String.format("Update Brake Mode Time: %f\n", Timer.getFPGATimestamp() - start_time));

        updateShoulderControl();

        System.out.print(String.format("Update Shoulder Control Time: %f\n", Timer.getFPGATimestamp() - start_time));

        updateExtension();

        System.out.print(String.format("Update Extension Time: %f\n", Timer.getFPGATimestamp() - start_time));

        updateUI();
        
        System.out.print(String.format("Update UI Time: %f\n", Timer.getFPGATimestamp() - start_time));
        

        SmartDashboard.putNumber("SpeakerPosition", FieldLayout.getSpeakerPose().getX());

        System.out.print(String.format("Update Speaker Position Time: %f\n", Timer.getFPGATimestamp() - start_time));

    }
    

    /****************************/
    /* Subsystem Helper Methods */
    /****************************/
    private void updateAutoAlignCommand(){
        // TODO update armAutoAlignCommand to work with MotorMechanismBase
        
        // Get field position
        Translation2d cur_pos = Drive.getInstance().getEstimatedPos().getTranslation();
        Translation2d goal_pos = FieldLayout.getShootSpeakerPose().getTranslation();
        double goal_dist = cur_pos.getDistance(goal_pos);

        // Calculate arm offset distance
        Rotation2d arm_angle = getArmAngle().minus(Rotation2d.fromDegrees(11));   // TODO Move to settings
        double arm_pos_offset = 0.2413; // TODO Move to settings
        double arm_offset = ((arm_angle.getCos() * Constants.armLength) + arm_pos_offset);

        // Calculate Distance 
        double shooter_distance = goal_dist + arm_offset;
        double shooter_height =  FieldLayout.stageHeight - Constants.armHeightOffset - (arm_angle.getSin() * Constants.armLength);
        double target_angle = 90 - Math.toDegrees(Math.atan2(shooter_height, shooter_distance));
        
        target_angle = MathUtil.clamp(target_angle, 23, 100);
          
        setState(Rotation2d.fromDegrees(target_angle), 0); 
    }


    /**
     * Updates the brake mode control of the
     */
    private void updateBrakeMode() {
        boolean brake_pressed = brakeModeDisableBtn.get();
        if(brake_pressed != last_brake_pressed) shoulder_joint.setBrakeMode(!brake_pressed);
        last_brake_pressed = brake_pressed;
    }

    /**
     * Updates the shoulder control parameters and soft limits
     */
    private void updateShoulderControl() {
        int current_ext = getArmExtension();

        // Update Shoulder Joint rate stage
        shoulder_joint.setStageIndex(current_ext);
    }

    /**
     * Updates the control of the arm extension
     */
    private void updateExtension() {
        int currentState = getArmExtension();

        // Set target extension valve state
        if (target_ext == 2 && getArmAngle().getDegrees() > Constants.minArmS2Angle.getDegrees()) {
            arm_ext_1.set(DoubleSolenoid.Value.kForward);
            arm_ext_2.set(DoubleSolenoid.Value.kForward);
        } else if (target_ext == 1) {
            arm_ext_1.set(DoubleSolenoid.Value.kForward);
            arm_ext_2.set(DoubleSolenoid.Value.kReverse);
        } else {
            arm_ext_1.set(DoubleSolenoid.Value.kReverse);
            arm_ext_2.set(DoubleSolenoid.Value.kReverse);
        }

        // Reset extension timer of the extension state has chanced
        if (currentState != target_ext) extenderTimer.restart();
    }

    /**
     * Updates shuffleboard
     */
    private void updateUI() {
        String cmd_name = "";
        Command cmd = getCurrentCommand();
        if(cmd != null) cmd_name = cmd.getName();

        sb_armMode.setString(cmd_name);
        sb_extStage1.setString(arm_ext_1.get().name());
        sb_extStage2.setString(arm_ext_2.get().name());
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
            arm = new Arm(Constants.arm_settings);
        }
        return arm;
    }

}
