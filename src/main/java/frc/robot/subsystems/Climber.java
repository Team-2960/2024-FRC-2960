package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Climber extends SubsystemBase {

    /**
     * Climber idle operation command
     */
    public class StopCommand extends Command {
        public StopCommand() {
            addRequirements(Climber.this);
        }

        @Override
        public void initialize() {
            setRatchet(true);
            setMotor(0);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    /**
     * Climber Retract Command
     */
    public class RetractCommand extends Command {
        private final double speed;

        public RetractCommand(double speed) {
            this.speed = speed;
            addRequirements(Climber.this);
        }

        @Override
        public void initialize() {
            setRatchet(true);
            setMotor(-speed);
        }

        @Override
        public boolean isFinished() {
            return isRetracted();
        }

        @Override
        public void end(boolean interrupted) {
            setMotor(0);
        }
    }

    public class ExtendCommand extends Command {
        private final double speed;

        public ExtendCommand(double speed) {
            this.speed = speed;
            addRequirements(Climber.this);
        }

        @Override
        public void initialize() {
            setRatchet(false);
            setMotor(0);
        }

        @Override
        public void execute() {
            if(isRatchetReleased()) setMotor(speed);
        }

        @Override
        public boolean isFinished() {
            return isExtended();
        }

        @Override
        public void end(boolean interrupted) {
            setMotor(0);
        }
    }

    private static Climber climber = null;

    public ClimberSettings settings;

    private SparkMax motors[];

    private SparkLimitSwitch winch_limit;
    private RelativeEncoder winch_encoder;

    private DoubleSolenoid ratchet_release;
    private Timer ratchet_timer;

    private boolean limit_enabled = true;

    private GenericEntry sb_state;
    private GenericEntry sb_isDown;
    private GenericEntry sb_isClearOfArm;
    private GenericEntry sb_motorVolt[];
    private GenericEntry sb_winchExt;
    private GenericEntry sb_ratchetTime;
    private GenericEntry sb_ratchetValve;

    private StopCommand stop_cmd;
    private RetractCommand match_start_cmd;
    private ExtendCommand climb_ext_cmd;
    private RetractCommand climb_ret_cmd;


    /**
     * Constructor
     */
    private Climber(ClimberSettings settings) {
        this.settings = settings;

        // Initialize Motors
        motors = new SparkMax[settings.motor_settings.length];

        for(int i = 0; i < motors.length; i++) {
            // Create motor
            motors[i] = new SparkMax(settings.motor_settings[i].id, MotorType.kBrushless);
            
            // Configure motor
            SparkMaxConfig motor_config = new SparkMaxConfig();
            motor_config.inverted(settings.motor_settings[i].inverted);
            motor_config.idleMode(IdleMode.kBrake);

            // Configure limit switch
            if(i == settings.limit_index) {
                LimitSwitchConfig config = new LimitSwitchConfig();
                config.forwardLimitSwitchType(LimitSwitchConfig.Type.kNormallyOpen);
                motor_config.apply(config);
            }

            motors[i].configure(motor_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        // Initialize limit switch
        winch_limit = motors[settings.limit_index].getForwardLimitSwitch();

        // Initialize Encoder
        winch_encoder = motors[settings.encoder_index].getEncoder();

        // Initialize Commands
        stop_cmd = new StopCommand();
        match_start_cmd = new RetractCommand(.2);
        climb_ext_cmd = new ExtendCommand(.5);
        climb_ret_cmd = new RetractCommand(1);

        setDefaultCommand(stop_cmd);
        match_start_cmd.schedule();

        // Initialize Ratchet Release
        ratchet_release = new DoubleSolenoid(
            settings.latch_settings.ph_can_id, 
            settings.latch_settings.module_type,
            settings.latch_settings.fwd_port, 
            settings.latch_settings.rev_port
        );

        ratchet_release.set(DoubleSolenoid.Value.kReverse);

        // Initialize Ratchet Timer
        ratchet_timer = new Timer();

        // Initialize Shuffleboard
        init_ui();
    }

    /**
     * Initialize Shuffleboard
     */
    private void init_ui () {

        var layout = Shuffleboard.getTab("Status")
                .getLayout("Climber", BuiltInLayouts.kList)
                .withSize(2, 6);

        String cmd_name = "";
        Command cmd = getCurrentCommand();
        if(cmd != null) cmd_name = cmd.getName();
        
        sb_state = layout.add("State", cmd_name).getEntry();
        sb_isDown = layout.add("Is Down", false).getEntry();
        sb_isClearOfArm = layout.add("Is Clear of Arm", false).getEntry();
        
        sb_motorVolt = new GenericEntry[motors.length];
        for(int i = 0; i < motors.length; i++) {
            sb_motorVolt[i] = layout.add(settings.motor_settings[i].name + "Voltage", 0).getEntry();
        }
        
        sb_winchExt = layout.add("Winch extension", 0).getEntry();
        sb_ratchetTime = layout.add("Ratchet timer", 0).getEntry();
        sb_ratchetValve = layout.add("Ratchet Valve", "").getEntry();
    }

    /**
     * Gets the current extension distance
     * 
     * @return distance the climber is extended
     */
    public double getExtension() {
        double distance = winch_encoder.getPosition() * settings.winch_circumfrance;

        if(settings.encoder_invert) distance *= -1;

        return distance;
    }

    /**
     * Runs the extend command
     */
    public void extend() {
        if(getCurrentCommand() != climb_ext_cmd) climb_ext_cmd.schedule();
    }

    /**
     * Runs the retract command
     */
    public void retract() {
        if(getCurrentCommand() != climb_ret_cmd) climb_ret_cmd.schedule();
    }

    /**
     * Runs the stop command
     */
    public void stop() {
        var current_cmd = getCurrentCommand();
        if(current_cmd != null && current_cmd != stop_cmd) current_cmd.cancel(); 
    }

    /**
     * Checks if the climber is fully retracted
     * 
     * @return true if the climber is fully retracted, false otherwise.
     */
    public boolean isRetracted() {
        return winch_limit.isPressed();
    }

    /**
     * Check if climber fully extended
     * 
     * @return true if climber is fully extended, false otherwise
     */
    public boolean isExtended() {
        return getExtension() >= settings.max_extension;
    }

    /**
     * Resets the climber encoder
     */
    public void resetClimber() {
        winch_encoder.setPosition(0);
    }

    /**
     * Checks if the climber is in a position that is clear of the arm
     * 
     * @return true if the climber is in a position clear of the arm
     */
    public boolean isClearOfArm() {
        double armContactHeight = 15;
        return isRetracted() || getExtension() < armContactHeight;
    }

    /**
     * Sets if the ratchet is engaged
     * @param enabled   true to engage the ratchet
     */
    public void setRatchet(boolean enabled) {
        if (enabled) {
            ratchet_release.set(Value.kForward);
        } else {
            ratchet_release.set(Value.kReverse);
            ratchet_timer.restart();
        }
    }

    /**
     * Checks if the ratchet is released
     * @return
     */
    public boolean isRatchetReleased() {
        return ratchet_release.get() == Value.kReverse && ratchet_timer.get() > settings.ratchet_delay;
    }
    
    /**
     * Sets the motor output
     *  - Limit switch is enabled
     * @param value         Motor output value
     */
    public void setMotor(double value) {
        for(var motor : motors) motor.set(value);
    }

    /**
     * Sets if the limit switch is enabled
     * @param   enabled     true to enable to limit switch, false otherwise
     */
    public void enableLimitSwtich(boolean enabled) {
        if(limit_enabled != enabled) {
            SparkMaxConfig motor_config = new SparkMaxConfig();
            LimitSwitchConfig limit_config = new LimitSwitchConfig();

            limit_config.forwardLimitSwitchEnabled(enabled);
            motor_config.apply(limit_config);

            motors[settings.limit_index].configure(motor_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

            limit_enabled = enabled;
        }
    }

    /**
     * Climber periodic update
     */
    @Override
    public void periodic() {
        // Reset the climber encoder if the limit switch is set
        if (isRetracted()) resetClimber();

        updateUI();
    }

    /**
     * Updates Shuffleboard
     */
    private void updateUI() {
        String cmd_name = "";
        Command cmd = getCurrentCommand();
        if(cmd != null) cmd_name = cmd.getName();

        sb_state.setString(cmd_name);
        sb_isDown.setBoolean(isRetracted());
        sb_isClearOfArm.setBoolean(isClearOfArm());

        for(int i = 0; i < motors.length; i++) {
            double voltage = motors[i].getBusVoltage() * motors[i].getAppliedOutput();
            sb_motorVolt[i].setDouble(voltage);
        }

        sb_winchExt.setDouble(getExtension());
        sb_ratchetTime.setDouble(ratchet_timer.get());
        sb_ratchetValve.setString(ratchet_release.get().name());
    }

    /**
     * Static initializer
     */
    public static Climber getInstance() {
        if (climber == null) {
            climber = new Climber(Constants.climber_settings);
        }

        return climber;
    }
}
