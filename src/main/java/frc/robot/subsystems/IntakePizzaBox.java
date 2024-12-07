package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib2960.pathplanner.PathPlanner;
import frc.robot.Constants;

public class IntakePizzaBox extends SubsystemBase {

    public class StopCommand extends Command {
        public StopCommand() {
            addRequirements(IntakePizzaBox.this);
        }

        @Override
        public void initialize() {
            runShooter(0);
            intake_motor.setVoltage(0);
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }

    public class IntakeCommand extends Command {
        public IntakeCommand() {
            addRequirements(IntakePizzaBox.this);
        }

        @Override
        public void execute() {
            if (!isNotePresent()) {
                if (isIntakeNotePresent()) {
                    intake_motor.setVoltage(settings.intake_slow_volt);
                }else{
                    intake_motor.setVoltage(settings.intake_in_volt);
                }
            }
        }

        @Override
        public boolean isFinished() {
            return isNotePresent();
        }

        @Override
        public void end(boolean interrupted) {
            intake_motor.setVoltage(0);
            if(Arm.getInstance().getArmAngle().getDegrees() <= 10){
                Arm.getInstance().gotoPreset("home");
            } 
        }
    }

    public class ShootPrepCommand extends Command {
        public ShootPrepCommand() {
            addRequirements(IntakePizzaBox.this);
        }

        @Override
        public void initialize() {
            runShooter(settings.shooter_prep_volt);
        }

        @Override
        public void end(boolean interrupted) {
            runShooter(0);
        }

    }

    public class ShootCommand extends Command {
        private final double speed;

        public ShootCommand(double speed) {
            this.speed = speed;
            addRequirements(IntakePizzaBox.this);
        }

        @Override
        public void initialize() {
            runShooter(settings.shooter_shoot_volt);
            intake_motor.setVoltage(0); 
        }

        @Override
        public void execute() {
            // Check if shooter is ready to shoot
            if (shooterAtSpeed(speed)) intake_motor.setVoltage(settings.intake_in_volt); 
        }

        @Override
        public boolean isFinished() {
            return !isNotePresent();
        }

        @Override
        public void end(boolean interrupted) {
            runShooter(0);
            intake_motor.setVoltage(0);
        }
    }

    public class ReverseCommand extends Command {
        public ReverseCommand() {
            addRequirements(IntakePizzaBox.this);
        }

        @Override
        public void initialize() {
            runShooter(settings.shooter_rev_volt);
            intake_motor.setVoltage(settings.intake_out_volt);
        }

        @Override
        public void end(boolean interrupted) {
            runShooter(0);
            intake_motor.setVoltage(0);
        }

    }

    private static IntakePizzaBox intake = null;

    private final IntakePizzaboxSettings settings;

    private final TalonFX intake_motor;

    private final SparkFlex[] shooter_motors;
    
    private final RelativeEncoder[] shooter_encoders;

    private final DigitalInput shooterPhotoeye;
    private final DigitalInput intakePhotoeye;

    private final StopCommand stop_cmd;
    private final IntakeCommand intake_cmd;
    private final ShootPrepCommand shoot_prep_cmd;
    private final ShootCommand shoot_slow_cmd;
    private final ShootCommand shoot_fast_cmd;
    private final ReverseCommand rev_cmd;

    private GenericEntry sb_state;
    private GenericEntry[] sb_shooterVolt;
    private GenericEntry[] sb_shooterRate;
    private GenericEntry sb_intakeRollerVolt;
    private GenericEntry sb_intakeRollerCurrent;
    private GenericEntry sb_intakeRollerRate;
    private GenericEntry sb_shooterNotePresent;
    private GenericEntry sb_intakeNotePresent;

    /**
     * Constructor
     */
    private IntakePizzaBox(IntakePizzaboxSettings settings) {
        this.settings = settings;

        // Initialize Intake Motor
        intake_motor = new TalonFX(settings.intake_settings.id);
        intake_motor.setInverted(settings.intake_settings.inverted);    // TODO implement with config system

        // Initialize Shooter Motors
        shooter_motors = new SparkFlex[settings.shooter_settings.length];
        shooter_encoders = new RelativeEncoder[settings.shooter_settings.length];

        for(int i = 0; i < shooter_motors.length; i++) {
            shooter_motors[i] = new SparkFlex(settings.shooter_settings[i].id, MotorType.kBrushless);
            shooter_motors[i].setInverted(true);    // TODO implement with config system
            shooter_encoders[i] = shooter_motors[i].getEncoder();
        }

        // Initialize shooterPhotoeye
        shooterPhotoeye = new DigitalInput(settings.shooter_pe_port);
        intakePhotoeye = new DigitalInput(settings.intake_pe_port);

        // Initialize commands
        stop_cmd = new StopCommand();
        intake_cmd = new IntakeCommand();
        shoot_prep_cmd = new ShootPrepCommand();
        shoot_slow_cmd = new ShootCommand(settings.min_slow_shoot_speed);
        shoot_fast_cmd = new ShootCommand(settings.min_slow_shoot_speed);
        rev_cmd = new ReverseCommand();

        setDefaultCommand(stop_cmd);

        // Initialize Shuffleboard
        init_ui();

        // Add named commands to PathPlanner
        PathPlanner.registerCommand("Pizzabox Stop", stop_cmd);
        PathPlanner.registerCommand("Pizzabox Intake", intake_cmd);
        PathPlanner.registerCommand("Pizzabox Shoot Prep", shoot_prep_cmd);
        PathPlanner.registerCommand("Pizzabox Shoot Slow", shoot_slow_cmd);
        PathPlanner.registerCommand("Pizzabox Shoot Fast", shoot_fast_cmd);
        PathPlanner.registerCommand("Pizzabox Reverse", rev_cmd);
    }

    /**
     * Initialize Shuffleboard
     */
    private void init_ui() {
        var layout = Shuffleboard.getTab("Status")
                .getLayout("Pizzabox", BuiltInLayouts.kList)
                .withSize(2, 6);

        sb_shooterVolt = new GenericEntry[settings.shooter_settings.length];
        sb_shooterRate = new GenericEntry[settings.shooter_settings.length];

        for(int i = 0; i < settings.shooter_settings.length; i++) {
            sb_shooterVolt[i] = layout.add(settings.shooter_settings[i] + " Voltage", 0).getEntry();
            sb_shooterRate[i] = layout.add(settings.shooter_settings[i] + " Rate", 0).getEntry();
        }

        sb_intakeRollerVolt = layout.add("Intake Roller Voltage", 0).getEntry();
        sb_intakeRollerRate = layout.add("Intake Roller Rate", 0).getEntry();
        sb_shooterNotePresent = layout.add("Shooter Note Present", false).getEntry();
        sb_intakeNotePresent = layout.add("Intake Note Present", false).getEntry();
        sb_intakeRollerCurrent = layout.add("Intake Roller Current", 0).getEntry();
    }

    /**
     * Stop intake pizzabox
     */
    public void stop() {
        Command current_cmd = getCurrentCommand();
        if(current_cmd != stop_cmd) current_cmd.cancel();
    }

    /**
     * Run intake
     */
    public void intake() {
        Command current_cmd = getCurrentCommand();
        if(current_cmd != intake_cmd) intake_cmd.schedule();
    }

    /**
     * Start Shoot Prep
     */
    public void shootPrep() {
        Command current_cmd = getCurrentCommand();
        if(current_cmd != shoot_prep_cmd) shoot_prep_cmd.schedule();
    }

    /**
     * Stop intake pizzabox
     */
    public void shootSlow() {
        Command current_cmd = getCurrentCommand();
        if(current_cmd != shoot_slow_cmd) shoot_slow_cmd.schedule();
    }

    /**
     * Stop intake pizzabox
     */
    public void shootFast() {
        Command current_cmd = getCurrentCommand();
        if(current_cmd != shoot_fast_cmd) shoot_fast_cmd.schedule();
    }

    /**
     * Stop intake pizzabox
     */
    public void reverse() {
        Command current_cmd = getCurrentCommand();
        if(current_cmd != rev_cmd) rev_cmd.schedule();
    }

    /**
     * Checks if a game piece is present
     * 
     * @return true if a gamepiece is present, false otherwise
     */
    public void runShooter(double voltage){
        for(var motor : shooter_motors) motor.setVoltage(voltage);
    }
    
     public boolean isNotePresent() {
        return shooterPhotoeye.get();
    }

    public boolean isIntakeNotePresent(){
        return intakePhotoeye.get();
    }

    /**
     * Checks if all the shooter wheels are at speed
     * @return  true if all shooter wheels are at speed, false otherwise
     */
    public boolean shooterAtSpeed(double speed) {
        boolean at_speed = true;

        for(var encoder : shooter_encoders){
            at_speed &= encoder.getVelocity() > speed;
        } 

        return at_speed;
    }

    /**
     * Subsystem periodic function
     */
    @Override
    public void periodic() {

        updateUI();
    }

    private void updateUI() {
        for(int i = 0; i < settings.shooter_settings.length; i++) {
            double voltage = shooter_motors[i].getBusVoltage() * shooter_motors[i].getAppliedOutput();
            double rate = shooter_encoders[i].getVelocity();
            sb_shooterVolt[i].setDouble(voltage); 
            sb_shooterRate[i].setDouble(rate);
        }

        sb_intakeRollerVolt.setDouble(intake_motor.getMotorVoltage().getValueAsDouble());
        sb_intakeRollerRate.setDouble(intake_motor.getVelocity().getValueAsDouble());
        sb_shooterNotePresent.setBoolean(isNotePresent());
        sb_intakeNotePresent.setBoolean(isIntakeNotePresent());
        sb_intakeRollerCurrent.setDouble(intake_motor.getStatorCurrent().getValueAsDouble());
    }

    /**
     * Static Initializer
     */
    public static IntakePizzaBox getInstance() {
        if (intake == null) {
            intake = new IntakePizzaBox(Constants.pizzabox_settings);
        }
        return intake;
    }
}
