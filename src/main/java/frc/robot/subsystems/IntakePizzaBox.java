package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePizzaBox extends SubsystemBase{
    public enum PizzaboxState {
        IDLE,
        INTAKE,
        SHOOT,
        SHOOT_PREP,
        REVERSE
    }

    private static IntakePizzaBox intake = null;

    private TalonFX intakeRollers;

    private CANSparkMax shooterTop;
    private CANSparkMax shooterBot;

    private RelativeEncoder shootEncoder1;
    private RelativeEncoder shootEncoder2;

    private DigitalInput photoeye;

    private PizzaboxState state;

    private GenericEntry sb_state;
    private GenericEntry sb_shooterTopVolt;
    private GenericEntry sb_shooterBotVolt;
    private GenericEntry sb_shooterTopRate;
    private GenericEntry sb_shooterBotRate;
    private GenericEntry sb_intakeRollerVolt;
    private GenericEntry sb_intakeRollerRate;
    private GenericEntry sb_notePresent;

    /**
     * Constructor
     */
    private IntakePizzaBox(){
        // Initialize Intake Motor  
        intakeRollers = new TalonFX(Constants.intakeRollers);

        // Initialize Shooter Motors
        shooterTop = new CANSparkMax(Constants.shooterTop, MotorType.kBrushless);
        shooterBot = new CANSparkMax(Constants.shooterBot, MotorType.kBrushless);
        shooterBot.follow(shooterTop, true);

        // Initialize Shooter Encoders
        shootEncoder1 = shooterTop.getEncoder();
        shootEncoder2 = shooterBot.getEncoder();

        // Initialize photoeye
        photoeye = new DigitalInput(0);
        
        // Initialize state
        state = PizzaboxState.IDLE;

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status").getLayout("Pizzabox");

        sb_state = layout.add("State", state.name()).getEntry();
        sb_shooterTopVolt = layout.add("Shooter 1 Voltage", 0).getEntry();
        sb_shooterBotVolt = layout.add("Shooter 2 Voltage", 0).getEntry();
        sb_shooterTopRate = layout.add("Shooter 1 Rate", 0).getEntry();
        sb_shooterBotRate = layout.add("Shooter 2 Rate", 0).getEntry();
        sb_intakeRollerVolt = layout.add("Intake Roller Voltage", 0).getEntry();
        sb_intakeRollerRate = layout.add("Intake Roller Rate", 0).getEntry();
        sb_notePresent = layout.add("Note Present", false).getEntry();
    }

    /**
     * Sets the Pizzabox State
     * @param   new pizzabox state
     */
    public void setState(PizzaboxState state) {
        this.state = state;
    }

    /**
     * Gets the pizzabox state
     * @return  current pizza box state
     */
    public PizzaboxState getState() {
        return state;
    }

    /**
     * Checks if a game piece is present 
     * @return  true if a gamepiece is present, false otherwise
     */
    public boolean isNotePresent() {
        return !photoeye.get();
    }
    
    /**
     * Subsystem periodic function
     */
    @Override
    public void periodic() {
        if(state == PizzaboxState.INTAKE) {
            // Check if a gamepiece is present
            if (isNotePresent()) {
                intakeRollers.set(0);  // Stop intake if a gamepiece is present
            } else {
                intakeRollers.set(1);  // Run motor if no intake is present
            }
        }else if(state == PizzaboxState.SHOOT_PREP) {
            shooterTop.set(Constants.shootPrepPower);    // Turn shooter to idle speed
        } else if (state == PizzaboxState.SHOOT) {
            shooterTop.set(1);    // Turn shooter to max power

            // Check if shooter is ready to shoot
            if(shootEncoder1.getVelocity() > Constants.minShootSpeed && shootEncoder2.getVelocity() > Constants.minShootSpeed) {
                intakeRollers.set(1);  // Run intake 
            }else{
                intakeRollers.set(0);  // Turn Intake Off
            }
 
        } else if (state == PizzaboxState.REVERSE) {
            // Reverse shooter and intake
            shooterTop.set(-1);
            intakeRollers.set(-1);
        } else {
            // Turn shooter and intake off
            shooterTop.set(0);
            intakeRollers.set(0);
        }

        updateUI();
    }

    private void updateUI() {
        sb_state.setString(state.name());
        sb_shooterTopVolt.setDouble(shooterTop.getBusVoltage() * shooterTop.getAppliedOutput());
        sb_shooterBotVolt.setDouble(shooterTop.getBusVoltage() * shooterTop.getAppliedOutput());
        sb_shooterTopRate.setDouble(shootEncoder1.getVelocity()); 
        sb_shooterBotRate.setDouble(shootEncoder2.getVelocity());
        sb_intakeRollerVolt.setDouble(intakeRollers.getMotorVoltage().getValueAsDouble());
        sb_intakeRollerRate.setDouble(intakeRollers.getVelocity().getValueAsDouble());
        sb_notePresent.setBoolean(isNotePresent());
    }

    /**
     * Static Initializer
     */
    public static IntakePizzaBox getInstance() {
        if (intake == null) {
            intake = new IntakePizzaBox();
        }
        return intake;
    }
}
