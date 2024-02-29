package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    private CANSparkMax shooter1;
    private CANSparkMax shooter2;

    private RelativeEncoder shootEncoder1;
    private RelativeEncoder shootEncoder2;

    private DigitalInput photoeye;

    PizzaboxState state;

    private GenericEntry sb_state;
    private GenericEntry sb_shooter1Volt;
    private GenericEntry sb_shooter2Volt;
    private GenericEntry sb_shooter1Rate;
    private GenericEntry sb_shooter2Rate;
    private GenericEntry sb_intakeRollerVolt;
    private GenericEntry sb_intakeRollerRate;
    private GenericEntry sb_notePresent;

    /**
     * Constructor
     */
    private IntakePizzaBox(){
        // Initialize Intake Motor  
        intakeRollers = new TalonFX(0);

        // Initialize Shooter Motors
        shooter1 = new CANSparkMax(0, MotorType.kBrushless);
        shooter2 = new CANSparkMax(0, MotorType.kBrushless);
        shooter2.follow(shooter1, true);

        // Initialize Shooter Encoders
        shootEncoder1 = shooter1.getEncoder();
        shootEncoder2 = shooter1.getEncoder();

        // Initialize photoeye
        photoeye = new DigitalInput(0);
        
        // Initialize state
        state = PizzaboxState.IDLE;

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status").getLayout("Pizzabox");

        sb_state = layout.add("State", state.name());
        sb_shooter1Volt = layout.add("Shooter 1 Voltage", 0);
        sb_shooter2Volt = layout.add("Shooter 2 Voltage", 0);
        sb_shooter1Rate = layout.add("Shooter 1 Rate", 0);
        sb_shooter2Rate = layout.add("Shooter 2 Rate", 0);
        sb_intakeRollerVolt = layout.add("Intake Roller Voltage", 0);
        sb_intakeRollerRate = layout.add("Intake Roller Rate", 0);
        sb_notePresent = layout.add("Note Present", false);
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
            shooter1.set(.2);    // Turn shooter to idle speed
        } else if (state == PizzaboxState.SHOOT) {
            shooter1.set(1);    // Turn shooter to max power

            double minShootSpeed = 500;     // TODO Move to constants
            
            // Check if shooter is ready to shoot
            if(shootEncoder1.getVelocity() > minShootSpeed && shootEncoder2.getVelocity() > minShootSpeed) {
                intakeRollers.set(1);  // Run intake 
            }else{
                intakeRollers.set(0);  // Turn Intake Off
            }
 
        } else if (state == PizzaboxState.REVERSE) {
            // Reverse shooter and intake
            shooter1.set(-1);
            intakeRollers.set(-1);
        } else {
            // Turn shooter and intake off
            shooter1.set(0);
            intakeRollers.set(0);
        }

    }

    private void updateUI() {
        sb_state.setString(state.name());
        sb_shooter1Volt.setDouble(shooter1.getBusVoltage() * shooter1.getAppliedOutput());
        sb_shooter2Volt.setDouble(shooter1.getBusVoltage() * shooter1.getAppliedOutput());
        sb_shooter1Rate.setDouble(shootEncoder1.getVelocity()); 
        sb_shooter2Rate.setDouble(shootEncoder2.getVelocity());
        sb_intakeRollerVolt.setDouble(intakeRollers.getMotorVoltage());
        sb_intakeRollerRate.setDouble(intakeRollers.getVelocity());
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
