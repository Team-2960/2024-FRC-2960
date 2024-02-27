package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

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

    /**
     * Constructor
     */
    private IntakePizzaBox(){
       intakeRollers = new TalonFX(0);

       shooter1 = new CANSparkMax(0, MotorType.kBrushless);
       shooter2 = new CANSparkMax(0, MotorType.kBrushless);
       shooter2.follow(shooter1, true);

       shootEncoder1 = shooter1.getEncoder();
       shootEncoder2 = shooter1.getEncoder();

       photoeye = new DigitalInput(0);
       
       state = IDLE;
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
    @override
    public void periodic() {
        if(state == INTAKE) {
            // Check if a gamepiece is present
            if (isNotePresent()) {
                intake.set(0);  // Stop intake if a gamepiece is present
            } else {
                intake.set(1);  // Run motor if no intake is present
            }
        }else if(state == SHOOT_PREP) {
            shooter1.set(.2);    // Turn shooter to idle speed
        } else if (state == SHOOT) {
            shooter1.set(1);    // Turn shooter to max power

            double minShootSpeed = 500;     // TODO Move to constants
            
            // Check if shooter is ready to shoot
            if(shootEncoder1.getVelocity() > minShootSpeed && shootEncoder2.getVelocity() > minShootSpeed) {
                intake.set(1);  // Run intake 
            }else{
                intake.set(0);  // Turn Intake Off
            }
 
        } else if (state == REVERSE) {
            // Reverse shooter and intake
            shooter1.set(-1);
            intake.set(-1);
        } else {
            // Turn shooter and intake off
            shooter1.set(0);
            intake.set(0);
        }

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
