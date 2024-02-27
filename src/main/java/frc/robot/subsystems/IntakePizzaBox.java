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

    public void setState(PizzaboxState state) {
        this.state = state;
    }

    public PizzaboxState getState() {
        return state;
    }

    public boolean isNotePresent() {
        return photoeye.get();
    }
    
    @override
    public void periodic() {
        if(state == INTAKE) {
            if (isNotePresent()) {
                intake.set(1);
            } else {
                intake.set(0);
            }
        } else if (state == SHOOT) {
            shooter1.set(1);

            double minShootSpeed = 500;     // TODO Move to constants
            
            if(shootEncoder1.getVelocity() > minShootSpeed && shootEncoder2.getVelocity() > minShootSpeed) {
                intake.set(1);
            }else{
                intake.set(0);
            }
 
        } else if (state == REVERSE) {
            shooter1.set(-1);
            intake.set(-1);
        } else {
            shooter1.set(0);
            intake.set(0);
        }

    }

    public static IntakePizzaBox getInstance() {
        if (intake == null) {
            intake = new IntakePizzaBox();
        }
        return intake;
    }
}
