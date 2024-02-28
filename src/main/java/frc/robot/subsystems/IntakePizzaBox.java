package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakePizzaBox extends SubsystemBase {
    private static IntakePizzaBox intake = null;
    private TalonFX intakeRollers;
    private CANSparkMax shooter1;
    private CANSparkMax shooter2;
    private DigitalInput photoeye;
    private SimpleMotorFeedforward intakeFeedForward;

    private IntakePizzaBox() {
        TalonFX intakeRollers = new TalonFX(0);
        CANSparkMax shooter1 = new CANSparkMax(0, MotorType.kBrushless);
        CANSparkMax shooter2 = new CANSparkMax(0, MotorType.kBrushless);
        DigitalInput photoeye = new DigitalInput(0);
    }

    public void setShooter(double speed) {
        shooter1.set(speed);
        shooter2.set(-speed);
        // TODO Test motors and reverse speeds if needed
    }

    public void setIntake(double speed, boolean override) {
        double feedForward = intakeFeedForward.calculate(intakeRollers.getVelocity().getValueAsDouble());
        if ((photoeye.get() == false) || (photoeye.get() && override == true)) {
            intakeRollers.set(speed + feedForward);
        } else if (photoeye.get() && override == false) {
            intakeRollers.set(0);
        }
    }
    //TODO ask Fitz what to do about the note slipping out of Pizza Box and fix it

    public static IntakePizzaBox get_intake() {
        if (intake == null) {
            intake = new IntakePizzaBox();
        }
        return intake;
    }
}