package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class Arm {
    private static Arm arm;

    private TalonFX armMotor1;
    private TalonFX armMotor2;
    private DoubleSolenoid armExtender1;
    private DoubleSolenoid armExtender2;
    private SparkAbsoluteEncoder armEncoder;


    private static PIDController armPID;

    public enum ArmState{
        HOME,
        EXTENDEDHOME,
        SPEAKER,
        AMP,
        TRAPCLIMB,
        TRAPSCORE


    }
    private Arm() {
        armMotor1 = new TalonFX(Constants.armMotor1);
        armMotor2 = new TalonFX(Constants.armMotor2);
        armExtender1 = new DoubleSolenoid(null, 0, 0);
        //armEncoder = new SparkAbsoluteEncoder();
        armPID = new PIDController(Constants.kpArm1, Constants.kiArm1, Constants.kdArm1);
    }

    public static Arm get_Arm() {
        if (arm == null) {
            arm = new Arm();
        }
        return arm;
    }

    public void calcPIDArm1(double speed) {
        
    }

    public void armExtendForward(){
        armExtender1.set(Value.kForward);
        armExtender2.set(Value.kForward);
    }

    public void armExtenderReverse(){
        armExtender1.set(Value.kReverse);
        armExtender2.set(Value.kReverse);
    }


  


}
