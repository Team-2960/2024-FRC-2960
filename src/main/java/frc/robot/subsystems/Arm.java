package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

public class Arm {
    private static Arm arm;

    private TalonFX armMotor1;
    private TalonFX armMotor2;

    private static PIDController armPID;

    private Arm() {
        armMotor1 = new TalonFX(Constants.armMotor1);
        armMotor2 = new TalonFX(Constants.armMotor2);

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

}
