package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private static Arm arm;

    private TalonFX armMotor1;
    private TalonFX armMotor2;
    private DoubleSolenoid armExtender1;
    private DoubleSolenoid armExtender2;
    private Encoder quadArmEncoder;
    private DutyCycleEncoder absoluteArmEncoder;
    private SimpleMotorFeedforward armFeedForward;
    private static PIDController armPID;
    private static SimpleMotorFeedforward armFF;

    public enum ArmState {
        HOME, // Resting on bumper
        EXTENDEDHOME,
        SPEAKER, // Point blank speaker shot
        AMP,
        CLIMBPREP,
        TRAPCLIMB,
        TRAPSCORE
    }

    private ArmState currentState;
    private ArmState targState;
    private Timer extenderTimer;
    private double targetPos = 0;

    private Arm() {
        armMotor1 = new TalonFX(Constants.armMotor1);
        armMotor2 = new TalonFX(Constants.armMotor2);

        armExtender1 = new DoubleSolenoid(null, 0, 0);
        armExtender2 = new DoubleSolenoid(null, 0, 0);

        quadArmEncoder = new Encoder(1, 2);

        absoluteArmEncoder = new DutyCycleEncoder(0);

        armPID = new PIDController(Constants.kpArm1, Constants.kiArm1, Constants.kdArm1);
        // TODO Init FF

        quadArmEncoder.setDistancePerPulse(360 / 4960);
        absoluteArmEncoder.setDistancePerRotation(360);
        // TODO Set abs encoder offset
    }

    public Rotation2d getCurrentPos() {
        double curPosDegrees = absoluteArmEncoder.getAbsolutePosition();
        return Rotation2d.fromDegrees(curPosDegrees);
    }

    public double getVelocity() {
        return quadArmEncoder.getRate();
    }

    private static double getTargetPos(ArmState target) {
        switch (target) {
            case HOME:
                return Constants.homePos;
            case EXTENDEDHOME:
                return Constants.homePos;
            case SPEAKER:
                return Constants.speakerPos;
            case AMP:
                return Constants.ampPos;
            case CLIMBPREP:
                return Constants.climbPrepPos;
            case TRAPCLIMB:
                return Constants.trapClimbPos;
            case TRAPSCORE:
                return Constants.trapScorePos;
            default:
                return Constants.homePos;
        }
    }

    public void setState(ArmState state){
        targState = state;
    }

    public double calcSpeed(double cur, double tar) {
        double speed = 0;
        if ((getCurrentPos().getDegrees() < Constants.maxArmPos) && (getCurrentPos().getDegrees() > Constants.minArmPos)) {
            double calcPID = armPID.calculate(cur, tar);
            double calcFF = armFeedForward.calculate(getVelocity());
            speed = calcPID + calcFF;
        }else{
            speed = 0;
        }
        return speed;
    }

    public static Arm get_Arm() {
        if (arm == null) {
            arm = new Arm();
        }
        return arm;
    }

    public void setExtender1(Value state) {
        armExtender1.set(state);
    }

    public void setExtender2(Value state) {
        armExtender2.set(state);
    }
    /*
     * public void armExtendForward(){
     * armExtender1.set(Value.kForward);
     * armExtender2.set(Value.kForward);
     * }
     * 
     * public void armExtenderReverse(){
     * armExtender1.set(Value.kReverse);
     * armExtender2.set(Value.kReverse);
     * }
     */

    @Override
    public void periodic(){
        
    }

}
