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
import java.util.*;

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

    public enum ExtensionState {
        STAGE0,
        STAGE1,
        STAGE2
    }

    public class ArmStateValues {
        public Rotation2d targetAngle;
        public Rotation2d angleTol;
        public ExtensionState extState;
        
        public ArmStateValues(Rotation2d targetAngle, ExtensionState extState){
            this(targetAngle, Rotation2d.fromDegrees(2), extState);
        }

        public ArmStateValues(Rotation2d targetAngle, Rotation2d angleTol, ExtensionState extState){
            this.targetAngle = targetAngle;
            this.angleTol = angleTol;
            this.extState = extState;
        }
    }

    private final ArmStateValues defaultState = new ArmStateValues(Rotation2d.fromDegrees(10), ExtensionState.STAGE0);

    private ArmStateValues targetState = defaultState;
    private Timer extenderTimer;
    private double targetPos = 0;

  /*  private Map<String, ArmStateValues> armStates = HashMap.of(
        "Match Start", new ArmStateValue(Rotation2d.fromDegrees(60), Stage0),
        "Home", defaultState,
        "Intake", new ArmStateValue(Rotation2d.fromDegrees(-5), Stage1),
        "Speaker", new ArmStateValue(Rotation2d.fromDegrees(10), Stage0),
        "Amp", new ArmStateValue(Rotation2d.fromDegrees(60), Stage1),
        "Climb", new ArmStateValue(Rotation2d.fromDegrees(90), Stage2),
        "Climb Balance", new ArmStateValue(Rotation2d.fromDegrees(80), Stage2),
        "Trap Score",   new ArmStateValue(Rotation2d.fromDegrees(70), Stage2)
    )*/


       


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

        // Set target state to current state
        targetState = new ArmStateValues(getArmAngle(), getArmExtension());

    }
        ArmStateValues matchStart = new ArmStateValues(Rotation2d.fromDegrees(60), ExtensionState.STAGE0);
        ArmStateValues Home = defaultState;
        ArmStateValues Intake = new ArmStateValues(Rotation2d.fromDegrees(-5), ExtensionState.STAGE1);
        ArmStateValues Speaker = new ArmStateValues(Rotation2d.fromDegrees(10), ExtensionState.STAGE0);
        ArmStateValues Amp = new ArmStateValues(Rotation2d.fromDegrees(60), ExtensionState.STAGE1);
        ArmStateValues Climb = new ArmStateValues(Rotation2d.fromDegrees(90), ExtensionState.STAGE2);
        ArmStateValues ClimbBalance = new ArmStateValues(Rotation2d.fromDegrees(80), ExtensionState.STAGE2);
        ArmStateValues TrapScore = new ArmStateValues(Rotation2d.fromDegrees(70), ExtensionState.STAGE2);
    
        public ArmStateValues armStates(String armState) {
        switch(armState){
            case "Match Start":
            return matchStart;
            case "Home":
            return Home;
            case "Intake":
            return Intake;
            case "Speaker":
            return Speaker;
            case "Amp":
            return Amp;
            case "Climb":
            return Climb;
            case "Climb Balance":
            return ClimbBalance;
            case "Trap Score":
            return TrapScore;
            default: return Home;
        }
    }

        
    

    /**
     * Gets the current arm angle
     * @return current arm angle
     */
    public Rotation2d getArmAngle() {
        return Rotation2d.fromDegrees(absoluteArmEncoder.getAbsolutePosition());
    }

    /**
     * Gets the current arm angle rate
     * @return current arm angle rate
     */
    public double getArmVelocity() {
        return quadArmEncoder.getRate();
    }

    /**
     * Checks the current extension state
     * @return  current extension state
     */
    public ExtensionState getArmExtension() {
        boolean isLowerExt = armExtender1.get() == Value.kForward;
        boolean isUpperExt = armExtender2.get() == Value.kForward;
        ExtensionState state = ExtensionState.STAGE0;

        if (isLowerExt) {
            if(isUpperExt) {
                state = ExtensionState.STAGE2;
            } else {
                state = ExtensionState.STAGE1;
            }
        }

        return state;
    }



    /**
     * Check if the arm is at its target angle
     * @return  true if the angle are at their target
     */
    public boolean atAngle() {
        Rotation2d currentAngle = getArmAngle();
        Rotation2d targetAngle = targetState.targetAngle;
        Rotation2d angleTol = targetState.angleTol;

        return Math.abs(targetAngle.minus(currentAngle).getDegrees()) < angleTol.getDegrees();
    }


    /**
     * Check if the arm is at its target extension
     * @return  true if the extension are at their target
     */
    public boolean atExtention() {
        return getArmExtension() == targetState.extState && extenderTimer.get() > .25;  // TODO Get extension time and move to constants
    }

    /**
     * Check if the arm is at its target angle and extension
     * @return  true if the angle and extension are at their targets
     */
    public boolean atTarget() {
        return atAngle() && atExtention();
    }

    public boolean isInClimberZone() {
        Rotation2d climberZoneLowerAngle =  Rotation2d.fromDegrees(60);  // TODO Move to constants
        Rotation2d climberZoneUpperAngle =  Rotation2d.fromDegrees(75);  // TODO Move to constants 
        Rotation2d currentAngle = getArmAngle();

        boolean in_zone = currentAngle.getDegrees() > climberZoneLowerAngle.getDegrees();
        in_zone &= currentAngle.getDegrees() < climberZoneUpperAngle.getDegrees();

        return in_zone;
    }

    /**
     * Looks up a standard target state
     * @param   Name of the standard state. If an unknown name is supplied, 
     *              the state will be set to the home position
     */
    public void setState(String name){
        setState(getTargetValues(name));
    }

    /**
     * Sets the target state for the arm
     * @param targetState   Current targetState value for the arm
     */
    public void setState(ArmStateValues targetState) {
        this.targetState = targetState;
    }

    /**
     * Subsystem periodic method
     */
    @Override
    public void periodic(){
        updateAngle();
        updateExtension();
    }

    /**
     * Looks up standard target values
     */
    private ArmStateValues getTargetValues(String name) {
        ArmStateValues targetState = armStates("name");

        if(targetState == null) targetState = defaultState;

        return targetState;
    }

    /**
     * Updates the control of the arm angle
     */
    private void updateAngle() {
        Rotation2d rampDownDist = Rotation2d.fromDegrees(10); // TODO Move to constants
        double maxAngleRate = Math.PI;

        // Calculate trapizoidal profile
        Rotation2d currentAngle = getArmAngle();
        Rotation2d targetAngle = targetState.targetAngle;
        Rotation2d angleError = targetAngle.minus(currentAngle);
        
        double targetSpeed = maxAngleRate * (angleError.getRadians() > 0 ? 1 :+ -1);
        double rampDownSpeed = angleError.getRadians() / rampDownDist.getRadians() * maxAngleRate;

        if (Math.abs(rampDownSpeed) < Math.abs(targetSpeed)) targetSpeed = rampDownSpeed;

        // Set soft limits
        if(currentAngle.getDegrees() >= Constants.maxArmPos && targetSpeed > 0) targetSpeed = 0;
        if(currentAngle.getDegrees() <= Constants.minArmPos && targetSpeed > 0) targetSpeed = 0;

        // Protect against climber collisions
      //  if(!Climber.get_Climber().isClearOfArm() && isInClimberZone()) targetSpeed = 0;

        // Calculate motor voltage output
        double calcPID = armPID.calculate(getArmVelocity(), targetSpeed);
        double calcFF = armFeedForward.calculate(currentAngle.getRadians(), targetSpeed);
        double targetVoltage = calcPID + calcFF;
        
        // Set Motors
        armMotor1.setVoltage(targetVoltage);
        armMotor2.setVoltage(targetVoltage);
    }

    /**
     * Updates the control of the arm extension
     */
    private void updateExtension() {
        ExtensionState currentState = getArmExtension();
        ExtensionState targetState = this.targetState.extState;

        Rotation2d minState2Angle = Rotation2d.fromDegrees(30); // TODO Move to constants
        boolean aboveState2Angle = getArmAngle().getDegrees() > minState2Angle.getDegrees();

        // Set target extension valve state
        if(targetState == ExtensionState.STAGE2 && aboveState2Angle) {
                armExtender1.set(Value.kForward);
                armExtender2.set(Value.kForward);
        } else if (targetState == ExtensionState.STAGE1 || targetState == ExtensionState.STAGE2 && !aboveState2Angle) {
                armExtender1.set(Value.kForward);
                armExtender2.set(Value.kReverse);
        } else {
            armExtender1.set(Value.kReverse);
            armExtender2.set(Value.kReverse);
        }

        // Reset extension timer of the extension state has chanced
        if(currentState != targetState) extenderTimer.restart();
    }

    /**
     * Static initializer for the arm class
     */
    public static Arm getInstance() {
        if (arm == null) {
            arm = new Arm();
        }
        return arm;
    }

}
