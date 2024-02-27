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

import java.lang.Map;

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

    public class ArmStateValues() {
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

    private final ArmStateValues defaultState = new ArmStateValue(Rotation2d.fromDegrees(10), Stage0);

    private ArmStateValues targetState = defaultState;
    private Timer extenderTimer;
    private double targetPos = 0;

    private Map<String, ArmStateValues> armStates = Map.of(
        "Match Start", new ArmStateValue(Rotation2d.fromDegrees(60), Stage0),
        "Home", defaultState,
        "Intake", new ArmStateValue(Rotation2d.fromDegrees(-5), Stage1),
        "Speaker", new ArmStateValue(Rotation2d.fromDegrees(10), Stage0),
        "Amp", new ArmStateValue(Rotation2d.fromDegrees(60), Stage1),
        "Climb", new ArmStateValue(Rotation2d.fromDegrees(90), Stage2),
        "Climb Balance", new ArmStateValue(Rotation2d.fromDegrees(80), Stage2),
        "Trap Score",   new ArmStateValue(Rotation2d.fromDegrees(70), Stage2)
    )

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
        targetState = new targetState(getArmAngle(), getArmExtension());

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
        boolean isLowerExt = armExtender1.get() == DoubleSolenoid.kReverse;
        boolean isUpperExt = armExtender2.get() == DoubleSolenoid.kReverse;
        ExtensionState state = STAGE0;

        if (isLowerExt) {
            if(isUpperExt) {
                state = STAGE2;
            } else {
                state = STAGE1;
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

        return Math.abs(tarAngle.minus(curAngle).toDegrees()) < angleTol.toDegrees();
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
        ArmStateValues targetState = armStates.get(name);

        if(targetState == null) targetState = defaultState;

        return targetState;
    }

    /**
     * Updates the control of the arm angle
     */
    private void updateAngle() {
        Rotation2d rampDownDist = Rotatation2d.fromDegrees(10); // TODO Move to constants
        double maxAngleRate = Math.pi;

        // Calculate trapizoidal profile
        Rotation2d currentAngle = getArmAngle();
        Rotation2d targetAngle = targetState.targetAngle;
        Rotation2d angleError = targetAngle.minus(currentAngle);
        
        double targetSpeed = maxAngleRate * (angleError.toRadians() > 0 ? 1 : -1);
        double rampDownSpeed = angleError.toRadians() / rampDownDist * maxAngleRate;

        if (Math.abs(rampDownSpeed) < Math.abs(targetSpeed)) targetSpeed = rampDownSpeed;

        // Set soft limits
        if(currentAngle.getDegrees() >= Constants.maxArmPos && targetSpeed > 0) targetSpeed = 0;
        if(currentAngle.getDegrees() <= Constants.minArmPos && targetSpeed > 0) targetSpeed = 0;

        // Calculate motor voltage output
        double calcPID = armPID.calculate(getArmVelocity(), targetSpeed);
        double calcFF = armFeedForward.calculate(currentAngle.toRadians(), targetSpeed);
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
        ExtensionState targetState = targetState.extState;

        Rotation2d minState2Angle = Rotation2d.fromDegrees(30); // TODO Move to constants
        boolean aboveState2Angle = getArmAngle().toDegrees() > minState2Angle.toDegrees()

        // Set target extension valve state
        if(targetState == STAGE2 && aboveState2Angle) {
                armExtender1.set(DoubleSolenoid.kForward);
                armExtender2.set(DoubleSolenoid.kForward);
        } else if (targetState == STAGE1 || targetState == STAGE2 && !aboveState2Angle) {
                armExtender1.set(DoubleSolenoid.kForward);
                armExtender2.set(DoubleSolenoid.kReverse);
        } else {
            armExtender1.set(DoubleSolenoid.kReverse);
            armExtender2.set(DoubleSolenoid.kReverse);
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
