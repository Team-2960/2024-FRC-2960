package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.Map;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private static Arm arm;

    public enum ArmControlMode {
        MANUAL,
        AUTOMATIC
    }

    public enum ExtensionState {
        STAGE0,
        STAGE1,
        STAGE2
    }

    /**
     * Defines an arm position state
     */
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

    private TalonFX armMotor1;
    private TalonFX armMotor2;

    private DoubleSolenoid armExtender1;
    private DoubleSolenoid armExtender2;

    private Encoder quadArmEncoder;

    private DutyCycleEncoder absoluteArmEncoder;

    private PIDController armPID;
    private ArmFeedforward armFF;

    private final ArmStateValues defaultState = new ArmStateValues(Rotation2d.fromDegrees(10), ExtensionState.STAGE0);

    private ArmControlMode control_mode;

    private ArmStateValues targetState = defaultState;
    private Timer extenderTimer;

    private double manual_rate;
    private ExtensionState manual_ext;

    private Map<String, ArmStateValues> armStates = Map.of(
        "Match Start", new ArmStateValues(Rotation2d.fromDegrees(60), ExtensionState.STAGE0),
        "Home", defaultState,
        "Intake", new ArmStateValues(Rotation2d.fromDegrees(-5), ExtensionState.STAGE1),
        "Speaker", new ArmStateValues(Rotation2d.fromDegrees(10), ExtensionState.STAGE0),
        "Amp", new ArmStateValues(Rotation2d.fromDegrees(60), ExtensionState.STAGE1),
        "Climb", new ArmStateValues(Rotation2d.fromDegrees(90), ExtensionState.STAGE2),
        "Climb Balance", new ArmStateValues(Rotation2d.fromDegrees(80), ExtensionState.STAGE2),
        "Trap Score",   new ArmStateValues(Rotation2d.fromDegrees(70), ExtensionState.STAGE2)
    );

    private GenericEntry sb_anglePosCurrent;
    private GenericEntry sb_anglePosSetPoint;
    private GenericEntry sb_angleRateCurrent;
    private GenericEntry sb_angleRateSetPoint;
    private GenericEntry sb_angleM1Volt;
    private GenericEntry sb_angleM2Volt;
    private GenericEntry sb_extStage1;
    private GenericEntry sb_extStage2;

    /**
     * Constructor
     */
    private Arm() {
        
        armMotor1 = new TalonFX(Constants.armMotor1);
        armMotor2 = new TalonFX(Constants.armMotor2);

        armExtender1 = new DoubleSolenoid(20, PneumaticsModuleType.REVPH, 9, 8);
        armExtender2 = new DoubleSolenoid(20, PneumaticsModuleType.REVPH, 7, 6);

        quadArmEncoder = new Encoder(1, 2);

        absoluteArmEncoder = new DutyCycleEncoder(0);

        armPID = new PIDController(Constants.armPID.kP, Constants.armPID.kP, Constants.armPID.kP);
        armFF = new ArmFeedforward(Constants.armFF.kS, Constants.armFF.kV, Constants.armFF.kG);
        
        quadArmEncoder.setDistancePerPulse(2 * Math.PI / 4960);
        absoluteArmEncoder.setDistancePerRotation(2 * Math.PI);
        // TODO Set abs encoder offset

        // Set control mode
        control_mode = ArmControlMode.MANUAL;
        manual_rate = 0;
        manual_ext = ExtensionState.STAGE0;

        // Set target state to current state
        targetState = new ArmStateValues(getArmAngle(), getArmExtension());

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status")
            .getLayout("Arm", BuiltInLayouts.kList)
            .withSize(2,4);

        sb_anglePosCurrent = layout.add("Angle Position Current", 0.).getEntry();
        sb_anglePosSetPoint = layout.add("Angle Position Set Point", 0).getEntry();
        sb_angleRateCurrent = layout.add("Angle Rate Current", 0).getEntry();
        sb_angleRateSetPoint = layout.add("Angle Rate Set Point", 0).getEntry();
        sb_angleM1Volt = layout.add("Angle Motor 1 Voltage", 0).getEntry();
        sb_angleM2Volt = layout.add("Angle Motor 2 Voltage", 0).getEntry();
        sb_extStage1 = layout.add("Ext Stage 1 State", armExtender1.get().name()).getEntry();
        sb_extStage2 = layout.add("Ext State 2 State", armExtender2.get().name()).getEntry();
    }

    /**
     * Gets the current arm angle
     * @return current arm angle
     */
    public Rotation2d getArmAngle() {
        double angle = 107-Rotation2d.fromRotations(absoluteArmEncoder.get()).getDegrees();
        return Rotation2d.fromDegrees(angle);
    }

    /**
     * Gets the current arm angle rate
     * @return current arm angle rate in radians per second
     */
    public double getArmVelocity() {
        return quadArmEncoder.getRate() ;
    }

    /**
     * Checks the current extension state
     * @return  current extension state
     */
    public ExtensionState getArmExtension() {
        boolean isLowerExt = armExtender1.get() == DoubleSolenoid.Value.kForward;
        boolean isUpperExt = armExtender2.get() == DoubleSolenoid.Value.kForward;
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
     * Sets the arm's target rate. Puts the arm into manual mode. If the arm is 
     *  not in manual mode already, the extension state is set to its current
     *  state.
     * @param   rate    new arm rate
     */
    public void setArmRate(double rate) {
        manual_rate = rate;

        if (control_mode != ArmControlMode.MANUAL) manual_ext = getArmExtension();
        
        control_mode = ArmControlMode.MANUAL;
    }

    /**
     * Sets the arm's extension state. Puts the arm into manual mode. If the arm is 
     *  not in manual mode already, the arm rate is set to 0.
     * @param   state   extension state
     */
    public void setExtState(ExtensionState state) {
        manual_ext = state;

        if (control_mode != ArmControlMode.MANUAL) manual_rate = 0;
        
        control_mode = ArmControlMode.MANUAL;
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
        return getArmExtension() == targetState.extState && extenderTimer.get() > Constants.armExtDelayTime;
    }

    /**
     * Check if the arm is at its target angle and extension
     * @return  true if the angle and extension are at their targets
     */
    public boolean atTarget() {
        return atAngle() && atExtention();
    }

    public boolean isInClimberZone() {
        Rotation2d currentAngle = getArmAngle();

        boolean in_zone = currentAngle.getDegrees() > Constants.climberZoneLowerAngle.getDegrees();
        in_zone &= currentAngle.getDegrees() < Constants.climberZoneUpperAngle.getDegrees();

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
        control_mode = ArmControlMode.AUTOMATIC;
    }

    /**
     * Subsystem periodic method
     */
    @Override
    public void periodic(){
        double targetArmRate = getTargetArmRate();
        updateAngleControl(targetArmRate);
        updateExtension();
        updateUI(targetArmRate);
        SmartDashboard.getNumber("Arm Angle", getArmAngle().getDegrees());
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
     * Determines the current target arm control rate
     * @return  target arm control rate based on current settings
     */
    private double getTargetArmRate() {
        Rotation2d minS2Angle = Rotation2d.fromDegrees(30);
        Rotation2d currentAngle = getArmAngle();
        double targetSpeed = 0;

        switch(control_mode) {
            case AUTOMATIC:
                targetSpeed = calcTrapezoidalRate();
                break;
            case MANUAL:
                targetSpeed = manual_rate;
                break;
        }

        // Set soft limits
        if(currentAngle.getDegrees() >= Constants.maxArmPos && targetSpeed > 0) targetSpeed = 0;
        if(currentAngle.getDegrees() <= Constants.minArmPos && targetSpeed > 0) targetSpeed = 0;

        // Protect against climber collisions
        if(!Climber.getInstance().isClearOfArm() && isInClimberZone()) targetSpeed = 0;

        // Keep arm in package
        if(getArmExtension() == ExtensionState.STAGE2 && currentAngle.getDegrees() <= minS2Angle.getDegrees()) targetSpeed = 0;

        return targetSpeed;
    }

    /**
     * Calculate the trapezoidal control rate for the current arm target position
     * @return  target arm control rate 
     */
    private double calcTrapezoidalRate() {
        double maxAngleRate = Math.PI;

        // Calculate trapezoidal profile
        Rotation2d currentAngle = getArmAngle();
        Rotation2d targetAngle = targetState.targetAngle;
        Rotation2d angleError = targetAngle.minus(currentAngle);
        
        double targetSpeed = maxAngleRate * (angleError.getRadians() > 0 ? 1 :+ -1);
        double rampDownSpeed = angleError.getRadians() / Constants.armRampDownDist.getRadians() * maxAngleRate;

        if (Math.abs(rampDownSpeed) < Math.abs(targetSpeed)) targetSpeed = rampDownSpeed;

        return targetSpeed;
    }

    /**
     * Updates the control of the arm rate
     * @param   targetSpeed     target
     */
    private void updateAngleControl(double targetSpeed) {
        Rotation2d currentAngle = getArmAngle();
        double angleRate = getArmVelocity();


        // Calculate motor voltage output
        double calcPID = armPID.calculate(angleRate, targetSpeed);
        double calcFF = armFF.calculate(currentAngle.getRadians(), targetSpeed);
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
        ExtensionState targetState = ExtensionState.STAGE0;

        switch(control_mode) {
            case AUTOMATIC:
                targetState = this.targetState.extState;
                break;
            case MANUAL:
                targetState = manual_ext;
                break;
        }

        boolean aboveState2Angle = getArmAngle().getDegrees() > Constants.armMinState2Angle.getDegrees();

        // Set target extension valve state
        if(targetState == ExtensionState.STAGE2 && aboveState2Angle) {
                armExtender1.set(DoubleSolenoid.Value.kForward);
                armExtender2.set(DoubleSolenoid.Value.kForward);
        } else if (targetState == ExtensionState.STAGE1 || targetState == ExtensionState.STAGE2 && !aboveState2Angle) {
                armExtender1.set(DoubleSolenoid.Value.kForward);
                armExtender2.set(DoubleSolenoid.Value.kForward);
        } else {
            armExtender1.set(DoubleSolenoid.Value.kForward);
            armExtender2.set(DoubleSolenoid.Value.kForward);
        }

        // Reset extension timer of the extension state has chanced
        if(currentState != targetState) extenderTimer.restart();
    }

    /**
     * Updates shuffleboard
     */
    private void updateUI(double targetRate) {
        sb_anglePosCurrent.setDouble(getArmAngle().getDegrees());
        sb_anglePosSetPoint.setDouble(targetState.targetAngle.getDegrees());
        sb_angleRateCurrent.setDouble(quadArmEncoder.getRate());
        sb_angleRateSetPoint.setDouble(targetRate);
        sb_angleM1Volt.setDouble(armMotor1.getMotorVoltage().getValueAsDouble());
        sb_angleM2Volt.setDouble(armMotor2.getMotorVoltage().getValueAsDouble());
        sb_extStage1.setString(armExtender1.get().name());
        sb_extStage2.setString(armExtender2.get().name());
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
