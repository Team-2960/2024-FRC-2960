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
        MANUAL_VOLT,
        MANUAL_RATE,
        AUTOMATIC
    }

    /**
     * Defines an arm position state
     */
    public class ArmStateValues {
        public Rotation2d targetAngle;
        public Rotation2d angleTol;
        public int extState;
        
        public ArmStateValues(Rotation2d targetAngle, int extState){
            this(targetAngle, Rotation2d.fromDegrees(2), extState);
        }

        public ArmStateValues(Rotation2d targetAngle, Rotation2d angleTol, int extState){
            this.targetAngle = targetAngle;
            this.angleTol = angleTol;
            this.extState = Math.max(0, Math.min(2, extState));
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

    private final ArmStateValues defaultState = new ArmStateValues(Rotation2d.fromDegrees(10), 0);

    private ArmControlMode control_mode;

    private ArmStateValues targetState = defaultState;
    private Timer extenderTimer;

    private double manual_volt;
    private double manual_rate;
    private int manual_ext;

    private Map<String, ArmStateValues> armStates = Map.of(
        "Match Start", new ArmStateValues(Rotation2d.fromDegrees(60), 0),
        "Home", defaultState,
        "Intake", new ArmStateValues(Rotation2d.fromDegrees(-5), 1),
        "Speaker", new ArmStateValues(Rotation2d.fromDegrees(10), 0),
        "Amp", new ArmStateValues(Rotation2d.fromDegrees(60), 1),
        "Climb", new ArmStateValues(Rotation2d.fromDegrees(90), 2),
        "Climb Balance", new ArmStateValues(Rotation2d.fromDegrees(80), 2),
        "Trap Score",   new ArmStateValues(Rotation2d.fromDegrees(70), 2)
    );

    private GenericEntry sb_armMode;
    private GenericEntry sb_anglePosCurrent;
    private GenericEntry sb_anglePosSetPoint;
    private GenericEntry sb_angleRateCurrent;
    private GenericEntry sb_angleRateSetPoint;
    private GenericEntry sb_angleM1Volt;
    private GenericEntry sb_angleM2Volt;
    private GenericEntry sb_angleTargetVolt;
    private GenericEntry sb_extStage1;
    private GenericEntry sb_extStage2;
    private GenericEntry sb_armClearOfClimber;

    /**
     * Constructor
     */
    private Arm() {
        
        armMotor1 = new TalonFX(Constants.armMotor1);
        armMotor2 = new TalonFX(Constants.armMotor2);

        armExtender1 = new DoubleSolenoid(Constants.phCANID, PneumaticsModuleType.REVPH, Constants.armExt1Rev, Constants.armExt1For);
        armExtender2 = new DoubleSolenoid(Constants.phCANID, PneumaticsModuleType.REVPH, Constants.armExt2Rev, Constants.armExt2For);
        
        absoluteArmEncoder = new DutyCycleEncoder(Constants.armDCEncoderPort);

        quadArmEncoder = new Encoder(Constants.armQuadEncoderAPort, Constants.armQuadEncoderBPort);
        quadArmEncoder.setDistancePerPulse(Constants.armEncAnglePerRot.getRadians() / Constants.revTBEncCountPerRev);

        armPID = new PIDController(Constants.armPID.kP, Constants.armPID.kP, Constants.armPID.kP);
        armFF = new ArmFeedforward(Constants.armFF.kS, Constants.armFF.kG, Constants.armFF.kV);
        
        // TODO Set abs encoder offset

        // Set control mode
        control_mode = ArmControlMode.MANUAL_VOLT;
        manual_volt = 0;
        manual_rate = 0;
        manual_ext = 0;

        // Set target state to current state
        targetState = new ArmStateValues(getArmAngle(), getArmExtension());

        // Initialize Timer
        extenderTimer = new Timer();


        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status")
            .getLayout("Arm", BuiltInLayouts.kList)
            .withSize(2,4);

        sb_armMode = layout.add("Arm Control Mode", control_mode.name()).getEntry();
        sb_anglePosCurrent = layout.add("Angle Position Current", 0).getEntry();
        sb_anglePosSetPoint = layout.add("Angle Position Set Point", 0).getEntry();
        sb_angleRateCurrent = layout.add("Angle Rate Current", 0).getEntry();
        sb_angleRateSetPoint = layout.add("Angle Rate Set Point", 0).getEntry();
        sb_angleM1Volt = layout.add("Angle Motor 1 Voltage", 0).getEntry();
        sb_angleM2Volt = layout.add("Angle Motor 2 Voltage", 0).getEntry();
        sb_angleTargetVolt = layout.add("Angle Target Voltage", 0).getEntry();
        sb_extStage1 = layout.add("Ext Stage 1 State", armExtender1.get().name()).getEntry();
        sb_extStage2 = layout.add("Ext State 2 State", armExtender2.get().name()).getEntry();
        sb_armClearOfClimber = layout.add("Arm clear of climber", false).getEntry();
    }

    /**
     * Gets the current arm angle
     * @return current arm angle
     */
    public Rotation2d getArmAngle() {
        double angle = Constants.armEncAngleOffset.getDegrees() - Rotation2d.fromRotations(absoluteArmEncoder.get()).getDegrees();
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
    public int getArmExtension() {
        boolean isLowerExt = armExtender1.get() == DoubleSolenoid.Value.kForward;
        boolean isUpperExt = armExtender2.get() == DoubleSolenoid.Value.kForward;
        int state = 0;

        if (isLowerExt) {
            if(isUpperExt) {
                state = 2;
            } else {
                state = 1;
            }
        }

        return state;
    }

    /** 
     * Sets the arm's output voltage to the motor. Puts the arm into manual 
     * voltage mode. If the arm is not in a manual mode already, the extension 
     * state is set to its current state.
     * @param   voltage     voltage to set to the motor
     */
    public void setArmVolt(double voltage) {
        manual_volt = voltage;

        if (control_mode == ArmControlMode.AUTOMATIC) manual_ext = getArmExtension();

        control_mode = ArmControlMode.MANUAL_VOLT;
    }

    /**
     * Sets the arm's target rate. Puts the arm into manual mode. If the arm is 
     *  not in manual mode already, the extension state is set to its current
     *  state.
     * @param   rate    new arm rate
     */
    public void setArmRate(double rate) {
        manual_rate = rate;

        if (control_mode == ArmControlMode.AUTOMATIC) manual_ext = getArmExtension();
        
        control_mode = ArmControlMode.MANUAL_RATE;
    }

    /**
     * Sets the arm's extension state. Puts the arm into manual mode. If the arm is 
     *  not in manual mode already, the arm rate is set to 0.
     * @param   state   extension state
     */
    public void setExtState(int state) {
        manual_ext = Math.max(0, Math.min(2, state));

        if (control_mode == ArmControlMode.AUTOMATIC) {
            manual_rate = 0;
            control_mode = ArmControlMode.MANUAL_RATE;
        }
    }

    /**
     * Steps the arm extension one stage out.
     */
    public void stepExtOut() {
        setExtState(getArmExtension() + 1);
    }

    /**
     * Steps the arm extension one stage out.
     */
    public void stepExtIn() {
        setExtState(getArmExtension() - 1);
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
        double voltage = getAngleControlVolt(targetArmRate);

        setMotorVolt(voltage);
        updateExtension();
        updateUI(targetArmRate, voltage);
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
            case MANUAL_RATE:
                targetSpeed = manual_rate;
                break;
            default:
                targetSpeed = 0;
                break;
        }

        // Protect against climber collisions
        if(!Climber.getInstance().isClearOfArm() && isInClimberZone()) targetSpeed = 0; // TODO Improve Collision Avoidance to eliminate deadlock possiblity

        // Keep arm in package
        if(getArmExtension() == 2 && currentAngle.getDegrees() <= minS2Angle.getDegrees()) targetSpeed = 0;

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
    private double getAngleControlVolt(double targetSpeed) {
        double result = this.manual_volt;
        if (this.control_mode != ArmControlMode.MANUAL_VOLT) {
            Rotation2d currentAngle = getArmAngle();
            double angleRate = getArmVelocity();


            // Calculate motor voltage output
            double calcPID = armPID.calculate(angleRate, targetSpeed);
            double calcFF = armFF.calculate(currentAngle.getRadians(), targetSpeed);

            result = calcPID + calcFF;
        }

        return result;
    }

    /**
     * Sets the motor voltage for the arm angle control. Manages soft limits as well.
     * @param   voltage     desired motor voltage
     */
    private void setMotorVolt(double voltage) {
        Rotation2d currentAngle = getArmAngle();

        // Set soft limits
        if(currentAngle.getDegrees() <= Constants.minArmS0Pos.getDegrees()) voltage = Math.max(0, voltage);
        if(currentAngle.getDegrees() >= Constants.maxArmPos.getDegrees()) voltage = Math.min(0, voltage);

        // Set Motors
        armMotor1.setVoltage(voltage);
        armMotor2.setVoltage(voltage);
    }

    /**
     * Updates the control of the arm extension
     */
    private void updateExtension() {
        int currentState = getArmExtension();
        int targetState = 0;

        switch(control_mode) {
            case AUTOMATIC:
                targetState = this.targetState.extState;
                break;
            default:
                targetState = manual_ext;
                break;
        }

        boolean aboveState2Angle = getArmAngle().getDegrees() > Constants.armMinState2Angle.getDegrees();

        // Set target extension valve state
        if(targetState == 2 && aboveState2Angle) {
                armExtender1.set(DoubleSolenoid.Value.kForward);
                armExtender2.set(DoubleSolenoid.Value.kForward);
        } else if (targetState == 1 || targetState == 2 && !aboveState2Angle) {
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
    private void updateUI(double targetRate, double targetVolt) {
        sb_armMode.setString(control_mode.name());
        sb_anglePosCurrent.setDouble(getArmAngle().getDegrees());
        sb_anglePosSetPoint.setDouble(targetState.targetAngle.getDegrees());
        sb_angleRateCurrent.setDouble(getArmVelocity());
        sb_angleRateSetPoint.setDouble(targetRate);
        sb_angleM1Volt.setDouble(armMotor1.getMotorVoltage().getValueAsDouble());
        sb_angleM2Volt.setDouble(armMotor2.getMotorVoltage().getValueAsDouble());
        sb_angleTargetVolt.setDouble(targetVolt);
        sb_extStage1.setString(armExtender1.get().name());
        sb_extStage2.setString(armExtender2.get().name());
        sb_armClearOfClimber.setBoolean(!isInClimberZone());
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
