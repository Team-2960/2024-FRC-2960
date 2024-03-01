package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climber extends SubsystemBase {

    public enum ClimberStates {
        MatchStart,
        ClimbStart,
        Climb,
        Idle
    }

    private static Climber climber = null;

    private CANSparkMax winchL;
    private CANSparkMax winchR;


    private SparkLimitSwitch winchLimit;
    private RelativeEncoder winchEncoder;

    private DoubleSolenoid ratchetRelease;
    private Timer ratchetTimer;

    private ClimberStates climbState;

    private GenericEntry sb_state;
    private GenericEntry sb_isDown;
    private GenericEntry sb_isClearOfArm;
    private GenericEntry sb_mLVoltage;
    private GenericEntry sb_mRVoltage;

    /**
     * Constructor
     */
    private Climber() {
        // Initialize Motors
        winchL = new CANSparkMax(10,MotorType.kBrushless);
        winchR = new CANSparkMax(9,MotorType.kBrushless);
        winchR.follow(winchL, true);

        // Initialize Encoder
        winchEncoder = winchL.getEncoder();
        winchEncoder.setPositionConversionFactor(Constants.winchCircum); 

        // Initialize limit switch
        winchLimit = winchL.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        // Initialize climber state
        climbState = ClimberStates.MatchStart;

        // Setup Shuffleboard
        var layout = Shuffleboard.getTab("Status").getLayout("Climber");

        sb_state = layout.add("State", climbState.name()).getEntry();
        sb_isDown = layout.add("Is Down", false).getEntry();
        sb_isClearOfArm = layout.add("Is Clear of Arm", false).getEntry();
        sb_mLVoltage = layout.add("Left Motor Voltage", 0).getEntry();
        sb_mRVoltage = layout.add("Right Motor Voltage", 0).getEntry();
    }

    /**
     * Sets the current climber state
     * 
     * @param climbState new climber state
     */
    public void setClimbState(ClimberStates climbState) {
        this.climbState = climbState;
    }

    /**
     * Gets the current set state for the climber
     * 
     * @return current set climber state
     */
    public ClimberStates getClimbState() {
        return climbState;
    }

    /**
     * Gets the current extension distance
     * 
     * @return distance the climber is extended
     */
    public double getExtension() {
        return winchEncoder.getPosition();
    }

    /**
     * Checks if the climber is hitting the limit sensor
     * 
     * @return true if the climber is hitting the limit sensor, false otherwise.
     */
    public boolean isDown() {
        return winchLimit.isPressed();
    }

    /**
     * Resets the climber encoder
     */
    public void resetClimber() {
        winchEncoder.setPosition(0);
    }

    /**
     * Checks if the climber is in a position that is clear of the arm
     * 
     * @return true if the climber is in a position clear of the arm
     */
    public boolean isClearOfArm() {
        double armContactHeight = 15;
        return isDown() || getExtension() < armContactHeight;
    }

    /**
     * Climber periodic update
     */
    @Override
    public void periodic() {
        switch (climbState) {
            case MatchStart:
                retractClimber(.2);
                break;
            case Climb:
                retractClimber(1);
                break;
            case ClimbStart:
                extendClimber();
                break;
            case Idle:
            default:
                winchL.set(0);
                ratchetRelease.set(DoubleSolenoid.Value.kForward);
        }

        // Reset the climber encoder if the limit switch is set
        if (isDown())
            resetClimber();

        updateUI();
    }

    /**
     * Retracts the climber until the limit sensor is tripped
     */
    private void retractClimber(double winchSpeed) {
        ratchetRelease.set(DoubleSolenoid.Value.kReverse);

        if(!isDown()) {
            winchL.set(winchSpeed);
        }else {
            winchL.set(0);
            ratchetRelease.set(DoubleSolenoid.Value.kForward);
            resetClimber();
        }
    }

    /**
     * Extends the climber to the max height
     */
    private void extendClimber() {
        // Check if the arm is clear of the climber
        if(!Arm.getInstance().isInClimberZone()) {
            // Check if the climber is at its max extention
            if(getExtension() < Constants.winchMaxExtension) { 
                // Disengage ratchet
                if(ratchetRelease.get() != DoubleSolenoid.Value.kForward) ratchetTimer.restart();
                ratchetRelease.set(DoubleSolenoid.Value.kForward);
                
                // Extend the climber if the ratchet is disengaged
                if (ratchetTimer.get() > Constants.winchRatchedDelay) winchL.set(-1);

            } else {
                winchL.set(0);
                ratchetRelease.set(DoubleSolenoid.Value.kReverse);
            }
        }
    }

    /**
     * Updates Shuffleboard
     */
    private void updateUI() {
        sb_state.setString(climbState.name());
        sb_isDown.setBoolean(isDown());
        sb_isClearOfArm.setBoolean(isClearOfArm());
        sb_mLVoltage.setDouble(winchL.getBusVoltage() * winchL.getAppliedOutput());
        sb_mRVoltage.setDouble(winchR.getBusVoltage() * winchR.getAppliedOutput());
    }

    /**
     * Static initializer
     */
    public static Climber getInstance() {
        if (climber == null) {
            climber = new Climber();
        }

        return climber;
    }
}
