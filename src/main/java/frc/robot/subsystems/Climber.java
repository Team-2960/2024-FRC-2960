package frc.robot.subsystems;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Climber extends SubsystemBase {

    public enum ClimberStates {
        MatchStart,
        ClimbStart,
        Climb
    }

    private static Climber climber = null;

    private CANSparkMax winchL;
    private CANSparkMax winchR;


    private SparkLimitSwitch winchLimit;
    private RelativeEncoder winchEncoder;

    private DoubleSolenoid ratchetRelease;
    private Timer ratchetTimer;

    private ClimberStates climbState;

    /**
     * Constructor
     */
    private Climber() {
        winchL = new CANSparkMax(10,MotorType.kBrushless);
        winchR = new CANSparkMax(9,MotorType.kBrushless);
        winchR.follow(winchL, true);

        winchEncoder = winchL.getEncoder();
        winchEncoder.setPositionConversionFactor(Math.PI * 1.5); // TODO Move circumfrance to Constants

        winchLimit = winchL.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        climbState = ClimberStates.MatchStart;
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
        }

        // Reset the climber encoder if the limit switch is set
        if (isDown())
            resetClimber();
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
            resetClimber();
        }
    }

    /**
     * Extends the climber to the max height
     */
    private void extendClimber() {
        double maxExtension = 20;   // TODO Determine maximum extension and move to constants 
        double ratchedDelay = .25;  // TODO Move ratchet delay time to constants

        // Check if the arm is clear of the climber
        if(!Arm.getInstance().isInClimberZone()) {
            // Check if the climber is at its max extention
            if(getExtension() < maxExtension) { 
                // Disengage ratchet
                if(ratchetRelease.get() != DoubleSolenoid.Value.kForward) ratchetTimer.restart();
                ratchetRelease.set(DoubleSolenoid.Value.kForward);
                
                // Extend the climber if the ratchet is disengaged
                if (ratchetTimer.get() > ratchedDelay) winchL.set(-1);

            } else {
                winchL.set(0);
                ratchetRelease.set(DoubleSolenoid.Value.kReverse);
            }
        }
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
