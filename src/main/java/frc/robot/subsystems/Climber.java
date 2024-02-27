package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;

public class Climber extends SubsystemBase{
    
    public enum ClimberStates {
        MatchStart,
        ClimbStart,
        Climb
    }

    private static Climber climber = null;

    private CANSparkMax winchL;
    private CANSparkMax winchR;

    private RelativeEncoder winchEncodre;

    private SparkLimitSwitch winchLimit;

    private DoubleSolenoid ratchedRelease;
    private Timer ratchetTimer;

    private ClimberState climbState;


    private Climber() {
        winchL = CANSparkMax(/* TODO Set Left Winch ID */);
        winchR = CANSparkMax(/* TODO Set Left Winch ID */);
        winchR.follow(winchL, true);

        winchEncodre = winchL.getEncoder();
        winchEncodre.setPositionConversionFactor(Math.Pi * 1.5); // TODO Move circumfrance to Constants

        winchLimit = winchL.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);

        climbState = MatchStart;
    }

    public void setClimbState(ClimberState climbState) {

    }

    public ClimberState getClimbState() {
        return climbState;
    }

    public double getExtension() {
        return winchEncodre.getPosition();
    }

    public boolean isDown() {
        return winchLimit.isPressed();
    }

    public void resetClimber() {
        winchEncoder.setPosition(0);
    }

    public boolean isClearOfArm() {
        double armContactHeight = 15;
        return isDown() || getExtension() < armContactHeight;
    }

    @Override
    public void periodic() {
        switch(climbState) {
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
        
    }

    private void retractClimber(double winchSpeed) {
        ratchedRelease.set(DoubleSolenoid.kReverse);

        if(!isDown()) {
            winchL.set(winchSpeed);
        }else {
            winchL.set(0);
            resetClimber()
        }
    }

    private void extendClimber() {
        boolean armClear = !arm.getInstance().isInClimberZone();

        if(armClear) {
            if(getExtension() < 20) { // TODO Determine maximum extension and move to constants 
                // Disengage ratchet
                if(ratchedRelease.get() != DoubleSolenoid.kForward) ratchetTimer.restart();
                ratchedRelease.set(DoubleSolenoid.kForward);

                if (ratchetTimer.get() > .25) { // TODO Move ratchet delay time to constants
                    
                    winchL.set(-1);
                }

            } else {
                winchL.set(0)
                ratchedRelease.set(DoubleSolenoid.kReverse);
            }

        }
    }
    

    public static Climber getInstance() {
        if (climber == null) {
            climber = new Climber();
        }

        return climber;
    }
}

