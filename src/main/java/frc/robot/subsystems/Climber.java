package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.deser.impl.CreatorCandidate;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    private static Climber climber = null;
    private CANSparkMax winchL;
    private CANSparkMax winchR;
    private double winchFeedforward = 0;
    private SparkLimitSwitch limitSwitchL;
    private SparkLimitSwitch limitSwitchR;

    private Climber() {
        winchL = new CANSparkMax(Constants.winchL, MotorType.kBrushless);
        winchR = new CANSparkMax(Constants.winchR, MotorType.kBrushless);
        limitSwitchL = winchL.getForwardLimitSwitch(Type.kNormallyOpen);
        limitSwitchR = winchR.getForwardLimitSwitch(Type.kNormallyOpen);
        winchR.setInverted(true);
    }

    public void setClimberState(int winchDirection, boolean enableLimitSwitches) {
        if (winchDirection < 0 && enableLimitSwitches == false) {
            winchL.set(Constants.winchSpeedDown);
            winchR.set(Constants.winchSpeedDown);

        } else if (winchDirection > 0 && enableLimitSwitches == false) {
            winchL.set(Constants.winchSpeedUp);
            winchR.set(Constants.winchSpeedUp);

        } else if (winchDirection < 0 && enableLimitSwitches) {
            limitSwitchL.enableLimitSwitch(enableLimitSwitches);
            limitSwitchR.enableLimitSwitch(enableLimitSwitches);

            if (!limitSwitchL.isPressed() || !limitSwitchR.isPressed()) {
                winchL.set(Constants.winchSpeedDown);
                winchR.set(Constants.winchSpeedDown);
            }else if (limitSwitchL.isPressed() && limitSwitchR.isPressed()){
                winchL.set(winchFeedforward);
                winchR.set(winchFeedforward);
            }
        }else if(winchDirection == 0){
            winchL.set(0);
            winchR.set(0);
        }
    }

    public static Climber get_Climber() {
        if (climber == null) {
            climber = new Climber();
        }
        return climber;
    }

    @Override
    public void periodic(){
        System.out.println("I'm working");
        SmartDashboard.getBoolean("Left Pressed", limitSwitchL.isPressed());
        SmartDashboard.getBoolean("Right Pressed", limitSwitchR.isPressed());
        SmartDashboard.getBoolean("Left Switch Enabled", limitSwitchL.isLimitSwitchEnabled());
        SmartDashboard.getBoolean("Right Switch Enabled", limitSwitchR.isLimitSwitchEnabled());
      

    }

}
