package frc.robot;

import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drive;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OperatorInterface extends SubsystemBase {
    // INSTANCE
    public static OperatorInterface oi = null;

    // JOYSTICKS
    private static Joystick driverController;
    private static Joystick operatorController;

    // Create the OI if doesn't exist
    public static OperatorInterface get_OperatorInterface() {
        if (oi == null) {
            oi = new OperatorInterface();
        }
        return oi;
    }

    private OperatorInterface() {
        // Initiating Subystems

        // Create Joysticks
        driverController = new Joystick(0);
        operatorController = new Joystick(1);

    }

    @Override
    public void periodic() {

        if (DriverStation.isTeleop()) {
            boolean fieldRelative = !driverController.getRawButton(1);
            

            double xSpeed = MathUtil.applyDeadband(driverController.getRawAxis(0), 0.1) * Constants.kMaxSpeed;
            double ySpeed = -MathUtil.applyDeadband(driverController.getRawAxis(1), 0.1) * Constants.kMaxSpeed;
            double rSpeed =  MathUtil.applyDeadband(driverController.getRawAxis(4), 0.1) * Constants.kMaxAngularSpeed;

            

            Drive.get_instance().set_speed(xSpeed, ySpeed, rSpeed, fieldRelative);
            SmartDashboard.putNumber("ySpeed", ySpeed);
            SmartDashboard.putNumber("xSpeed", xSpeed);
            SmartDashboard.putNumber("rSpeed", rSpeed);
            SmartDashboard.putBoolean("FieldRelative", fieldRelative);

          /*   if (driverController.getRawAxis(5)<0.2 && driverController.getRawAxis(5)>-0.2){
                rSpeed = 0;
            }
            */
        }

        // Arm Control
      



    }

    public static OperatorInterface get_instance() {
        if (oi == null) {
            oi = new OperatorInterface();
        }

        return oi;
    }
}
