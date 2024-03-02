package frc.robot;

import javax.swing.text.rtf.RTFEditorKit;

import edu.wpi.first.math.geometry.*;
import frc.robot.Util.*;

public class Constants {
    public static final Transform2d fieldCenterOffset = new Transform2d(-8.270875, -4.105275, new Rotation2d(0.0));

    // Robot constants
    public final static double updatePeriod = 0.02;//seconds
    
    public final static double robotWidth = 29.5 * .0254;   // Meters 
    public final static double robotLength = 29.5 * .0254;  // Meters 
    public final static double robotDiag = Math.sqrt(Math.pow(robotWidth, 2) + Math.pow(robotLength, 2)); // Meters

    public static final double autoClearance = .25; // Meters

    public static final double driveGearRatio = 5.08;
    public static final double wheelCirc = 2.95 * .0254 * Math.PI; // Meters
    public static final double driveRatio =  Constants.wheelCirc / Constants.driveGearRatio;   // Meters

    public static final Transform3d robotToCamera = new Transform3d(
        new Translation3d(10*.0254, 0, 10 * .0254), 
        new Rotation3d(30 * Math.PI / 180, 0, Math.PI)
    );  

    
    public static final double winchDiam = 1.5; // in.
    public static final double winchCircum = Math.PI * winchDiam; // in.

    // Motor IDs
    public final static int shooterTop = 14;
    public final static int shooterBot = 13;

    public final static int intakeRollers = 15;

    public final static int winchMotorL = 10;
    public final static int winchMotorR = 9;
    
    public final static int armMotor1 =11;
    public final static int armMotor2 = 12;

    public final static int frontLeftDriveM = 3;
    public final static int frontLeftAngleM = 4;
    public final static int frontRightDriveM = 1;
    public final static int frontRightAngleM = 2;

    public final static int backLeftDriveM = 5;
    public final static int backLeftAngleM = 6;
    public final static int backRightDriveM = 7;
    public final static int backRightAngleM = 8;

    // PID & Feed Forward values
    public static PIDParam armPID = new PIDParam(0.0, 0.0, 0.0);
    public static FFParam armFF = FFParam.arm(0.0, 0.0, 0.0, 0.0);

    public static PIDParam drivePID = new PIDParam(.1, 0.0, 0.0);
    public static FFParam driveFF = FFParam.simpleMotor(0.0, 0.05, 0.0);

    public static PIDParam driveAngPID = new PIDParam(0.0, 0.0, 0.0);
    public static FFParam driveAngFF = FFParam.simpleMotor(0.0, 0.195, 0.9);

    // Robot Limits
    public static final double swerveAngleRampRate = 20;
    public static final Rotation2d swerveAngleRampDist = Rotation2d.fromDegrees(30);

    public static final double maxSwerveAngularSpeed = Math.PI * 4;     //Rad per second
    public static final double maxSwerveAngularAccel = Math.PI * 10;    //Rad per second ^ 2

    //public static final Rotation2d swerveAngleOffset = Rotation2d.fromDegrees(180);
    public static final double maxSpeed = 15;
    public static final double maxAngularSpeed = 12 * 2 * Math.PI;
    public static final Rotation2d driveAngleRampDistance = Rotation2d.fromRadians(0.7);

    public static final Rotation2d armRampDownDist = Rotation2d.fromDegrees(10);
    public static final Rotation2d armMinState2Angle = Rotation2d.fromDegrees(30);
    public static final Rotation2d climberZoneLowerAngle =  Rotation2d.fromDegrees(60); 
    public static final Rotation2d climberZoneUpperAngle =  Rotation2d.fromDegrees(75);
    public static final double armExtDelayTime = .25; // Seconds

    public static final double winchMaxExtension = 20;   // in.
    public static final double winchRatchedDelay = .25;  // seconds

    public static final double minShootSpeed = 500;     // rev / s
    public static final double shootPrepPower = .2;     

    //Auton
    public static double autonRampDownSpeed = 0.5;
    public static double minSpeed = 2;

    //Camera
    public static double cameraToRobotX;
    public static double cameraToRobotY;
    public static double cameraToRobotHeight;

    public static double cameraRoll;
    public static double cameraPitch;
    public static double cameraYaw;

    //Arm
    public static final double maxArmPos = 0;
    public static final double minArmPos = 0;
    public static double maxArmSpeed;
    public static double maxArmAccel;

    public static final double homePos = 0;
    public static final double extendedHomePos = 0;
    public static final double speakerPos = 0;
    public static final double ampPos = 0;
    public static final double climbPrepPos = 0;
    public static final double trapClimbPos = 0;
    public static final double trapScorePos = 0;

    //TODO test and change these values

}
