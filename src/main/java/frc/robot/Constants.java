package frc.robot;

import edu.wpi.first.units.*;
import edu.wpi.first.math.geometry.*;
import frc.robot.Util.*;

public class Constants {
    public static final Transform2d fieldCenterOffset = new Transform2d(8.270875, 4.105275, new Rotation2d(0.0));

    // Robot constants
    // TODO Convert constants to units library for clarity
    public static final double updatePeriod = 0.02;//seconds
    
    public static final double robotWidth = 29.5 * .0254;   // Meters 
    public static final double robotLength = 29.5 * .0254;  // Meters 
    public static final double wheelInset = 1.75 * .0254;   // Meters
    public static final double robotDiag = Math.sqrt(Math.pow(robotWidth, 2) + Math.pow(robotLength, 2)); // Meters

    public static final double autoClearance = .25; // Meters

    public static final double driveGearRatio = 5.08;
    public static final double wheelCirc = 2.95 * .0254 * Math.PI; // Meters
    public static final double driveRatio =  Constants.wheelCirc / Constants.driveGearRatio;   // Meters

    public static final Transform3d robotToCamera = new Transform3d(
        new Translation3d(-robotLength/2+.040, 0, .206), 
        new Rotation3d(32 * Math.PI / 180, 0, Math.PI)
    );  

    public static final double winchDiam = 1.5; // in.
    public static final double winchCircum = Math.PI * winchDiam * (15/36); // in.

    public static final int revTBEncCountPerRev = 4096;

    // CAN IDs
    public static final int shooterTop = 14;
    public static final int shooterBot = 13;

    public static final int intakeRollers = 15;

    public static final int winchMotorL = 10;
    public static final int winchMotorR = 9;
    
    public static final int armMotor1 =11;
    public static final int armMotor2 = 12;

    public static final int frontLeftDriveM = 3;
    public static final int frontLeftAngleM = 4;
    public static final int frontRightDriveM = 1;
    public static final int frontRightAngleM = 2;

    public static final int backLeftDriveM = 5;
    public static final int backLeftAngleM = 6;
    public static final int backRightDriveM = 7;
    public static final int backRightAngleM = 8;

    public static final int phCANID = 20;

    // Digital Input Ports
    public static final int armDCEncoderPort = 0;
    public static final int armQuadEncoderAPort = 1;
    public static final int armQuadEncoderBPort = 2;
    public static final int pbPhotoeyePort = 3;
    public static final int armBrakeModeBtn = 4;

    // PH Solenoid Port
    public static final int armExt1Rev = 8;
    public static final int armExt1For = 9;
    public static final int armExt2Rev = 7;
    public static final int armExt2For = 6;
    public static final int climbRatchetRev = 4;
    public static final int climbRatchetFor = 5;

    // Auton
    public static double autonRampDownSpeed = 0.5;  
    public static double minSpeed = 2;              // m/s


    // Drive
    public static PIDParam drivePID = new PIDParam(.1, 0.0, 0.0);
    public static FFParam driveFF = FFParam.simpleMotor(0.0, 0.75, 0.0);

    public static PIDParam driveAngPID = new PIDParam(0.05, 0.0, 0.001);
    public static FFParam driveAngFF = FFParam.simpleMotor(0.1, 0.1, 0);

    public static final double maxSpeed = 15;
    public static final double maxAngularSpeed = 12 * 2 * Math.PI;

    public static final double swerveAngleRampRate = 7; 
    public static final Rotation2d swerveAngleRampDist = Rotation2d.fromDegrees(30);

    public static final double maxSwerveAngularSpeed = Math.PI * 4;     //Rad per second
    public static final double maxSwerveAngularAccel = Math.PI * 10;    //Rad per second ^ 2

    public static final Rotation2d driveAngleRampDistance = Rotation2d.fromRadians(0.7);

    // Arm
    public static PIDParam armPID = new PIDParam(0.0, 0.0, 0.0);
    public static FFParam armFF = FFParam.arm(.1, 3.24, 0.28, 0.03);

    public static final Rotation2d minArmS0Pos = Rotation2d.fromDegrees(20);
    public static final Rotation2d minArmIntakePos = Rotation2d.fromDegrees(2);
    public static final Rotation2d maxArmPos = Rotation2d.fromDegrees(96.5);

    public static final Rotation2d armMinState2Angle = Rotation2d.fromDegrees(30);

    public static final Rotation2d armRampDownDist = Rotation2d.fromDegrees(15);

    public static final Rotation2d climberZoneLowerAngle =  Rotation2d.fromDegrees(46); 
    public static final Rotation2d climberZoneUpperAngle =  Rotation2d.fromDegrees(70);

    public static final double armExtDelayTime = .25;   // Second
    public static final double maxArmSpeed = Math.PI * 0.5;   // radians / s

    public static final Rotation2d armEncAnglePerRot = Rotation2d.fromDegrees(360);
    public static final Rotation2d armEncAngleOffset = Rotation2d.fromDegrees(168.5);

    public static final double lowerEncLimit = .455;
    public static final double upperEncLimit = .19;
    public static final double LowerEncLimitS0 = .42;

    // Pizzabox
    public static final double intakeInPower = .75;
    public static final double intakeShootPower = 1;
    public static final double intakeOutPower = 1;

    public static final double shooterPrepPower = .2;    
    public static final double shooterShootPower = 1;
    public static final double shooterRevPower = 1;
    public static final double shooterMinShootSpeed = 3000 ;     // rpm

    // Climber
    public static final double winchMaxExtension = 425;   // in.
    public static final double winchRatchedDelay = .25;  // seconds

    // Pneumatics
    public static final double minPressure = 0; //TODO change back to 80
    public static final double maxPressure = 120;
}
