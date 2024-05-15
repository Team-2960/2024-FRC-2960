package frc.robot;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.ArmFeedforward;
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
        new Rotation3d(36 * Math.PI / 180, 0, Math.PI)
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

    //Preset Auton Positions
    public static final Pose2d redSourceSide = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d redCenter = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d redAmpSide = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d blueSourceSide = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d blueCenter = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d blueAmpSide = new Pose2d(0, 0, Rotation2d.fromDegrees(0));



    // Drive
    public static PIDParam drivePID = new PIDParam(.5, 0.0, 0.0);
    public static PIDConstants drivePIDConstants = new PIDConstants(.5, 0.0, 0.0);
    public static FFParam driveFF = FFParam.simpleMotor(0.0, 2.25, 0.0);

    public static PIDParam driveAngPID = new PIDParam(0.05, 0.0, 0.001);
    public static PIDConstants driveAngPIDConstants = new PIDConstants(0.05, 0.0, 0.001);
    public static FFParam driveAngFF = FFParam.simpleMotor(0.1, 0.1, 0);

    public static final double maxSpeed = 4.5;
    public static final double maxAngularSpeed = 1.5 * 2 * Math.PI;
    public static final double maxAutoAngularSpeed = 0.5 * 2 * Math.PI;

    public static final double swerveAngleRampRate = 7; 
    public static final Rotation2d swerveAngleRampDist = Rotation2d.fromDegrees(30);

    public static final double maxSwerveAngularSpeed = Math.PI * 4;     //Rad per second
    public static final double maxSwerveAngularAccel = Math.PI * 10;    //Rad per second ^ 2

    public static final Rotation2d driveAngleRampDistance = Rotation2d.fromDegrees(20);
    public static final PIDParam angleAlignPID = new PIDParam(4, 0, 0.3);

    // Arm
    public static PIDParam armPIDS0 = new PIDParam(0.01, 0.0, 0.0);
    public static FFParam armFFS0 = FFParam.arm(.1, 2, 0.25, 0.0);

    public static PIDParam armPIDS1 = new PIDParam(0.01, 0.0, 0.0);
    public static FFParam armFFS1 = FFParam.arm(.1, 2, 0.25, 0.0);
    
    public static PIDParam armPIDS2 = new PIDParam(0.01, 0.0, 0.0);
    public static FFParam armFFS2 = FFParam.arm(.1, 2, 0.25, 0.0);

    public static final Rotation2d minArmS0Pos = Rotation2d.fromDegrees(20 + 16);
    public static final Rotation2d minArmS0Angle = Rotation2d.fromDegrees(2 + 16);
    public static final Rotation2d minArmS2Angle = Rotation2d.fromDegrees(46 + 16);
    public static final Rotation2d maxArmS2Angle = Rotation2d.fromDegrees(96.5 + 16);
    public static final Rotation2d minArmIntakePos = Rotation2d.fromDegrees(2 + 16);
    public static final Rotation2d maxArmPos = Rotation2d.fromDegrees(96.5 + 16);
    public static final Rotation2d minArm2dAngle = Rotation2d.fromDegrees(46 + 16);
    public static final Rotation2d maxArm2dAngle = Rotation2d.fromDegrees(77 + 16);

    public static final Rotation2d armMinState2Angle = Rotation2d.fromDegrees(30 + 16);

    public static final Rotation2d armRampDownDist = Rotation2d.fromDegrees(20);

    public static final Rotation2d climberZoneLowerAngle =  Rotation2d.fromDegrees(46); 
    public static final Rotation2d climberZoneUpperAngle =  Rotation2d.fromDegrees(70);

    public static final double armExtDelayTime = .25;   // Second
    public static final double maxArmSpeed = Math.PI;   // radians / s
    public static final double maxArmAutoSpeed = 1 * Math.PI;  //radians /s

    public static final Rotation2d armEncAnglePerRot = Rotation2d.fromDegrees(360);
    public static final Rotation2d armEncAngleOffset = Rotation2d.fromDegrees(168.5);

    public static final double armOffset = 0;

    public static final double armAlignAngleOffset = 0;

    public static final double armLength = 0.4953;

    public static final double armHeightOffset = 0.26;

    // STAGE1 SOFT LIMIT RANGE 46 - 78.1
    public static final double lowerEncLimit = .449;
    public static final double upperEncLimit = .184;
    public static final double LowerEncLimitS0 = .42 - 16/360;
    public static final double lowerEncLimitS2 = .2;

    // Pizzabox
    public static final double intakeInVoltage = 8.3;
    public static final double intakeShootVoltage = 8.3;
    public static final double intakeOutVoltage = 8.3;
    public static final double intakeSlowVoltage = 4;
    public static final double intakeSlowCurrent = 20;

    public static final double shooterPrepPower = .75;    
    public static final double shooterShootVoltage = 10.8;
    public static final double shooterRevVoltage = 10.8;
    public static final double shooterMinShootSpeed = 4000 ;     // rpm
    public static final double shooterFastShootSpeed = 5500;//rpm

    // Climber
    public static final double winchMaxExtension = 88;   // in.
    public static final double winchMinLimit = 1.5; //in
    public static final double 
    winchRatchedDelay = .25;  // seconds

    // Pneumatics
    public static final double minPressure = 100;
    public static final double maxPressure = 120;
}
