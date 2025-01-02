package frc.robot;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.subsystems.ArmSettings;
import frc.robot.subsystems.ClimberSettings;
import frc.robot.subsystems.DriveSettings;
import frc.robot.subsystems.IntakePizzaboxSettings;
import frc.robot.subsystems.SwerveSettings;

import frc.lib2960.controllers.*;
import frc.lib2960.photonvision.*;
import frc.lib2960.subsystems.*;
import frc.lib2960.util.*;
import frc.lib2960_ctre.*;

public class Constants {
    public static final Transform2d fieldCenterOffset = new Transform2d(8.270875, 4.105275, new Rotation2d(0.0));

    
    /****************************/
    /* Robot Constants Settings */
    /****************************/
    // TODO Convert constants to units library for clarity
    public static final double updatePeriod = 0.02;//seconds
    
    public static final double robotWidth = 29.5 * .0254;   // Meters 
    public static final double robotLength = 29.5 * .0254;  // Meters 
    public static final double wheelInset = 1.75 * .0254;   // Meters
    public static final double robotDiag = Math.sqrt(Math.pow(robotWidth, 2) + Math.pow(robotLength, 2)); // Meters

    public static final double autoClearance = .25; // Meters

    public static final int revTBEncCountPerRev = 4096;

    /*******************/
    /* CAN ID Settings */
    /*******************/
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


    /***********************/
    /* Digital Input Ports */
    /***********************/
    public static final int armDCEncoderPort = 0;
    public static final int armQuadEncoderAPort = 1;
    public static final int armQuadEncoderBPort = 2;
    public static final int pbPhotoeyePort = 3;
    public static final int armBrakeModeBtn = 4;

    
    /*********************/
    /* PH Solenoid Ports */
    /*********************/
    public static final int armExt1Rev = 8;
    public static final int armExt1For = 9;
    public static final int armExt2Rev = 7;
    public static final int armExt2For = 6;
    public static final int climbRatchetRev = 4;
    public static final int climbRatchetFor = 5;


    /******************/
    /* Auton Settings */
    /******************/
    public static double autonRampDownSpeed = 0.5;  
    public static double minSpeed = 2;                  // m/s

    //Preset Auton Positions
    public static final Pose2d redSourceSide = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d redCenter = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d redAmpSide = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d blueSourceSide = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d blueCenter = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    public static final Pose2d blueAmpSide = new Pose2d(0, 0, Rotation2d.fromDegrees(0));


    /***********************/
    /* Drivetrain Settings */
    /***********************/
    // Drivetrain Constants
    public static final double driveGearRatio = 5.08;
    public static final double wheelDiam = 2.95 * .0254;                    // Meters      
    public static final double wheelRadius = wheelDiam / 2;                 // Meters      
    public static final double wheelCirc = wheelDiam  * Math.PI;            // Meters
    public static final double driveRatio =  wheelCirc / driveGearRatio;    // Meters 

    // Calculate swerve drive module offset from center of robot
    public static final double swerve_x_offset = (robotLength / 2 - wheelInset);    // Meters
    public static final double swerve_y_offset = (robotWidth / 2 - wheelInset);     // Meters

    // Swerve Base Settings
    public static final SwerveDriveBaseSettings drive_base_settings = new SwerveDriveBaseSettings(
        4.5,            // Max linear speed (meter/s)
        1.5 * 360,      // Max angle rate (degrees/s)
        1.5 * 360,      // Max angle tracking acceleration (degrees/s^2)
        1.5 * 360       // Max angle tracking deceleration (degrees/s^2)
    );
    

    /**************************/
    /* Swerve Module Settings */
    /**************************/
    // Swerve Module Angle Position Control Settings
    public static final PositionControllerSettings angle_pos_settings = new PositionControllerSettings(
        360 * 5,    // Maximum Acceleration (degrees/s^2)
        360 * 5,    // Maximum Deceleration (degrees/s^2)
        360 * 2,    // Maximum Rate (degrees/s)
        true,
        new Limits(0, 360)
    );

    // Swerve Module Angle Rate Control Settings
    public static final RateControllerSettings angle_rate_settings = new RateControllerSettings(
        FFParam.simpleMotor(0.1, 0.1, 0),
        new PIDParam(0.05, 0.0, 0.001)
    );

    // Swerve Module Drive Rate Control Settings
    public static final RateControllerSettings drive_rate_settings = new RateControllerSettings(
        FFParam.simpleMotor(0.0, 2.25, 0.0),
        new PIDParam(.5, 0.0, 0.0)
    );

    // Front Left Swerve Module Settings
    public static final SwerveSettings fl_swerve_settings = new SwerveSettings(
        "Front Left",
        new Translation2d(swerve_x_offset, swerve_y_offset),
        driveRatio,             
        wheelRadius,
        angle_pos_settings,
        angle_rate_settings,
        drive_rate_settings,
        new MotorSettings("Front Left Angle", frontLeftAngleM, true),
        new MotorSettings("Front Left Drive", frontLeftDriveM, true),
        true
    );

    // Front Right Swerve Module Settings
    public static final SwerveSettings fr_swerve_settings = new SwerveSettings(
        "Front Right",
        new Translation2d(swerve_x_offset, swerve_y_offset),
        driveRatio,             
        wheelRadius,
        angle_pos_settings,
        angle_rate_settings,
        drive_rate_settings,
        new MotorSettings("Front Right Angle", frontRightAngleM, true),
        new MotorSettings("Front Right Drive", frontRightDriveM, false),
        true
    );

    // Rear Left Swerve Module Settings
    public static final SwerveSettings rl_swerve_settings = new SwerveSettings(
        "Rear Left",
        new Translation2d(swerve_x_offset, swerve_y_offset),
        driveRatio,             
        wheelRadius,
        angle_pos_settings,
        angle_rate_settings,
        drive_rate_settings,
        new MotorSettings("Rear Left Angle", backLeftAngleM, true),
        new MotorSettings("Rear Left Drive", backLeftDriveM, true),
        true
    );

    // Rear Right Swerve Module Settings
    public static final SwerveSettings rr_swerve_settings = new SwerveSettings(
        "Rear Right",
        new Translation2d(swerve_x_offset, swerve_y_offset),
        driveRatio,             
        wheelRadius,
        angle_pos_settings,
        angle_rate_settings,
        drive_rate_settings,
        new MotorSettings("Front Right Angle", backRightAngleM, true),
        new MotorSettings("Front Right Drive", backRightDriveM, false),
        true
    );

    //Drive Settings
    public static final DriveSettings drive_settings = new DriveSettings(
        drive_base_settings, 
        new SwerveSettings[] {
            fl_swerve_settings, 
            fr_swerve_settings, 
            rl_swerve_settings, 
            rr_swerve_settings
        }
    );

    /****************/
    /* Arm Settings */
    /****************/
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
    public static final double lowerEncLimitS0 = .42 - 16/360;
    public static final double lowerEncLimitS2 = .2;

    // Shoulder Joint Settings
    public static final double shoulder_max_rate = 0.5 * 360;       // degrees/s
    public static final double shoulder_max_accel = 2.25 * 360;     // degrees/s^2

    public static final FFParam arm_ff_param = FFParam.arm(0.1, 2.0, 0.25, 0.0);
    public static final PIDParam arm_pid_param = new PIDParam(0.01, 0.0, 0.0);

    public static final ArmSettings arm_settings = new ArmSettings (
        new MotorMechTalonFXSettings(
            "Shoulder Joint", 
            "Arm", 
            new PositionControllerSettings(shoulder_max_accel, shoulder_max_accel, 
                                        shoulder_max_rate, true, new Limits(0,360) 
            ), 
            new MotorMechStageSettings[] {
                new MotorMechStageSettings(
                    new RateControllerSettings(arm_ff_param, arm_pid_param), 
                    new Limits(lowerEncLimitS0, upperEncLimit)
                ),
                new MotorMechStageSettings(
                    new RateControllerSettings(arm_ff_param, arm_pid_param), 
                    new Limits(lowerEncLimit, upperEncLimit)
                ),
                new MotorMechStageSettings(
                    new RateControllerSettings(arm_ff_param, arm_pid_param), 
                    new Limits(lowerEncLimitS2, upperEncLimit)
                )
            }, 
            new Limits(-1, 1), 
            new MotorSettings[] {
                new MotorSettings("Arm Motor 1", armMotor1, false),
                new MotorSettings("Arm Motor 2", armMotor2, false),
            }, 
            new QuadEncoderSettings("Arm Quad Encoder", armQuadEncoderAPort, armQuadEncoderBPort, false, armEncAnglePerRot.getDegrees() / revTBEncCountPerRev), 
            new AbsEncoderSettings("Arm Abs Encoder", armDCEncoderPort, false, armEncAngleOffset.getDegrees())
        ),
        new DoubleSolinoidSettings(
            "Arm Extension 1",
            Constants.phCANID, 
            PneumaticsModuleType.REVPH, 
            Constants.armExt1Rev,
            Constants.armExt1For
        ),
        new DoubleSolinoidSettings(
            "Arm Extension 2",
            Constants.phCANID, 
            PneumaticsModuleType.REVPH, 
            Constants.armExt2Rev,
            Constants.armExt2For
        ),
        armBrakeModeBtn
    );

    // Climber Zone Limit
    public static final Limits climber_zone = new Limits(
        46,         // Climber Zone Lower Limit (Degrees)
        70          // Climber zone upper Limit (Degrees)
    );

    /*********************/
    /* Pizzabox Settings */
    /*********************/
    public static final double intakeInVoltage = 8.3;
    public static final double intakeShootVoltage = 8.3;
    public static final double intakeOutVoltage = 8.3;
    public static final double intakeSlowVoltage = 4;
    public static final double intakeSlowCurrent = 20;

    
    public static final double shooterShootVoltage = 10.8;
    public static final double shooterPrepVoltage = .75 * shooterShootVoltage;    
    public static final double shooterRevVoltage = 10.8;
    public static final double shooterMinShootSpeed = 4000 ;     // rpm
    public static final double shooterFastShootSpeed = 5500;//rpm

    public static final IntakePizzaboxSettings pizzabox_settings = new IntakePizzaboxSettings(
        new MotorSettings("Intake", intakeRollers, false), 
        new MotorSettings[] {
            new MotorSettings("Shooter Top", shooterTop, true),
            new MotorSettings("Shooter Bottom", shooterBot, false)
        }, 
        3, 
        5,
        shooterMinShootSpeed,
        shooterFastShootSpeed,
        intakeSlowVoltage,
        intakeInVoltage,
        intakeOutVoltage,
        shooterPrepVoltage,
        shooterShootVoltage,
        shooterRevVoltage
    );
    
    /********************/
    /* Climber Settings */
    /********************/
    // Winch Constants
    public static final double winchDiam = 1.5; // in.
    public static final double winchCircum = Math.PI * winchDiam * (15/36); // in.

    // Winch Settings
    public static final double winchMaxExtension = 88;   // in.
    public static final double winchMinLimit = 1.5; //in
    public static final double winchRatchedDelay = .25;  // seconds
    
    public static final ClimberSettings climber_settings = new ClimberSettings(
        new MotorSettings[] {
            new MotorSettings("Left Winch", winchMotorL, false),
            new MotorSettings("Right Winch", winchMotorR, true)
        }, 
        new DoubleSolinoidSettings(
            "Ratchet Valve", 
            phCANID, 
            PneumaticsModuleType.REVPH, 
            climbRatchetRev,
            climbRatchetFor
        ),
        winchCircum,
        0, 
        1, 
        true,
        Constants.winchMaxExtension,
        winchRatchedDelay
    );
    
    /*******************/
    /* Vision Settings */
    /*******************/
    public static final Transform3d robotToCamera = new Transform3d(
        new Translation3d(-robotLength/2+.040, 0, .206), 
        new Rotation3d(36 * Math.PI / 180, 0, Math.PI)
    ); 

    public static final AprilTagPipelineSettings vision_settings = new AprilTagPipelineSettings(
        "Camera_Module_v1",         // Camera Name
        Constants.robotToCamera     // Robot to Camera Transform
    );

    /***********************/
    /* Pneumatics Settings */
    /***********************/
    public static final double minPressure = 100;
    public static final double maxPressure = 120;
}
