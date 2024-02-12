package frc.robot;

import com.kauailabs.navx.frc.AHRS;

public class Constants {
    // Robot constants
    public final static double updatePeriod = 0.02;//seconds
    // Motor IDs
    public final static int armMotor1 = 1;
    public final static int armMotor2 = 2;

    public final static int frontLeftDriveM = 4;
    public final static int frontLeftAngleM = 3;
    public final static int frontLeftAngleENC = 10;
    public final static int frontRightDriveM = 6;
    public final static int frontRightAngleM = 5;
    public final static int frontRightAngleENC = 11;

    public final static int backLeftDriveM = 2;
    public final static int backLeftAngleM = 1;
    public final static int backLeftAngleENC = 9;
    public final static int backRightDriveM = 8;
    public final static int backRightAngleM = 7;
    public final static int backRightAngleENC = 12;

    // Arm PID values
    public static double kpArm1 = 0;
    public static double kiArm1 = 0;
    public static double kdArm1 = 0;

    public static double driveSwerveP = 0.1;
    public static double driveSwerveI = 0;
    public static double driveSwerveD = 0;

    public static double angleSwerveP = .095;
    public static double angleSwerveI = 0;
    public static double angleSwerveD = 0;

    public static double maxSwerveAngularSpeed = Math.PI * 5;//Rad per second
    public static double maxSwerveAngularAccel = Math.PI * 10;//Rad per second

    public static double angleSwerveSFF = 0;//.451;//volts
    public static double angleSwerveVFF = .195;//volts * seconds/radian
    public static double angleSwerveAFF = 0;

    public static double driveSwerveSFF = 0;//volts
    public static double driveSwerveVFF = 0.5;//volts * seconds/meters
    public static double driveSwerveAFF = 0;

    public static final double kMaxSpeed = 15;
    public static final double kMaxAngularSpeed = 12 * 2 * Math.PI;

    public static double xSpeed;
    public static double ySpeed;
    public static double rotationalSpeed;
    public static double periodSeconds;
    public static boolean fieldRelative;

    public static double driveGearRatio = 6.75;
    public static double wheelCirc = 4 * .0254 * Math.PI; //In Meters

    public static AHRS navx;

    //Auton
    public static double autonRampDownSpeed = 0.5;
    public static double minSpeed = 2;

}
