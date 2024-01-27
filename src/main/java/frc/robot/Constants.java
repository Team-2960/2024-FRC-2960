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

    public static double angleSwerveP = .5;
    public static double angleSwerveI = 0;
    public static double angleSwerveD = 0;

    public static double maxSwerveAngularSpeed = 3;//Rad per second
    public static double maxSwerveAngularAccel = Math.PI * 6;//Rad per second

    public static double angleSwerveSFF = .451;//volts
    public static double angleSwerveVFF = .385;//volts * seconds/radian
    public static double angleSwerveAFF = 0;

    public static double driveSwerveSFF = 0;//volts
    public static double driveSwerveVFF = 1.193;//volts * seconds/meters
    public static double driveSwerveAFF = 0;

    public static final double kMaxSpeed = 5;
    public static final double kMaxAngularSpeed = Math.PI;

    public static double xSpeed;
    public static double ySpeed;
    public static double rotationalSpeed;
    public static double periodSeconds;
    public static boolean fieldRelative;

    public static double driveGearRatio = 6.75;
    public static double wheelCirc = 0.319185544; //In Meters

    public static AHRS navx;

}
