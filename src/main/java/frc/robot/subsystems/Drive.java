package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.AnalogGyro;

public class Drive extends SubsystemBase {
    private static Drive drive = null; // Statically initialized instance
    private final Translation2d frontLeftLocation = new Translation2d(0, 0);
    private final Translation2d frontRightLocation = new Translation2d(0, 0);
    private final Translation2d backLeftLocation = new Translation2d(0, 0);
    private final Translation2d backRightLocation = new Translation2d(0, 0);

    private final Swerve frontLeft = new Swerve(1, 2, 3);
    private final Swerve frontRight = new Swerve(1, 2, 3);
    private final Swerve backLeft = new Swerve(1, 2, 3);
    private final Swerve backRight = new Swerve(1, 2, 3);



    private Drive() {

    }

    /**
     * Static Initializer
     * 
     * @return Common object of Drive
     */
    public static Drive get_drive() {
        if (drive == null) {
            drive = new Drive();
        }

        return drive;
    }
}
