package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {

    private final TalonFX mDrive;

    private final TalonFX mAngle;

    private final CANcoder encAngle;

    private final PIDController drivePIDcontroller;

    private final ProfiledPIDController angleprofiledPIDController;

    //TODO Move to constants
    private final TrapezoidProfile.Constraints AngleTrapezoidProfile;

    private final SimpleMotorFeedforward driveFeedforward;

    private final SimpleMotorFeedforward angleFeedforward;

    private SwerveModuleState desiredState;

    public Swerve(int driveMotorID, int anglemotorid, int anglemotorencid) {

        mDrive = new TalonFX(driveMotorID);
        mAngle = new TalonFX(anglemotorencid);
        encAngle = new CANcoder(anglemotorid);
        drivePIDcontroller = new PIDController(Constants.driveswerveP, Constants.driveswerveI, Constants.driveswerveD);
        AngleTrapezoidProfile = new TrapezoidProfile.Constraints(Constants.maxSwerveAngularSpeed,
                Constants.maxSwerveAngularAccel);
        angleprofiledPIDController = new ProfiledPIDController(Constants.angleswerveP, Constants.angleswerveI,
                Constants.angleswerveD,
                AngleTrapezoidProfile);
        angleFeedforward = new SimpleMotorFeedforward(Constants.angleSwerveSFF, Constants.angleSwerveVFF,
                Constants.angleSwerveAFF);
        driveFeedforward = new SimpleMotorFeedforward(Constants.driveSwerveSFF, Constants.driveSwerveVFF,
                Constants.driveSwerveAFF);
        angleprofiledPIDController.enableContinuousInput(-Math.PI, Math.PI);
        desiredState = new SwerveModuleState();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(mDrive.getVelocity().getValueAsDouble(),
                new Rotation2d(encAngle.getAbsolutePosition().getValueAsDouble()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(mDrive.getPosition().getValueAsDouble(),
                new Rotation2d(encAngle.getAbsolutePosition().getValueAsDouble()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        this.desiredState = desiredState;
    }

    @Override
    public void periodic() {
        var encoderRotation = new Rotation2d(encAngle.getAbsolutePosition().getValueAsDouble());

        // SwerveModuleState state = SwerveModuleState.optimize(desiredState,
        // encoderRotation);
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

        // Scale speed by cosine of angle error. This scales down movement perpendicular
        // to the desired
        // direction of travel that can occur when modules change directions. This
        // results in smoother
        // driving.
        // state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

        // Calculate the drive output from the drive PID controller.
        final double driveOutput = drivePIDcontroller.calculate(mDrive.getVelocity().getValueAsDouble(),
                state.speedMetersPerSecond);

        final double // driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
        outputDriveFF = driveFeedforward.calculate(state.speedMetersPerSecond);

        // Calculate the turning motor output from the turning PID controller.
        final double turnOutput = angleprofiledPIDController
                .calculate(encAngle.getAbsolutePosition().getValueAsDouble(), state.angle.getRadians());

        final double outputAngleFF = angleFeedforward.calculate(angleprofiledPIDController.getSetpoint().velocity);

        mDrive.setVoltage(driveOutput + outputDriveFF);

        mAngle.setVoltage(turnOutput + outputAngleFF);
    }

}
