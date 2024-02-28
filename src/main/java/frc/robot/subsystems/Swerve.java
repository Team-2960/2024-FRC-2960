package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.AnnotationIntrospector.ReferenceProperty.Type;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {

        private final TalonFX mDrive;

        private final CANSparkMax mAngle;

        private final SparkAbsoluteEncoder encAngle;

        private final PIDController drivePIDcontroller;

        private final ProfiledPIDController angleProfiledPIDController;
        
        private final PIDController AnglePIDController;

        // TODO Move to constants
        private final TrapezoidProfile.Constraints AngleTrapezoidProfile;

        private final SimpleMotorFeedforward driveFeedforward;

        private final SimpleMotorFeedforward angleFeedforward;

        private SwerveModuleState desiredState;

        private String swerveName;

        public Swerve(int driveMotorID, int angleMotorID, int angleMotorEncID, String swerveName) {

                mDrive = new TalonFX(driveMotorID);
                mAngle = new CANSparkMax(angleMotorID, MotorType.kBrushless);
                encAngle = mAngle.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

                this.swerveName = swerveName;

                
                //Drive control
                drivePIDcontroller = new PIDController(Constants.driveSwerveP, Constants.driveSwerveI,
                                Constants.driveSwerveD);
                driveFeedforward = new SimpleMotorFeedforward(Constants.driveSwerveSFF, Constants.driveSwerveVFF,
                                Constants.driveSwerveAFF);

                //Angle control
                AngleTrapezoidProfile = new TrapezoidProfile.Constraints(Constants.maxSwerveAngularSpeed,
                                Constants.maxSwerveAngularAccel);
                angleProfiledPIDController = new ProfiledPIDController(Constants.angleSwerveP, Constants.angleSwerveI,
                                Constants.angleSwerveD,
                                AngleTrapezoidProfile);

                AnglePIDController = new PIDController(Constants.angleSwerveP, Constants.angleSwerveI, Constants.angleSwerveD);
                angleFeedforward = new SimpleMotorFeedforward(Constants.angleSwerveSFF, Constants.angleSwerveVFF,
                                Constants.angleSwerveAFF);

                //Configure anglePID
                angleProfiledPIDController.enableContinuousInput(-Math.PI, Math.PI);

                
                //Initialize desired state
                desiredState = new SwerveModuleState();
        }

         public Rotation2d getCurrentAngle(){
                return
                new Rotation2d(encAngle.getPosition() * (2 * Math.PI));
        }
        public double getCurrentVelocity(){
                return
                (mDrive.getVelocity().getValueAsDouble()/Constants.driveGearRatio)
                        * Constants.wheelCirc;
        }
        public double getCurrentDrivePos(){
                return
                (mDrive.getPosition().getValueAsDouble()/Constants.driveGearRatio)
                        * Constants.wheelCirc;
        }
        public SwerveModulePosition getPosition() {
                return new SwerveModulePosition(getCurrentDrivePos(),
                                getCurrentAngle());
        }
        public SwerveModuleState getState() {
                return new SwerveModuleState(getCurrentVelocity(),
                                getCurrentAngle());
        }

       

        public void setDesiredState(SwerveModuleState desiredState) {
                this.desiredState = desiredState;
        }

       

        @Override
        public void periodic() {
                var encoderRotation = getCurrentAngle();

                //Optimize desired state
                SwerveModuleState state = SwerveModuleState.optimize(desiredState, encoderRotation);

                // Scale speed by cosine of angle error. This scales down movement perpendicular
                // to the desired
                // direction of travel that can occur when modules change directions. This
                // results in smoother
                // driving.
                // state.speedMetersPerSecond *= state.angle.minus(encoderRotation).getCos();

                // Calculate the drive output from the drive PID controller.
                double curMetersPerSecond = getCurrentVelocity();
                 double driveOutput = drivePIDcontroller.calculate(curMetersPerSecond,
                                state.speedMetersPerSecond);

                final double outputDriveFF = driveFeedforward.calculate(state.speedMetersPerSecond);

                // Calculate the turning motor output from the turning PID controller.
                double turnOutput = angleProfiledPIDController
                                .calculate(encoderRotation.getRadians(), state.angle.getRadians());
                
                double error = encoderRotation.getRadians() - state.angle.getRadians();
                double compError = 2*Math.PI - (Math.abs(error));
                double compareError = Math.min(Math.abs(error), compError);
                double direction;
                double rampRate = 20;
                
                if(error < 0){
                        direction = -1;
                }else{
                        direction = 1;
                }

                if(compareError == compError){
                        direction *= -1;
                }
                double finalError = compareError * direction;
                double angleVelocity = finalError * rampRate;
                double turnOutput1 = AnglePIDController
                                .calculate(angleVelocity);
                //System.out.println("Current" + encoderRotation.getRadians() + " Desired" + state.angle.getRadians());
                double outputAngleFF = angleFeedforward
                                .calculate(angleVelocity);

                mDrive.setVoltage(driveOutput + outputDriveFF);

                mAngle.setVoltage(turnOutput1 + outputAngleFF);

                SmartDashboard.putNumber("A number", 0);

                
                
                /* 
                SmartDashboard.putNumber("driveOutput", driveOutput);
                SmartDashboard.putNumber("turnOutput", turnOutput);
                SmartDashboard.putNumber("outputDriveFF", outputDriveFF);
                SmartDashboard.putNumber("outputAngleFF", outputAngleFF);

                SmartDashboard.putNumber("driveVelocity", mDrive.getVelocity().getValueAsDouble());
                SmartDashboard.putNumber("state.SpeedMeters/second", state.speedMetersPerSecond);
                */
                /*
                var swerve_tab = Shuffleboard.getTab("Swerve");
                var module_layout = swerve_tab.getLayout(swerveName, BuiltInLayouts.kList);

                module_layout.add(swerveName + "_AngleFF", outputAngleFF);
                module_layout.add(swerveName + "PIDAngleOutput",turnOutput1);
                module_layout.add(swerveName + "_CurrentAngleSpeed", encAngle.getVelocity().getValueAsDouble() * 2*Math.PI);
                module_layout.add(swerveName + "DesiredAngle", state.angle.getRadians());
                module_layout.add(swerveName+ "CurrentAngle", encoderRotation.getRadians());
                module_layout.add(swerveName + "_Error", error);
                */
        }

}
