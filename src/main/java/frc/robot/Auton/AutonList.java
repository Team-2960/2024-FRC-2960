package frc.robot.Auton;

import java.io.IOException;
import java.sql.Driver;
import java.util.Map;
import java.util.Optional;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc.robot.auton.commands.drive.*;
import frc.robot.auton.commands.pizzabox.*;
import frc.robot.Util.FieldLayout;
import frc.robot.subsystems.Drive;
/*
public class AutonList {
        public static final Command autonPreamble = new SequentialCommandGroup(
                        new intakeNote(),
                        new ParallelCommandGroup(
                                        new armToPreset("Speaker"),
                                        new prepShootNote()),
                        new shootFastNote());
        public static final Command sourceSideRed = new SequentialCommandGroup(
                        new setFieldPose(new Pose2d(0, 0, Rotation2d.fromDegrees(120))),
                        new setFieldRelative(false),
                        new ParallelCommandGroup(
                                        new armToPreset("AmpSideShoot"),
                                        new prepShootNote()),
                        new WaitCommand(1),
                        new shootNote(),
                        new driveToTime(4, Rotation2d.fromDegrees(0), 2.5));
        public static final Command sourceSideBlue = new SequentialCommandGroup(
                        new setFieldPose(new Pose2d(0, 0, Rotation2d.fromDegrees(60))),
                        new setFieldRelative(false),
                        new ParallelCommandGroup(
                                        new armToPreset("AmpSideShoot"),
                                        new prepShootNote()),
                        new WaitCommand(1),
                        new shootNote(),
                        new driveToTime(4, Rotation2d.fromDegrees(0), 2.5));

        public static final Command AmpSideSimpleBlue = new SequentialCommandGroup(
                        new setFieldPose(new Pose2d(0, 0, Rotation2d.fromDegrees(300))),
                        new setFieldRelative(false),
                        new ParallelCommandGroup(
                                        new armToPreset("AmpSideShoot"),
                                        new prepShootNote()),
                        new WaitCommand(1),
                        new shootNote(),
                        new WaitCommand(12.5),
                        new driveToTime(6, Rotation2d.fromDegrees(-45), 2.5));
        public static final Command AmpSideSimpleRed = new SequentialCommandGroup(
                        new setFieldPose(new Pose2d(0, 0, Rotation2d.fromDegrees(240))),
                        new setFieldRelative(false),
                        new ParallelCommandGroup(
                                        new armToPreset("AmpSideShoot"),
                                        new prepShootNote()),
                        new WaitCommand(1),
                        new shootNote(),
                        new WaitCommand(12.5),
                        new driveToTime(6, Rotation2d.fromDegrees(45), 2.5));

        public static final Command middleSimpleRed = new SequentialCommandGroup(
                        new setFieldPose(new Pose2d(0, 0, Rotation2d.fromDegrees(180))),
                        new setFieldRelative(false),
                        new ParallelCommandGroup(
                                        new armToPreset("Speaker"),
                                        new prepShootNote()),
                        new WaitCommand(1),
                        new shootNote(),
                        new WaitCommand(5),
                        new driveToTime(2, Rotation2d.fromDegrees(0), 2));
        public static final Command middleSimpleBlue = new SequentialCommandGroup(
                        new setFieldPose(new Pose2d(0, 0, Rotation2d.fromDegrees(0))),
                        new setFieldRelative(false),
                        new ParallelCommandGroup(
                                        new armToPreset("Speaker"),
                                        new prepShootNote()),
                        new WaitCommand(1),
                        new shootNote(),
                        new WaitCommand(5),
                        new driveToTime(2, Rotation2d.fromDegrees(0), 2));

        public static final Command middle2NoteRed = new SequentialCommandGroup(
                        new setFieldRelative(true),
                        new setEnableCamera(true),
                        new setFieldPose(new Pose2d(6.53635, -1.443, Rotation2d.fromDegrees(180))),
                        autonPreamble,
                        new armToPreset("Intake"),
                        new ParallelRaceGroup(
                                        new intakeNote(),
                                        new goToPosition(FieldLayout.getNote(FieldLayout.NoteType.NEAR_SPEAKER), 1,
                                                        .5),
                                        new alignToPoint(FieldLayout.getNote(FieldLayout.NoteType.NEAR_SPEAKER),
                                                        Rotation2d.fromDegrees(0), false)), //TODO have it align first
                        new ParallelCommandGroup(
                                        new alignToPoint(FieldLayout.getSpeakerPose().getTranslation(),
                                                        Rotation2d.fromDegrees(0), true),
                                        new armToPreset("AmpSideShoot"),
                                        new prepShootNote()),
                        new shootFastNote());
        public static final Command middle2NoteBlue = new SequentialCommandGroup(
                        new setFieldRelative(true),
                        new setEnableCamera(true),
                        new setFieldPose(new Pose2d(-6.53635, -1.443, Rotation2d.fromDegrees(0))),
                        new intakeNote(),
                        new ParallelCommandGroup(
                                        new armToPreset("Speaker"),
                                        new prepShootNote()),
                        new shootNote(),
                        new armToPreset("Intake"),
                        new ParallelRaceGroup(
                                        new driveToTime(2, Rotation2d.fromDegrees(0), 3),
                                        // new waitForAprilTag(),
                                        new intakeNote()),
                        // new ParallelRaceGroup(
                        // new intakeNote(),
                        // new goToPosition(FieldLayout.getNote(FieldLayout.NoteType.NEAR_SPEAKER), 2,
                        // .5)
                        // ),
                        new ParallelCommandGroup(
                                        new armToPreset("AmpSideShoot"),
                                        new prepShootNote()),
                        new shootFastNote());

        public static final Command ampSide1NoteRed = new SequentialCommandGroup(
                new setFieldRelative(true),
                new setEnableCamera(true),
                new setFieldPose(new Pose2d(7.6073 , -2.6622, Rotation2d.fromDegrees(120))),
                new intakeNote(),
                        new ParallelCommandGroup(
                                        new armToPreset("Speaker"),
                                        new prepShootNote()),
                        new shootFastNote(),
                new ParallelCommandGroup(
                        new goToPosition(6, -2.5, 1, 0.2)
                )
        );

        public static final Command sourceSide3NoteRed= new SequentialCommandGroup(
                new setFieldRelative(true),
                new setEnableCamera(true),
                new setFieldPose(new Pose2d(0,0,Rotation2d.fromDegrees(180))),
                new intakeNote(),
                        new ParallelCommandGroup(
                                        new armToPreset("Speaker"),
                                        new prepShootNote()),
                        new shootFastNote(),
                new ParallelCommandGroup(
                        new goToAngle(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(1)),
                        new goToPosition(0, 0, 1, 0.2)
                ),
                new ParallelCommandGroup(
                        new goToAngle(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(1)),
                        new goToPosition(0, 0, 1, 0.2)
                ),
                new ParallelCommandGroup(
                        new goToAngle(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(1)),
                        new goToPosition(0, 0, 1, 0.2)
                ),
                new ParallelRaceGroup(
                        new alignToPoint(FieldLayout.cNoteMid, Rotation2d.fromDegrees(0), false),
                        new goToPosition(FieldLayout.cNoteMid, 1, 0),
                        new intakeNote()
                ),
                new ParallelCommandGroup(
                        new goToAngle(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(1)),
                        new goToPosition(0, 0, 1, 0.2)
                ),
                new ParallelCommandGroup(
                        new goToAngle(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(1)),
                        new goToPosition(0, 0, 1, 0.2)
                ),
                new ParallelCommandGroup(
                        new goToAngle(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(1)),
                        new goToPosition(0, 0, 1, 0.2)
                ),
                new intakeNote(),
                        new ParallelCommandGroup(
                                        new armToPreset("Speaker"),
                                        new prepShootNote()),
                        new shootFastNote(),
                new ParallelRaceGroup(
                        new alignToPoint(FieldLayout.rNoteStage, Rotation2d.fromDegrees(0), false),
                        new goToPosition(FieldLayout.rNoteStage, 1, 0),
                        new intakeNote()
                ),
                new ParallelRaceGroup(
                        new goToPosition(0, 0, 0, 0),
                        new alignToPoint(FieldLayout.getSpeakerPose().getTranslation(), Rotation2d.fromDegrees(0), false)
                ),
                
                new intakeNote(),
                        new ParallelCommandGroup(
                                        new armToPreset("Speaker"),
                                        new prepShootNote()),
                        new shootFastNote()

                
        );


        public static final Command testAuton = new SequentialCommandGroup(
                        new setFieldRelative(true),
                        new setFieldPose(new Pose2d(0, 0, Rotation2d.fromDegrees(180))),
                        new ParallelCommandGroup(
                                        new goToPosition(-1, 0, 1, 0.1),
                                        new goToAngle(Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(1))));
        
        public static final Command testAuton2 =
                Drive.getInstance().followPathCommand("Example Path");
       
        public static final Map<String, Map<Alliance, Command>> auton_list = Map.of(
                        // "Shoot and Drive", Map.of(
                        // Alliance.Red, shootAndDrive,
                        // Alliance.Blue, shootAndDrive
                        // )
                        "Shoot and Drive Simple", Map.of(
                                        Alliance.Red, sourceSideRed,
                                        Alliance.Blue, sourceSideBlue),
                        "Amp Side Simple", Map.of(
                                        Alliance.Red, AmpSideSimpleRed,
                                        Alliance.Blue, AmpSideSimpleBlue),
                        "Middle Shoot Simple", Map.of(
                                        Alliance.Red, middle2NoteRed,
                                        Alliance.Blue, middleSimpleBlue));

        public static Optional<Command> getCommands(String name) {
                Map<Alliance, Command> named_auton = auton_list.get(name);
                Command auton = null;
                var alliance = DriverStation.getAlliance();

                if (named_auton != null && alliance.isPresent())
                        auton = named_auton.get(alliance.get());

                return Optional.ofNullable(auton);
        }

        public static Optional<Command> getDefaultCommands() {
                return Optional.ofNullable(testAuton2);

        }
        
}*/
