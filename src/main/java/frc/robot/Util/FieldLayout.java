package frc.robot.Util;

import java.util.Map;

import javax.sql.rowset.spi.TransactionalWriter;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class FieldLayout {
    public enum StageFace {
        AMP,
        SOURCE,
        FAR
    }

    public enum NoteType {
        NEAR_STAGE,
        NEAR_SPEAKER,
        NEAR_AMP,
        FAR_SOURCE2,
        FAR_SOURCE1,
        FAR_MID,
        FAR_AMP1,
        FAR_AMP2
    }

    public static final Pose2d rSpeaker = new Pose2d(8.308, -1.443, Rotation2d.fromDegrees(180));
    public static final Pose2d bSpeaker = new Pose2d(-8.309, -1.443, Rotation2d.fromDegrees(0));

    public static final Pose2d rAmp = new Pose2d(6.430, -4.099, Rotation2d.fromDegrees(270));
    public static final Pose2d bAmp = new Pose2d(-6.430, -4.099, Rotation2d.fromDegrees(270));

    public static final Pose2d rStageAmp = new Pose2d(3.634, -0.393, Rotation2d.fromDegrees(60));
    public static final Pose2d rStageSource = new Pose2d(3.634, 0.392, Rotation2d.fromDegrees(300));
    public static final Pose2d rStageFar = new Pose2d(2.949, 0.000, Rotation2d.fromDegrees(180));

    public static final Pose2d bStageAmp = new Pose2d(-3.634, -0.393, Rotation2d.fromDegrees(120));
    public static final Pose2d bStageSource = new Pose2d(-3.634, 0.392, Rotation2d.fromDegrees(240));
    public static final Pose2d bStageFar = new Pose2d(-2.949, 0.000, Rotation2d.fromDegrees(0));

    public static final Map<StageFace, Pose2d> rStage = Map.of(
            StageFace.AMP, rStageAmp,
            StageFace.SOURCE, rStageSource,
            StageFace.FAR, rStageFar);

    public static final Map<StageFace, Pose2d> bStage = Map.of(
            StageFace.AMP, bStageAmp,
            StageFace.SOURCE, bStageSource,
            StageFace.FAR, bStageFar);

    public static final Translation2d rNoteStage = new Translation2d(5.185, 0.000);
    public static final Translation2d rNoteSpeaker = new Translation2d(5.185, -1.397);
    public static final Translation2d rNoteAmp = new Translation2d(5.185, -2.793);

    public static final Translation2d bNoteStage = new Translation2d(-5.185, 0.000);
    public static final Translation2d bNoteSpeaker = new Translation2d(-5.185, -1.397);
    public static final Translation2d bNoteAmp = new Translation2d(-5.185, -2.793);

    public static final Translation2d cNoteSource2 = new Translation2d(0.000, 3.234);
    public static final Translation2d cNoteSource1 = new Translation2d(0.000, 3.234);
    public static final Translation2d cNoteMid = new Translation2d(0.000, 3.234);
    public static final Translation2d cNoteAmp1 = new Translation2d(0.000, 3.234);
    public static final Translation2d cNoteAmp2 = new Translation2d(0.000, 3.234);

    public static final Map<NoteType, Translation2d> rNotes = Map.of(
            NoteType.NEAR_STAGE, rNoteStage,
            NoteType.NEAR_SPEAKER, rNoteSpeaker,
            NoteType.NEAR_AMP, rNoteAmp,
            NoteType.FAR_SOURCE2, cNoteSource2,
            NoteType.FAR_SOURCE1, cNoteSource1,
            NoteType.FAR_MID, cNoteMid,
            NoteType.FAR_AMP1, cNoteAmp1,
            NoteType.FAR_AMP2, cNoteAmp2);

    public static final Map<NoteType, Translation2d> bNotes = Map.of(
            NoteType.NEAR_STAGE, bNoteStage,
            NoteType.NEAR_SPEAKER, bNoteSpeaker,
            NoteType.NEAR_AMP, bNoteAmp,
            NoteType.FAR_SOURCE2, cNoteSource2,
            NoteType.FAR_SOURCE1, cNoteSource1,
            NoteType.FAR_MID, cNoteMid,
            NoteType.FAR_AMP1, cNoteAmp1,
            NoteType.FAR_AMP2, cNoteAmp2);

    public static final double rAutoLineX = 6.137; // Meters
    public static final double bAutoLineX = -6.137; // Meters

    public static final double rWingLineX = 2.338; // Meters
    public static final double bWingLineX = -2.338; // Meters

    /**
     * Gets the pose of the speaker for the current alliance
     * 
     * @return pose of the speaker for the current alliance
     */
    public static Pose2d getSpeakerPose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return rSpeaker;
        } else {
            return bSpeaker;
        }
    }

    /**
     * Gets the pose of the amp for the current alliance
     * 
     * @return pose of the amp for the current alliance
     */
    public static Pose2d getAmpPose() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return rAmp;
        } else {
            return bAmp;
        }
    }

    /**
     * Gets the pose of the stage for the current alliance
     * 
     * @param face face to retrieve
     * @return pose of the stage for the current alliance
     */
    public static Pose2d getStage(StageFace face) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return rStage.get(face);
        } else {
            return bStage.get(face);
        }
    }

    /**
     * Gets the pose of the nearest stage for the current alliance
     * 
     * @param pos current position
     * @return pose of the nearest stage for the current alliance
     */
    public static Pose2d getNearestStage(Translation2d pos) {
        Map<StageFace, Pose2d> stage_list = bStage;
        var alliance = DriverStation.getAlliance();
        Pose2d nearest = null;
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            stage_list = rStage;

            double min_dist = Double.MAX_VALUE;

            for (var pose : stage_list.entrySet()) {
                double distance = pos.getDistance(pose.getValue().getTranslation());
                if (distance < min_dist) {
                    min_dist = distance;
                    nearest = pose.getValue();
                }
            }
        }

        return nearest;
    }

    /**
     * Gets the position of a pre-staged note
     * 
     * @param note_type id for the desired pre-staged notes
     * @return position of the selected note
     */
    public static Translation2d getNote(NoteType note_type) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return rNotes.get(note_type);
        } else {
            return bNotes.get(note_type);
        }
    }

    public static Translation2d getNoteOffset(NoteType noteType, double x, double y){
        var notePos = getNote(noteType);
        return new Translation2d(notePos.getX() + x, notePos.getY() + y);
    }

    /**
     * Gets the x position of the autoline for the current alliance
     * 
     * @return x position of the autoline for the current alliance
     */
    public static double getAutoLineX() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return rAutoLineX;
        } else {
            return bAutoLineX;
        }
    }

    /**
     * Gets the x position of the wing line for the current alliance
     * 
     * @return x position of the wing line for the current alliance
     */
    public static double getWingLineX() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return rWingLineX;
        } else {
            return bWingLineX;
        }
    }

    /**
     * Gets the x position that will ensure the robot is clear of the auto zone line
     * 
     * @return x position that will ensure the robot is clear of the auto zone line
     */
    public static double getAutoClearX() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return rAutoLineX - Constants.robotDiag - Constants.autoClearance;
        } else {
            return bAutoLineX + Constants.robotDiag + Constants.autoClearance;
        }
    }

    /**
     * Gets the forward angle for the robot for the current alliance
     * 
     * @return forward angle for the robot for the current alliance
     */
    public static Rotation2d getForwardAngle() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return Rotation2d.fromDegrees(180);
        } else {
            return Rotation2d.fromDegrees(0);
        }
    }
}