package frc.robot.util;

import java.util.Map;

import edu.wpi.first.math.geometry.*;

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


    public static final Pose2d rSpeaker = new Pose2d(8.308, 1.443, Rotation2d.fromDegrees(180));
    public static final Pose2d bSpeaker = new Pose2d(-8.309, 1.443, Rotation2d.fromDegrees(0));

    public static final Pose2d rAmp = new Pose2d(6.430, 4.099, Rotation2d.fromDegrees(270));
    public static final Pose2d bAmp = new Pose2d(-6.430, 4.099, Rotation2d.fromDegrees(270));

    public static final Pose2d rStageAmp = new Pose2d(3.634, 0.393, Rotation2d.fromDegrees(60));
    public static final Pose2d rStageSource = new Pose2d(3.634, -0.392, Rotation2d.fromDegrees(300));
    public static final Pose2d rStageFar = new Pose2d(2.949,0.000, Rotation2d.fromDegrees(180));

    public static final Pose2d bStageAmp = new Pose2d(-3.634, 0.393, Rotation2d.fromDegrees(120));
    public static final Pose2d bStageSource = new Pose2d(-3.634, -0.392, Rotation2d.fromDegrees(240));
    public static final Pose2d bStageFar = new Pose2d(-2.949,0.000, Rotation2d.fromDegrees(0));

    public static final Map<StageFace, Pose2d> rStage = Map.of(
        StageFace.AMP, rStageAmp, 
        StageFace.SOURCE, rStageSource, 
        StageFace.FAR, rStageFar
    );

    public static final Map<StageFace, Pose2d> bStage = Map.of(
        StageFace.AMP, bStageAmp, 
        StageFace.SOURCE, bStageSource, 
        StageFace.FAR, bStageFar
    );

    public static final Translation2d rNoteStage = new Translation2d(5.185,0.000);
    public static final Translation2d rNoteSpeaker = new Translation2d(5.185,1.397);
    public static final Translation2d rNoteAmp = new Translation2d(5.185,2.793);

    public static final Translation2d bNoteStage = new Translation2d(-5.185,0.000);
    public static final Translation2d bNoteSpeaker = new Translation2d(-5.185,1.397);
    public static final Translation2d bNoteAmp = new Translation2d(-5.185,2.793);

    public static final Translation2d cNoteSource2 = new Translation2d(0.000,-3.234);
    public static final Translation2d cNoteSource1 = new Translation2d(0.000,-3.234);
    public static final Translation2d cNoteMid = new Translation2d(0.000,-3.234);
    public static final Translation2d cNoteAmp1 = new Translation2d(0.000,-3.234);
    public static final Translation2d cNoteAmp2 = new Translation2d(0.000,-3.234);

    public static final Map<NoteType, Translation2d> rNotes = Map.of(
        NoteType.NEAR_STAGE, rNoteStage,
        NoteType.NEAR_SPEAKER, rNoteSpeaker,
        NoteType.NEAR_AMP, rNoteAmp,
        NoteType.FAR_FEED2, cNoteSource2,
        NoteType.FAR_FEED1, cNoteSource1,
        NoteType.FAR_MID, cNoteMid,
        NoteType.FAR_AMP1, cNoteAmp1,
        NoteType.FAR_AMP2, cNoteAmp2
    )
    
    public static final Map<NoteType, Translation2d> bNotes = Map.of(
        NoteType.NEAR_STAGE, bNoteStage,
        NoteType.NEAR_SPEAKER, bNoteSpeaker,
        NoteType.NEAR_AMP, bNoteAmp,
        NoteType.FAR_FEED2, cNoteSource2,
        NoteType.FAR_FEED1, cNoteSource1,
        NoteType.FAR_MID, cNoteMid,
        NoteType.FAR_AMP1, cNoteAmp1,
        NoteType.FAR_AMP2, cNoteAmp2
    )

    /**
     * Gets the pose of the speaker for the current alliance
     * @return  pose of the speaker for the current alliance
     */
    public static Pose2d getSpeakerPose() {
        if(Drivestation.getAlliance() == Drivestation.Alliance.Red) {
            return rSpeaker;
        } else {
            return bSpeaker;
        }
    }

    /**
     * Gets the pose of the amp for the current alliance
     * @return  pose of the amp for the current alliance
     */
    public static Pose2d getAmpPose() {
        if(Drivestation.getAlliance() == Drivestation.Alliance.Red) {
            return rAmp;
        } else {
            return bAmp;
        }
    }

    /**
     * Gets the pose of the stage for the current alliance
     * @param   face    face to retrieve
     * @return  pose of the stage for the current alliance
     */
    public static Pose2d getStage(StageFace face) {
        if(Drivestation.getAlliance() == Drivestation.Alliance.Red) {
            return rStage.get(face);
        } else {
            return bStage.get(face);
        }
    }

    /**
     * Gets the pose of the nearest stage for the current alliance
     * @param   pos     current position
     * @return  pose of the nearest stage  for the current alliance
     */
    public static Pose2d getNearestStage(Translation2d pos) {
        Map<StageFace, Pose2d> stage_list = bStage;
        if(Drivestation.getAlliance() == Drivestation.Alliance.Red) stage_list = rStage;

        double min_dist = Double.MAX_VALUE;
        Pose2d nearest = null;

        for(var pose : stage_list.entrySet()) {
            double distance = pos.getDistance(pose.getValue().getTranslation());
            if(distance < min_dist) {
                min_dist = distance;
                nearest = pose.getValue();
            }
        }

        return nearest;
    }


    /**
     * Gets the position of a pre-staged note
     * @param   note_type   id for the desired pre-staged notes
     * @return  position of the selected note
     */
    public Translation2d getNote(NoteType note_type) {
        if(Drivestation.getAlliance() == Drivestation.Alliance.Red) {
            return rNotes.get(note_type);
        } else {
            return bNotes.get(note_type);
        }
    }
}