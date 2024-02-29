package frc.robot.util;

import edu.wpi.first.math.geometry.*;

public class FieldLayout {
    public static final Pose2d rSpeaker = new Pose2d(8.308, 1.443, Rotation2d.fromDegrees(180));
    public static final Pose2d bSpeaker = new Pose2d(-8.309, 1.443, Rotation2d.fromDegrees(0));

    public static final Pose2d rAmp = new Pose2d(6.430, 4.099, Rotation2d.fromDegrees(270));
    public static final Pose2d bAmp = new Pose2d(-6.430, 4.099, Rotation2d.fromDegrees(270));

    public static final Pose2d rStageSpeaker = new Pose2d(3.634, 0.393, Rotation2d.fromDegrees(60));
    public static final Pose2d rStageFeader = new Pose2d(3.634, -0.392, Rotation2d.fromDegrees(300));
    public static final Pose2d rStageFar = new Pose2d(2.949,0.000, Rotation2d.fromDegrees(180));

    public static final Pose2d bStageSpeaker = new Pose2d(-3.634, 0.393, Rotation2d.fromDegrees(120));
    public static final Pose2d bStageFeader = new Pose2d(-3.634, -0.392, Rotation2d.fromDegrees(240));
    public static final Pose2d bStageFar = new Pose2d(-2.949,0.000, Rotation2d.fromDegrees(0));

    public static final Pose2d[] rStage = {rStageSpeaker, rStageFeader, rStageFar};
    public static final Pose2d[] bStage = {bStageSpeaker, bStageFeader, bStageFar};


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
     * Gets the pose of the speaker side stage for the current alliance
     * @return  pose of the speaker side stage for the current alliance
     */
    public static Pose2d getStageSpeakerPose() {
        if(Drivestation.getAlliance() == Drivestation.Alliance.Red) {
            return rStageSpeaker;
        } else {
            return bStageSpeaker;
        }
    }

    /**
     * Gets the pose of the feader side stage for the current alliance
     * @return  pose of the feader side stage for the current alliance
     */
    public static Pose2d getStageFeaderPose() {
        if(Drivestation.getAlliance() == Drivestation.Alliance.Red) {
            return rStageFeader;
        } else {
            return bStageFeader;
        }
    }

    /**
     * Gets the pose of the far side stage for the current alliance
     * @return  pose of the far side stage for the current alliance
     */
    public static Pose2d getStageFarPose() {
        if(Drivestation.getAlliance() == Drivestation.Alliance.Red) {
            return rStageFar;
        } else {
            return bStageFar;
        }
    }

    /**
     * Gets the pose of the nearest stage for the current alliance
     * @param   pos     current position
     * @return  pose of the nearest stage  for the current alliance
     */
    public static Pose2d getNearestStage(Translation2d pos) {
        Pose2d[] stage_list = bStage;
        if(Drivestation.getAlliance() == Drivestation.Alliance.Red) stage_list = rStage;

        double min_dist = Double.MAX_VALUE;
        Pose2d nearest = null;

        for(var pose : stage_list) {
            double distance = pos.getDistance(pose.getTranslation());
            if(distance < min_dist) {
                min_dist = distance;
                nearest = pose;
            }
        }

        return nearest;
    }
}