package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        Pose2d scorePosition = new Pose2d(-54, -54, Math.toRadians(45));
        double bucketScoreTime = 4;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(137.87183), Math.toRadians(90), 5.57)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-34,-62, Math.PI))
                        .addTemporalMarker(0, () -> {})
                        .addTemporalMarker(2, () -> {})//extend vertical slides and score
                        .splineToLinearHeading(scorePosition, Math.toRadians(45))
                        .addDisplacementMarker(()->{})
                        .waitSeconds(bucketScoreTime)
                        .addTemporalMarker(() -> {}) //retract vertical slides
                        .splineToLinearHeading(new Pose2d(-30.5, -37, Math.toRadians(155)), Math.toRadians(155))
                        .addDisplacementMarker(() -> {})
                        .forward(1)
                        .addTemporalMarkerOffset(.1, () -> {})
                        .addTemporalMarkerOffset(.35, () -> {}) //perform intake transition
                        .waitSeconds(.2)
                        .addTemporalMarker(() -> {})
                        .waitSeconds(1)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
   }