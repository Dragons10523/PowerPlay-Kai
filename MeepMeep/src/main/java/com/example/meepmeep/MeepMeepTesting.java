package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.SampleMecanumDrive;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.Vector;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d scorePosition = new Pose2d(53, 53, Math.toRadians(225));
        Pose2d AutoRightPos = new Pose2d(-14, 62, 0);
        Pose2d AutoLeftPos = new Pose2d(33, 62, Math.toRadians(270));
        Pose2d AutoRightScorePos = new Pose2d(-14,62, Math.toRadians(180));
        double bucketScoreTime = 2;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(137.87183), Math.toRadians(90), 5.57)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(AutoLeftPos)
//                        .addTemporalMarker(0, () -> {})
//                        .addTemporalMarker(0.5, () -> {})//extend vertical slides and score
                        .splineToLinearHeading(scorePosition, Math.toRadians(225))
                        .strafeLeft(1)
                        .waitSeconds(bucketScoreTime)
//                        .addTemporalMarker(() -> {}) //retract vertical slides
                        .forward(3)
                        .splineToLinearHeading(new Pose2d(35.5, 42, Math.toRadians(315)), Math.toRadians(315))
                        .strafeLeft(.5)
//                        .addDisplacementMarker(() -> {})
                        .forward(1)
//                        .addTemporalMarkerOffset(.2,() -> {}) //perform intake transition
                        .waitSeconds(.2)
//                        .addTemporalMarker(() -> {})
                        .waitSeconds(2)
                        .splineToLinearHeading(scorePosition, Math.toRadians(225))
                        .back(.5)
//                        .addTemporalMarkerOffset(0.5,() -> {}) //extend vertical slides
                        .waitSeconds(bucketScoreTime)
//                        .addTemporalMarker(() -> {}) //retract vertical slides
                        .waitSeconds(1.5)
//                        .addTemporalMarkerOffset(.5,()->{})
                        .splineToLinearHeading(new Pose2d(40, 33, Math.toRadians(335)), Math.toRadians(335))

                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
   }