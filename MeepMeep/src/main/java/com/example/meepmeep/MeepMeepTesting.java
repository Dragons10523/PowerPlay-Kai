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
        double bucketScoreTime = 2;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, Math.toRadians(137.87183), Math.toRadians(90), 5.57)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(32.5, 62, Math.toRadians(270)))
                        .addTemporalMarker(0, () -> {
                            //autoUtils.armFlip(Utils.ArmFlipState.GROUND);
                        })
                        .splineToLinearHeading(scorePosition, Math.toRadians(225))
                        .forward(-0.5)
                        .addTemporalMarker(2,()->{
                        })//extend vertical slides and score
                        .waitSeconds(bucketScoreTime)
                        .addTemporalMarker(0, () -> {
                        }) //retract vertical slides
                        .splineToLinearHeading(new Pose2d(36, 39, Math.toRadians(315)), Math.toRadians(315))
                        .waitSeconds(2)
                        .forward(2)
                        .addDisplacementMarker(() -> {
                        }) //perform intake transition
                        .build());




        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
   }