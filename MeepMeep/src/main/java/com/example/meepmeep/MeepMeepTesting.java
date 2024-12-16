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
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-7.36,-46.489, 0))
                        .splineToLinearHeading(scorePosition, Math.toRadians(45))
                        .waitSeconds(bucketScoreTime)
                        .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
   }