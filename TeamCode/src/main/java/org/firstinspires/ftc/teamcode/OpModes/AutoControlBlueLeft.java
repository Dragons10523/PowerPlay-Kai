package org.firstinspires.ftc.teamcode.OpModes;


import android.annotation.SuppressLint;
import com.acmerobotics.roadrunner.trajectory.TimeProducer;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.drive.SampleMecanumDrive;

@Autonomous(name = "Auto_Left")
public class AutoControlBlueLeft extends AutoControl {

    private final Utils.FieldSide fieldSide = Utils.FieldSide.BLUE_LEFT;


    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        super.initialize();

        double bucketScoreTime = 2;

        SparkFunOTOS.Pose2D pos = new SparkFunOTOS.Pose2D(34, 62, Math.toRadians(270));

        Pose2d scorePosition = new Pose2d(53, 53, Math.toRadians(225));

        robot.opticalSensor.setPosition(pos);

        TrajectorySequence firstScore = drive.trajectorySequenceBuilder(new Pose2d(35, 62, Math.toRadians(270)))
                .addTemporalMarker(0, () -> {
                    autoUtils.armFlip(Utils.ArmFlipState.GROUND);
                })
                .splineToLinearHeading(scorePosition, Math.toRadians(225))
                .forward(-.5)
                .addTemporalMarker(2, ()->{
                    autoUtils.verticalSlide(Utils.LiftState.HIGH);
                })//extend vertical slides and score
                .waitSeconds(bucketScoreTime)
                .build();
        TrajectorySequence moveToFirstPiece = drive.trajectorySequenceBuilder(firstScore.end())
                .addTemporalMarker(0, () -> {
                    autoUtils.verticalSlide(Utils.LiftState.GROUND);
                }) //retract vertical slides
                .waitSeconds(2)
                .forward(3)
                .splineToLinearHeading(new Pose2d(36, 39, Math.toRadians(315)), Math.toRadians(315))
                //TODO: optimize times
                .waitSeconds(2)
                .addDisplacementMarker(()->{
                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(.75);
                })
                .forward(2)
                .addDisplacementMarker(() -> {
                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(0);
                }) //perform intake transition
                .addDisplacementMarker(()->{
                    autoUtils.intakeTransition();
                })
                .waitSeconds(2)
                .build();
        TrajectorySequence secondScore = drive.trajectorySequenceBuilder(moveToFirstPiece.end())
                .splineToLinearHeading(scorePosition, Math.toRadians(225))
                .forward(-2)
                .addDisplacementMarker(() -> {
                    autoUtils.verticalSlide(Utils.LiftState.HIGH);
                }) //extend vertical slides
                .waitSeconds(bucketScoreTime)
                .build();
        TrajectorySequence moveToSecondPiece = drive.trajectorySequenceBuilder(secondScore.end())
                .addTemporalMarker(0, () -> {
                    autoUtils.verticalSlide(Utils.LiftState.GROUND);
                })//retract vertical slides
                .splineToLinearHeading(new Pose2d(58.5, 39, Math.toRadians(270)), Math.toRadians(270))
                .UNSTABLE_addTemporalMarkerOffset(1,() -> {
                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(0.75);
                })
                .forward(2)
                .addDisplacementMarker(() -> {
                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(0);
                    autoUtils.intakeTransition();
                })
                .build();
        TrajectorySequence thirdScore = drive.trajectorySequenceBuilder(moveToSecondPiece.end())
                .splineToLinearHeading(scorePosition, Math.toRadians(225))
                .forward(-2)
                .addTemporalMarker(1, () -> {
                    autoUtils.verticalSlide(Utils.LiftState.HIGH);
                })
                .addTemporalMarker(4,()->{
                    robot.Servos.get(RobotClass.SERVOS.BUCKET).setPosition(0);
                })
                .waitSeconds(bucketScoreTime)
                .build();
        TrajectorySequence moveToThirdPiece = drive.trajectorySequenceBuilder(thirdScore.end())
                .addTemporalMarker(0, () -> {
                    autoUtils.verticalSlide(Utils.LiftState.GROUND);
                })
                .splineToLinearHeading(new Pose2d(58, 26, 0), 0)
                .addDisplacementMarker(()->{
                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(0.75);
                })
                .forward(3)
                .addDisplacementMarker(()->{
                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(0);
                    autoUtils.intakeTransition();
                })//do intake stuff
                .build();
        TrajectorySequence fourthScore = drive.trajectorySequenceBuilder(moveToThirdPiece.end())
                .forward(-12)
                .splineToLinearHeading(scorePosition, Math.toRadians(225))
                .forward(-2)
                .addDisplacementMarker(()->{
                    autoUtils.verticalSlide(Utils.LiftState.HIGH);
                })
                .waitSeconds(bucketScoreTime)
                .build();
        TrajectorySequence moveToPark = drive.trajectorySequenceBuilder(fourthScore.end())
                .addTemporalMarker(0,()->{
                    autoUtils.verticalSlide(Utils.LiftState.GROUND);
                })
                .splineTo(new Vector2d(40, 0), Math.toRadians(270))
                .strafeRight(13)
                .build();

        while (!isStarted()) {
            SparkFunOTOS.Pose2D pose = robot.opticalSensor.getPosition();
            telemetry.addData("posX", pose.x);
            telemetry.addData("posY", pose.y);
            telemetry.addData("posH", pose.h);
            telemetry.update();
        }
        waitForStart();

        drive.followTrajectorySequence(firstScore);
        drive.followTrajectorySequence(moveToFirstPiece);
//        drive.followTrajectorySequence(secondScore);
//        drive.followTrajectorySequence(moveToSecondPiece);
//        drive.followTrajectorySequence(thirdScore);
//        drive.followTrajectorySequence(moveToThirdPiece);
//        drive.followTrajectorySequence(fourthScore);
//        drive.followTrajectorySequence(moveToPark);

        while (opModeIsActive()) {

            SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();

            telemetry.addLine(String.format("XYH %.3f %.3f %.3f", pose2D.x, pose2D.y, pose2D.h));

            telemetry.update();
        }
    }
}