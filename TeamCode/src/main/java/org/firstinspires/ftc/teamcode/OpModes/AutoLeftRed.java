package org.firstinspires.ftc.teamcode.OpModes;


import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utils;

import java.util.Objects;

@Autonomous(name = "Auto_Left_Red")
public class AutoLeftRed extends AutoControl {
    ElapsedTime time = new ElapsedTime();

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("opMode started");
        telemetry.update();
        super.runOpMode();
        telemetry.addLine("opMode INIT");
        telemetry.update();
        super.initialize();
        telemetry.addLine("headingInit");
        telemetry.update();
        super.initialHeading(Math.toRadians(180), false);
        double startTime = time.seconds();
        do {
            sleep(20);
            telemetry.addLine("isLooping");
            telemetry.addData("SuccessfulLocalizationCount", autoUtils.getSuccessfulLocalizationCount());
            telemetry.update();
            autoUtils.updateOpticalSensorToPoseEstimateCamera();
            if (startTime + 5 < time.seconds() || isStarted()) {
                break;
            }
        }
        while (autoUtils.getSuccessfulLocalizationCount() < 20);
        SparkFunOTOS.Pose2D pos = robot.opticalSensor.getPosition();
        double bucketScoreTime = 2;
        Pose2d scorePosition = new Pose2d(-54, -54, Math.toRadians(45));
        Pose2d startPos = new Pose2d(pos.x, pos.y, pos.h);
        Trajectory moveToScoreRed = drive.trajectoryBuilder(startPos)
                .splineToConstantHeading(new Vector2d(-30, -55), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-54, -54, Math.toRadians(45)), Math.toRadians(225))
                .build();

        TrajectorySequence firstScore = drive.trajectorySequenceBuilder(startPos)
                .addTemporalMarker(1, () -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.armFlip(Utils.ArmFlipState.GROUND, 0.6);
                            autoUtils.armExtension(Utils.ArmState.IN);
                        }
                    };
                    t1.start();
                })
                .addTrajectory(moveToScoreRed)
                .build();
        telemetry.addLine("firstScore success");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {

            SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();
            LLResult result = robot.limelight.getLatestResult();
            telemetry.addData("XYH: ", "%.3f %.3f %.3f", pose2D.x, pose2D.y, pose2D.h);
            telemetry.addData("successfulLocalizations", autoUtils.getSuccessfulLocalizationCount());
            if (result != null) {
                telemetry.addData("staleness", result.getStaleness());
                telemetry.addData("validResult?", result.isValid());
            } else {
                telemetry.addLine("No data available");
            }
            telemetry.update();

        }
        waitForStart();

        //Score pre-loaded sample
        drive.followTrajectorySequence(firstScore);
        //Ensure horizontal extension is IN
        autoUtils.armExtension(Utils.ArmState.IN);
        //check displacement and re-position if inaccurate
        double displacementFromTarget = opticalSensorClass.getDisplacementFromTarget(scorePosition.getX(), scorePosition.getY());
        if (displacementFromTarget > 3) {
            drive.followTrajectory(trajectoryHandler.lineToScoreRed(5));
            drive.turn(Math.toRadians(45));
        }
        //extend, score, retract
        autoUtils.scorePiece(1);
        //move and grab piece
        drive.followTrajectorySequence(trajectoryHandler.moveToFirstPieceRed());
        //transition piece into bucket
        autoUtils.intakeTransition();
        //ensure piece is in bucket
        autoUtils.sampleWiggle();
        //move to score position
        drive.followTrajectorySequence(trajectoryHandler.scoreRed());
        //check displacement and re-position if inaccurate
        displacementFromTarget = opticalSensorClass.getDisplacementFromTarget(scorePosition.getX(), scorePosition.getY());
        if (displacementFromTarget > 3) {
            drive.followTrajectory(trajectoryHandler.lineToScoreRed(5));
            drive.turn(Math.toRadians(45));
        }
        //position arm down
        autoUtils.armFlip(Utils.ArmFlipState.GROUND, 0.6);
        //extend, score, retract
        autoUtils.scorePiece(1);
        //move and grab second piece
        drive.followTrajectorySequence(trajectoryHandler.moveToSecondPieceRed());
        //transition piece into bucket
        autoUtils.intakeTransition();
        //ensure piece is in bucket
        autoUtils.sampleWiggle();
        //move to scoreRed
        drive.followTrajectorySequence(trajectoryHandler.scoreRed());
        //ensure accurate position
        displacementFromTarget = opticalSensorClass.getDisplacementFromTarget(scorePosition.getX(), scorePosition.getY());
        if (displacementFromTarget > 3) {
            drive.followTrajectory(trajectoryHandler.lineToScoreRed(5));
            drive.turn(Math.toRadians(45));
        }
        //position arm down
        autoUtils.armFlip(Utils.ArmFlipState.GROUND, 0.6);
        //extend, score, retract
        autoUtils.scorePiece(1);
        //position arm up
        autoUtils.armFlip(Utils.ArmFlipState.UP, 0.6);
        //PARK
        drive.followTrajectory(trajectoryHandler.splineToParkRed());
        //position arm to touch bar for parking points
        autoUtils.armFlip(Utils.ArmFlipState.MIDDLE, 0.2);

//        drive.followTrajectorySequence(moveToSecondPiece);
//        drive.followTrajectorySequence(thirdScore); // and park
//        drive.followTrajectorySequence(moveToThirdPiece);
//        drive.followTrajectorySequence(fourthScore);
        //drive.followTrajectorySequence(moveToPark);
        while (!getStopRequested()) {
            SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();
            LLResult result = robot.limelight.getLatestResult();
            telemetry.addData("XYH: ", "%.3f %.3f %.3f", pose2D.x, pose2D.y, pose2D.h);
            telemetry.addData("successfulLocalizations", autoUtils.getSuccessfulLocalizationCount());
            telemetry.addData("Displacement", displacementFromTarget);
            if (result != null) {
                telemetry.addData("staleness", result.getStaleness());
                telemetry.addData("validResult?", result.isValid());
            } else {
                telemetry.addLine("No data available");
            }
            telemetry.update();
        }
        robot.limelight.close();
    }
}