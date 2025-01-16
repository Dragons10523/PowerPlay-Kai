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
        while (autoUtils.getSuccessfulLocalizationCount() < 60);
        SparkFunOTOS.Pose2D pos = robot.opticalSensor.getPosition();
        double bucketScoreTime = 2;
        Pose2d scorePosition = new Pose2d(-54, -54, Math.toRadians(45));

        TrajectorySequence firstScore = drive.trajectorySequenceBuilder(new Pose2d(pos.x, pos.y, pos.h))
                .addTemporalMarker(1, () -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.armFlip(Utils.ArmFlipState.GROUND, 0.6);
                            autoUtils.armExtension(Utils.ArmState.IN);
                        }
                    };
                    t1.start();
                })
                .lineToLinearHeading(new Pose2d(pos.x - 10, pos.y + 10, Math.toRadians(90)))
                .setReversed(true)
                .splineTo(new Vector2d(-54, -54), Math.toRadians(225))
                .setReversed(false)
                .build();
        telemetry.addLine("firstScore success");
        telemetry.update();


//        telemetry.addLine("secondPiece success");
//        telemetry.update();
//        TrajectorySequence thirdScore = drive.trajectorySequenceBuilder(moveToSecondPiece.end())
//                .splineToLinearHeading(scorePosition, Math.toRadians(225))
//                .back(.5)
//                .addTemporalMarker(0.5, () -> {
//                    Thread t1 = new Thread() {
//                        public void run() {
//                            autoUtils.verticalSlide(Utils.LiftState.HIGH);
//                        }
//                    };
//                    t1.start();
//                }) //extend vertical slides
//                .waitSeconds(bucketScoreTime)
//                .addTemporalMarker(() -> {
//                    Thread t1 = new Thread() {
//                        public void run() {
//                            autoUtils.verticalSlide(Utils.LiftState.GROUND);
//                        }
//                    };
//                    t1.start();
//                }) //retract vertical slides
//                .back(1)
//                .build();
//        telemetry.addLine("thirdScore success");
//        telemetry.update();
//        TrajectorySequence moveToThirdPiece = drive.trajectorySequenceBuilder(thirdScore.end())
//                .addTemporalMarker(0, () -> {
//                    Thread t1 = new Thread() {
//                        public void run() {
//                            autoUtils.verticalSlide(Utils.LiftState.GROUND);
//                        }
//                    };
//                    t1.start();
//                })
//                .splineToLinearHeading(new Pose2d(58, 26, 0), 0)
//                .addDisplacementMarker(() -> {
//                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(0.75);
//                })
//                .forward(3)
//                .addDisplacementMarker(() -> {
//                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(0);
//                    Thread t2 = new Thread() {
//                        public void run() {
//                            autoUtils.intakeTransition();
//                        }
//                    };
//                    t2.start();
//                })
//                .build();
//        telemetry.addLine("thirdPiece success");
//        telemetry.update();
//        TrajectorySequence fourthScore = drive.trajectorySequenceBuilder(moveToThirdPiece.end())
//                .forward(-12)
//                .splineToLinearHeading(scorePosition, Math.toRadians(225))
//                .forward(-2)
//                .addDisplacementMarker(() -> {
//                    Thread t1 = new Thread() {
//                        public void run() {
//                            autoUtils.verticalSlide(Utils.LiftState.HIGH);
//                        }
//                    };
//                    t1.start();
//                })
//                .waitSeconds(bucketScoreTime)
//                .build();
//        telemetry.addLine("fourthScore success");
//        telemetry.update();
//        TrajectorySequence moveToPark = drive.trajectorySequenceBuilder(firstScore.end())
//                .waitSeconds(1.5)
//                .addTemporalMarker(2, () -> {
//                    autoUtils.armFlip(Utils.ArmFlipState.UP, .8);
//                })
//                .splineTo(new Vector2d(45, 13), Math.toRadians(180))
//                .forward(20)
//                .build();
        telemetry.addLine("park success");
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
            drive.followTrajectory(trajectoryHandler.lineToLinearHeadingScoreRed(5));
        }
        //extend, score, retract
        autoUtils.scorePiece(2);
        //move and grab piece
        drive.followTrajectorySequence(trajectoryHandler.moveToFirstPieceRed());
        //transition piece into bucket
        autoUtils.intakeTransition();
        //ensure piece is in bucket

        double wiggleStartTime = time.seconds();
        while (wiggleStartTime + 0.25 > time.seconds()) {
            robot.drivetrain.simpleDrive(-1);
        }
        wiggleStartTime = time.seconds();
        while (wiggleStartTime + 0.1 > time.seconds()) {
            robot.drivetrain.simpleDrive(1);
        }
        robot.drivetrain.simpleDrive(0);

        //move to score position
        drive.followTrajectorySequence(trajectoryHandler.scoreRed());
        //check displacement and re-position if inaccurate
        displacementFromTarget = opticalSensorClass.getDisplacementFromTarget(scorePosition.getX(), scorePosition.getY());
        if (displacementFromTarget > 3) {
            drive.followTrajectory(trajectoryHandler.lineToLinearHeadingScoreRed(5));
        }
        //position arm down
        autoUtils.armFlip(Utils.ArmFlipState.GROUND, 0.6);
        //extend, score, retract
        autoUtils.scorePiece(2);

        drive.followTrajectorySequence(trajectoryHandler.moveToSecondPieceRed());

        autoUtils.intakeTransition();

        wiggleStartTime = time.seconds();
        while (wiggleStartTime + 0.25 > time.seconds()) {
            robot.drivetrain.simpleDrive(-1);
        }
        wiggleStartTime = time.seconds();
        while (wiggleStartTime + 0.1 > time.seconds()) {
            robot.drivetrain.simpleDrive(1);
        }
        robot.drivetrain.simpleDrive(0);

        drive.followTrajectorySequence(trajectoryHandler.scoreRed());

        displacementFromTarget = opticalSensorClass.getDisplacementFromTarget(scorePosition.getX(), scorePosition.getY());
        if (displacementFromTarget > 3) {
            drive.followTrajectory(trajectoryHandler.lineToLinearHeadingScoreRed(5));
        }

        //position arm down
        autoUtils.armFlip(Utils.ArmFlipState.GROUND, 0.6);
        //extend, score, retract
        autoUtils.scorePiece(2);
        //position arm up
        autoUtils.armFlip(Utils.ArmFlipState.UP, 0.6);
        //PARK
        drive.followTrajectory(trajectoryHandler.splineToPark());
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