package org.firstinspires.ftc.teamcode.OpModes;


import android.annotation.SuppressLint;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.trajectorysequence.TrajectorySequence;

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
        Pose2d startPos = new Pose2d(pos.x, pos.y, pos.h);

        TrajectorySequence firstScore = trajectoryHandler.firstScore(startPos);
        TrajectorySequence moveToFirstPieceRed = trajectoryHandler.moveToFirstPieceRed(firstScore.end());
        TrajectorySequence scoreFirstPieceRed = trajectoryHandler.scoreRed(moveToFirstPieceRed.end());
        TrajectorySequence moveToSecondPiece = trajectoryHandler.moveToSecondPieceRed(scoreFirstPieceRed.end());
        TrajectorySequence scoreSecondPieceRed = trajectoryHandler.scoreRed(moveToSecondPiece.end());

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
        //retract lift, move, and grab piece
        drive.followTrajectorySequence(moveToFirstPieceRed);
        //move to score position and extend lift
        drive.followTrajectorySequence(scoreFirstPieceRed);
        robot.drivetrain.simpleDrive(0);
        //retract lift, move, and grab second piece
        drive.followTrajectorySequence(moveToSecondPiece);
        //move to scoreRed and score
        drive.followTrajectorySequence(scoreSecondPieceRed);
        //PARK
        drive.followTrajectory(trajectoryHandler.splineToParkRed());
        //position arm to touch bar for parking points

        while (!getStopRequested()) {
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
        robot.limelight.close();
    }
}