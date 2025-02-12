package org.firstinspires.ftc.teamcode.OpModes;


import android.annotation.SuppressLint;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utils;

@Autonomous(name = "Auto_Left_Blue")
public class AutoLeftBlue extends AutoControl {
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
        super.initialHeading(Math.toRadians(0), false);
        double startTime = time.seconds();
        double averagePosX = 0, totalPosX = 0;
        double averagePosY = 0, totalPosY = 0;
        SparkFunOTOS.Pose2D pos;
        do {
            sleep(20);
            double successfulLocalizationCount = autoUtils.getSuccessfulLocalizationCount();
            telemetry.addLine("isLooping");
            telemetry.addData("SuccessfulLocalizationCount", successfulLocalizationCount);
            telemetry.update();
            boolean successfulLocalization = autoUtils.updateOpticalSensorToPoseEstimateCamera();
            pos = robot.opticalSensor.getPosition();
            if(successfulLocalization){
                totalPosX += pos.x;
                totalPosY += pos.y;
            }
            if(successfulLocalizationCount != 0){
                averagePosX = totalPosX / successfulLocalizationCount;
                averagePosY = totalPosY / successfulLocalizationCount;
            }

            if (startTime + 5 < time.seconds() || isStarted()) {
                break;
            }
        }
        while (autoUtils.getSuccessfulLocalizationCount() < 20);


        Pose2d startPos = new Pose2d(averagePosX, averagePosY, pos.h);

        TrajectorySequence auto_Left_Blue = trajectoryHandler.auto_Left(Utils.FieldSide.BLUE_LEFT, startPos);

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

        drive.followTrajectorySequence(auto_Left_Blue);

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