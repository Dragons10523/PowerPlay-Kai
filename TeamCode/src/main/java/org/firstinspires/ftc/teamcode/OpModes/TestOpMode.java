package org.firstinspires.ftc.teamcode.OpModes;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.Camera.Limelight;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utils;

import java.util.Arrays;
import java.util.List;


@TeleOp
public class TestOpMode extends AutoControl {
    ElapsedTime time = new ElapsedTime();
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
        super.initialHeading(Math.toRadians(0), true);
        double startTime = time.seconds();
        do {
            sleep(20);
            telemetry.addLine("isLooping");
            telemetry.addData("SuccessfulLocalizationCount", autoUtils.getSuccessfulLocalizationCount());
            autoUtils.updateOpticalSensorToPoseEstimateCamera();
            telemetry.update();
            if (startTime + 5 < time.seconds() || isStarted()) {
                break;
            }
        }
        while (autoUtils.getSuccessfulLocalizationCount() < 20);
        SparkFunOTOS.Pose2D pos = robot.opticalSensor.getPosition();

        Pose2d startPos = new Pose2d(pos.x, pos.y, pos.h);

        waitForStart();

        drive.followTrajectory(drive.trajectoryBuilder(startPos).splineTo(new Vector2d(pos.x + 10, pos.y - 10), Math.toRadians(270)).build());
    }
}
