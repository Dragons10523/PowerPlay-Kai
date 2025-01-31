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
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        super.initialize();
        super.initialHeading(Math.toRadians(180), false);

        ElapsedTime time = new ElapsedTime();
        waitForStart();
        int didCameraUpdate = 0;
        double range = 0;
        double[] stdDevMt2 = {0, 0};
        Position pose2D = new Position();
        SparkFunOTOS.Pose2D pos_Sensor;
        while (opModeIsActive()) {
            LLResult result = robot.limelight.getLatestResult();
            pos_Sensor = robot.opticalSensor.getPosition();
            robot.limelight.updateRobotOrientation(Math.toDegrees(robot.getHeading()));

            if (result != null && result.isValid()) {
                pose2D = result.getBotpose_MT2().getPosition();
                stdDevMt2 = result.getStddevMt2();

                if (stdDevMt2[0] * 39.37 + stdDevMt2[1] * 39.37 < 1.0) {
                    didCameraUpdate++;
                }
            } else {
                telemetry.addLine("result is not valid");
            }

            telemetry.addData("range", range);
            telemetry.addData("stdDevMt2", stdDevMt2[0] * 39.37 + stdDevMt2[1] * 39.37);
            telemetry.addData("didCameraUpdate", didCameraUpdate);
            telemetry.addData("cameraPos", "xy %.3f %.3f", pose2D.toUnit(DistanceUnit.INCH).x, pose2D.toUnit(DistanceUnit.INCH).y);
            telemetry.addData("sensorPos", "xyh %.3f %.3f %.3f", pos_Sensor.x, pos_Sensor.y, pos_Sensor.h);
            telemetry.update();

        }
    }
}
