package org.firstinspires.ftc.teamcode.OpModes;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoController;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utils;


@TeleOp
public class TestOpMode extends AutoControl {
    private final Utils.FieldSide fieldSide = Utils.FieldSide.BLUE_LEFT;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        super.initialize();

        while(!isStarted()){
            SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();
            telemetry.addLine(String.format("XYH %.3f %.3f %.3f", pose2D.x, pose2D.y, pose2D.h));
            telemetry.update();
        }
        waitForStart();

        autoUtils.armFlip(Utils.ArmFlipState.MIDDLE, .8);

        //drive.followTrajectorySequence(trajSeq);
//        while(opModeIsActive()){
//            SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();
//            telemetry.addLine(String.format("XYH %.3f %.3f %.3f", pose2D.x, pose2D.y, pose2D.h));
//            telemetry.update();
//        }
    }


}
