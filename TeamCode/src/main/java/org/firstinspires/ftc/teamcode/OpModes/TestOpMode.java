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
import com.qualcomm.robotcore.hardware.ServoController;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.Camera.Limelight;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utils;

import java.util.Arrays;
import java.util.List;


@TeleOp
public class TestOpMode extends AutoControl {
    private final Utils.FieldSide fieldSide = Utils.FieldSide.BLUE_LEFT;

    @SuppressLint("DefaultLocale")
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        super.initialize();

        robot.limelight.pipelineSwitch(0);
        robot.limelight.start();

        while(!isStarted()){
            LLStatus status = robot.limelight.getStatus();
            telemetry.addData("Name", "%s", status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = robot.limelight.getLatestResult();
            if(result != null){
                Pose3D botpose = result.getBotpose_MT2();
                botpose.getPosition().toUnit(DistanceUnit.INCH);


                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);
                if(result.isValid()){
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("MT2Stddev", Arrays.toString(result.getStddevMt2()));
                    telemetry.addData("Botpose IN", "XYZ: %.3f %.3f %.3f", botpose.getPosition().toUnit(DistanceUnit.INCH).x, botpose.getPosition().toUnit(DistanceUnit.INCH).y, botpose.getPosition().z);
                    List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

                    for(LLResultTypes.FiducialResult fr : fiducialResults){
                        telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                    }
                }
                else{
                    telemetry.addData("Limelight", "No data available");
                }
            }
            SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();
            telemetry.addLine(String.format("XYH %.3f %.3f %.3f", pose2D.x, pose2D.y, pose2D.h));
            telemetry.update();
        }
        waitForStart();
        SparkFunOTOS.Pose2D pos = new SparkFunOTOS.Pose2D(0, 0, Math.toRadians(90));
        robot.opticalSensor.setPosition(pos);
        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(0,0,Math.toRadians(90)))
                .addTemporalMarker(0,()->{
                    Thread t1 = new Thread() {
                        public void run() {
                            while(!isStopRequested()){
                                SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();
                                telemetry.addLine(String.format("XYH %.3f %.3f %.3f", pose2D.x, pose2D.y, pose2D.h));
                                telemetry.update();
                            }
                        }
                    };
                    t1.start();
                })
                .lineToConstantHeading(new Vector2d(20,0))
                .waitSeconds(5)
                .lineToConstantHeading(new Vector2d(0,0))
                .build();

        drive.followTrajectorySequence(traj);

        while(!isStopRequested()){
            SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();
            telemetry.addLine(String.format("XYH %.3f %.3f %.3f", pose2D.x, pose2D.y, pose2D.h));
            telemetry.update();
        }
        //drive.followTrajectorySequence(trajSeq);
//        while(opModeIsActive()){
//            SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();
//            telemetry.addLine(String.format("XYH %.3f %.3f %.3f", pose2D.x, pose2D.y, pose2D.h));
//            telemetry.update();
//        }
    }


}
