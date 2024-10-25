package org.firstinspires.ftc.teamcode.OpModes;


import android.annotation.SuppressLint;

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

@Autonomous(name = "Auto_Blue_Left")
public class AutoControlBlueLeft extends AutoControl {

    private final Utils.FieldSide fieldSide = Utils.FieldSide.BLUE_LEFT;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        super.initialize();

        waitForStart();

//        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d()).lineTo(
//                        new Vector2d(60,0),
//                        SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .build();

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(new Pose2d())
                .lineTo(new Vector2d(70,0))
                .splineToConstantHeading(new Vector2d(70,-40),0)
                .build();
        TrajectorySequence trajSeqRev = drive.trajectorySequenceBuilder(trajSeq.end())
                .splineToConstantHeading(new Vector2d(75,0),0)
                .lineTo(new Vector2d(0,0))
                .build();

        drive.followTrajectorySequence(trajSeq);
        sleep(2000);
        drive.followTrajectorySequence(trajSeqRev);






        while(opModeIsActive()){

            SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();

            telemetry.addLine(String.format("XYH %.3f %.3f %.3f", pose2D.x, pose2D.y, pose2D.h));

            telemetry.update();
        }
    }
}