package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utils;

public class TrajectoryHandler {
    RobotClass robot;
    SampleMecanumDrive drive;
    AutoUtils autoUtils;

    public TrajectoryHandler(RobotClass robot, SampleMecanumDrive drive, AutoUtils autoUtils) {
        this.robot = robot;
        this.drive = drive;
        this.autoUtils = autoUtils;

    }

    public TrajectorySequence moveToFirstPieceRed(){
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectorySequenceBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .splineToLinearHeading(new Pose2d(-48, -47, Math.toRadians(90)), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-.75);
                    robot.Servos.get(RobotClass.SERVOS.INTAKE_SERVO).setPosition(.4);
                    autoUtils.armExtension(Utils.ArmState.EXTENDED);
                })
                .forward(1)
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    autoUtils.armExtension(Utils.ArmState.IN);

                }) //perform intake transition
                .waitSeconds(2)
                .build();
    }
    public TrajectorySequence secondScoreRed(){
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return  drive.trajectorySequenceBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .addTemporalMarker(1, () -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.armExtension(Utils.ArmState.IN);
                        }
                    };
                    t1.start();
                })
                .setReversed(true)
                .splineTo(new Vector2d(-54, -54), Math.toRadians(225))
                .setReversed(false)
                .build();
    }

    public Trajectory wiggle(){
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .forward(5)
                .build();
    }
    public Trajectory splineToLinearHeadingScoreRed() {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .splineToLinearHeading(new Pose2d(-54, -54, Math.toRadians(45)), Math.toRadians(45))
                .build();
    }

    public Trajectory splineToScoreRed() {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .splineTo(new Vector2d(-54, -54), Math.toRadians(45))
                .build();
    }

    public Trajectory lineToScoreRed() {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .lineTo(new Vector2d(-54, -54))
                .build();
    }

    public Trajectory lineToLinearHeadingScoreRed(double max_VELOCITY) {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .lineToLinearHeading(new Pose2d(-54, -54, Math.toRadians(45)),
                        SampleMecanumDrive.getVelocityConstraint(max_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
    }

    public Trajectory splineToVector(Vector2d vec, double endHeading_RADIANS) {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .splineTo(vec, endHeading_RADIANS)
                .build();
    }

    public Trajectory splineToLinearHeadingPos(Pose2d pose2d, double endHeading_RADIANS) {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .splineToLinearHeading(pose2d, endHeading_RADIANS)
                .build();
    }
}