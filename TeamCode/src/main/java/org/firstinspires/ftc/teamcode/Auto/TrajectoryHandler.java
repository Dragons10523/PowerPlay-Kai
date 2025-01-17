package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utils;

public class TrajectoryHandler {
    RobotClass robot;
    SampleMecanumDrive drive;
    AutoUtils autoUtils;
    ElapsedTime time = new ElapsedTime();

    public TrajectoryHandler(RobotClass robot, SampleMecanumDrive drive, AutoUtils autoUtils) {
        this.robot = robot;
        this.drive = drive;
        this.autoUtils = autoUtils;

    }

    public TrajectorySequence moveToFirstPieceRed() {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectorySequenceBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .addTemporalMarker(0, () -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.verticalSlide(Utils.LiftState.GROUND);
                        }
                    };
                    t1.start();
                })
                .splineToLinearHeading(new Pose2d(-48, -47, Math.toRadians(90)), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-.75);
                    robot.Servos.get(RobotClass.SERVOS.INTAKE_SERVO).setPosition(.4);
                    Thread t1 = new Thread(){
                        public void run(){
                            double startTime = time.seconds();
                            while(startTime + 1 > time.seconds()){
                                autoUtils.armExtension(Utils.ArmState.EXTENDED);
                            }
                        }
                    };
                    t1.start();

                })
                .forward(1)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    autoUtils.armExtension(Utils.ArmState.IN);

                }) //perform intake transition
                .build();
    }

    public TrajectorySequence moveToFirstPieceBlue() {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectorySequenceBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .splineToLinearHeading(new Pose2d(48, 47, Math.toRadians(270)), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-.75);
                    robot.Servos.get(RobotClass.SERVOS.INTAKE_SERVO).setPosition(.4);
                    autoUtils.armExtension(Utils.ArmState.EXTENDED);
                })
                .forward(1)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    autoUtils.armExtension(Utils.ArmState.IN);

                }) //perform intake transition
                .waitSeconds(2)
                .build();
    }

    public TrajectorySequence scoreRed() {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectorySequenceBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .addTemporalMarker(1, () -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.armExtension(Utils.ArmState.IN);
                            autoUtils.armFlip(Utils.ArmFlipState.GROUND, 0.6);
                        }
                    };
                    t1.start();
                })
                .addTemporalMarker(1.5, () -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.verticalSlide(Utils.LiftState.HIGH);
                        }
                    };
                    t1.start();
                })
                .setReversed(true)
                .splineTo(new Vector2d(-54, -54), Math.toRadians(225))
                .setReversed(false)
                .waitSeconds(1)
                .build();
    }

    public TrajectorySequence scoreBlue() {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectorySequenceBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .addTemporalMarker(1, () -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.armExtension(Utils.ArmState.IN);
                        }
                    };
                    t1.start();
                })
                .setReversed(true)
                .splineTo(new Vector2d(54, 54), Math.toRadians(45))
                .setReversed(false)
                .build();
    }

    public TrajectorySequence moveToSecondPieceRed() {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectorySequenceBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .addTemporalMarker(() -> {
                    autoUtils.verticalSlide(Utils.LiftState.GROUND);
                })
                .splineToLinearHeading(new Pose2d(-58.5, -47, Math.toRadians(90)), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-.75);
                    robot.Servos.get(RobotClass.SERVOS.INTAKE_SERVO).setPosition(.4);
                    autoUtils.armExtension(Utils.ArmState.EXTENDED);
                })
                .forward(1)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    autoUtils.armExtension(Utils.ArmState.IN);
                })
                .build();
    }

    public TrajectorySequence moveToSecondPieceBlue() {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectorySequenceBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .splineToLinearHeading(new Pose2d(58.5, 47, Math.toRadians(270)), Math.toRadians(270))
                .addDisplacementMarker(() -> {
                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-.75);
                    robot.Servos.get(RobotClass.SERVOS.INTAKE_SERVO).setPosition(.4);
                    autoUtils.armExtension(Utils.ArmState.EXTENDED);
                })
                .forward(1)
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    autoUtils.armExtension(Utils.ArmState.IN);
                })
                .waitSeconds(2)
                .build();
    }

    public Trajectory splineToParkRed() {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .addTemporalMarker(0, () -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.verticalSlide(Utils.LiftState.GROUND);
                            autoUtils.armExtension(Utils.ArmState.IN);
                            autoUtils.armFlip(Utils.ArmFlipState.UP, 0.6);
                        }
                    };
                    t1.start();
                })
                .splineTo(new Vector2d(-25, -10), 0)
                .build();
    }

    public Trajectory splineToParkBlue() {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .splineTo(new Vector2d(25, 10), Math.toRadians(180))
                .build();
    }

    public Trajectory splineToLinearHeadingScoreRed() {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .splineToLinearHeading(new Pose2d(-54, -54, Math.toRadians(45)), Math.toRadians(45))
                .build();
    }

    public Trajectory splineToLinearHeadingScoreBlue() {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .splineToLinearHeading(new Pose2d(54, 54, Math.toRadians(225)), Math.toRadians(225))
                .build();
    }

    public Trajectory splineToScoreRed() {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .splineTo(new Vector2d(-54, -54), Math.toRadians(45))
                .build();
    }

    public Trajectory splineToScoreBlue() {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .splineTo(new Vector2d(54, 54), Math.toRadians(225))
                .build();
    }

    public Trajectory lineToScoreRed() {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .lineTo(new Vector2d(-54, -54))
                .build();
    }

    public Trajectory lineToScoreBlue() {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .lineTo(new Vector2d(54, 54))
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

    public Trajectory lineToLinearHeadingScoreBlue(double max_VELOCITY) {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .lineToLinearHeading(new Pose2d(54, 54, Math.toRadians(225)),
                        SampleMecanumDrive.getVelocityConstraint(max_VELOCITY, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
    }

    public Trajectory lineToScoreRed(double max_VELOCITY) {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .lineTo(new Vector2d(-54, -54))
                .build();
    }

    public Trajectory lineToScoreBlue(double max_VELOCITY) {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .lineTo(new Vector2d(54, 54))
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