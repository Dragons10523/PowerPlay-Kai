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
                            autoUtils.armFlip(Utils.ArmFlipState.GROUND, 1);
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
        TrajectorySequence moveToFirstPiece = drive.trajectorySequenceBuilder(firstScore.end())
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
                    autoUtils.armExtension(Utils.ArmState.EXTENDED);
                })

                .forward(1)
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    autoUtils.armExtension(Utils.ArmState.IN);

                }) //perform intake transition
                .waitSeconds(2)
                .build();
        telemetry.addLine("firstPiece success");
        telemetry.update();
        TrajectorySequence secondScore = drive.trajectorySequenceBuilder(moveToFirstPiece.end())
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
        telemetry.addLine("secondScore success");
        telemetry.update();
//        TrajectorySequence moveToSecondPiece = drive.trajectorySequenceBuilder(secondScore.end())
//                .forward(3)
//                .splineToLinearHeading(new Pose2d(40, 34, Math.toRadians(335)), Math.toRadians(335))
//                .strafeLeft(.5)
//                .addDisplacementMarker(() -> {
//
//                    robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(0.54);
//                    robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(0.64);
//
//                })
//                .forward(1)
//                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
//                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-.75);
//                })
//                .UNSTABLE_addTemporalMarkerOffset(.25, () -> {
//                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(0);
//
//                    robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(0.67);
//                    robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(0.52);
//
//                }) //perform intake transition
//                .waitSeconds(.2)
//                .addTemporalMarker(() -> {
//                    Thread t4 = new Thread() {
//                        public void run() {
//                            autoUtils.intakeTransition();
//                        }
//                    };
//                    t4.start();
//                })
//                .waitSeconds(1)
//                .build();
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


        drive.followTrajectorySequence(firstScore);
        autoUtils.armExtension(Utils.ArmState.IN);
        double displacementFromTarget = opticalSensorClass.getDisplacementFromTarget(scorePosition.getX(), scorePosition.getY());
        Trajectory correctionTraj = null;
        if (displacementFromTarget > 3) {
            SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
            correctionTraj = drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                    .lineToLinearHeading(new Pose2d(scorePosition.getX(), scorePosition.getY(), Math.toRadians(45)),
                            SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                            SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .build();
            drive.followTrajectory(correctionTraj);
        }

        autoUtils.scorePiece(2);

        if (Objects.isNull(correctionTraj)) {
            drive.followTrajectorySequence(moveToFirstPiece);
        } else {
            assert correctionTraj != null;
            TrajectorySequence moveToFirstPieceCorrected = drive.trajectorySequenceBuilder(correctionTraj.end())
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
                        autoUtils.armExtension(Utils.ArmState.EXTENDED);
                    })
                    .forward(1)
                    .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                        autoUtils.armExtension(Utils.ArmState.IN);

                    }) //perform intake transition
                    .waitSeconds(2)
                    .build();
            drive.followTrajectorySequence(moveToFirstPieceCorrected);
        }
        autoUtils.intakeTransition();
        drive.followTrajectorySequence(secondScore);

        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        correctionTraj = drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .lineToLinearHeading(new Pose2d(scorePosition.getX(), scorePosition.getY(), Math.toRadians(45)),
                        SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        drive.followTrajectory(correctionTraj);

        autoUtils.armFlip(Utils.ArmFlipState.GROUND, 1);
        autoUtils.scorePiece(2);
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