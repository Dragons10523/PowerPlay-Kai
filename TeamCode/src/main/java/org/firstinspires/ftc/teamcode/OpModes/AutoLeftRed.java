package org.firstinspires.ftc.teamcode.OpModes;


import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utils;

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
        telemetry.addLine("cameraLocalization");
        telemetry.update();
        super.cameraLocalization();
        telemetry.addLine("headingInit");
        telemetry.update();
        super.initialHeading(0, true);
        double startTime = time.seconds();
        do{
            sleep(20);
            autoUtils.updateOpticalSensorToPoseEstimateCamera();;
        }
        while(startTime + 5 < time.seconds());
        SparkFunOTOS.Pose2D pos = robot.opticalSensor.getPosition();
        double bucketScoreTime = 2;
        Pose2d scorePosition = new Pose2d(-54, -54, Math.toRadians(45));

        TrajectorySequence firstScore = drive.trajectorySequenceBuilder(new Pose2d(pos.x, pos.y, pos.h))
                .addTemporalMarker(0, () -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.armFlip(Utils.ArmFlipState.GROUND, 1);
                        }
                    };
                    t1.start();
                })
                .addTemporalMarker(2, () -> {
                    Thread t2 = new Thread() {
                        public void run() {
                            autoUtils.verticalSlide(Utils.LiftState.HIGH);
                        }
                    };
                    t2.start();
                })//extend vertical slides and score
                .splineToLinearHeading(scorePosition, Math.toRadians(45))
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
                .splineToLinearHeading(new Pose2d(-30.5, -37, Math.toRadians(155)), Math.toRadians(155))
                .addDisplacementMarker(() -> {
                    robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(0.56);
                    robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(0.64);

                })
                .forward(1)
                .UNSTABLE_addTemporalMarkerOffset(.1, () -> {
                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-.75);
                })
                .UNSTABLE_addTemporalMarkerOffset(.35, () -> {

                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(0);

                    robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(0.69);
                    robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(0.52);

                }) //perform intake transition
                .waitSeconds(.2)
                .addTemporalMarker(() -> {
                    Thread t4 = new Thread() {
                        public void run() {
                            autoUtils.intakeTransition();
                        }
                    };
                    t4.start();
                })
                .waitSeconds(1)
                .build();
        telemetry.addLine("firstPiece success");
        telemetry.update();
        TrajectorySequence secondScore = drive.trajectorySequenceBuilder(moveToFirstPiece.end())
                .splineToLinearHeading(scorePosition, Math.toRadians(45))
                .addTemporalMarker(0.5, () -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.verticalSlide(Utils.LiftState.HIGH);
                        }
                    };
                    t1.start();
                }) //extend vertical slides
                .waitSeconds(bucketScoreTime)
                .addTemporalMarker(() -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.verticalSlide(Utils.LiftState.GROUND);
                        }
                    };
                    t1.start();
                }) //retract vertical slides
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
        waitForStart();

        drive.followTrajectorySequence(firstScore);
        double displacementFromTarget = opticalSensorClass.getDisplacementFromTarget(scorePosition.getX(), scorePosition.getY());
        if(displacementFromTarget > 3){
            SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
            Trajectory correctionTraj = drive.trajectoryBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                    .lineToConstantHeading(new Vector2d(scorePosition.getX(), scorePosition.getY()))
                    .build();
            drive.followTrajectory(correctionTraj);
        }
        autoUtils.scorePiece(2);
//        drive.followTrajectorySequence(moveToFirstPiece);
//        drive.followTrajectorySequence(secondScore);
//        drive.followTrajectorySequence(moveToSecondPiece);
//        drive.followTrajectorySequence(thirdScore); // and park
//        drive.followTrajectorySequence(moveToThirdPiece);
//        drive.followTrajectorySequence(fourthScore);
        //drive.followTrajectorySequence(moveToPark);
        while(!getStopRequested()){

        }
    }
}