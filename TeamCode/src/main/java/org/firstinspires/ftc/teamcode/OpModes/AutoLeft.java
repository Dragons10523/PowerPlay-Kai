package org.firstinspires.ftc.teamcode.OpModes;


import android.annotation.SuppressLint;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

@Autonomous(name = "Auto_Left")
public class AutoLeft extends AutoControl {

    private final Utils.FieldSide fieldSide = Utils.FieldSide.BLUE_LEFT;

    enum AutoState{
        FIRST_SCORE,
        MOVE_TO_BLOCK,

    }
    AutoState autoState = AutoState.FIRST_SCORE;

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        telemetry.addLine("opMode started");
        telemetry.update();
        super.initialize();
        telemetry.addLine("opMode INIT");
        telemetry.update();
        double bucketScoreTime = 1;

        SparkFunOTOS.Pose2D pos = new SparkFunOTOS.Pose2D(33, 62, Math.toRadians(270));

        Pose2d scorePosition = new Pose2d(54, 54, Math.toRadians(225));

        robot.opticalSensor.setPosition(pos);


        TrajectorySequence firstScore = drive.trajectorySequenceBuilder(new Pose2d(33, 62, Math.toRadians(270)))
                .addTemporalMarker(0, () -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.armFlip(Utils.ArmFlipState.GROUND, 1);
                        }
                    };
                    t1.start();
                })
                .addTemporalMarker(0.5, () -> {
                    Thread t2 = new Thread() {
                        public void run() {
                            autoUtils.verticalSlide(Utils.LiftState.HIGH);
                        }
                    };
                    t2.start();
                })//extend vertical slides and score
                .splineToLinearHeading(scorePosition, Math.toRadians(225))
                .strafeLeft(1)
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
        telemetry.addLine("firstScore success");
        telemetry.update();
        TrajectorySequence moveToFirstPiece = drive.trajectorySequenceBuilder(firstScore.end())
                .forward(3)
                .splineToLinearHeading(new Pose2d(35.5, 42, Math.toRadians(315)), Math.toRadians(315))
                .strafeLeft(.5)
                .addDisplacementMarker(() -> {
                    Thread t2 = new Thread() {
                        public void run() {
                            robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-.75);

                            robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(0.56);
                            robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(0.64);
                        }
                    };
                    t2.start();
                })
                .forward(1)
                .UNSTABLE_addTemporalMarkerOffset(.1,() -> {
                    Thread t3 = new Thread() {
                        public void run() {
                            robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-0.1);

                            robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(0.69);
                            robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(0.52);
                        }
                    };
                    t3.start();
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
                .waitSeconds(2)
                .build();
        telemetry.addLine("firstPiece success");
        telemetry.update();
        TrajectorySequence secondScore = drive.trajectorySequenceBuilder(moveToFirstPiece.end())
                .splineToLinearHeading(scorePosition, Math.toRadians(225))
                .back(.5)
                .addTemporalMarker(0.5,() -> {
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
        TrajectorySequence moveToSecondPiece = drive.trajectorySequenceBuilder(secondScore.end())
                .forward(3)
                .splineToLinearHeading(new Pose2d(40, 34, Math.toRadians(335)), Math.toRadians(335))
                .strafeLeft(.5)
                .addDisplacementMarker(() -> {
                    Thread t2 = new Thread() {
                        public void run() {
                            robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-.75);

                            robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(0.54);
                            robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(0.64);
                        }
                    };
                    t2.start();
                })
                .forward(1)
                .UNSTABLE_addTemporalMarkerOffset(.15,() -> {
                    Thread t3 = new Thread() {
                        public void run() {
                            robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-0.1);

                            robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(0.67);
                            robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(0.52);
                        }
                    };
                    t3.start();
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
                .waitSeconds(2)
                .build();
        telemetry.addLine("secondPiece success");
        telemetry.update();
        TrajectorySequence thirdScore = drive.trajectorySequenceBuilder(moveToSecondPiece.end())
                .splineToLinearHeading(scorePosition, Math.toRadians(225))
                .back(.5)
                .addTemporalMarker(0.5,() -> {
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
                .back(2)
                .build();
        telemetry.addLine("thirdScore success");
        telemetry.update();
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
        TrajectorySequence moveToPark = drive.trajectorySequenceBuilder(secondScore.end())
                .waitSeconds(1.5)
                .addTemporalMarker(2,()->{
                    autoUtils.armFlip(Utils.ArmFlipState.UP, .8);
                })
                .splineTo(new Vector2d(45, 13), Math.toRadians(180))
                .forward(20)
                .build();
        telemetry.addLine("park success");
        telemetry.update();
        while (!isStarted()) {
            SparkFunOTOS.Pose2D pose = robot.opticalSensor.getPosition();
            telemetry.addData("posX", pose.x);
            telemetry.addData("posY", pose.y);
            telemetry.addData("posH", pose.h);
            telemetry.update();
        }
        waitForStart();

        robot.initResetLift();

        drive.followTrajectorySequence(firstScore);
        drive.followTrajectorySequence(moveToFirstPiece);
        drive.followTrajectorySequence(secondScore);
        drive.followTrajectorySequence(moveToSecondPiece);
        drive.followTrajectorySequence(thirdScore);
//        drive.followTrajectorySequence(moveToThirdPiece);
//        drive.followTrajectorySequence(fourthScore);
//        drive.followTrajectorySequence(moveToPark);

        while (opModeIsActive()) {


            SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();

            telemetry.addLine(String.format("XYH %.3f %.3f %.3f", pose2D.x, pose2D.y, pose2D.h));

            telemetry.update();
        }
    }
}