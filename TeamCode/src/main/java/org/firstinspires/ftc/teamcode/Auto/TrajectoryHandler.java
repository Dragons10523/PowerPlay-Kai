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
    final double bucketScoreTime = 0.8;
    final double intakeTransitionTime = 1.6;
    final double armRetractTime = 0.9;
    final double extensionTime = 0.5;
    final double armMoveOutWayOfLiftTime = 0.2;

    public TrajectoryHandler(RobotClass robot, SampleMecanumDrive drive, AutoUtils autoUtils) {
        this.robot = robot;
        this.drive = drive;
        this.autoUtils = autoUtils;

    }
    public TrajectorySequence auto_Left(Utils.FieldSide fieldSide, Pose2d startPos){
        int inverseSide;
        double inverseHeading;
        Pose2d scorePosition;
        if(fieldSide == Utils.FieldSide.RED_LEFT){
            inverseSide = -1;
            inverseHeading = 180.0;
             scorePosition = new Pose2d(55.5 * inverseSide, 55.5 * inverseSide, Math.toRadians(225 - inverseHeading));

        }
        else{
            inverseSide = 1;
            inverseHeading = 0.0;
            scorePosition = new Pose2d(55 * inverseSide, 55 * inverseSide, Math.toRadians(225 - inverseHeading));

        }
        return drive.trajectorySequenceBuilder(startPos)
                .addTemporalMarker(0, () -> autoUtils.armExtension(Utils.ArmState.IN))
                .UNSTABLE_addTemporalMarkerOffset(1, ()->{
                    Thread t1 = new Thread(){
                        public void run(){
                            autoUtils.armFlip(Utils.ArmFlipState.GROUND, 0.6);
                        }
                    };
                    Thread t2 = new Thread(){
                        public void run(){
                            double startTime = time.seconds();
                            while(startTime + armMoveOutWayOfLiftTime > time.seconds()){
                                boolean isWaiting = true;
                            }
                            autoUtils.verticalSlide(Utils.LiftState.HIGH);
                        }
                    };
                    t1.start();
                    t2.start();
                })
                .splineToSplineHeading(scorePosition, Math.toRadians(90 - inverseHeading))
                .addTemporalMarker(()-> autoUtils.scorePiece(bucketScoreTime))
                .waitSeconds(bucketScoreTime)
                .addTemporalMarker(() -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.verticalSlide(Utils.LiftState.GROUND);
                        }
                    };
                    t1.start();
                })
                .splineToLinearHeading(new Pose2d(47.3 * inverseSide, 46 * inverseSide, Math.toRadians(270 - inverseHeading)), Math.toRadians(270 - inverseHeading))
                .addDisplacementMarker(() -> {
                    robot.Servos.get(RobotClass.SERVOS.INTAKE_SERVO).setPosition(.4);
                    autoUtils.armExtension(Utils.ArmState.EXTENDED);
                    Thread t1 = new Thread(){
                        public void run(){
                            double startTime = time.seconds();
                            while(startTime + 0.6 > time.seconds()){
                                robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-.75);
                            }
                            startTime = time.seconds();
                            while(startTime + 0.2 > time.seconds()){
                                robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(.3);
                            }
                            robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-.75);
                        }
                    };
                    t1.start();

                })
                .forward(1)
                .UNSTABLE_addTemporalMarkerOffset(extensionTime, () -> autoUtils.armExtension(Utils.ArmState.IN)) //perform intake transition
                .waitSeconds(armRetractTime)
                .addTemporalMarker(() -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.intakeTransition();
                        }
                    };
                    t1.start();
                })
                .waitSeconds(intakeTransitionTime)
                .addTemporalMarker(() -> {
                    autoUtils.armExtension(Utils.ArmState.IN);
                    Thread t1 = new Thread(){
                        public void run(){
                            autoUtils.armFlip(Utils.ArmFlipState.GROUND, 0.6);
                        }
                    };
                    Thread t2 = new Thread(){
                        public void run(){
                            double startTime = time.seconds();
                            while(startTime + armMoveOutWayOfLiftTime > time.seconds()){
                                boolean isWaiting = true;
                            }
                            autoUtils.verticalSlide(Utils.LiftState.HIGH);
                        }
                    };
                    t1.start();
                    t2.start();
                })
                .lineToLinearHeading(scorePosition)
                .addTemporalMarker(() -> autoUtils.scorePiece(bucketScoreTime))
                .waitSeconds(bucketScoreTime)
                .addTemporalMarker(()->{
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.verticalSlide(Utils.LiftState.GROUND);
                        }
                    };
                    t1.start();
                })
                .splineToLinearHeading(new Pose2d(57.3 * inverseSide, 46 * inverseSide, Math.toRadians(270 - inverseHeading)), Math.toRadians(270 - inverseHeading))
                .addDisplacementMarker(() -> {
                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-.75);
                    robot.Servos.get(RobotClass.SERVOS.INTAKE_SERVO).setPosition(.4);
                    autoUtils.armExtension(Utils.ArmState.EXTENDED);

                })
                .forward(1)
                .UNSTABLE_addTemporalMarkerOffset(extensionTime, () -> autoUtils.armExtension(Utils.ArmState.IN)) //perform intake transition
                .waitSeconds(armRetractTime)
                .addTemporalMarker(() -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.intakeTransition();
                        }
                    };
                    t1.start();
                })
                .waitSeconds(intakeTransitionTime)
                .addTemporalMarker(() -> {
                    autoUtils.armExtension(Utils.ArmState.IN);
                    Thread t1 = new Thread(){
                        public void run(){
                            autoUtils.armFlip(Utils.ArmFlipState.GROUND, 0.6);
                        }
                    };
                    Thread t2 = new Thread(){
                        public void run(){
                            double startTime = time.seconds();
                            while(startTime + armMoveOutWayOfLiftTime > time.seconds()){
                                boolean isWaiting = true;
                            }
                            autoUtils.verticalSlide(Utils.LiftState.HIGH);
                        }
                    };
                    t1.start();
                    t2.start();
                })
                .lineToLinearHeading(scorePosition)
                .addTemporalMarker(() -> autoUtils.scorePiece(bucketScoreTime))
                .waitSeconds(bucketScoreTime)
                .addTemporalMarker(() -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.verticalSlide(Utils.LiftState.GROUND);
                        }
                    };
                    t1.start();
                    autoUtils.armExtension(Utils.ArmState.IN);
                })
                .splineToLinearHeading(new Pose2d(56 * inverseSide,43 * inverseSide, Math.toRadians(305 - inverseHeading)), Math.toRadians(0 - inverseHeading))
                .addDisplacementMarker(() -> {
                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-.75);
                    robot.Servos.get(RobotClass.SERVOS.INTAKE_SERVO).setPosition(.4);
                    autoUtils.armExtension(Utils.ArmState.EXTENDED);

                })
                .back(1)
                .UNSTABLE_addTemporalMarkerOffset(extensionTime, () -> autoUtils.armExtension(Utils.ArmState.IN)) //perform intake transition
                .waitSeconds(armRetractTime)
                .addTemporalMarker(() -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.intakeTransition();
                        }
                    };
                    t1.start();
                })
                .waitSeconds(intakeTransitionTime)
                .addTemporalMarker(() -> {
                    autoUtils.armExtension(Utils.ArmState.IN);
                    Thread t1 = new Thread(){
                        public void run(){
                            autoUtils.armFlip(Utils.ArmFlipState.GROUND, 0.6);
                        }
                    };
                    Thread t2 = new Thread(){
                        public void run(){
                            autoUtils.verticalSlide(Utils.LiftState.HIGH);
                        }
                    };
                    t1.start();
                    double startTime = time.seconds();
                    while(startTime + armMoveOutWayOfLiftTime > time.seconds()){
                        boolean isWaiting = true;
                    }
                    t2.start();
                })
                .lineToLinearHeading(scorePosition)
                .addTemporalMarker(() -> {
                    autoUtils.scorePiece(bucketScoreTime);
                })
                .waitSeconds(bucketScoreTime)
                .addTemporalMarker(() -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.verticalSlide(Utils.LiftState.GROUND);
                            autoUtils.armFlip(Utils.ArmFlipState.UP, 0.6);
                        }
                    };
                    t1.start();
                    autoUtils.armExtension(Utils.ArmState.IN);
                })
                .splineToSplineHeading(new Pose2d(40 * inverseSide, 20 * inverseSide, Math.toRadians(180 + inverseHeading)), Math.toRadians(90 + inverseHeading))
                .splineToSplineHeading(new Pose2d(20 * inverseSide, 10 * inverseSide, Math.toRadians(180 + inverseHeading)), Math.toRadians(0 + inverseHeading))

                .build();
    }


    public TrajectorySequence moveToFirstPieceRed(Pose2d pos) {
        return drive.trajectorySequenceBuilder(pos)
                .addTemporalMarker(0, () -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.verticalSlide(Utils.LiftState.GROUND);
                        }
                    };
                    t1.start();
                })
                .splineToLinearHeading(new Pose2d(-47.5, -47, Math.toRadians(90)), Math.toRadians(90))
                .addDisplacementMarker(() -> {
                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-.75);
                    robot.Servos.get(RobotClass.SERVOS.INTAKE_SERVO).setPosition(.4);
                    autoUtils.armExtension(Utils.ArmState.EXTENDED);

                })
                .forward(1)
                .UNSTABLE_addTemporalMarkerOffset(extensionTime, () -> autoUtils.armExtension(Utils.ArmState.IN))
                .waitSeconds(armRetractTime)
                .addTemporalMarker(() -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.intakeTransition();
                        }
                    };
                    t1.start();
                })
                .waitSeconds(intakeTransitionTime)
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

    public TrajectorySequence scoreFirstPieceRed(Pose2d pos) {
        return drive.trajectorySequenceBuilder(pos)
                .addTemporalMarker(0, () -> {
                    autoUtils.armExtension(Utils.ArmState.IN);
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.armFlip(Utils.ArmFlipState.GROUND, 0.6);
                            autoUtils.verticalSlide(Utils.LiftState.HIGH);
                        }
                    };
                    t1.start();
                })
                .lineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)))
                .addTemporalMarker(() -> {
                    autoUtils.scorePiece(bucketScoreTime);
                })
                .waitSeconds(bucketScoreTime)
                .build();
    }
    public TrajectorySequence scoreSecondPieceRed(Pose2d pos) {
        return drive.trajectorySequenceBuilder(pos)
                .addTemporalMarker(0, () -> {
                    autoUtils.armExtension(Utils.ArmState.IN);
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.armFlip(Utils.ArmFlipState.GROUND, 0.6);
                            autoUtils.verticalSlide(Utils.LiftState.HIGH);
                        }
                    };
                    t1.start();
                })
                .lineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(45)))
                .addDisplacementMarker(() -> {
                    autoUtils.scorePiece(bucketScoreTime);
                })
                .waitSeconds(bucketScoreTime)
                .build();
    }

    public TrajectorySequence scoreBlue() {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectorySequenceBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .addTemporalMarker(1, () -> {
                    autoUtils.armExtension(Utils.ArmState.IN);
                })
                .setReversed(true)
                .splineTo(new Vector2d(54, 54), Math.toRadians(45))
                .setReversed(false)
                .build();
    }
    public TrajectorySequence moveToSecondPieceRedFix(Pose2d pos){
        return drive.trajectorySequenceBuilder(pos)
                .addTemporalMarker(()->{
                    autoUtils.verticalSlide(Utils.LiftState.GROUND);
                })
                .lineToLinearHeading(new Pose2d(-57.5, -50, Math.toRadians(90)))
                .build();
    }

    public TrajectorySequence secondPieceGrabRed(Pose2d pos) {
        return drive.trajectorySequenceBuilder(pos)
                .addTemporalMarker(()->{
                    autoUtils.verticalSlide(Utils.LiftState.GROUND);
                })
                .lineToLinearHeading(new Pose2d(-57.5, -50, Math.toRadians(90)))
                .addTemporalMarker( () -> {
                    robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-.75);
                    robot.Servos.get(RobotClass.SERVOS.INTAKE_SERVO).setPosition(.4);
                    autoUtils.armExtension(Utils.ArmState.EXTENDED);
                })
                .waitSeconds(1)
                .forward(1)
                .UNSTABLE_addTemporalMarkerOffset(extensionTime, () -> {
                    autoUtils.armExtension(Utils.ArmState.IN);

                }) //perform intake transition
                .waitSeconds(armRetractTime)
                .addTemporalMarker(() -> {
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.intakeTransition();
                        }
                    };
                    t1.start();
                })
                .waitSeconds(intakeTransitionTime)
                .build();
    }

    public TrajectorySequence moveToSecondPieceBlue() {
        SparkFunOTOS.Pose2D currentPos = robot.opticalSensor.getPosition();
        return drive.trajectorySequenceBuilder(new Pose2d(currentPos.x, currentPos.y, currentPos.h))
                .splineToLinearHeading(new Pose2d(57, 47, Math.toRadians(270)), Math.toRadians(270))
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
                    autoUtils.armExtension(Utils.ArmState.IN);
                    Thread t1 = new Thread() {
                        public void run() {
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