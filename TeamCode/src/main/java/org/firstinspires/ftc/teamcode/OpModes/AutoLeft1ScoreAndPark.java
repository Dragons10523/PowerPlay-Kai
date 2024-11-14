package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utils;

@Autonomous(name = "Auto_Red_Left")
public class AutoLeft1ScoreAndPark extends AutoControl {

    private final Utils.FieldSide fieldSide = Utils.FieldSide.RED_LEFT;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        super.initialize();

        double bucketScoreTime = 2;

        SparkFunOTOS.Pose2D pos = new SparkFunOTOS.Pose2D(33, 62, Math.toRadians(270));

        Pose2d scorePosition = new Pose2d(54, 53, Math.toRadians(225));

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
                .splineToLinearHeading(scorePosition, Math.toRadians(225))
                .back(.5)
                .addTemporalMarker(2, () -> {
                    Thread t2 = new Thread() {
                        public void run() {
                            autoUtils.verticalSlide(Utils.LiftState.HIGH);
                        }
                    };
                    t2.start();
                })//extend vertical slides and score
                .waitSeconds(bucketScoreTime)
                .addDisplacementMarker(() -> {
                    Thread t3 = new Thread() {
                        public void run() {
                            autoUtils.armFlip(Utils.ArmFlipState.UP, .6);
                        }
                    };
                    t3.start();
                })
                .splineTo(new Vector2d(36, 35), Math.toRadians(270))
                .splineTo(new Vector2d(25, 13), Math.toRadians(180))
                .build();
        waitForStart();

        drive.followTrajectorySequence(firstScore);

        while (!isStopRequested()) {
            //aprilTagPipeline.updateAprilTagPipeline();
            SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();
            telemetry.addLine(String.format("XYH %6.2f %6.2f %6.2f", pose2D.x, pose2D.y, pose2D.h));
            telemetry.update();
        }

    }

}