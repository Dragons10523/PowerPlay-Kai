package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Auto.AutoUtils;
import org.firstinspires.ftc.teamcode.Susbsystem.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Utils;

@Autonomous(name = "Auto_Right_1Score")
public class AutoRight1ScoreAndPark extends AutoControl {
    private final Utils.FieldSide fieldSide = Utils.FieldSide.RED_RIGHT;
    AutoUtils autoUtils;
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        SparkFunOTOS.Pose2D pos = new SparkFunOTOS.Pose2D(-14,62, Math.toRadians(180));

        robot.opticalSensor.setPosition(pos);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(new Pose2d(-14, 62, Math.toRadians(180)))
                .addTemporalMarker(0,()->{
                    Thread t1 = new Thread() {
                        public void run() {
                            autoUtils.armFlip(Utils.ArmFlipState.GROUND, 1);
                        }
                    };
                    t1.start();
                })
                .setReversed(true)
                .splineTo(new Vector2d(53, 53), Math.toRadians(45))
                .addDisplacementMarker(() -> {
                    Thread t2 = new Thread() {
                        public void run() {
                            autoUtils.verticalSlide(Utils.LiftState.HIGH);
                        }
                    };
                    t2.start();
                })
                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                    Thread t3 = new Thread() {
                        public void run() {
                            autoUtils.verticalSlide(Utils.LiftState.GROUND);
                            autoUtils.armFlip(Utils.ArmFlipState.UP, .6);
                        }
                    };
                    t3.start();
                })
                .waitSeconds(5)
                .setReversed(false)
                .splineTo(new Vector2d(35,35), Math.toRadians(270))
                .splineTo(new Vector2d(25, 13), Math.toRadians(180))
                .build();

        waitForStart();

        drive.followTrajectorySequence(traj1);

        while (!isStopRequested()) {
            //aprilTagPipeline.updateAprilTagPipeline();
            SparkFunOTOS.Pose2D pose2D = robot.opticalSensor.getPosition();
            telemetry.addLine(String.format("XYH %6.2f %6.2f %6.2f", pose2D.x, pose2D.y, pose2D.h));
            telemetry.update();
        }
    }

}