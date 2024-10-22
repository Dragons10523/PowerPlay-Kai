package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.RobotClass;
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

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initialize();

        waitForStart();

        Trajectory traj = drive.trajectoryBuilder(new Pose2d()).splineToConstantHeading(
                new Vector2d(30, 30), 0)
                .build();
        drive.followTrajectory(traj);


    }
}