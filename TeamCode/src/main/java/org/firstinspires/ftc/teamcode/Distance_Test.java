package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "RAW distance output")
public class Distance_Test extends Localization{

    @Override
    public void runOpMode() {
        init(hardwareMap, false, Side.BLUE);
        robot.initializeDistanceSensors();
        robot.initializeCameraDistance();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Left: ", robot.distanceSensors[0].getDistance(DistanceUnit.INCH));
            telemetry.addData("Right", robot.distanceSensors[1].getDistance(DistanceUnit.INCH));
            telemetry.addData("Front: ", robot.distanceSensors[3].getDistance(DistanceUnit.INCH));
            telemetry.addData("Camera: ", robot.cameraDis.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
