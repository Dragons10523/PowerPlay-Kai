package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "RIGHT_DISTANCE")
public class Distance_Test extends Localization{

    @Override
    public void runOpMode() {
        init(hardwareMap, false, Side.BLUE);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("Right", robot.distanceSensors[1].getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
