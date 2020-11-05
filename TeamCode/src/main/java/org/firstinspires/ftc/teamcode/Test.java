package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test")
public class Test extends Localization {
    @Override
    public void runOpMode() throws InterruptedException {
        alize();
        waitForStart();
        startLocalization();
        while (opModeIsActive()) {
            telemetry.addData("Position", updatePosition());
            telemetry.update();
        }
    }
}
