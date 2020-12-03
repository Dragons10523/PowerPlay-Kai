package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Test")
public class Test extends Control {
    @Override
    public void runOpMode() throws InterruptedException {
        alize();
        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("X",thalatte.mouse.x);
            telemetry.addData("Y",thalatte.mouse.y);
            telemetry.update();
        }
    }
}
