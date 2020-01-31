package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "ServoTest")
public class ServoTourqueTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ServoImplEx s = hardwareMap.get(ServoImplEx.class, "leftClaw");
        waitForStart();

        s.setPosition(0.85);
        while(opModeIsActive()){

            telemetry.addData("Servo Position", "");
            telemetry.update();
        }
    }
}
