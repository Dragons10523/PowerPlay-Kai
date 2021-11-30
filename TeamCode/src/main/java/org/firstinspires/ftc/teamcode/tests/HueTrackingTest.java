package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AbstractBarcode;

@Autonomous(name = "Hue Tracking Test", group = "Testing")
public class HueTrackingTest extends AbstractBarcode {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        startOpenCV();

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addData("Frametime", ahi.camera.getPipelineTimeMs());
            telemetry.update();
        }
    }
}
