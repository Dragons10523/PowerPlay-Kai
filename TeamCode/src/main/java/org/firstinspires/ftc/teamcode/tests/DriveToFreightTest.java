package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AbstractBarcode;

@Autonomous(name = "DriveToFreightTest", group = "Testing")
public class DriveToFreightTest extends AbstractBarcode {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeValues();
        startOpenCV();

        waitForStart();

        ahi.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ahi.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hueTrackingPipeline.startVideo();
        driveToFreight();
        drive(0, 0);
        sleep(500);
        hueTrackingPipeline.stopVideo();
        /*drive(1, 1);
        runIntake(1);
        sleep(250);
        drive(0, 0);
        runIntake(0);*/
    }
}
