package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AbstractBarcode;

@Autonomous(name = "DriveToFreightTest", group = "Testing")
public class DriveToFreightTest extends AbstractBarcode {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeValues();
        startOpenCV();

        waitForStart();

        driveToFreight();
        /*drive(1, 1);
        runIntake(true);
        sleep(250);
        drive(0, 0);
        runIntake(false);*/
    }
}
