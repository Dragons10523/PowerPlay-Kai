package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.AbstractBarcode;

@Autonomous(name = "DriveToHubTest", group = "Testing")
public class DriveToHubTest extends AbstractBarcode {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeValues();
        startOpenCV("red");

        waitForStart();

        ahi.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        ahi.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveToShippingHub(FieldSide.RED);
        /*drive(1, 1);
        runIntake(1);
        sleep(250);
        drive(0, 0);
        runIntake(0);*/
    }
}
