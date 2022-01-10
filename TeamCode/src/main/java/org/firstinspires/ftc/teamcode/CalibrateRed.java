package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CalibrateRed", group = "Calibration")
public class CalibrateRed extends AbstractCalibrate {
    @Override
    public void runOpMode() throws InterruptedException {
        calibrate("red");
    }
}
