package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CalibrateBlue", group = "Calibration")
public class CalibrateBlue extends AbstractCalibrate {
    @Override
    public void runOpMode() throws InterruptedException {
        calibrate("blu");
    }
}
