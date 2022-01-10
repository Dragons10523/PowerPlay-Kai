package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "CalibrateFreight", group = "Calibration")
public class CalibrateFreight extends AbstractCalibrate {
    @Override
    public void runOpMode() throws InterruptedException {
        calibrate("fr8");
    }
}
