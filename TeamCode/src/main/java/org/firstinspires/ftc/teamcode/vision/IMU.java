package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import java.util.Arrays;

public interface IMU extends com.qualcomm.robotcore.hardware.IMU {
    class Parameters extends com.qualcomm.robotcore.hardware.IMU.Parameters {
        public Parameters(ImuOrientationOnRobot imuOrientationOnRobot, OpMode opMode, Byte[] byteData) {
            super(imuOrientationOnRobot);
            opMode.telemetry.addAction(() -> opMode.telemetry.addLine(new String(byteData)));
        }
    }
}
