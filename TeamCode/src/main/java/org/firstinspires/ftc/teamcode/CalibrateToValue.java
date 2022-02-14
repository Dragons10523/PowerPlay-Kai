package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;

@Disabled
@Autonomous(name = "Calibrate To Value", group = "Calibration")
public class CalibrateToValue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double[] centerColor = {81.6765, 136.87, 92.05};

        waitForStart();

        try {
            FileOutputStream fos = new FileOutputStream("/storage/emulated/0/FIRST/CalibrationData/blu.dat");
            ByteBuffer data = (ByteBuffer) ByteBuffer.allocate(8 * 3).putDouble(centerColor[0]).putDouble(centerColor[1]).putDouble(centerColor[2]).rewind();
            byte[] dataArray = new byte[8*3];
            data.get(dataArray, 0, dataArray.length);
            fos.write(dataArray);
            fos.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
