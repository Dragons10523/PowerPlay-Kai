package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

public class Kai {
    public DcMotor frontLeft, frontRight, backLeft, backRight;
    public DcMotor intake;

    public BNO055IMU imu;

    OpenCvWebcam frontCamera;

    public HardwareMap hwmap;

    public Kai(HardwareMap hwmap) {
        this.hwmap = hwmap;

        frontLeft = hwmap.get(DcMotor.class, "frontLeft");
        frontRight = hwmap.get(DcMotor.class, "frontRight");
        backLeft = hwmap.get(DcMotor.class, "backLeft");
        backRight = hwmap.get(DcMotor.class, "backRight");

        intake = hwmap.get(DcMotor.class, "intake");

        BNO055IMU.Parameters parameters             = new BNO055IMU.Parameters();
        parameters.angleUnit                        = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit                        = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile              = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled                   = false;
        parameters.loggingTag                       = "IMU";

        imu = hwmap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        WebcamName frontWebcamName = hwmap.get(WebcamName.class, "Front Camera");
        frontCamera = OpenCvCameraFactory.getInstance().createWebcam(frontWebcamName);
    }
}
