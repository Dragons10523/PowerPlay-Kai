package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
@TeleOp(name="IMUTEST")
public class IMUTest extends OpMode {

    BNO055IMU imu;
    double angleError = 0;
    @Override
    public void init() {
        HardwareConfig config = new HardwareConfig(hardwareMap);

        config.initializeIMU();
        imu = config.imu;
        angleError = -imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES).firstAngle;
    }

    @Override
    public void loop() {
        telemetry.addData("Angle:", getAngle());
        telemetry.update();
    }
    double getAngle(){
        double ang = imu.getAngularOrientation(AxesReference.INTRINSIC, ZYX, AngleUnit.DEGREES).firstAngle + angleError;
        double newangle = 0;

        if(ang <= 0 && ang > -90){
            newangle = 90-Math.abs(ang);
        }
        else if(ang < -90){
            newangle = 450 - Math.abs(ang);
        }
        else if(ang > 0){
            newangle = 90 + Math.abs(ang);
        }

        return newangle;
    }
}
