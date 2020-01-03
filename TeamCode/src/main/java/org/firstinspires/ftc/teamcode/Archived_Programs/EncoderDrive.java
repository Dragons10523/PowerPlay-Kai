package org.firstinspires.ftc.teamcode.Archived_Programs;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.DriveTrain;
import org.firstinspires.ftc.teamcode.Encoders;
import org.firstinspires.ftc.teamcode.HardwareConfig;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;

@TeleOp(name="Encoder Drive Test")
@Disabled
public class EncoderDrive extends OpMode {
    private MecanumDrive funcs;
    private Encoders encode;
    private double Inches = 0;
    double angleError;
    BNO055IMU imu;
    private int goal = 24;
    @Override
    public void init() {
        HardwareConfig robot = new HardwareConfig(hardwareMap);
        DriveTrain dt = new DriveTrain(robot.frontLeft, robot.frontRight, robot.rearRight, robot.rearLeft);
        funcs = new MecanumDrive(dt);
        encode = new Encoders(dt);

        encode.resetEncoders();
        Inches = encode.getInches();
        BNO055IMU.Parameters gyrometers = new BNO055IMU.Parameters();
        gyrometers.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        gyrometers.calibrationDataFile = "BNO055IMUCalibration.json";
        gyrometers.loggingEnabled = true;
        gyrometers.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyrometers);
        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        ElapsedTime time = new ElapsedTime();
        while(time.milliseconds()<100){telemetry.update();} //Changing modes requires a delay before doing anything else

        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);

        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);

        imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        time.reset();
        while(time.milliseconds()<100){telemetry.update();} //Changing modes again requires a delay
        angleError = -imu.getAngularOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
        encode.resetEncoders();
    }
    private double getAngle(){
        return imu.getAngularOrientation(INTRINSIC, ZYX, DEGREES).firstAngle +angleError;
    }

    @Override
    public void loop() {
        if(Inches <= goal){
            Inches = encode.getInches();
            if(Inches > goal+6) {
                funcs.absMove(270,0.5, getAngle());
            }
            else {
                funcs.absMove(270, 1, getAngle());
            }
        }
        else{
            funcs.stopNow();
        }
        telemetry.addData("Target", (goal* Encoders.COUNTS_ROTATION)/(Encoders.WHEEL_DIAMETER*Math.PI));
        telemetry.addData("Current Rotations", encode.getTicks());
        telemetry.update();
    }
}
