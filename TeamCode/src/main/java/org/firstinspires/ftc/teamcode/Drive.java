package org.firstinspires.ftc.teamcode;

import android.os.AsyncTask;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.OOPO.Relativity;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;

@TeleOp(name = "TeleOP")
public class Drive extends OpMode {

    private ProtoConfig robot;
    private OOPO funcs;
    private static BNO055IMU imu;
    private static Relativity relative = Relativity.FIELD;
    private static double speed = 1;
    private static Gamepad gp1;
    private static Gamepad gp2;
    private AsyncTask lol;
    private static double angleError;
    private Servo blockintake, leftClaw, rightClaw;

    @Override
    public void init() {
        robot = new ProtoConfig();
        robot.init(hardwareMap);
        funcs = new OOPO(robot.frontLeft, robot.frontRight, robot.rearRight, robot.rearLeft);
        blockintake = robot.blockIntake;
        leftClaw = robot.leftClaw;
        rightClaw = robot.rightClaw;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        byte AXIS_MAP_CONFIG_BYTE = 0x6; //This is what to write to the AXIS_MAP_CONFIG register to swap x and z axes
        byte AXIS_MAP_SIGN_BYTE = 0x1; //This is what to write to the AXIS_MAP_SIGN register to negate the z axis

        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

        ElapsedTime time = new ElapsedTime();
        while(time.milliseconds()<100){telemetry.update();}
        imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG,AXIS_MAP_CONFIG_BYTE & 0x0F);
        imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN,AXIS_MAP_SIGN_BYTE & 0x0F);
        imu.write8(BNO055IMU.Register.OPR_MODE,BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        time.reset();
        while(time.milliseconds()<100){telemetry.update();}

        angleError = -imu.getAngularOrientation(INTRINSIC,ZYX,DEGREES).firstAngle;
        gp1 = gamepad1;
        gp2 = gamepad2;
        lol = new buttonUpdates(parameters).execute();
    }
    @Override
    public void loop() {
        if (relative == Relativity.FIELD) {
            telemetry.addData("Relativity", "FIELD");
            double a = getAngle();
            telemetry.addData("Angle", a);
            funcs.absoluteMove(gamepad1.left_stick_y * speed, gamepad1.left_stick_x * speed, Math.toRadians(a), -gamepad1.right_stick_x * speed);
        } else {
            telemetry.addData("Relativity", "ROBOT");
            funcs.joystickMove(gamepad1.left_stick_y * speed, gamepad1.left_stick_x * speed, -gamepad1.right_stick_x * speed);
        }

        robot.ramp1.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        robot.ramp2.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        robot.intake1.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        robot.intake2.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        robot.lift.setPower(gamepad2.left_stick_y);
        robot.horizontal.setPower(gamepad2.right_stick_y);

        telemetry.addData("ServoLeft", robot.leftClaw.getPosition());
        telemetry.addData("ServoRight", robot.rightClaw.getPosition());
        telemetry.addData("Speed", (speed * 100) + "%");
        telemetry.update();
    }
    @Override
    public void stop(){
        lol.cancel(true);
    }
    private double getAngle(){
        return imu.getAngularOrientation(INTRINSIC, ZYX, DEGREES).firstAngle +angleError;
    }
    class buttonUpdates extends AsyncTask<Void, Void, Void>{
        BNO055IMU.Parameters parameters;
        buttonUpdates(BNO055IMU.Parameters para){
            parameters = para;
        }
        @Override
        protected Void doInBackground(Void ... voids) {
            boolean lastX = false;
            boolean lastY = false;
            boolean lastLeftStick = false;
            boolean lastA = false;
            boolean lastLeftBump = false;
            boolean lastRightBump = false;
            Gamepad gamepad1 = Drive.gp1;
            Gamepad gamepad2 = Drive.gp2;
            while(true){
                if(!gamepad1.x && lastX){
                    lastX =false;
                    if (Drive.speed == 1) {
                        Drive.speed = 0.5;
                    } else {
                        Drive.speed = 1;
                    }
                }
                else if(gamepad1.x){
                    lastX = true;
                }
                if(!gamepad1.y && lastY){
                    lastY = false;
                    if(Drive.relative == Relativity.ROBOT){
                        Drive.relative = Relativity.FIELD;
                    }
                    else{
                        Drive.relative = Relativity.ROBOT;
                    }
                }
                else if(gamepad1.y){
                    lastY = true;
                }
                if(!gamepad1.left_stick_button && lastLeftStick) {
                    lastLeftStick = false;
                    Drive.imu.initialize(parameters);
                    angleError = -imu.getAngularOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
                }
                else if(gamepad1.left_stick_button){
                    lastLeftStick = true;
                }
                if(!gamepad2.a && lastA){
                    lastA = false;
                    if(!(blockintake.getPosition() > 0.2)) {
                        blockintake.setPosition(0.55);
                    }
                    else{
                        blockintake.setPosition(0);
                    }
                }
                else if(gamepad2.a){
                    lastA = true;
                }
                if (!gamepad1.left_bumper && lastLeftBump) {
                    lastLeftBump = false;
                    if (leftClaw.getPosition() < 0.75){
                        leftClaw.setPosition(0.75);
                    }
                    else{
                        leftClaw.setPosition(0);
                    }
                }
                else if(gamepad1.left_bumper){
                    lastLeftBump = true;
                }

                if (!gamepad1.right_bumper && lastRightBump) {
                    lastRightBump = false;
                    if (rightClaw.getPosition() > 0){
                        rightClaw.setPosition(0);
                    }
                    else{
                        rightClaw.setPosition(0.75);
                    }
                }
                else if(gamepad1.right_bumper){
                    lastRightBump = true;
                }
            }
        }

    }
}
