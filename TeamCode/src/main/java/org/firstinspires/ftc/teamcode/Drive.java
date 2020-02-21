package org.firstinspires.ftc.teamcode;

import android.os.AsyncTask;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive.Relativity;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;

@TeleOp(name = "TeleOP")
public class Drive extends OpMode {

    private HardwareConfig robot;
    private MecanumDrive Mecanums;
    private BNO055IMU imu;
    private Relativity relative = Relativity.FIELD;
    private double speed = 1;
    private Gamepad gp1;
    private Gamepad gp2;
    private AsyncTask lol;
    private double angleError;
    private Servo blockintake, leftClaw, rightClaw, leftHook, rightHook;

    @Override
    public void init() {
        robot = new HardwareConfig(hardwareMap);

        robot.initializeDriveTrain();
        robot.initializeTools();
        robot.initializeServos();

        Mecanums = new MecanumDrive(new DriveTrain(robot.frontLeft, robot.frontRight, robot.rearLeft, robot.rearRight));
        blockintake = robot.blockIntake;
        leftClaw = robot.leftClaw;
        rightClaw = robot.rightClaw;
        leftHook = robot.leftHook;
        rightHook = robot.rightHook;

        robot.initializeIMU();
        imu = robot.imu;

        angleError = -imu.getAngularOrientation(INTRINSIC,ZYX,DEGREES).firstAngle;
        gp1 = gamepad1;
        gp2 = gamepad2;
        lol = new buttonUpdates().execute();
    }
    @Override
    public void loop() {
        if (relative == Relativity.FIELD) {
            telemetry.addData("Relativity", "FIELD");
            double a = getAngle();
            telemetry.addData("Angle", a);
            Mecanums.absoluteMove(gamepad1.left_stick_y * speed, gamepad1.left_stick_x * speed, Math.toRadians(a), -gamepad1.right_stick_x * speed);
        } else {
            telemetry.addData("Relativity", "ROBOT");
            Mecanums.joystickMove(gamepad1.left_stick_y * speed, gamepad1.left_stick_x * speed, -gamepad1.right_stick_x * speed);
        }

        robot.ramp1.setPower((gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad2.left_trigger + gamepad1.left_trigger ));
        robot.ramp2.setPower((gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad2.left_trigger + gamepad1.left_trigger ));
        robot.intake1.setPower((gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad2.left_trigger + gamepad1.left_trigger ));
        robot.intake2.setPower((gamepad1.right_trigger + gamepad2.right_trigger) - (gamepad2.left_trigger + gamepad1.left_trigger ));
        robot.lift.setPower(gamepad2.left_stick_y);
        robot.horizontal.setPower(gamepad2.right_stick_y);

        telemetry.addData("Acceleration", imu.getAcceleration().xAccel + "    " + imu.getAcceleration().yAccel);
        telemetry.addData("ServoLeft", robot.leftClaw.getPosition());
        telemetry.addData("ServoRight", robot.rightClaw.getPosition());
        telemetry.addData("Speed", (speed * 100) + "%");
        telemetry.update();
    }
    @Override
    public void stop(){  lol.cancel(true); }
    private double getAngle(){
        return imu.getAngularOrientation(INTRINSIC, ZYX, DEGREES).firstAngle +angleError;
    }
     class buttonUpdates extends AsyncTask<Void, Void, Void>{


        @Override
        protected Void doInBackground(Void ... voids) {
            boolean lastX = false;
            boolean lastY = false;
            boolean lastLeftStick = false;
            boolean lastA = false;
            boolean lastLeftBump = false;
            boolean lastRightBump = false;
            boolean lastHookLeft = false;
            boolean lastHookRight = false;
            Gamepad gamepad1 = gp1;
            Gamepad gamepad2 = gp2;
            while(!this.isCancelled()){
                if(!gamepad1.x && lastX){
                    lastX =false;
                    if (speed == 1) {
                        speed = 0.5;
                    } else {
                        speed = 1;
                    }
                }
                else if(gamepad1.x){
                    lastX = true;
                }
                if(!gamepad1.y && lastY){
                    lastY = false;
                    if(relative == Relativity.ROBOT){
                        relative = Relativity.FIELD;
                    }
                    else{
                        relative = Relativity.ROBOT;
                    }
                }
                else if(gamepad1.y){
                    lastY = true;
                }
                if(!gamepad1.left_stick_button && lastLeftStick) {
                    lastLeftStick = false;
                    angleError = -imu.getAngularOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
                }
                else if(gamepad1.left_stick_button){
                    lastLeftStick = true;
                }
                if(!gamepad2.a && lastA){
                    lastA = false;
                    if(!(blockintake.getPosition() > 0.2)) {
                        blockintake.setPosition(0);
                    }
                    else{
                        blockintake.setPosition(0.55);
                    }
                }
                else if(gamepad2.a){
                    lastA = true;
                }

                if (!gamepad2.y && lastHookLeft) {
                    lastHookLeft = false;
                    if (leftHook.getPosition() > 0.2){
                        leftHook.setPosition(0);
                    }
                    else{
                        leftHook.setPosition(0.75);
                    }
                }
                else if(gamepad2.y){
                    lastHookLeft = true;
                }

                if (!gamepad2.y && lastHookRight) {
                    lastHookRight = false;
                    if (rightHook.getPosition() < 0.65){
                        rightHook.setPosition(0.75);
                    }
                    else{
                        rightHook.setPosition(0);
                    }
                }
                else if(gamepad2.y){
                    lastHookRight = true;
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
                        rightClaw.setPosition(0.85);
                    }
                }
                else if(gamepad1.right_bumper){
                    lastRightBump = true;
                }
            }
        return null;
        }

    }
}
