package org.firstinspires.ftc.teamcode;

import android.os.AsyncTask;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
@TeleOp(name = "GOD-MODE")
public class GodMode extends OpMode {

    private Localization localization;
    private OOPO.Relativity relative;
    private DcMotor ramp1, ramp2, lift, horizontal;
    private CRServo intake1, intake2;
    private Servo leftClaw, rightClaw, blockintake;
    private Gamepad gp1, gp2;
    private AsyncTask lol;
    private double speed = 1;
    @Override
    public void init() {
        localization = new Localization();
        localization.initialize(hardwareMap);
        ramp1 = localization.bahumut.ramp1;
        ramp2 = localization.bahumut.ramp2;
        lift = localization.bahumut.lift;
        horizontal = localization.bahumut.horizontal;
        intake1 = localization.bahumut.intake1;
        intake2 = localization.bahumut.intake2;
        leftClaw = localization.bahumut.leftClaw;
        rightClaw = localization.bahumut.rightClaw;
        blockintake = localization.bahumut.blockIntake;
        gp1 = gamepad1;
        gp2 = gamepad2;
        lol = new buttonUpdates(localization.gyrometers).execute();
    }

    @Override
    public void stop(){
        lol.cancel(true);
    }

    @Override
    public void loop() {
        localization.updatePosition();
        if (relative == OOPO.Relativity.FIELD) {
            telemetry.addData("Relativity", "FIELD");
            double a = localization.getAngle();
            telemetry.addData("Angle", a);
            localization.funcs.absoluteMove(gamepad1.left_stick_y * speed, gamepad1.left_stick_x * speed, Math.toRadians(a), gamepad1.right_stick_x * speed);
        } else {
            telemetry.addData("Relativity", "ROBOT");
            localization.funcs.joystickMove(gamepad1.left_stick_y * speed, gamepad1.left_stick_x * speed, gamepad1.right_stick_x * speed);
        }

        ramp1.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        ramp2.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        intake1.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        intake2.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        lift.setPower(gamepad2.left_stick_y);
        horizontal.setPower(gamepad2.right_stick_y);
        telemetry.addData("X = ",localization.X + " Y = " + localization.Y);
        telemetry.addData("ServoLeft", leftClaw.getPosition());
        telemetry.addData("ServoRight", rightClaw.getPosition());
        telemetry.addData("Speed", (speed * 100) + "%");
        telemetry.addData("frontLazer", Math.round(localization.frontDis));
        telemetry.addData("leftLazer", localization.leftDis);
        telemetry.addData("rightLazer", localization.rightDis);
        telemetry.addData("rearLazer", localization.rearDis);
        telemetry.addData("cameraLazer", localization.cameraDisDis);
        telemetry.update();
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
            Gamepad gamepad1 = gp1;
            Gamepad gamepad2 = gp2;
            while(true){
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
                    if(relative == OOPO.Relativity.ROBOT){
                        relative = OOPO.Relativity.FIELD;
                    }
                    else{
                        relative = OOPO.Relativity.ROBOT;
                    }
                }
                else if(gamepad1.y){
                    lastY = true;
                }
                if(!gamepad1.left_stick_button && lastLeftStick) {
                    lastLeftStick = false;
                    localization.imu.initialize(parameters);
                    localization.angleError = -localization.imu.getAngularOrientation(INTRINSIC, ZYX, DEGREES).firstAngle;
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