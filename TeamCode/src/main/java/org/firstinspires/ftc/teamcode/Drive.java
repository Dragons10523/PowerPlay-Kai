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
    private static BNO055IMU imu;
    private static Relativity relative = Relativity.FIELD;
    private static double speed = 1;
    private static Gamepad gp1;
    private static Gamepad gp2;
    private AsyncTask lol;
    private static double angleError;
    private static Servo blockintake, leftClaw, rightClaw;

    @Override
    public void init() {
        robot = new HardwareConfig(hardwareMap);

        robot.initializeDriveTrain();
        robot.initializeTools();

        Mecanums = new MecanumDrive(new DriveTrain(robot.frontLeft, robot.frontRight, robot.rearLeft, robot.rearRight));
        blockintake = robot.blockIntake;
        leftClaw = robot.leftClaw;
        rightClaw = robot.rightClaw;

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
    public void stop(){  lol.cancel(true); }
    private double getAngle(){
        return imu.getAngularOrientation(INTRINSIC, ZYX, DEGREES).firstAngle +angleError;
    }
    static class buttonUpdates extends AsyncTask<Void, Void, Void>{


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
