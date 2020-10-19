package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "ShootyShoot")
public class Test extends Control implements Control.ControllerSpec {

    double power = 0;
    short negator = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        alize();
        setSpec(this);
        waitForStart();
        while (opModeIsActive()) {
            shoot(negator * power);
            telemetry.addData("Shooting Power", negator * power);
            telemetry.update();
        }
    }

    @Override
    public void dpadUp(GamepadId id) {
        power = 1.0;
    }

    @Override
    public void dpadDown(GamepadId id) {
        power = 0.0;
    }

    @Override
    public void dpadLeft(GamepadId id) {
        power = clamp(power + 0.2, 0.0, 1.0);
    }

    @Override
    public void dpadRight(GamepadId id) {

    }

    @Override
    public void aButton(GamepadId id) {

    }

    @Override
    public void bButton(GamepadId id) {

    }

    @Override
    public void xButton(GamepadId id) {
        negator = (short)(-negator);
    }

    @Override
    public void yButton(GamepadId id) {

    }

    @Override
    public void startButton(GamepadId id) {

    }

    @Override
    public void backButton(GamepadId id) {

    }

    @Override
    public void lBumper(GamepadId id) {

    }

    @Override
    public void rBumper(GamepadId id) {

    }

    @Override
    public void lStickButton(GamepadId id) {

    }

    @Override
    public void rStickButton(GamepadId id) {

    }

    @Override
    public void sticks(GamepadId id, float lx, float ly, float rx, float ry) {

    }

    @Override
    public void triggers(GamepadId id, float l, float r) {

    }
}
