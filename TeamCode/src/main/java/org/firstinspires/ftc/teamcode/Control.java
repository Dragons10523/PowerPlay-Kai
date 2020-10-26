package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Control extends LinearOpMode{
    Thalatte thalatte;

    // CONSTANTS
    final double INTAKE = 1.0;
    final double CLAMP = 1.0;

    public GamepadPrev prev1;

    public GamepadPrev prev2;

    public ElapsedTime time;

    public void drive(double l, double r) {
        thalatte.backRight.setPower(r);
        thalatte.frontRight.setPower(r);
        thalatte.backLeft.setPower(l);
        thalatte.frontLeft.setPower(l);
    }

    public void shoot(double p) {
        thalatte.shooterFront.setPower(p);
        thalatte.shooterBack.setPower(p);
    }

    public void intake(boolean on) {
        thalatte.intake.setPower(on ? INTAKE : 0);
    }

    public boolean toggleIntake() {
        boolean set = thalatte.intake.getPower() == 0;
        intake(set);
        return set;
    }

    public void vwompArm(double p) {
        thalatte.vwomp.setPower(p);
    }

    public void vwompClamp(boolean c) {
        thalatte.vwompClampLeft.setPosition(c ? CLAMP : 0);
        thalatte.vwompClampRight.setPosition(c ? CLAMP : 0);
    }

    public boolean toggleVwompClamp() {
        boolean set = thalatte.vwompClampLeft.getPosition() == 0;
        vwompClamp(set);
        return set;
    }

    public void alize() {
        thalatte = new Thalatte(hardwareMap);
        prev1 = new GamepadPrev(gamepad1);
        prev2 = new GamepadPrev(gamepad2);
        time = new ElapsedTime();
    }

    public double clamp(double num, double min, double max) {
        return Math.max(Math.min(num, max), min);
    }

}