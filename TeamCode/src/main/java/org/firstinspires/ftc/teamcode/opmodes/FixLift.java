package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.processors.Control;

@TeleOp(name = "Fix Lift")
public class FixLift extends Control {
    public void init() {
        super.init();
        kai.armLiftA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        kai.armLiftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        kai.liftExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop() {
        kai.armLiftA.setPower(-gamepad2.left_stick_y / 4);
        kai.armLiftB.setPower(-gamepad2.left_stick_y / 4);
        kai.liftExtension.setPower(-gamepad2.right_stick_y);
    }
}
