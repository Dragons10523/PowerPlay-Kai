package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.processors.Control;

@TeleOp(name = "Fix Lift")
public class FixLift extends Control {
    private float position = 0;
    private ElapsedTime deltaTime;

    public void init() {
        super.init();
        kai.armLiftA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        kai.armLiftB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        kai.liftExtension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        kai.armLiftA.setPower(0);
        kai.armLiftB.setPower(0);
        kai.liftExtension.setPower(0);

        deltaTime = new ElapsedTime();
    }

    public void init_loop() {
        deltaTime.reset();
    }

    public void loop() {
        kai.armLiftA.setPower(-gamepad2.left_stick_y / 3);
        kai.armLiftB.setPower(-gamepad2.left_stick_y / 3);
        kai.liftExtension.setPower(-gamepad2.right_stick_y);

        if(gamepad2.right_bumper) {
            position -= deltaTime.seconds()*300;
        } else if(gamepad2.left_bumper) {
            position += deltaTime.seconds()*300;
        }
        deltaTime.reset();

        kai.turntable.setTargetPosition((int)position);

        telemetry.addData("Lift A", kai.armLiftA.getCurrentPosition());
        telemetry.addData("Lift B", kai.armLiftB.getCurrentPosition());
        telemetry.update();
    }
}
