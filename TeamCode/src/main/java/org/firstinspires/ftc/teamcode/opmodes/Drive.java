package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.processors.Control;
import org.firstinspires.ftc.teamcode.processors.VecUtils;

@TeleOp(name = "Drive")
public class Drive extends Control {
    // Numerical values
    float xSum = 0;
    float ySum = 0;

    // Prev values
    boolean clawPrev = false;

    boolean resetLift = false;

    ElapsedTime deltaTimer = new ElapsedTime();
    double deltaTime = 1;

    @Override
    public void loop() {
        deltaTime = deltaTimer.seconds();
        deltaTimer.reset();
        super.loop();

        processDriverControls();
        processManipulatorControls();

        telemetry.addData("Target Level", kai.armLiftA.getTargetPosition());
        telemetry.addData("Current Level A", kai.armLiftA.getCurrentPosition());
        telemetry.addData("Current Level B", kai.armLiftA.getCurrentPosition());
        telemetry.addData("PIDF Coefficients", kai.armLiftA.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));

        telemetry.update();
    }

    private void processDriverControls() {
        // Calibration
        if(gamepad1.back) {
            kai.deadwheels.setTransform(0, 0, 0);
        }

        // Turning
        double turn = gamepad1.right_trigger - gamepad1.left_trigger;

        // Driving
        float driveX;
        float driveY;

        if(gamepad1.right_bumper) {
            double offsetX = 10 * deltaTime * ((gamepad1.left_stick_x*0.7) - xSum);
            double offsetY = 10 * deltaTime * ((-gamepad1.left_stick_y*0.7) - ySum);

            xSum += offsetX;
            ySum += offsetY;

            driveX = xSum;
            driveY = ySum;
        } else {
            driveX = gamepad1.left_stick_x;
            driveY = -gamepad1.left_stick_y;
        }

        DriveMode driveMode = DriveMode.LOCAL;

        mecanumDrive(driveX, driveY, turn, driveMode);
    }

    private void processManipulatorControls() {
        if (gamepad2.x) {
            armControl.setLiftHeight(GoalHeight.NONE);
        } else if (gamepad2.y) {
            armControl.setLiftHeight(GoalHeight.HIGH);
        } else if (gamepad2.b) {
            armControl.setLiftHeight(GoalHeight.MID);
        } else if (gamepad2.a) {
            armControl.setLiftHeight(GoalHeight.LOW);
        }

        if (Math.hypot(gamepad2.left_stick_y, gamepad2.left_stick_x) > 0.1) {
            double angle = Math.atan2(
                    -gamepad2.left_stick_y,
                    gamepad2.left_stick_x);

            angle -= VecUtils.HALF_PI;
            angle = collapseAngle(angle);

            telemetry.addData("Requested Angle", angle);
            telemetry.addData("Actual Angle", armControl.tableAngle());

            armControl.setAngleOverride(angle);
        }

        if (gamepad2.dpad_up) {
            armControl.setExtensionDistance(5);
        }

        if(gamepad2.dpad_down) {
            armControl.setExtensionDistance(0);
        }

        if(Math.abs(gamepad2.right_stick_y) > 0.5) {
            kai.armLiftA.setPower(-gamepad2.right_stick_y);
            kai.armLiftB.setPower(-gamepad2.right_stick_y);
            kai.armLiftA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            kai.armLiftA.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            resetLift = true;
        } else if(resetLift) {
            kai.armLiftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            kai.armLiftA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            kai.armLiftA.setPower(1);
            kai.armLiftB.setPower(1);
            kai.armLiftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            kai.armLiftA.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            resetLift = false;
        }

        //armControl.setExtensionDistance(gamepad2.right_trigger * 13.5);

        boolean clawToggle = gamepad2.right_bumper;
        if(clawToggle != clawPrev && clawToggle) {
            armControl.toggleClaw();
        }

        clawPrev = clawToggle;

        if(gamepad2.dpad_down) {
            orientClaw(WristState.FLIPPED);
        } else if(gamepad2.dpad_up) {
            orientClaw(WristState.NORMAL);
        }
    }
}