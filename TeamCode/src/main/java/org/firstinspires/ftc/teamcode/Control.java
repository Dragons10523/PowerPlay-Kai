package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Control extends LinearOpMode {
    Ahi ahi;

    final int armOffset = 0;
    final double CONVERSION_FACTOR = 8192*(4.0+(2.0/7.0)*Math.PI);

    enum ArmPosition {
        START,
        LOW,
        MED,
        HIGH,
        PICKUP
    }

    public void drive(double left, double right) {
        ahi.leftA.setPower(left);
        ahi.leftB.setPower(left);
        ahi.rightA.setPower(right);
        ahi.rightB.setPower(right);
    }

    public void driveLoop() {
        double left = ahi.drivetrainReverse ? -gamepad1.right_stick_y : -gamepad1.left_stick_y;
        double right = ahi.drivetrainReverse ? -gamepad1.left_stick_y : -gamepad1.right_stick_y;

        boolean sneak = gamepad1.left_stick_button || gamepad1.right_stick_button;

        left *= sneak ? 0.3 : 1.0;
        right *= sneak ? 0.3 : 1.0;

        drive(left, right);
    }

    public void driveDist(double dist) {
        int ticks = (int)(dist*CONVERSION_FACTOR);

        int leftTarget = ticks+ahi.leftA.getCurrentPosition();
        int rightTarget = ticks+ahi.rightA.getCurrentPosition();

        // Defining PID variables
        double kP = 1;
        double kI = 1;
        double kD = 1;

        double PLeft = leftTarget-ahi.leftA.getCurrentPosition();
        double PLeftPrev = PLeft;
        double ILeft = 0;
        double DLeft = 0;

        double PRight = rightTarget-ahi.rightA.getCurrentPosition();
        double PRightPrev = PRight;
        double IRight = 0;
        double DRight = 0;

        // Setting up timer for deltaTime
        ElapsedTime time = new ElapsedTime(); // Potentially change to a global timer

        double lastTime = time.milliseconds();
        double deltaTime = 0;

        double timeInBounds = 0;

        while(opModeIsActive()) {
            deltaTime = time.milliseconds()-lastTime;

            PLeft = leftTarget-ahi.leftA.getCurrentPosition();
            PRight = rightTarget-ahi.rightA.getCurrentPosition();

            ILeft += PLeft*deltaTime;
            IRight += PRight*deltaTime;

            try { // In case of divide by zero error
                DLeft = (PLeft - PLeftPrev) / deltaTime;
                DRight = (PRight - PRightPrev) / deltaTime;
            } catch (ArithmeticException e) {
                DLeft = 0;
                DRight = 0;
            }

            drive(kP*PLeft + kI*ILeft + kD*DLeft, kP*PRight + kI*IRight + kD*DRight);

            PRightPrev = PRight;
            PLeftPrev = PLeft;
            lastTime = time.milliseconds();

            if(Math.abs(PLeft) < 350 && Math.abs(PRight) < 350) {
                timeInBounds += deltaTime;
            } else {
                timeInBounds = 0;
            }

            if(timeInBounds > 300) {
                break;
            }
        }
    }

    public void armControl(ArmPosition armPosition) {
        switch(armPosition) {
            case LOW:
                ahi.arm.setTargetPosition(3640-armOffset);
                break;
            case MED:
                ahi.arm.setTargetPosition(3300-armOffset);
                break;
            case HIGH:
                ahi.arm.setTargetPosition(2960-armOffset);
                break;
            case PICKUP:
                ahi.arm.setTargetPosition(0-armOffset);
                break;
            default:
                ahi.arm.setTargetPosition(0);
                break;
        }
        ahi.arm.setPower(1.0);
    }

    public void runIntake(boolean on) {
        ahi.succc.setPower(on ? 1.0 : 0.0);
    }

    public void playDDR(double power) {
        ahi.ddr.setPower(power);
    }

    public void setFlup(boolean open) {
        ahi.flup.setPosition(open ? 1.0 : 0.0);
    }

    public void setLiftPower(double power) {
        ahi.capLift.setPower(power);
    }
}
