package org.firstinspires.ftc.teamcode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.processors.Control;

@TeleOp(name = "Calibrate Deadwheels", group = "Calibration")
public class CalibrateDeadwheels extends Control {
    double firstX = 0;
    double firstY = 0;
    double firstTheta = 0;
    double firstIterations = 0;

    double secondX = 0;
    double secondY = 0;
    double secondTheta = 0;
    double secondIterations = 0;

    double thirdX = 0;
    double thirdY = 0;
    double thirdTheta = 0;
    double thirdIterations = 0;

    @Override
    public void start() {
        super.start();

        telemetry.addLine("This program will output a calculated matrix based on recorded values");
        telemetry.addLine("Press A to continue");
        telemetry.update();
        waitForA();

        kai.deadwheels.calibrationMode = true;

        firstTest();
        secondTest();
        thirdTest();

        firstX /= firstIterations * 60;
        firstY /= firstIterations * 60;
        firstTheta /= firstIterations * 60;

        secondX /= secondIterations * 60;
        secondY /= secondIterations * 60;
        secondTheta /= secondIterations * 60;

        thirdX /= thirdIterations * 5;
        thirdY /= thirdIterations * 5;
        thirdTheta /= thirdIterations * 5;

        telemetry.addLine("Test Results:");
        telemetry.addLine(String.format("| %1, %2, %3 |", secondX, secondY, secondTheta));
        telemetry.addLine(String.format("| %1, %2, %3 |", firstX, firstY, firstTheta));
        telemetry.addLine(String.format("| %1, %2, %3 |", thirdX, thirdY, thirdTheta));
    }

    @Override
    public void loop() {
        requestOpModeStop();
    }

    private void firstTest() {
        telemetry.addLine("First Test: 5 feet forwards");
        telemetry.addLine("Press A when ready");
        telemetry.update();
        waitForA();

        do {
            kai.deadwheels.setTransform(0, 0, 0);

            firstIterations++;

            while(!(gamepad1.a || gamepad2.a)) {
                kai.deadwheels.wheelLoop();

                telemetry.addLine("Test 1");
                telemetry.addData("Measured X", kai.deadwheels.currentX);
                telemetry.addData("Measured Y", kai.deadwheels.currentY);
                telemetry.addData("Measured Theta", kai.deadwheels.currentAngle);
                telemetry.addLine("Press A to end the test");
                telemetry.update();
            }

            firstX += kai.deadwheels.currentX;
            firstY += kai.deadwheels.currentY;
            firstTheta += kai.deadwheels.currentAngle;

            telemetry.addLine("Press B to go to the next test. Press X once ready to run this test again");

            while(!(gamepad1.b || gamepad2.b || gamepad1.x || gamepad2.x)) {
                sleep(100);
            }

            if(gamepad1.b || gamepad2.b) break;

        } while(true);
    }

    private void secondTest() {
        telemetry.addLine("Second Test: 5 feet right");
        telemetry.addLine("Press A when ready");
        telemetry.update();
        waitForA();

        do {
            kai.deadwheels.setTransform(0, 0, 0);

            secondIterations++;

            while(!(gamepad1.a || gamepad2.a)) {
                kai.deadwheels.wheelLoop();

                telemetry.addLine("Test 2");
                telemetry.addData("Measured X", kai.deadwheels.currentX);
                telemetry.addData("Measured Y", kai.deadwheels.currentY);
                telemetry.addData("Measured Theta", kai.deadwheels.currentAngle);
                telemetry.addLine("Press A to end the test");
                telemetry.update();
            }

            secondX += kai.deadwheels.currentX;
            secondY += kai.deadwheels.currentY;
            secondTheta += kai.deadwheels.currentAngle;

            telemetry.addLine("Press B to go to the next test. Press X once ready to run this test again");

            while(!(gamepad1.b || gamepad2.b || gamepad1.x || gamepad2.x)) {
                sleep(100);
            }

            if(gamepad1.b || gamepad2.b) break;

        } while(true);
    }

    private void thirdTest() {
        telemetry.addLine("Third Test: 5 full rotations");
        telemetry.addLine("Press A when ready");
        telemetry.update();
        waitForA();

        do {
            kai.deadwheels.setTransform(0, 0, 0);

            thirdIterations++;

            while(!(gamepad1.a || gamepad2.a)) {
                kai.deadwheels.wheelLoop();

                telemetry.addLine("Test 3");
                telemetry.addData("Measured X", kai.deadwheels.currentX);
                telemetry.addData("Measured Y", kai.deadwheels.currentY);
                telemetry.addData("Measured Theta", kai.deadwheels.currentAngle);
                telemetry.addLine("Press A to end the test");
                telemetry.update();
            }

            thirdX += kai.deadwheels.currentX;
            thirdY += kai.deadwheels.currentY;
            thirdTheta += kai.deadwheels.currentAngle;

            telemetry.addLine("Press B to go to the next test. Press X once ready to run this test again");

            while(!(gamepad1.b || gamepad2.b || gamepad1.x || gamepad2.x)) {
                sleep(100);
            }

            if(gamepad1.b || gamepad2.b) break;

        } while(true);
    }

    private void waitForA() {
        while(!(gamepad1.a || gamepad2.a)) {
            sleep(100);
        }
    }
}
