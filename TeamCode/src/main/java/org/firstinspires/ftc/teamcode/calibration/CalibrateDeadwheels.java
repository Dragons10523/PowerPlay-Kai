package org.firstinspires.ftc.teamcode.calibration;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.processors.Control;
import org.firstinspires.ftc.teamcode.processors.VecUtils;

@TeleOp(name = "Calibrate Deadwheels", group = "Calibration")
public class CalibrateDeadwheels extends Control {
    double firstX = 0;
    double firstY = 0;
    double firstTheta = 0;
    double firstIterations = 1;

    double secondX = 0;
    double secondY = 0;
    double secondTheta = 0;
    double secondIterations = 1;

    double thirdX = 0;
    double thirdY = 0;
    double thirdTheta = 0;
    double thirdIterations = 1;

    int lastTestCompleted = 0;

    public void start() {
        super.start();
        kai.deadwheels.calibrationMode = true;
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        super.loop();

        switch(lastTestCompleted) {
            case 0:
            case 2:
            case 4:
                telemetry.addLine("This program will output a calculated matrix based on recorded values");
                telemetry.addLine("Press A to continue");
                telemetry.update();
                checkForA();
                kai.deadwheels.setTransform(-0.5, -0.5, 0);
                break;
            case 1:
                firstTest();
                break;
            case 3:
                secondTest();
                break;
            case 5:
                thirdTest();
                break;
            case 6:
                sleep(5000);

                firstX /= firstIterations * 60;
                firstY /= firstIterations * 60;
                firstTheta /= firstIterations * 60;

                secondX /= secondIterations * 60;
                secondY /= secondIterations * 60;
                secondTheta /= secondIterations * 60;

                thirdX /= thirdIterations * 5 * VecUtils.TAU;
                thirdY /= thirdIterations * 5 * VecUtils.TAU;
                thirdTheta /= thirdIterations * 5 * VecUtils.TAU;

                telemetry.addLine("Test Results:");
                telemetry.addLine(String.format("| %1$f, %2$f, %3$f |", secondX, secondY, secondTheta));
                telemetry.addLine(String.format("| %1$f, %2$f, %3$f |", firstX, firstY, firstTheta));
                telemetry.addLine(String.format("| %1$f, %2$f, %3$f |", thirdX, thirdY, thirdTheta));
                telemetry.update();

                while(!isStopRequested) sleep(100);
        }
    }

    private void firstTest() {
        kai.deadwheels.setTransform(-0.5, -0.5, 0);

        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() <= 30) {
            kai.deadwheels.wheelLoop();

            telemetry.addLine("Test 1");
            telemetry.addData("Measured X", kai.deadwheels.currentX);
            telemetry.addData("Measured Y", kai.deadwheels.currentY);
            telemetry.addData("Measured Theta", kai.deadwheels.currentAngle);
            telemetry.addData("Time Remaining", 30 - timer.seconds());
            telemetry.update();
        }

        firstX += kai.deadwheels.currentX;
        firstY += kai.deadwheels.currentY;
        firstTheta += kai.deadwheels.currentAngle;

        lastTestCompleted++;
    }

    private void secondTest() {
        kai.deadwheels.setTransform(-0.5, -0.5, 0);

        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() <= 30) {
            kai.deadwheels.wheelLoop();

            telemetry.addLine("Test 2");
            telemetry.addData("Measured X", kai.deadwheels.currentX);
            telemetry.addData("Measured Y", kai.deadwheels.currentY);
            telemetry.addData("Measured Theta", kai.deadwheels.currentAngle);
            telemetry.addData("Time Remaining", 30 - timer.seconds());
            telemetry.update();
        }

        secondX += kai.deadwheels.currentX;
        secondY += kai.deadwheels.currentY;
        secondTheta += kai.deadwheels.currentAngle;

        lastTestCompleted++;
    }

    private void thirdTest() {
        mecanumDrive(0, 0, gamepad1.right_trigger - gamepad1.left_trigger, DriveMode.LOCAL);

        telemetry.addLine("Test 3");
        telemetry.addData("Measured X", kai.deadwheels.currentX);
        telemetry.addData("Measured Y", kai.deadwheels.currentY);
        telemetry.addData("Measured Theta", kai.deadwheels.currentAngle);
        telemetry.update();

        if(gamepad1.b) {
            thirdX += kai.deadwheels.currentX;
            thirdY += kai.deadwheels.currentY;
            thirdTheta += kai.deadwheels.currentAngle;

            lastTestCompleted++;
        }
    }

    private void checkForA() {
        if(gamepad1.a || gamepad2.a) {
            lastTestCompleted++;
        }
    }
}
