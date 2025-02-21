package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.RobotClass;
import org.firstinspires.ftc.teamcode.Utils;

import java.util.Map;

@TeleOp
public class FunctionalityTest extends AutoControl {
    ElapsedTime time = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("opMode starting");
        telemetry.update();
        super.runOpMode();
        telemetry.addLine("opMode INIT");
        telemetry.update();
        super.initialize();
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine("functionality Testing:");
            telemetry.addLine("Press A to test SENSORS");
            telemetry.addLine("Press START to test BUCKET");
            telemetry.addLine("Press B to test INTAKE");
            telemetry.addLine("Press BACK to test ARM_EXTENSION");
            telemetry.addLine("Press X to test ARM_FLIP");
            telemetry.addLine("Press Y to test LIFT");
            telemetry.update();
            if (gamepad1.a) {
                telemetry.clear();
                sleep(500);
                testSensors();
                telemetry.clear();
                sleep(500);
            }
            if (gamepad1.b) {
                telemetry.clear();
                sleep(500);
                testIntake();
                telemetry.clear();
                sleep(500);
            }
            if (gamepad1.x) {
                telemetry.clear();
                sleep(500);
                testArmFlip();
                telemetry.clear();
                sleep(500);
            }
            if (gamepad1.y) {
                telemetry.clear();
                sleep(500);
                testLift();
                telemetry.clear();
                sleep(500);
            }
            if (gamepad1.start) {
                telemetry.clear();
                sleep(500);
                testBucket();
                telemetry.clear();
                sleep(500);
            }
            if (gamepad1.back) {
                telemetry.clear();
                sleep(500);
                testArmExtension();
                telemetry.clear();
                sleep(500);
            }
        }
    }

    public boolean testMotor(DcMotorEx motor) {
        double startPosition = motor.getCurrentPosition();
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        int failureCount = 0;
        double startTime = time.seconds();
        while (opModeIsActive()) {
            telemetry.addLine("testing " + motor.getDeviceName());
            telemetry.addData("startPos", startPosition);
            telemetry.addData("currentPos", motor.getCurrentPosition());
            telemetry.update();
            if (time.milliseconds() % 1000 > 500) {
                motor.setPower(-0.7);
            } else {
                motor.setPower(0.7);
            }
            if (startPosition != motor.getCurrentPosition()) {
                motor.setPower(0);
                return true;
            }
            if (startTime + 1.0 < time.seconds()) {
                motor.setPower(0);
                return false;
            }
        }
        motor.setPower(0);
        return false;
    }


    public void testSensors() {
        while (opModeIsActive() && !gamepad1.a) {
            telemetry.addLine("testing limelight and OTOS");
            telemetry.addLine("Press A to exit");

            LLStatus status = robot.limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());
            SparkFunOTOS.Pose2D pos = robot.opticalSensor.getPosition();
            telemetry.addData("sensorPos", "XYH %.3f %.3f %.3f", pos.x, pos.y, pos.h);
            telemetry.update();
        }
    }

    public void testArmFlip() {
        telemetry.addLine("Testing armFlip");
        telemetry.update();
        boolean armFlipSuccess = testMotor(robot.Motors.get(RobotClass.MOTORS.ARM_FLIP));
        if (!armFlipSuccess) {
            while (opModeIsActive() && !gamepad1.a) {
                telemetry.addLine("armFlip FAILURE");
                telemetry.addLine("Press A to exit");
                telemetry.update();
            }
        } else {
            while (opModeIsActive() && !gamepad1.a) {
                telemetry.addLine("armFlip SUCCESS");
                telemetry.addLine("Press A to exit");
                telemetry.update();
            }
        }
    }

    public void testLift() {
        boolean liftSuccess = testMotor(robot.Motors.get(RobotClass.MOTORS.LIFT));
        if (!liftSuccess) {
            while (opModeIsActive() && !gamepad1.a) {
                telemetry.addLine("lift FAILURE");
                telemetry.addLine("Press A to exit");
                telemetry.update();
            }
        } else {
            while (opModeIsActive() && !gamepad1.a) {
                telemetry.addLine("lift SUCCESS");
                telemetry.addLine("Press A to exit");
                telemetry.update();
            }
        }
    }

    public void testIntake() {
        while (opModeIsActive() && !gamepad1.a) {
            telemetry.addLine("testing intakeServo and intakeGate");
            telemetry.addLine("Press A to exit");
            telemetry.update();
            if (time.seconds() % 2 > 1) {
                robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-0.75);
                robot.Servos.get(RobotClass.SERVOS.INTAKE_SERVO).setPosition(0);
            } else {
                robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(0.75);
                robot.Servos.get(RobotClass.SERVOS.INTAKE_SERVO).setPosition(0.75);
            }
        }
        robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(0);
        telemetry.clear();
    }

    public void testBucket() {
        while (opModeIsActive() && !gamepad1.a) {
            telemetry.addLine("testing bucket");
            telemetry.addLine("Press A to exit");
            telemetry.update();
            if (time.seconds() % 2 > 1) {
                robot.Servos.get(RobotClass.SERVOS.BUCKET).setPosition(0.75);
            } else {
                robot.Servos.get(RobotClass.SERVOS.BUCKET).setPosition(0.25);
            }
        }
        telemetry.clear();
    }

    public void testArmExtension() {
        while (opModeIsActive() && !gamepad1.a) {
            telemetry.addLine("Press X to test LEFT EXTENSION");
            telemetry.addLine("Press Y to test RIGHT EXTENSION");
            telemetry.addLine("Press B to test BOTH");
            telemetry.addLine("Press A to exit");
            telemetry.update();
            if (gamepad1.x) {
                while (opModeIsActive() && !gamepad1.a) {
                    telemetry.addLine("testing LEFT EXTENSION");
                    telemetry.addLine("Press A to exit");
                    telemetry.update();
                    if (time.seconds() % 2 > 1) {
                        robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(0.75);
                    } else {
                        robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(0.25);
                    }
                }

            }
            if (gamepad1.y) {
                while (opModeIsActive() && !gamepad1.a) {
                    telemetry.addLine("testing RIGHT EXTENSION");
                    telemetry.addLine("Press A to exit");
                    telemetry.update();
                    if (time.seconds() % 2 > 1) {
                        robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(0.75);
                    } else {
                        robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(0.25);
                    }
                }

            }
            if (gamepad1.b) {
                while (opModeIsActive() && !gamepad1.a) {
                    telemetry.addLine("testing EXTENSION");
                    telemetry.addLine("Press A to exit");
                    telemetry.update();
                    if (time.seconds() % 3 > 1.5) {
                        robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(Utils.arm_leftPos[0]);
                        robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(Utils.arm_rightPos[0]);
                    } else {
                        robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(Utils.arm_leftPos[Utils.arm_leftPos.length - 1]);
                        robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(Utils.arm_rightPos[Utils.arm_rightPos.length - 1]);
                    }
                }

            }
        }
        telemetry.clear();
    }
}
