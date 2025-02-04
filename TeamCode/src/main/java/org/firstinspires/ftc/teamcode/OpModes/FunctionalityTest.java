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
    Map<RobotClass.MOTORS, DcMotorEx> Motors = robot.Motors;

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
            telemetry.addLine("Press A to test SENSORS      Press START to test BUCKET");
            telemetry.addLine("Press B to test INTAKE       Press BACK to test ARM_EXTENSION");
            telemetry.addLine("Press X to test ARM_FLIP");
            telemetry.addLine("Press Y to test LIFT");
            if (gamepad1.a) {
                telemetry.clear();
                sleep(500);
                testSensors();
            }
            if (gamepad1.b) {
                telemetry.clear();
                sleep(500);
                testIntake();
            }
            if (gamepad1.x) {
                telemetry.clear();
                sleep(500);
                testArmFlip();
            }
            if (gamepad1.y) {
                telemetry.clear();
                sleep(500);
                testLift();
            }
            if (gamepad1.start) {
                telemetry.clear();
                sleep(500);
                testBucket();
            }
            if (gamepad1.back) {
                telemetry.clear();
                sleep(500);
                testArmExtension();
            }
        }
    }


    public boolean testMotor(DcMotorEx motor) {
        double startPosition = motor.getCurrentPosition();
        int failureCount = 0;
        double startTime = time.seconds();
        while (opModeIsActive() && failureCount < 100) {
            telemetry.addLine("testing " + motor.getDeviceName());
            telemetry.addData("startPos", startPosition);
            telemetry.addData("currentPos", motor.getCurrentPosition());
            telemetry.addData("failureCount", failureCount);
            telemetry.update();
            if (time.milliseconds() % 1000 > 500) {
                motor.setPower(-0.3);
            } else {
                motor.setPower(0.3);
            }
            if (startPosition == motor.getCurrentPosition()) {
                failureCount++;
            }
            if (startTime + 1.0 < time.seconds()) {
                motor.setPower(0);
                return true;
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
        boolean armFlipSuccess = testMotor(Motors.get(RobotClass.MOTORS.ARM_FLIP));
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
        boolean liftSuccess = testMotor(Motors.get(RobotClass.MOTORS.LIFT));
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
            telemetry.addLine("testing intakeServo, intakeGate");
            telemetry.addLine("Press A to exit");
            telemetry.update();
            if (time.seconds() % 2 > 1) {
                robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(-0.75);
                robot.Servos.get(RobotClass.SERVOS.INTAKE_SERVO).setPosition(0.75);
            } else {
                robot.CR_Servos.get(RobotClass.CR_SERVOS.INTAKE).setPower(0.75);
                robot.Servos.get(RobotClass.SERVOS.INTAKE_SERVO).setPosition(0.75);
            }

        }
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
