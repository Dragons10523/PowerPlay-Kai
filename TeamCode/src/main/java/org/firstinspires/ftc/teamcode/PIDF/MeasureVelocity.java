package org.firstinspires.ftc.teamcode.PIDF;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.ProtoConfig;

@TeleOp(name = "Measure Max Velocity")

public class MeasureVelocity extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ProtoConfig config = new ProtoConfig();
        config.initDcMotorEx(hardwareMap);

        DcMotorEx[] motors = {config.frontLeftEx, config.frontRightEx,
                config.rearLeftEx, config.rearRightEx};
        double[] currentVelocity = {0, 0, 0, 0};
        double[] maxVelocity = {0, 0, 0, 0};
        waitForStart();

        while(opModeIsActive()){
            for(int i = 0; i < motors.length; i++){
                currentVelocity[i] = motors[i].getVelocity();
                if(currentVelocity[i] > maxVelocity[i]){
                    maxVelocity[i] = currentVelocity[i];
                }
            }

            telemetry.addData("FrontLeft", maxVelocity[0]);
            telemetry.addData("frontRight", maxVelocity[1]);
            telemetry.addData("rearLeft", maxVelocity[2]);
            telemetry.addData("rearRight", maxVelocity[3]);
            telemetry.update();
        }
    }
}
