package org.firstinspires.ftc.teamcode.PIDF;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ProtoConfig;

@TeleOp(name = "Measure Max Velocity")

public class MeasureVelocity extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ProtoConfig config = new ProtoConfig();
        config.init(hardwareMap);

        DcMotorEx[] motors = {config.frontLeft, config.frontRight,
                config.rearLeft, config.rearRight
        };
        double[] currentVelocity = {0, 0, 0, 0};
        double[] maxVelocity = {0, 0, 0, 0};
        waitForStart();


        ElapsedTime time = new ElapsedTime();
        while(opModeIsActive() && time.seconds() < 1.5){
            for(int i = 0; i < motors.length; i++){
                motors[i].setPower(1);
                currentVelocity[i] = motors[i].getVelocity();
                if(currentVelocity[i] > maxVelocity[i]){
                    maxVelocity[i] = currentVelocity[i];
                }
            }

            telemetry.addData("FrontLeft", maxVelocity[0]);
            telemetry.addData("frontRight", maxVelocity[1]);
            telemetry.addData("rearLeft", maxVelocity[2]);
            telemetry.addData("rearRight", maxVelocity[3]);
            telemetry.addData("Average", (maxVelocity[0] + maxVelocity[1] + maxVelocity[2] + maxVelocity[3])/4);
            telemetry.update();
        }
        for(int i = 0; i< motors.length; i++){
            motors[i].setPower(0);
        }
        while(opModeIsActive()){}
    }
}
