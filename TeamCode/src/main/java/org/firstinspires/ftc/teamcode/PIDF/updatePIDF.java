package org.firstinspires.ftc.teamcode.PIDF;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.HardwareConfig;

@TeleOp(name = "UpdatePIDF")
@Disabled
public class updatePIDF extends LinearOpMode {
    final private static double P = 1.1294;
    final private static double I = 0.1129;
    final private static double D = 0;
    final private static double F = 11.2941;
    private DcMotorEx[] motors = {null, null, null, null};

    @Override
    public void runOpMode() {
        HardwareConfig config = new HardwareConfig(hardwareMap);
        motors[0] = config.frontRight;
        motors[1] = config.frontLeft;
        motors[2] = config.rearRight;
        motors[3] = config.rearLeft;

        double AverageP = 0;
        double AverageI = 0;
        double AverageD = 0;
        double AverageF = 0;

        for(int i = 0; i < motors.length; i++){
            AverageP += motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p;
            AverageI += motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i;
            AverageD += motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d;
            AverageF += motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f;
        }

        telemetry.addData("P", AverageP);
        telemetry.addData("I", AverageI);
        telemetry.addData("D", AverageD);
        telemetry.addData("F", AverageF);
        telemetry.update();
        waitForStart();

        for(int i = 0; i < motors.length; i++){
            motors[i].setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, I, D, F));
            motors[i].setPositionPIDFCoefficients(5);
        }
        for(int i = 0; i < motors.length; i++){
            AverageP += motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).p;
            AverageI += motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).i;
            AverageD += motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).d;
            AverageF += motors[i].getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER).f;
        }

        telemetry.addData("P", AverageP);
        telemetry.addData("I", AverageI);
        telemetry.addData("D", AverageD);
        telemetry.addData("F", AverageF);
        telemetry.update();

    }
}
