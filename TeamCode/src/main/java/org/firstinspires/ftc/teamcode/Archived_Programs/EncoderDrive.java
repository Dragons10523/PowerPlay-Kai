package org.firstinspires.ftc.teamcode.Archived_Programs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Encoders;
import org.firstinspires.ftc.teamcode.OOPO;
import org.firstinspires.ftc.teamcode.ProtoConfig;

@TeleOp(name="Encoder Drive Test")
@Disabled
public class EncoderDrive extends OpMode {
    private OOPO funcs;
    private Encoders encode;
    private double Inches = 0;
    @Override
    public void init() {
        ProtoConfig robot = new ProtoConfig();
        robot.init(hardwareMap);
        funcs = new OOPO(robot.frontLeft,robot.frontRight,robot.rearRight,robot.rearLeft);
        encode = new Encoders(robot.frontRight,robot.frontLeft, robot.rearRight, robot.rearLeft);

        encode.resetEncoders();
        Inches = encode.getTicks();
    }

    @Override
    public void loop() {
        if(Inches <= 560){
            Inches = encode.getTicks();
            funcs.absMove(0, 0.45, 90);
        }
        funcs.stopNow();
    }
}
