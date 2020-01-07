package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name =  "TurnAngleTest")
public class TurnAngleTest extends NewLocalization{
    @Override
    public void runOpMode(){
        init(hardwareMap, false);
        waitForStart();
        turnToAngle(180);
        telemetry.update();
    }
}
