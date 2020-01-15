package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name =  "TurnAngleTest")
@Disabled
public class TurnAngleTest extends Localization {
    @Override
    public void runOpMode(){
        init(hardwareMap, false, Side.BLUE);
        waitForStart();
        turnToAngle(270);
        telemetry.update();
    }
}
