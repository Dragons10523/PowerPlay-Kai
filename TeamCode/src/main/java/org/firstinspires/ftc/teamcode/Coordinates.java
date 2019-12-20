package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Coordinates")
@Disabled
public class Coordinates extends Localization {
    @Override
    public void runOpMode(){
        initialize(hardwareMap);
        waitForStart();

        while (!isStopRequested()) {
            updatePosition();
            telemetry.addData("Target Visibility", targetVisible);
            telemetry.addData("Pos (in)", "{X, Y} = %.1f, %.1f", X, Y);
            if (stoneVisible) {
                telemetry.addData("Stone", stoneVisible);
                telemetry.addData("Stone Target", "{X, Y}: %.1f, %.1f", stoneX, stoneY);
            }
            telemetry.update();
        }
        targetsSkyStone.deactivate();
    }
}
