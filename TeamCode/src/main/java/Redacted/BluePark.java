package Redacted;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "BluePark")
@Disabled
@Deprecated

public class BluePark extends Localization {
    @Override
    public void runOpMode(){
        initialize(hardwareMap);
        waitForStart();
        updatePosition();
        moveWithEncoder(0, 65);
    }
}
