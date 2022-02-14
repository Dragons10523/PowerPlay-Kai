package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Park Blue", group = "Auto Main", preselectTeleOp = "Vroom Vroom Blue")
public class ParkAutoBlue extends AbstractAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeValues();
        waitForStart();
        armControl(ArmPosition.LOW_FORE);
        if(protectedSleep(300)) return;
        driveDist(48);
        armControl(ArmPosition.PICKUP);
        if(protectedSleep(1000)) return;
    }
}
