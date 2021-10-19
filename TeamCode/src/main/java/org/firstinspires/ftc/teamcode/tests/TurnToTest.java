package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AbstractAutonomous;

@Autonomous(name = "TurnToTest", group = "Testing")
public class TurnToTest extends AbstractAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeValues();
        waitForStart();

        telemetry.addLine("Turning to 0");
        telemetry.update();
        startTurnTo(0);
        while(turningFlag) updateTurnTo();
        telemetry.addLine("Desired angle achieved");
        telemetry.update();

        if(protectedSleep(500)) return;

        telemetry.addLine("Turning to PI");
        telemetry.update();
        startTurnTo(Math.PI);
        while(turningFlag) updateTurnTo();
        telemetry.addLine("Desired angle achieved");
        telemetry.update();

        if(protectedSleep(500)) return;

        telemetry.addLine("Turning to PI/2");
        telemetry.update();
        startTurnTo(Math.PI/2);
        while(turningFlag) updateTurnTo();
        telemetry.addLine("Desired angle achieved");
        telemetry.update();

        if(protectedSleep(500)) return;

        zero();
    }
}
