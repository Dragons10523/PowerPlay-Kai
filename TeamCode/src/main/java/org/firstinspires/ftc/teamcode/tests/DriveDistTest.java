package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AbstractAutonomous;

@Autonomous(name = "DriveDistTest", group = "Testing")
public class DriveDistTest extends AbstractAutonomous {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        telemetry.addLine("Driving forwards 6\"");
        telemetry.update();
        if(protectedSleep(500)) return;
        if(driveDist(6)) return;
        telemetry.addLine("Desired angle achieved");
        telemetry.update();
        if(protectedSleep(500)) return;

        telemetry.addLine("Driving forwards 12\"");
        telemetry.update();
        if(protectedSleep(500)) return;
        if(driveDist(12)) return;
        telemetry.addLine("Desired angle achieved");
        telemetry.update();
        if(protectedSleep(500)) return;

        telemetry.addLine("Driving backwards 6\"");
        telemetry.update();
        if(protectedSleep(500)) return;
        if(driveDist(-6)) return;
        telemetry.addLine("Desired angle achieved");
        telemetry.update();
        if(protectedSleep(500)) return;

        telemetry.addLine("Driving backwards 12\"");
        telemetry.update();
        if(protectedSleep(500)) return;
        if(driveDist(-12)) return;
        telemetry.addLine("Desired angle achieved");
        telemetry.update();
        if(protectedSleep(500)) return;

        zero();
    }
}