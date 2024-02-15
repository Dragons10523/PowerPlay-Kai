package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mushu;

public class Test extends CommandBase {
    double testNumber = 0;
    Mushu mushu;
    Telemetry telemetry;
    public Test(Mushu mushu, Telemetry telemetry){
        this.mushu = mushu;
        this.telemetry = telemetry;
    }
    public void execute(){
        testNumber = 0;
        while (testNumber < 500) {
            testNumber++;
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            telemetry.addData("testNumber", testNumber);
            telemetry.update();

        }

    }
    public boolean isFinished(){
        return true;
    }
}
