package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoControl;
import org.firstinspires.ftc.teamcode.RobotClass;
@Autonomous
public class SimplePark extends AutoControl {
    ElapsedTime time = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        super.simpleInit();
        waitForStart();
        double startTime = time.seconds();
        while(startTime + 2 > time.seconds() && !isStopRequested()){
            robot.Motors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(0.5);
            robot.Motors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(0.5);
            robot.Motors.get(RobotClass.MOTORS.BACK_LEFT).setPower(0.5);
            robot.Motors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(0.5);
        }
        robot.Motors.get(RobotClass.MOTORS.FRONT_LEFT).setPower(0);
        robot.Motors.get(RobotClass.MOTORS.FRONT_RIGHT).setPower(0);
        robot.Motors.get(RobotClass.MOTORS.BACK_LEFT).setPower(0);
        robot.Motors.get(RobotClass.MOTORS.BACK_RIGHT).setPower(0);
    }
}
