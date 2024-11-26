package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClass;
@TeleOp
public class ServoTestExtensionOpMode extends LinearOpMode {
    RobotClass robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotClass(hardwareMap);
        robot.initMotorsComp();

        waitForStart();
        boolean firstPressA = true;
        boolean firstPressB = true;
        boolean firstPressY = true;
        boolean firstPressX = true;
        double servoPos1 = 0;
        double servoPos2 = 0;
        //servoPos1 rest = 0.78 ish
        //servoPos1 extended = 0 ish
//        robot.Servos.get(RobotClass.SERVOS.ARM_LEFT).setPosition(servoPos1);
//        robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(servoPos2);
        robot.Servos.get(RobotClass.SERVOS.INTAKE_SERVO).setPosition(servoPos1);
        while(opModeIsActive()){
            if(!gamepad1.a){
                firstPressA = true;
            }
            if(gamepad1.a && firstPressA){
                firstPressA = false;
                servoPos1 += .01;
            }
            if(!gamepad1.b){
                firstPressB = true;
            }
            if(gamepad1.b && firstPressB){
                firstPressB = false;
                servoPos1 -= .01;
            }
            //0.94 Closed
            //0.6 OPEN
            robot.Servos.get(RobotClass.SERVOS.INTAKE_SERVO).setPosition(servoPos1);
//            if(!gamepad1.x){
//                firstPressX = true;
//            }
//            if(gamepad1.x && firstPressX){
//                firstPressX = false;
//                servoPos2 += .01;
//                robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(servoPos2);
//            }
//            if(!gamepad1.y){
//                firstPressY = true;
//            }
//            if(gamepad1.y && firstPressY){
//                firstPressY = false;
//                servoPos2 -= .01;
//                robot.Servos.get(RobotClass.SERVOS.ARM_RIGHT).setPosition(servoPos2);
//            }
            telemetry.addData("servoPos1", servoPos1);
//            telemetry.addData("servoPos2", servoPos2);
            telemetry.update();
            //servoPos1 IN : 0.67
            //servoPos1 OUT : 0.42
        }
    }
}
