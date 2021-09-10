package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Control extends LinearOpMode {
    Thalatte thalatte;
    double sPower = 1.0;

          double INTAKE = 0.9;
          double FEEDER = 1.0;
    final double CLAMP  = 1.0;

    enum Speed {
        FASTER,
        FAST,
        NORMAL,
        SLOW,
        SLOWER,
        STOP
    }

    public void setSpeed(Speed speed){
        this.speed = speed;
    }

    Speed speed  = Speed.STOP;

    public GamepadPrev prev1;
    public GamepadPrev prev2;

    public ElapsedTime time;

    public void drive(double l, double r) {
        thalatte.backRight. setPower(r);
        thalatte.frontRight.setPower(r);
        thalatte.backLeft.  setPower(l);
        thalatte.frontLeft. setPower(l);
    }

    public void driveVel(double l, double r) {
        ((DcMotorEx)thalatte.backRight ).setVelocity(r);
        ((DcMotorEx)thalatte.frontRight).setVelocity(r);
        ((DcMotorEx)thalatte.backLeft  ).setVelocity(l);
        ((DcMotorEx)thalatte.frontLeft ).setVelocity(l);
    }

    public void driveDist(double dist, double power) {
        int ticks = (int)Math.round(dist*560/(4*Math.PI*1.5));

        thalatte.frontRight.setTargetPosition(ticks+thalatte.frontRight.getCurrentPosition());
        thalatte.frontLeft .setTargetPosition(ticks+thalatte.frontLeft .getCurrentPosition());
        thalatte.backRight .setTargetPosition(ticks+thalatte.backRight .getCurrentPosition());
        thalatte.backLeft  .setTargetPosition(ticks+thalatte.backLeft  .getCurrentPosition());

        thalatte.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        thalatte.frontLeft .setMode(DcMotor.RunMode.RUN_TO_POSITION);
        thalatte.backRight .setMode(DcMotor.RunMode.RUN_TO_POSITION);
        thalatte.backLeft  .setMode(DcMotor.RunMode.RUN_TO_POSITION);

        thalatte.frontRight.setPower(power);
        thalatte.frontLeft .setPower(power);
        thalatte.backRight .setPower(power);
        thalatte.backLeft  .setPower(power);

        while(thalatte.frontRight.isBusy() || thalatte.frontLeft.isBusy() || thalatte.backRight.isBusy() || thalatte.backLeft.isBusy()){
            sleep(10);
        }
        sleep(500);

        thalatte.frontRight.setPower(0);
        thalatte.frontLeft .setPower(0);
        thalatte.backRight .setPower(0);
        thalatte.backLeft  .setPower(0);

        thalatte.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        thalatte.frontLeft .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        thalatte.backRight .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        thalatte.backLeft  .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void shoot(double p) {
        thalatte.shooterFront.setPower( p);
        thalatte.shooterBack. setPower(-p );
    }

    public void shootDistance(double distance) throws InterruptedException{
        double power = distance / 15.0;
        shoot(power);
        Thread.sleep(333);
        shoot(0);
    }

    public boolean toggleShoot(double power){
        boolean set = thalatte.shooterFront.getPower() == 0;
        shoot(set ? power : 0);
        return set;
    }

    public void intake(boolean on) {
        thalatte.intake.setPower(on ? INTAKE : 0);
    }

    public void feeder(boolean on) {
        thalatte.feeder.setPower(on ? FEEDER : 0);
    }

    public void setFeederDirection(double direction){
        FEEDER = Math.signum(direction);
        feeder(thalatte.feeder.getPower() != 0);
    }

    public void setIntakeDirection(double direction){
        INTAKE = Math.signum(direction);
        intake(thalatte.intake.getPower() != 0);
    }

    public void toggleFeederDirection(){
        setFeederDirection(-Math.signum(FEEDER));
    }

    public boolean toggleIntake() {
        boolean set = thalatte.intake.getPower() == 0;
        intake(set);
        return set;
    }

    public void toggleIntakeDirection(){
        setIntakeDirection(-Math.signum(INTAKE));
    }

    public boolean toggleFeeder() {
        boolean set = thalatte.feeder.getPower() == 0;
        feeder(set);
        return set;
    }

    public void vwompArm(double p) {
        thalatte.vwomp.setPower(p);
    }

    public void vwompClamp(boolean c) {
        thalatte.vwompClamp.setPosition(c ? CLAMP : 0);
    }

    public boolean toggleVwompClamp() {
        boolean set = thalatte.vwompClamp.getPosition() == 0;
        vwompClamp(set);
        return set;
    }

    public void alize() {
        thalatte = new Thalatte(hardwareMap);
        prev1    = new GamepadPrev(gamepad1);
        prev2    = new GamepadPrev(gamepad2);
        time     = new ElapsedTime();
        zero();
        speed = Speed.FASTER;
//        thalatte.lights.setConstant(80);
    }

    public static double clamp(double num, double min, double max) {
        return Math.max(Math.min(num, max), min);
    }

    public void driveLoop() {
        double l = -gamepad1.left_stick_y;
        double r = -gamepad1.right_stick_y;

        switch(speed) {
            case SLOWER:
                drive(.2*l,.2*r);
                break;
            case SLOW:
                drive(.4*l,.4*r);
                break;
            case NORMAL:
                drive(.5*l,.5*r);
                break;
            case FAST:
                drive(.7*l,.7*r);
                break;
            case FASTER:
                drive(l,r);
                break;
            case STOP:
                drive(0,0);
                break;
        }
    }

    public void zero(){
        drive(0,0);
        vwompArm(0);
        intake(false);
        feeder(false);
        shoot(0);
    }
}

// ☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭