package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Control extends LinearOpMode {
    Thalatte thalatte;

    final double INTAKE = 1.0;
          double FLUP   = 1.0;
    final double CLAMP  = 1.0;

    enum Speed {
        FASTER,
        FAST,
        NORMAL,
        SLOW,
        SLOWER,
        STOP
    }

    enum PowerShot {
        LEFT,
        CENTER,
        RIGHT
    }

    enum Vwomp {
        DOWN,
        UP
    }

    enum Clamp {
        OPENED,
        CLOSED
    }

    Vwomp vwomp = Vwomp.UP;
    Clamp clamp = Clamp.CLOSED;

    private int psToInt(PowerShot ps){
        switch (ps){
            case LEFT:
                return 0;
            case CENTER:
                return 1;
            case RIGHT:
                return 2;
        }
        return 0;
    }

    private PowerShot intToPs(int i){
        switch (i){
            case 0:
                return PowerShot.LEFT;
            case 1:
                return PowerShot.CENTER;
            case 2:
                return PowerShot.RIGHT;
        }
        return PowerShot.LEFT;
    }

    public void setSpeed(Speed speed){
        this.speed = speed;
    }

    Speed speed  = Speed.STOP;
    PowerShot ps = PowerShot.LEFT;

    public GamepadPrev prev1;
    public GamepadPrev prev2;

    public ElapsedTime time;



    public void drive(double l, double r) {
        thalatte.backRight. setPower(r);
        thalatte.frontRight.setPower(r);
        thalatte.backLeft.  setPower(l);
        thalatte.frontLeft. setPower(l);
    }

    public void shoot(double p) {
        thalatte.shooterFront.setPower(p);
        thalatte.shooterBack. setPower(p);
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

    public void flup(boolean on) {
        thalatte.flup1.setPower(  on ? FLUP : 0);
        thalatte.flup2.setPower(-(on ? FLUP : 0));
    }

    public void setFlupDirection(int direction){
        FLUP = Math.signum(direction);
    }

    public boolean toggleIntake() {
        boolean set = thalatte.intake.getPower() == 0;
        intake(set);
        flup(set);
        return set;
    }

    public void vwompArm(double p) {
        thalatte.vwomp.setPower(p);
    }

    public void vwompClamp(boolean c) {
        thalatte.vwompClampLeft. setPosition(c ? CLAMP : 0);
        thalatte.vwompClampRight.setPosition(c ? CLAMP : 0);
    }

    public boolean toggleVwompClamp() {
        boolean set = thalatte.vwompClampLeft.getPosition() == 0;
        vwompClamp(set);
        return set;
    }

    public void alize() {
        thalatte = new Thalatte(hardwareMap);
        prev1    = new GamepadPrev(gamepad1);
        prev2    = new GamepadPrev(gamepad2);
        time     = new ElapsedTime();
        zero();
        speed = Speed.NORMAL;

    }

    public double clamp(double num, double min, double max) {
        return Math.max(Math.min(num, max), min);
    }

    public void powerShotCycle(int i){
        ps = intToPs((i + psToInt(ps))%3);
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
        flup(false);
        shoot(0);
    }
}

// ☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭