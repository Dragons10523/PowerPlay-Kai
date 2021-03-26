package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Control extends LinearOpMode {
    Thalatte thalatte;
    double sPower = 1;

          double INTAKE = -1.0;
          double FEEDER   = 1.0;
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

    public void driveVel(double l, double r) {
        ((DcMotorEx)thalatte.backRight ).setVelocity(r);
        ((DcMotorEx)thalatte.frontRight).setVelocity(r);
        ((DcMotorEx)thalatte.backLeft  ).setVelocity(l);
        ((DcMotorEx)thalatte.frontLeft ).setVelocity(l);
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
        feeder(false);
        shoot(0);
    }

    public void disableMotor(DcMotor motor){
        if(((DcMotorEx)motor).isMotorEnabled()){
            ((DcMotorEx)motor).setMotorDisable();
        }
    }

    public void enableMotor(DcMotor motor){
        if(!((DcMotorEx)motor).isMotorEnabled()){
            ((DcMotorEx)motor).setMotorEnable();
        }
    }

    public void disableIntake(){
        disableMotor(thalatte.intake);
    }

    public void disableShooter(){
        disableMotor(thalatte.shooterBack);
        disableMotor(thalatte.shooterFront);
    }

    public void disableDrive(){
        disableMotor(thalatte.frontLeft);
        disableMotor(thalatte.frontRight);
        disableMotor(thalatte.backLeft);
        disableMotor(thalatte.backRight);
    }

    public void disableVwomp(){
        disableMotor(thalatte.vwomp);
    }

    public void disableAll(){
        disableDrive();
        disableIntake();
        disableShooter();
        disableVwomp();
    }

    public void enableIntake(){
        enableMotor(thalatte.intake);
    }

    public void enableShooter(){
        enableMotor(thalatte.shooterBack);
        enableMotor(thalatte.shooterFront);
    }

    public void enableDrive(){
        enableMotor(thalatte.frontLeft);
        enableMotor(thalatte.frontRight);
        enableMotor(thalatte.backLeft);
        enableMotor(thalatte.backRight);
    }

    public void enableVwomp(){
        enableMotor(thalatte.vwomp);
    }

    public void enableAll(){
        enableDrive();
        enableIntake();
        enableShooter();
        enableVwomp();
    }

    @Deprecated
    public void disableForever(){
        new Thread(new Runnable() {
            @Override
            public void run() {
                while (true) {
                    disableAll();
                    disableForever();
                    sleep(10);
                }
            }
        });
    }
}

// ☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭