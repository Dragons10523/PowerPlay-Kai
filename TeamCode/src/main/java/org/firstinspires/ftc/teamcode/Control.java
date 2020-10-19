package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

public abstract class Control extends LinearOpMode implements Gamepad.GamepadCallback {
    Thalatte thalatte;

    // CONSTANTS
    final double INTAKE = 1.0;
    final double CLAMP = 1.0;

    private ControllerSpec spec = null;

    public GamepadPrev prev1 = new GamepadPrev(gamepad1);

    public GamepadPrev prev2 = new GamepadPrev(gamepad2);

    public interface ControllerSpec {

        enum GamepadId {
            GAMEPAD_1,
            GAMEPAD_2,
        }

        void dpadUp(GamepadId id);
        void dpadDown(GamepadId id);
        void dpadLeft(GamepadId id);
        void dpadRight(GamepadId id);
        void aButton(GamepadId id);
        void bButton(GamepadId id);
        void xButton(GamepadId id);
        void yButton(GamepadId id);
        void startButton(GamepadId id);
        void backButton(GamepadId id);
        void lBumper(GamepadId id);
        void rBumper(GamepadId id);
        void lStickButton(GamepadId id);
        void rStickButton(GamepadId id);

        void sticks(GamepadId id, float lx, float ly, float rx, float ry);

        void triggers(GamepadId id, float l, float r);

    }

    public void drive(double l, double r) {
        thalatte.backRight.setPower(r);
        thalatte.frontRight.setPower(r);
        thalatte.backLeft.setPower(l);
        thalatte.frontLeft.setPower(l);
    }

    public void shoot(double p) {
        thalatte.shooterFront.setPower(p);
        thalatte.shooterBack.setPower(p);
    }

    public void intake(boolean on) {
        thalatte.intake.setPower(on ? INTAKE : 0);
    }

    public boolean toggleIntake() {
        boolean set = thalatte.intake.getPower() == 0;
        intake(set);
        return set;
    }

    public void vwompArm(double p) {
        thalatte.vwomp.setPower(p);
    }

    public void vwompClamp(boolean c) {
        thalatte.vwompClampLeft.setPosition(c ? CLAMP : 0);
        thalatte.vwompClampRight.setPosition(c ? CLAMP : 0);
    }

    public boolean toggleVwompClamp() {
        boolean set = thalatte.vwompClampLeft.getPosition() == 0;
        vwompClamp(set);
        return set;
    }

    public void alize() {
        thalatte = new Thalatte(hardwareMap);
        try {
            Gamepad copy = gamepad1;
            gamepad1 = new Gamepad(this);
            gamepad1.copy(copy);
            copy = gamepad2;
            gamepad2 = new Gamepad(this);
            gamepad2.copy(copy);
        } catch(RobotCoreException e) {
            telemetry.addLine(e.getMessage());
            telemetry.update();
        }

    }

    public double clamp(double num, double min, double max) {
        return Math.max(Math.min(num, max), min);
    }

    private ControllerSpec.GamepadId getId(Gamepad gamepad){
        if(gamepad.id == gamepad1.id){
            return ControllerSpec.GamepadId.GAMEPAD_1;
        }
        return ControllerSpec.GamepadId.GAMEPAD_2;
    }

    @Override
    public void gamepadChanged(Gamepad gamepad) {
        if(opModeIsActive() && spec != null) {
            switch(getId(gamepad)){

                case GAMEPAD_1:

                    switch(prev1.getEvent()){
                        case UP:
                            spec.dpadUp(ControllerSpec.GamepadId.GAMEPAD_1);
                            break;
                        case DOWN:
                            spec.dpadDown(ControllerSpec.GamepadId.GAMEPAD_1);
                            break;
                        case LEFT:
                            spec.dpadLeft(ControllerSpec.GamepadId.GAMEPAD_1);
                            break;
                        case RIGHT:
                            spec.dpadRight(ControllerSpec.GamepadId.GAMEPAD_1);
                            break;
                        case A:
                            spec.aButton(ControllerSpec.GamepadId.GAMEPAD_1);
                            break;
                        case B:
                            spec.bButton(ControllerSpec.GamepadId.GAMEPAD_1);
                            break;
                        case X:
                            spec.xButton(ControllerSpec.GamepadId.GAMEPAD_1);
                            break;
                        case Y:
                            spec.yButton(ControllerSpec.GamepadId.GAMEPAD_1);
                            break;
                        case START:
                            spec.startButton(ControllerSpec.GamepadId.GAMEPAD_1);
                            break;
                        case BACK:
                            spec.backButton(ControllerSpec.GamepadId.GAMEPAD_1);
                            break;
                        case LB:
                            spec.lBumper(ControllerSpec.GamepadId.GAMEPAD_1);
                            break;
                        case RB:
                            spec.rBumper(ControllerSpec.GamepadId.GAMEPAD_1);
                            break;
                        case LSB:
                            spec.lStickButton(ControllerSpec.GamepadId.GAMEPAD_1);
                            break;
                        case RSB:
                            spec.rStickButton(ControllerSpec.GamepadId.GAMEPAD_1);
                            break;
                        case NONE:
                            spec.sticks(ControllerSpec.GamepadId.GAMEPAD_1,
                                        gamepad.left_stick_x,
                                       -gamepad.left_stick_y,
                                        gamepad.right_stick_x,
                                       -gamepad.right_stick_y);
                            spec.triggers(ControllerSpec.GamepadId.GAMEPAD_1,
                                          gamepad.left_trigger,
                                          gamepad.right_trigger);
                            break;
                    }

                    break;

                case GAMEPAD_2:

                    switch(prev2.getEvent()){
                        case UP:
                            spec.dpadUp(ControllerSpec.GamepadId.GAMEPAD_2);
                            break;
                        case DOWN:
                            spec.dpadDown(ControllerSpec.GamepadId.GAMEPAD_2);
                            break;
                        case LEFT:
                            spec.dpadLeft(ControllerSpec.GamepadId.GAMEPAD_2);
                            break;
                        case RIGHT:
                            spec.dpadRight(ControllerSpec.GamepadId.GAMEPAD_2);
                            break;
                        case A:
                            spec.aButton(ControllerSpec.GamepadId.GAMEPAD_2);
                            break;
                        case B:
                            spec.bButton(ControllerSpec.GamepadId.GAMEPAD_2);
                            break;
                        case X:
                            spec.xButton(ControllerSpec.GamepadId.GAMEPAD_2);
                            break;
                        case Y:
                            spec.yButton(ControllerSpec.GamepadId.GAMEPAD_2);
                            break;
                        case START:
                            spec.startButton(ControllerSpec.GamepadId.GAMEPAD_2);
                            break;
                        case BACK:
                            spec.backButton(ControllerSpec.GamepadId.GAMEPAD_2);
                            break;
                        case LB:
                            spec.lBumper(ControllerSpec.GamepadId.GAMEPAD_2);
                            break;
                        case RB:
                            spec.rBumper(ControllerSpec.GamepadId.GAMEPAD_2);
                            break;
                        case LSB:
                            spec.lStickButton(ControllerSpec.GamepadId.GAMEPAD_2);
                            break;
                        case RSB:
                            spec.rStickButton(ControllerSpec.GamepadId.GAMEPAD_2);
                            break;
                        case NONE:
                            spec.sticks(ControllerSpec.GamepadId.GAMEPAD_2,
                                        gamepad.left_stick_x,
                                       -gamepad.left_stick_y,
                                        gamepad.right_stick_x,
                                       -gamepad.right_stick_y);
                            spec.triggers(ControllerSpec.GamepadId.GAMEPAD_2,
                                          gamepad.left_trigger,
                                          gamepad.right_trigger);
                            break;
                    }

                    break;
            }
        }
    }

    public void setSpec(ControllerSpec spec) {
        this.spec = spec;
    }

}