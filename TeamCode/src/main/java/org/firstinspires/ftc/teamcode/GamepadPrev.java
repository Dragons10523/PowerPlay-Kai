package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadPrev {

    public enum ButtonEvent {
        UP,
        DOWN,
        LEFT,
        RIGHT,
        A,
        B,
        X,
        Y,
        START,
        BACK,
        LB,
        RB,
        LSB,
        RSB,
        NONE,
    }

    private Gamepad slave;

    boolean up = true;
    boolean down = true;
    boolean left = true;
    boolean right = true;
    boolean a = true;
    boolean b = true;
    boolean x = true;
    boolean y = true;
    boolean start = true;
    boolean back = true;
    boolean lb = true;
    boolean rb = true;
    boolean lsb = true;
    boolean rsb = true;

    public GamepadPrev(Gamepad slave){
        this.slave = slave;
    }

    public void update(){
        up = slave.dpad_up;
        down = slave.dpad_down;
        left = slave.dpad_left;
        right = slave.dpad_right;
        a = slave.a;
        b = slave.b;
        x = slave.x;
        y = slave.y;
        start = slave.start;
        back = slave.back;
        lb = slave.left_bumper;
        rb = slave.right_bumper;
        lsb = slave.left_stick_button;
        rsb = slave.right_stick_button;
    }

    public ButtonEvent getEvent(){

        ButtonEvent ret;

        if(slave.dpad_up && !up) ret = ButtonEvent.UP;
        else if(slave.dpad_down && !down) ret = ButtonEvent.DOWN;
        else if(slave.dpad_left && !left) ret = ButtonEvent.LEFT;
        else if(slave.dpad_right && !right) ret = ButtonEvent.RIGHT;
        else if(slave.a && !a) ret = ButtonEvent.A;
        else if(slave.b && !b) ret = ButtonEvent.B;
        else if(slave.x && !x) ret = ButtonEvent.X;
        else if(slave.y && !y) ret = ButtonEvent.Y;
        else if(slave.start && !start) ret = ButtonEvent.START;
        else if(slave.back && !back) ret = ButtonEvent.BACK;
        else if(slave.left_bumper && !lb) ret = ButtonEvent.LB;
        else if(slave.right_bumper && !rb) ret = ButtonEvent.RB;
        else if(slave.left_stick_button && !lsb) ret = ButtonEvent.LSB;
        else if(slave.right_stick_button && !rsb) ret = ButtonEvent.RSB;
        else ret = ButtonEvent.NONE;

        update();

        return ret;

    }

}
