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
        LT,
        RT,
        NONE
    }

    private Gamepad slave;

    boolean up    = false;
    boolean down  = false;
    boolean left  = false;
    boolean right = false;
    boolean a     = false;
    boolean b     = false;
    boolean x     = false;
    boolean y     = false;
    boolean start = false;
    boolean back  = false;
    boolean lb    = false;
    boolean rb    = false;
    boolean lsb   = false;
    boolean rsb   = false;
    boolean lt    = false;
    boolean rt    = false;

    public GamepadPrev(Gamepad slave){
        this.slave = slave;
    }

    public void update(){
        up    = slave.dpad_up;
        down  = slave.dpad_down;
        left  = slave.dpad_left;
        right = slave.dpad_right;
        a     = slave.a;
        b     = slave.b;
        x     = slave.x;
        y     = slave.y;
        start = slave.start;
        back  = slave.back;
        lb    = slave.left_bumper;
        rb    = slave.right_bumper;
        lsb   = slave.left_stick_button;
        rsb   = slave.right_stick_button;
        lt    = slave.left_trigger > 0;
        rt    = slave.right_trigger > 0;
    }

    public ButtonEvent getEvent(){
        ButtonEvent ret;

             if(slave.dpad_up            && !up)    ret = ButtonEvent.UP;
        else if(slave.dpad_down          && !down)  ret = ButtonEvent.DOWN;
        else if(slave.dpad_left          && !left)  ret = ButtonEvent.LEFT;
        else if(slave.dpad_right         && !right) ret = ButtonEvent.RIGHT;
        else if(slave.a                  && !a)     ret = ButtonEvent.A;
        else if(slave.b                  && !b)     ret = ButtonEvent.B;
        else if(slave.x                  && !x)     ret = ButtonEvent.X;
        else if(slave.y                  && !y)     ret = ButtonEvent.Y;
        else if(slave.start              && !start) ret = ButtonEvent.START;
        else if(slave.back               && !back)  ret = ButtonEvent.BACK;
        else if(slave.left_bumper        && !lb)    ret = ButtonEvent.LB;
        else if(slave.right_bumper       && !rb)    ret = ButtonEvent.RB;
        else if(slave.left_stick_button  && !lsb)   ret = ButtonEvent.LSB;
        else if(slave.right_stick_button && !rsb)   ret = ButtonEvent.RSB;
        else if(slave.left_trigger   > 0 && !lt)    ret = ButtonEvent.LT;
        else if(slave.right_trigger  > 0 && !rt)    ret = ButtonEvent.RT;
        else                                        ret = ButtonEvent.NONE;

        update();

        return ret;
    }
}
