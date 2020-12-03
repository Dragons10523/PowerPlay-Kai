package org.firstinspires.ftc.teamcode;

import android.os.Build;
import android.view.InputQueue;
import android.view.MotionEvent;
import android.view.View;

public class MouseTrap implements View.OnCapturedPointerListener {

    float x;
    float y;

    @Override
    public boolean onCapturedPointer(View view, MotionEvent event) {
        x = event.getRawX();
        y = event.getRawY();
        return true;
    }
}
