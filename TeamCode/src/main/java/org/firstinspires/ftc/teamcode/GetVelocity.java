package org.firstinspires.ftc.teamcode;

import android.os.AsyncTask;

import com.qualcomm.robotcore.util.ElapsedTime;

public class GetVelocity extends AsyncTask<Void, Void, Void> {
    Encoders encode = null;
    public double speed = 0;
    double dis1 = 0;
    double dis2 = 0;
    ElapsedTime timer;
    public GetVelocity(Encoders e){
        encode = e;
        timer = new ElapsedTime();
    }
    @Override
    protected Void doInBackground(Void ... voids) {
        while(!this.isCancelled()) {
            dis1 = encode.getInches();
            while (timer.milliseconds() < 251) {

            }
            dis2 = encode.getInches();
            speed = Math.abs(dis1 - dis2) / 0.25;
        }
        return null;
    }
}
