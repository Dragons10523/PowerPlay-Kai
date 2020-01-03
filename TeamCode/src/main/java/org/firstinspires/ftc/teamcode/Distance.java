package org.firstinspires.ftc.teamcode;

public class Distance {
     double dis = 0;
     boolean enable = true;
    Distance(double d){
        this.dis = 0;
    }
    void disable(){
        enable = false;
    }
    void enable(){
        enable = true;
    }
    void setDis(double d){
        this.dis = d;
    }
    double getDis(){
        return this.dis;
    }

    void setEnable(boolean b){
        enable= b;
    }
    boolean getEnable(){
        return enable;
    }
}
