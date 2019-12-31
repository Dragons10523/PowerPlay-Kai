package org.firstinspires.ftc.teamcode;

public class Distance {
    static double dis = 0;
    static boolean enable = true;
    Distance(double d){
        dis = 0;
    }
    void disable(){
        enable = false;
    }
    void enable(){
        enable = true;
    }
    void setDis(double d){
        dis = d;
    }
    double getDis(){
        return dis;
    }

    void setEnable(boolean b){
        enable= b;
    }
    boolean getEnable(){
        return enable;
    }
}
