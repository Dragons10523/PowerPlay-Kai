package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public abstract class Localization extends Control {
    Vector2D<Double> position = new Vector2D<>(0.0,0.0);
    IntegrationData intD;

    public void startLocalization(){
        thalatte.imu.startAccelerationIntegration(new Position(), new Velocity(), 500);
        intD = new IntegrationData(thalatte.imu, time);
        time.startTime();
    }

    public Vector2D<Double> getPosition(){
        double x = intD.x;
        double y = intD.y;
        // TODO Distance Sensor Sh!t
        position.x = x;
        position.y = y;
        return position;
    }

    public Vector2D<Double> updatePosition(){
        sleep(500);

        intD.integrate();

        return getPosition();
    }
}

class IntegrationData {
    double deltaT = 0.5;

    double[] pastDT = {0.5,0.5,0.5,0.5,0.5};
    int index       = 0;
    int counter     = 0;

    double x = 0, y = 0, vx = 0, vy = 0;

    BNO055IMU imu;
    ElapsedTime time;

    IntegrationData(BNO055IMU imu, ElapsedTime time){
        this.imu  = imu;
        this.time = time;
    }

    void updateTime(){
        if(++counter==10){
            counter %= 10;
            pastDT[index++] = time.seconds();
            index %= 5;

            deltaT = (pastDT[0] + pastDT[1] + pastDT[2] + pastDT[3] + pastDT[4]) / 5.0;
        }
        time.reset();
    }

    void integrate(){
        updateTime();
        Acceleration acc = imu.getAcceleration().toUnit(DistanceUnit.INCH);
        Orientation or   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double theta = -or.firstAngle;
        double axl   = acc.xAccel;
        double ayl   = acc.yAccel;

        double ax = Math.sin(theta)*axl + Math.cos(theta)*ayl;
        double ay = -Math.cos(theta)*axl + Math.sin(theta)*ayl;

        vx += ax*deltaT;
        vy += ay*deltaT;

        x += vx*deltaT;
        y += vy*deltaT;
    }
}