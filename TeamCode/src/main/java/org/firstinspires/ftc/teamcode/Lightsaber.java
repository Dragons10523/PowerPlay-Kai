package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;

public class Lightsaber {

    Geometry geometry;
    Geometry.Point position;
    double theta;

    public static class LightsaberUnit {
        DistanceSensor sensor;
        double xOffset, yOffset, direction;

        LightsaberUnit(double xOffset, double yOffset, double direction){
            this.xOffset   = xOffset;
            this.yOffset   = yOffset;
            this.direction = direction;
        }

        double getXDistance(double theta){
            double distance = sensor.getDistance(DistanceUnit.INCH);
            double xMagnitude = distance * Math.cos(direction);
            double yMagnitude = distance * Math.sin(direction);
            double x = xOffset + xMagnitude;
            double y = yOffset + yMagnitude;
            return (Math.cos(theta)*x) + (Math.sin(theta)*y);
        }

        double getYDistance(double theta){
            double distance = sensor.getDistance(DistanceUnit.INCH);
            double xMagnitude = distance * Math.cos(direction);
            double yMagnitude = distance * Math.sin(direction);
            double x = xOffset + xMagnitude;
            double y = yOffset + yMagnitude;
            return (-Math.sin(theta)*x) + (Math.cos(theta)*y);
        }

    }

    public void setTheta(double theta){
        this.theta = theta;
    }

    Geometry.Line lineate(LightsaberUnit unit){
        Geometry.Wall wall = geometry.intersects(unit,position,theta);
        Geometry.Line line = geometry.line(wall);

        line.p1.x -= unit.getXDistance(theta);
        line.p1.y -= unit.getYDistance(theta);
        line.p2.x -= unit.getXDistance(theta);
        line.p2.y -= unit.getYDistance(theta);

        return line;
    }

    final double range = 2.0;

    HashMap<String,LightsaberUnit> units;

    Lightsaber(HashMap<String,LightsaberUnit> units, HardwareMap hwmap, Geometry geometry) {
        this.units = units;
        this.geometry = geometry;
        position = geometry.point(0,0);
        for(String key : units.keySet()) {
            this.units.get(key).sensor = hwmap.get(DistanceSensor.class, key);
        }
    }

}
