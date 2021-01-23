package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Collection;
import java.util.HashMap;
import java.util.Vector;

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
        if(unit.sensor.getDistance(DistanceUnit.INCH) == DistanceUnit.infinity) return null;
        Geometry.Wall wall = geometry.intersects(unit,position,theta);
        Geometry.Line line = geometry.line(wall);

        line.p1.x -= unit.getXDistance(theta);
        line.p1.y -= unit.getYDistance(theta);
        line.p2.x -= unit.getXDistance(theta);
        line.p2.y -= unit.getYDistance(theta);

        return line;
    }

    public void estimate(){
        Collection<LightsaberUnit> units = this.units.values();
        Vector<Geometry.Line>      lines = new Vector<>();
        Vector<Geometry.Point>    points = new Vector<>();
        for(LightsaberUnit u : units){
            Geometry.Line line;
            if((line = lineate(u)) != null) lines.add(line);
        }
        for(int i = 0; i < lines.size(); i++){
            for(int j = lines.size() - 1; j >= 0; j--){
                Geometry.Point point;
                Geometry.Line  line;
                if(i == j)
                    break;
                if((point = geometry.intersection(lines.get(i), lines.get(j))) == null)
                    continue;
                if((line = geometry.line(point, position)).getDistance() > 2.0)
                    continue;
                points.add(point);
            }
        }
        Vector<Double> xs = new Vector<>();
        Vector<Double> ys = new Vector<>();
        for(Geometry.Point p : points){
            xs.add(p.x);
            ys.add(p.y);
        }
        double xavg = 0, yavg = 0;
        for(double x : xs) xavg += x;
        for(double y : ys) yavg += y;
        xavg /= xs.size();
        yavg /= ys.size();
        position = geometry.point(xavg, yavg);
    }

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
