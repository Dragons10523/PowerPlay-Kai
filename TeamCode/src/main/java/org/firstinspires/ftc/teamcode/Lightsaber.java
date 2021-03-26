package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.motors.GoBILDA5201Series;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.HashMap;
import java.util.Vector;

public class Lightsaber {

    Geometry geometry;
    Geometry.Point position;
    final Geometry.Point MIDPOINT = new Geometry.Point(48,72);
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
            return (Math.cos(theta - (Math.PI / 2))*x) + (Math.sin(theta - (Math.PI / 2))*y);
        }

        double getYDistance(double theta){
            double distance = sensor.getDistance(DistanceUnit.INCH);
            double xMagnitude = distance * Math.cos(direction);
            double yMagnitude = distance * Math.sin(direction);
            double x = xOffset + xMagnitude;
            double y = yOffset + yMagnitude;
            return (-Math.sin(theta - (Math.PI / 2))*x) + (Math.cos(theta - (Math.PI / 2))*y);
        }

    }

    public void setTheta(double theta){
        this.theta = theta;
    }

    Vector<Geometry.Line> lineate(LightsaberUnit unit){
        Vector<Geometry.Line> ret = new Vector<>();

        for(int i = 0; i < geometry.walls.length; i++) {
            if(unit.sensor.getDistance(DistanceUnit.METER) >= 2.0)
                break;
            Geometry.Wall wall = geometry.walls[i];
            Geometry.Line line = new Geometry.Line(wall);
            double p1_1, p2_1, p1_2, p2_2;
            line.p1.x -= unit.getXDistance(theta);
            line.p1.y -= unit.getYDistance(theta);
            line.p2.x -= unit.getXDistance(theta);
            line.p2.y -= unit.getYDistance(theta);

            p1_1 = new Geometry.Line(wall.p1,MIDPOINT).getDistance();
            p2_1 = new Geometry.Line(wall.p2,MIDPOINT).getDistance();
            p1_2 = new Geometry.Line(line.p1,MIDPOINT).getDistance();
            p2_2 = new Geometry.Line(line.p2,MIDPOINT).getDistance();

            if(p1_1 <= p1_2 && p2_1 <= p2_2)
                continue;

            ret.add(line);
        }

        return ret;
    }

    public void estimate() {

        Collection<LightsaberUnit> units = this.units.values();
        Vector<Geometry.Line>      lines = new Vector<>();
        Vector<Geometry.Point>    points = new Vector<>();

        for(LightsaberUnit u : units) {
            Vector<Geometry.Line> ls = lineate(u);
            lines.addAll(ls);
        }

        System.out.println("||| POINTS REPORT");

        for(int i = 0; i < lines.size(); i++) {
            for(int j = lines.size() - 1; j >= 0; j--) {
                Geometry.Point point;
                if(i == j)
                    break;
                if((point = geometry.intersection(lines.get(i), lines.get(j))) == null)
                    continue;
                System.out.printf("X: %f, Y: %f\n", point.x, point.y);
                points.add(point);
            }
        }

        double xs = 0;
        double ys = 0;

        int removed = 0;

        for(int i = 0; i < points.size(); i++) {

            double x = points.get(i - removed).x;
            double y = points.get(i - removed).y;

            if(Control.clamp(x,0,96) != x || Control.clamp(y,0,144) != y) {
                points.remove(i-(removed++));
                continue;
            }

            xs += x;
            ys += y;

        }

        if(points.size() == 0)
            return;

        xs /= (double)(points.size());
        ys /= (double)(points.size());

        position = new Geometry.Point(xs,ys);

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

// ☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭