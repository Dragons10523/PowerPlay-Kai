package org.firstinspires.ftc.teamcode;

public class Geometry {

    public static class Point {
        double x, y;
        Point(double x, double y){
            this.x = x;
            this.y = y;
        }
        double distance(){
            return Math.sqrt(x*x + y*y);
        }
    }

    public static class Wall {
        Point p1, p2;

        Wall(Point p1, Point p2) {
            this.p1 = p1;
            this.p2 = p2;
        }

        Point midpoint(){
            return new Point((p1.x + p2.x) / 2, (p1.y + p2.y) / 2);
        }

        boolean inRange(Line l){
            Line l1      = new Line(l.p1,this.p1   );
            Line l2      = new Line(l.p1,this.p2   );
            Line midline = new Line(l.p1,midpoint());
            double p1mid = l1.theta - midline.theta;
            double p2mid = midline.theta - l2.theta;
            double p1l   = l1.theta - l.theta;
            double p2l   = l.theta - l2.theta;
            return ((p1mid == Control.clamp(p1mid,0,Math.PI) || p1mid == Control.clamp(p1mid,-2 * Math.PI, -Math.PI)) && (p2mid == Control.clamp(p2mid,0,Math.PI) || p2mid == Control.clamp(p2mid,-2 * Math.PI, -Math.PI)))
                    == ((p1l == Control.clamp(p1l,0,Math.PI) || p1l == Control.clamp(p1l,-2 * Math.PI, -Math.PI)) && (p2l == Control.clamp(p2l,0,Math.PI) || p2l == Control.clamp(p2l,-2 * Math.PI, -Math.PI)));
        }
    }


    public static class Line {
        Point p1;
        Point p2;
        double theta;

        Line(Point p1, Point p2) {
            this.p1 = p1;
            this.p2 = p2;
            theta = Localization.collapseAngle(Math.atan2(p2.y - p1.y, p2.x - p1.x));
        }

        Line(Point p, double theta) {
            p1 = p;
            p2 = new Point(p.x + Math.cos(theta), p.y + Math.sin(theta));
            this.theta = Localization.collapseAngle(theta);
        }

        Line(Wall w){
            this(w.p1,w.p2);
        }

        double getDistance() {
            return Math.sqrt(((p2.x-p1.x)*(p2.x-p1.x))+((p2.y-p1.y)*(p2.y-p1.y)));
        }
    }

    Point intersection(Line l1, Line l2) {
        double x1 = l1.p1.x;
        double y1 = l1.p1.y;
        double x2 = l1.p2.x;
        double y2 = l1.p2.y;
        double x3 = l2.p1.x;
        double y3 = l2.p1.y;
        double x4 = l2.p2.x;
        double y4 = l2.p2.y;
        double denominator = ((x1 - x2) * (y3 - y4)) - ((y1 - y2) * (x3 - x4));
        if(denominator == 0) return null;
        return new Point((((((x1 * y2) - (y1 * x2)) * (x3 - x4)) - ((x1 - x2) * ((x3 * y4) - (y3 * x4)))) / denominator),
                ((((x1 * y2) - (y1 * x2)) * (y3 - y4)) - ((y1 - y2) * ((x3 * y4) - (y3 * x4)))) / denominator);
    }

    public Point point(double x, double y){
        return new Point(x, y);
    }
    public Line line(Point p1, Point p2) {
        return new Line(p1,p2);
    }
    public Line line(Point p, double theta){
        return new Line(p,theta);
    }
    public Line line(Wall wall){
        return new Line(wall);
    }

    Wall[] walls;

//    Wall intersects(Lightsaber.LightsaberUnit unit, Point estimate, double theta){
//        double direction = Math.atan2(unit.getYDistance(theta),unit.getXDistance(theta));
//        Line aim = new Line(estimate,direction);
//        return intersects(aim);
//    }
//
//    Wall intersects(Line line){
//        double distance = Double.POSITIVE_INFINITY;
//        Wall   closest  = null;
//
//        for(Wall w : walls){
//            Point  estimate         = line.p1;
//            double theta            = line.theta;
//            Point  intersection     = intersection(line,new Line(w));
//            Line   path             = new Line(estimate,intersection);
//            double possibleDistance = path.getDistance();
//
////            System.out.println("|| 0");
//            if((Math.abs(path.theta - line.theta) > Math.PI / 2) && (Math.abs(path.theta - line.theta) < Math.PI)) continue;
////            System.out.println("|| 1");
//            if(!w.inRange(line)                             ) continue;
////            System.out.println("|| 2");
//            if(possibleDistance        >= distance          ) continue;
////            System.out.println("|| 3");
//
//            distance = possibleDistance;
//            closest  = w;
//        }
//        return closest;
//    }

    Geometry(Wall[] walls){
        this.walls = walls;
    }

    Geometry(){
        Point lowerLeft  = new Point(0, 0);
        Point upperLeft  = new Point(0, 144);
        Point lowerRight = new Point(96, 0);
        Point upperRight = new Point(96, 144);

        Wall left  = new Wall(lowerLeft,upperLeft);
        Wall right = new Wall(lowerRight,upperRight);
        Wall up    = new Wall(upperLeft,upperRight);
        Wall down  = new Wall(lowerLeft,lowerRight);

        walls = new Wall[]{left, right, up, down};
    }

}

// ☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭☭