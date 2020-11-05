package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public abstract class AutoAim extends Localization {
    final double angAdj   = 0.0;
    final double powerAdj = 1.0;

    public Vector2D<Double> getAim(Vector2D<Double> pos2, Double height){
        Vector2D<Double> pos1 = getPosition();
        double relAng         = Math.atan2(pos2.y - pos1.y,pos2.x - pos1.x) + angAdj;
        return new Vector2D<>((double)-thalatte.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + relAng,((1/20)*((Math.sqrt(((pos2.y - pos1.y)*(pos2.y - pos1.y) + (pos2.x - pos1.x)*(pos2.x - pos1.x)) + (height)*(height)) - 20)) + 1)*powerAdj);
    }
}