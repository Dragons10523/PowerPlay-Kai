package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class VecUtils {
    public static double HALF_PI = Math.PI / 2;
    public static double TAU = Math.PI * 2;

    public static VectorF rotateVector(VectorF originalVector, double angle) {
        float x = originalVector.get(0);
        float y = originalVector.get(1);

        originalVector.put(0, (float) ((x * Math.cos(angle)) + (y * Math.sin(angle))));
        originalVector.put(1, (float) ((x * -Math.sin(angle)) + (y * Math.cos(angle))));

        return originalVector;
    }
}
