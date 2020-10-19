package org.firstinspires.ftc.teamcode;

import java.util.Vector;

public abstract class Localization extends Control {
    public Vector<Double> position = new Vector<>(2);

    public Vector<Double> getPosition() {
        // TODO update position
        return position;
    }
}
