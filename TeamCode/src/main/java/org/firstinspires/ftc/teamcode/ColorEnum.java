package org.firstinspires.ftc.teamcode;

public class ColorEnum {

    public enum Color{
        RED_UP,
        RED_DOWN,
        BLUE_UP,
        BLUE_DOWN;

    }
    public static Color color;
    public ColorEnum(Color color){
        ColorEnum.color = color;
    }

}
