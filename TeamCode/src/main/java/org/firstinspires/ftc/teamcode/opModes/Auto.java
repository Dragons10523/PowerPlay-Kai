package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;


import org.firstinspires.ftc.teamcode.Mushu;
import org.openftc.easyopencv.OpenCvCamera;

public class Auto extends CommandOpMode {
Mushu mushu;
int april;
//TODO: set up basic auto
    @Override
    public void initialize() {

        // plan to make a switch case based on the april tag
        // first need to place pixel on spike


        mushu = Mushu.GetInstance(this);

        switch (april){
            case 1:
            //april tag left

            case 2:
            //april tag middle

            case 3:
            //april tag right

        }
    }
    
}
