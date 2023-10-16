package org.firstinspires.ftc.teamcode.opModes;

import com.arcrobotics.ftclib.command.CommandOpMode;


import org.firstinspires.ftc.teamcode.Mushu;
import org.openftc.easyopencv.OpenCvCamera;

public class Auto extends CommandOpMode {
Mushu mushu;

    @Override
    public void initialize() {
        mushu = Mushu.GetInstance(this);
        switch (april);
    }
    
}
