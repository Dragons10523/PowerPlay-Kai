package SensorTests;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.regex.Pattern;

@TeleOp(name = "LEDS")
@Disabled
public class LEDS extends OpMode {

    RevBlinkinLedDriver blinkin;
    @Override
    public void init() {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "pretty");

    }

    @Override
    public void loop() {
        RevBlinkinLedDriver.BlinkinPattern pattern = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE;
        blinkin.setPattern(pattern);
    }
}
