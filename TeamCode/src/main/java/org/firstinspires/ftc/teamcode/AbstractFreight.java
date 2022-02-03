package org.firstinspires.ftc.teamcode;

/* CLASS SUMMARY:
 * Drives the autonomous path used for the freight side, toggle to change between red and blue
 *  */

// ORDER OF OPERATIONS: Preload -> Cycle -> Park

public abstract class AbstractFreight extends AbstractBarcode {
    protected ArmPosition fieldOrientation;

    public void run(FieldSide fieldSide) {
        initializeValues();
        startOpenCV();
        boolean onRed = fieldSide == FieldSide.RED ? true : false;

        waitForStart();

        hueTrackingPipeline.startVideo();

        armControl(ArmPosition.UP); // Raise arm for camera to see
        if(protectedSleep(1000)) return;

        fieldOrientation = getFieldOrientation(fieldSide); // Si

        telemetry.addData("Target", fieldOrientation.toString());
        telemetry.update();

        driveDist(4);

        startTurnTo(onRed ? 4*Math.PI/6 : 2*Math.PI/6); // Turn to face the shipping hub
        while(turningFlag) updateTurnTo();

        driveToShippingHub(fieldSide); // Go to the shipping hub

        drive(-.5, -.5);

        switch(fieldOrientation) {
            case HIGH_FORE:
                if(protectedSleep(300)) return;
                break;
            case MED_FORE:
                if(protectedSleep(400)) return;
                break;
            case LOW_FORE:
                if(protectedSleep(400)) return;
                break;
        }

        drive(0, 0);

        armControl(fieldOrientation);
        if(protectedSleep(500)) return; // Eject the block on the right height
        runIntake(.6);
        if(protectedSleep(1000)) return;

        armControl(ArmPosition.UP); // Raise the arm
        runIntake(0);
        if(protectedSleep(400)) return;

        /*
        while(getRuntime() < 20) {
            armControl(ArmPosition.UP);

            drive(-1, -1); // Back up
            if(protectedSleep(500)) return;
            drive(0, 0);

            startTurnTo(onRed ? Math.PI : 0); // Face the freight
            while(turningFlag) updateTurnTo();
            if(isStopRequested()) return; // Safety

            driveToFreight(); // Go to the freight
            if(isStopRequested()) return; // Safety

            armControl(ArmPosition.LOW_FORE);
            if(protectedSleep(150)) return;
            armControl(ArmPosition.PICKUP);
            if(protectedSleep(150)) return;

            drive(0.7, 0.7); // Grab the freight
            runIntake(-1);
            if(protectedSleep(500)) return;
            drive(-1, -1);
            runIntake(0);
            if(protectedSleep(100)) return;
            drive(0, 0);

            startTurnTo(onRed ? 0 : Math.PI); // Face the shiping hub
            while(turningFlag) updateTurnTo();
            if(isStopRequested()) return; // Safety

            driveToShippingHub(fieldSide); // Drive to the shipping hub
            if(isStopRequested()) return; // Safety
            armControl(ArmPosition.HIGH_FORE);
            while(ahi.arm.isBusy() && opModeIsActive()) sleep(50);
            runIntake(0.5);
            if(protectedSleep(500)) return;
            runIntake(0);
        }*/

        drive(-.7, -.7); // Back up
        if(protectedSleep(400)) return;
        drive(0, 0);

        startTurnTo(onRed ? -0.15 : Math.PI+0.15); // Face the freight
        while(turningFlag) updateTurnTo();

        armControl(ArmPosition.HIGH_FORE); // Lower arm a bit

        driveToFreight(); // Go to the freight

        armControl(ArmPosition.PICKUP); // Place the arm down
        if(protectedSleep(500)) return;

        stopOpenCV();
    }
}
