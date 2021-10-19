package org.firstinspires.ftc.teamcode;

/* CLASS SUMMARY:
 * Drives the autonomous path used for the freight side, toggle to change between red and blue
 *  */

// ORDER OF OPERATIONS: Preload -> Cycle -> Park

public abstract class AbstractFreight extends AbstractAutonomous {
    protected ArmPosition fieldOrientation;

    public void run(FieldSide fieldSide) {
        // TODO: Use fieldSide
        boolean onRed = fieldSide == FieldSide.RED;

        initializeValues();

        waitForStart();

        fieldOrientation = getFieldOrientation();

        if(driveDist(6)) return; // Get away from the wall

        startTurnTo(onRed ? 3*Math.PI/4 : Math.PI/4);
        while(turningFlag) updateTurnTo(); // Face the goal

        if(driveDist(16)) return; // Drive to the goal

        armControl(fieldOrientation); // Position the arm

        startTurnTo(onRed ? -Math.PI/4 : 5*Math.PI/4); // Align the arm with the goal
        while(turningFlag) updateTurnTo();

        setFlup(true); // Dump preload
        runIntake(true);
        if(protectedSleep(200)) return;

        setFlup(false); // Stop intake
        runIntake(false);
        armControl(ArmPosition.PICKUP); // Prepare arm for pickup

        startTurnTo(onRed ? Math.PI : 0); // Face the freight
        while(turningFlag) updateTurnTo();

        if(driveDist(20)) return; // Drive to the barrier

        drive(0.7, 0.7); // Drive over the barrier
        if(protectedSleep(500)) return;
        drive(0, 0);
        if(protectedSleep(200)) return;

        startTurnTo(onRed ? Math.PI : 0);
        while(turningFlag) updateTurnTo(); // Realign after crossing the barrier

        runIntake(true); // Attempt to grab freight
        drive(0.4, 0.4);
        if(protectedSleep(700)) return;
        // TODO: Branching program to detect failure
        drive(0, 0); // Stop attempting to grab freight
        runIntake(false);
        if(protectedSleep(200)) return;

        startTurnTo(onRed ? 7*Math.PI/6 : -Math.PI/6);
        while(turningFlag) updateTurnTo(); // Face back towards goal

        if(driveDist(-24)) return; // Drive to goal

        armControl(ArmPosition.HIGH); // Position arm to high goal

        startTurnTo(onRed ? 5*Math.PI/4 : -Math.PI/4); // Align arm with the goal
        while(turningFlag) updateTurnTo();

        setFlup(true); // Dump freight
        runIntake(true);
        if(protectedSleep(200)) return;

        setFlup(false); // Stop intake
        runIntake(false);
        armControl(ArmPosition.START);

        startTurnTo(onRed ? 7*Math.PI/6 : -Math.PI/6); // Face the barrier
        while(turningFlag) updateTurnTo();

        if(driveDist(24)) return; // Park
    }
}
