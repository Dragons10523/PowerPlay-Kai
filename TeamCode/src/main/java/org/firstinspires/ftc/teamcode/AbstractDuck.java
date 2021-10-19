package org.firstinspires.ftc.teamcode;

/* CLASS SUMMARY:
 * Drives the autonomous path used for the duck side, toggle to change between red and blue
 *  */

// ORDER OF OPERATIONS: Ducks -> Preload -> Cycle -> Park

public abstract class AbstractDuck extends AbstractAutonomous {
    protected ArmPosition fieldOrientation;

    public void run(FieldSide fieldSide) {
        // TODO: Use fieldSide
        boolean onRed = fieldSide == FieldSide.RED;

        initializeValues();

        waitForStart();

        fieldOrientation = getFieldOrientation();

        if(driveDist(6)) return; // Get away from the wall

        startTurnTo(onRed ? 0 : Math.PI);
        while(turningFlag) updateTurnTo(); // Turn to face the carousel

        if(driveDist(-18)) return; // Drive up to carousel

        startTurnTo(onRed ? Math.PI/2 : Math.PI);
        while(turningFlag) updateTurnTo(); // Put the spinner on the carousel

        playDDR(1); // Spin the carousel
        if(protectedSleep(1500)) return;
        playDDR(0);

        startTurnTo(onRed ? 7*Math.PI/6 : -Math.PI/6);
        while(turningFlag) updateTurnTo(); // Turn to face away from the goal

        if(driveDist(-28)) return; // Reverse to the goal

        startTurnTo(onRed ? 5*Math.PI/4 : -Math.PI/4);
        while(turningFlag) updateTurnTo(); // Turn to face away from the goal

        armControl(fieldOrientation); // Position arm
        if(protectedSleep(700)) return;

        setFlup(true); // Dump duck
        runIntake(true);
        if(protectedSleep(200)) return;

        setFlup(false); // Close and stop intake
        runIntake(false);
        armControl(ArmPosition.PICKUP); // Set arm for pickup

        startTurnTo(3*Math.PI/2);
        while(turningFlag) updateTurnTo(); // Face the wall

        if(driveDist(16)) return; // Drive to the wall

        startTurnTo(onRed ? Math.PI : 0);
        while(turningFlag) updateTurnTo(); // Face the freight

        if(driveDist(28)) return; // Drive to the barrier

        drive(.7, .7); // Drive over the barrier
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
        if(protectedSleep(650)) return;

        setFlup(true); // Dump freight
        runIntake(true);
        if(protectedSleep(200)) return;

        setFlup(false); // Stop intake
        runIntake(false);
        armControl(ArmPosition.START);

        if(driveDist(24)) return; // Park
    }
}
