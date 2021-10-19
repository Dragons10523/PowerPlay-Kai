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

        driveDist(6); // Get away from the wall

        startTurnTo(onRed ? 0 : Math.PI);
        while(turningFlag) updateTurnTo(); // Turn to face the carousel

        driveDist(-18); // Drive up to carousel

        startTurnTo(onRed ? Math.PI/2 : Math.PI);
        while(turningFlag) updateTurnTo(); // Put the spinner on the carousel

        playDDR(1); // Spin the carousel
        sleep(1500);
        playDDR(0);

        startTurnTo(onRed ? 7*Math.PI/6 : -Math.PI/6);
        while(turningFlag) updateTurnTo(); // Turn to face away from the goal

        driveDist(-28); // Reverse to the goal

        startTurnTo(onRed ? 5*Math.PI/4 : -Math.PI/4);
        while(turningFlag) updateTurnTo(); // Turn to face away from the goal

        armControl(fieldOrientation); // Position arm
        sleep(700);

        setFlup(true); // Dump duck
        runIntake(true);
        sleep(200);

        setFlup(false); // Close and stop intake
        runIntake(false);
        armControl(ArmPosition.PICKUP); // Set arm for pickup

        startTurnTo(3*Math.PI/2);
        while(turningFlag) updateTurnTo(); // Face the wall

        driveDist(16); // Drive to the wall

        startTurnTo(onRed ? Math.PI : 0);
        while(turningFlag) updateTurnTo(); // Face the freight

        driveDist(28); // Drive to the barrier

        drive(.7, .7); // Drive over the barrier
        sleep(500);
        drive(0, 0);
        sleep(200);

        startTurnTo(onRed ? Math.PI : 0);
        while(turningFlag) updateTurnTo(); // Realign after crossing the barrier

        runIntake(true); // Attempt to grab freight
        drive(0.4, 0.4);
        sleep(700);
        // TODO: Branching program to detect failure
        drive(0, 0); // Stop attempting to grab freight
        runIntake(false);
        sleep(200);

        startTurnTo(onRed ? 7*Math.PI/6 : -Math.PI/6);
        while(turningFlag) updateTurnTo(); // Face back towards goal

        driveDist(-24); // Drive to goal

        armControl(ArmPosition.HIGH); // Position arm to high goal
        sleep(650);

        setFlup(true); // Dump freight
        runIntake(true);
        sleep(200);

        setFlup(false); // Stop intake
        runIntake(false);
        armControl(ArmPosition.START);

        driveDist(24); // Park
    }
}
