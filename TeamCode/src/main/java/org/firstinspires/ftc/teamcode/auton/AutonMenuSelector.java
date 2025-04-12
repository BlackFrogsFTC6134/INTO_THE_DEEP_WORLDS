package org.firstinspires.ftc.teamcode.auton;

//import com.qualcomm.robotcore.eventloop.opmode.Telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmode.BaseOp.StartingPosition;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.concurrent.TimeUnit;

public class AutonMenuSelector {

    public static final long DEBOUNCE_DELAY_MS = 250;;

    // Helper method that runs inside your init loop (i.e., before waitForStart() is called)
    // and returns the selected starting position.
    public static StartingPosition selectStartingPosition(Gamepad gamepad, Telemetry telemetry, StartingPosition currentSelection) {

        // Loop until the A button is pressed.
        while (!gamepad.a) {
            // Display the menu on telemetry.
            telemetry.clear();
            telemetry.addLine("=== Autonomous Menu ===");
            telemetry.addLine("D-Pad UP/DOWN: Change starting position");
            telemetry.addLine("Press A to confirm your choice");
            telemetry.addData("Current selection:", currentSelection.toString());
            telemetry.update();

            // Check D-pad input to update currentSelection.
            if (gamepad.dpad_up) {
                currentSelection = nextStartingPosition(currentSelection);
                sleep(DEBOUNCE_DELAY_MS);
            } else if (gamepad.dpad_down) {
                currentSelection = previousStartingPosition(currentSelection);
                sleep(DEBOUNCE_DELAY_MS);
            }
            // Allow some time for the loop to avoid hogging the CPU.
            sleep(50);
        }
        // Clear telemetry one more time on confirmation.
        telemetry.clear();
        telemetry.addLine("Auton CONFIRMED: " + currentSelection.toString());
        telemetry.update();
        sleep(1300);
        // Wait for the A button to be released before returning,
        // so that we don't immediately re-enter this loop if called again.
        while (gamepad.a) {
            sleep(50);
        }
        return currentSelection;
    }

    private static StartingPosition nextStartingPosition(StartingPosition current) {
        switch(current) {
            case BLUE_BASKET:  return StartingPosition.BLUE_HUMAN;
            case BLUE_HUMAN:   return StartingPosition.RED_BASKET;
            case RED_BASKET:   return StartingPosition.RED_HUMAN;
            case RED_HUMAN:    return StartingPosition.BLUE_BASKET;
            default:           return StartingPosition.BLUE_BASKET;
        }
    }

    private static StartingPosition previousStartingPosition(StartingPosition current) {
        switch(current) {
            case BLUE_BASKET:  return StartingPosition.RED_HUMAN;
            case RED_HUMAN:    return StartingPosition.RED_BASKET;
            case RED_BASKET:   return StartingPosition.BLUE_HUMAN;
            case BLUE_HUMAN:   return StartingPosition.BLUE_BASKET;
            default:           return StartingPosition.BLUE_BASKET;
        }
    }

    // Simple sleep method using Thread.sleep()

    private static void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            // Handle interruption as needed.
            Thread.currentThread().interrupt();
        }
    }
}

