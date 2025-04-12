package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.opmode.BaseAutoOp;

@Autonomous(name="Auton", group="Auton")
public class AutonMenu extends BaseAutoOp {

    public AutonMenu() {
        super(mStartingPosition);  // mStartingPosition will be set in the menu loop in autonInitialize().
    }

    @Override
    public void runOpMode() throws InterruptedException {
        autonInitialize();
        // Now run your appropriate auton routine based on mStartingPosition.
        // For example, if mStartingPosition is BLUE_BASKET, run runBasketSpecimenOpMode(), etc.
        // You can use a switch or if-else block here:
        switch (mStartingPosition) {
            case BLUE_BASKET:
            case RED_BASKET:
                runBasketSpecimenOpMode();
                break;
            case BLUE_HUMAN:
            case RED_HUMAN:
                runHumanSpecimenOpMode();
                break;
            default:
                runBasketSpecimenOpMode();
                break;
        }
    }
}

