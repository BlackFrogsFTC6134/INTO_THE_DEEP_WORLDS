package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Basket Specimen", group="Auton")
public class AutonBlueBasketSpecimen extends BaseAutoOp {
    public AutonBlueBasketSpecimen() {
        super(StartingPosition.BLUE_BASKET);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        autonInitialize();
        runBasketSpecimenOpMode();
    }
}
