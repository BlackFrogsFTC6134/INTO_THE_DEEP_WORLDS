package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Basket Specimen", group="Auton")
public class AutonRedBasketSpecimen extends BaseAutoOp {
    public AutonRedBasketSpecimen() {
        super(StartingPosition.RED_BASKET);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        autonInitialize();
        runBasketSpecimenOpMode();
    }
}
