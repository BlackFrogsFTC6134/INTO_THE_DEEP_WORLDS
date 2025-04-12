package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmode.BaseAutoOp;

@Disabled
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
