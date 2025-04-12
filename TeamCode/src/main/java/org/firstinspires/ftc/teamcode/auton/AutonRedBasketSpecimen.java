package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmode.BaseAutoOp;

@Disabled
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
