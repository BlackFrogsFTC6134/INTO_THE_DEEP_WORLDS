package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Blue Human", group="Auton")
public class AutonBlueHuman extends BaseAutoOp {
    public AutonBlueHuman() {
        super(StartingPosition.BLUE_HUMAN);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        autonInitialize();
        runHumanSpecimenOpMode2();
    }
}
