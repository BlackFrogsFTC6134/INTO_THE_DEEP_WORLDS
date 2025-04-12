package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.opmode.BaseAutoOp;

@Disabled
@Autonomous(name="Blue Human", group="Auton")
public class AutonBlueHuman extends BaseAutoOp {
    public AutonBlueHuman() {
        super(StartingPosition.BLUE_HUMAN);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        autonInitialize();
        runHumanSpecimenOpMode();
    }
}
