package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

@Disabled
@Config
@TeleOp
public class ArmPivotManual extends OpMode {

    public static double ARM_UP_POWER = 0.7;
    public static double ARM_DOWN_POWER = -0.25;

    private DcMotorEx pivotMotor;
    private TouchSensor switchPivotHome;

    private double ticks_per_rev = 537.7; //goBilda 312 RPM
    private double gear_ratio = 28.0;
    private double ticks_per_deg = (ticks_per_rev * gear_ratio) / 360;

    //private final double ticks_per_deg = 2786.2 / 360; // goBilda 5203 60 RPM motor/360 degrees

    private double armPower = 0;
    private double armPosition = 0;
    private double armAngle = 0;

    @Override
    public void init() {

        pivotMotor = hardwareMap.get(DcMotorEx.class, "motorPivot");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setDirection(DcMotor.Direction.REVERSE);

        switchPivotHome = hardwareMap.get(TouchSensor.class, "switchPivotHome");

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void loop() {

        if (gamepad1.y) {
            armPower = ARM_UP_POWER;
        } else if (gamepad1.a) {
            if (!switchPivotHome.isPressed())
                armPower = ARM_DOWN_POWER;
        } else
            armPower = 0;

        pivotMotor.setPower(armPower);

        armPosition = pivotMotor.getCurrentPosition();
        armAngle = (armPosition / ticks_per_deg) - 50.2;

        telemetry.addData("position", armPosition);
        //telemetry.addData("positionR", armPositionR);
        //telemetry.addData("position", pivotPosition);
        telemetry.addData("angle", armAngle);
        telemetry.addData("power", armPower);
        telemetry.addData("current", pivotMotor.getCurrent(CurrentUnit.AMPS));
        //telemetry.addData("currentR", rightRotationMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}