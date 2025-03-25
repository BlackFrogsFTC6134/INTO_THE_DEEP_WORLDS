package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

//@Disabled
@Config
@TeleOp
public class ArmExtendManual extends OpMode {

    public static double EXTEND_UP_POWER = 0.5;
    public static double EXTEND_DOWN_POWER = -0.5;

    private double ticks_per_rev = 537.7; //goBilda 312 RPM
    private double gear_ratio = 1;
    private double pulley_diameter_inch = 38.2 / 25.4; //goBilda GT2 60T 38.2mm diameter
    private double ticks_per_inch = (ticks_per_rev * gear_ratio) / (pulley_diameter_inch * Math.PI);

    private DcMotorEx extendMotor1;
    private DcMotorEx extendMotor2;
    private DcMotorEx extendMotor3;

    private double extendPower = 0;
    private double extendPosition = 0;
    //private double armAngle = 0;

    @Override
    public void init() {

        extendMotor1 = hardwareMap.get(DcMotorEx.class, "motorExtend1");
        extendMotor2 = hardwareMap.get(DcMotorEx.class, "motorExtend2");
        extendMotor3 = hardwareMap.get(DcMotorEx.class, "motorExtend3");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        extendMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor1.setDirection(DcMotor.Direction.FORWARD);

        extendMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor2.setDirection(DcMotor.Direction.FORWARD);

        extendMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor3.setDirection(DcMotor.Direction.FORWARD);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void loop() {

        if (gamepad1.y) {
            extendPower = EXTEND_UP_POWER;
        } else if (gamepad1.a) {
            extendPower = EXTEND_DOWN_POWER;
        } else
            extendPower = 0;

        extendMotor1.setPower(extendPower);
        extendMotor2.setPower(extendPower);
        extendMotor3.setPower(extendPower);

        telemetry.addData("position1", extendMotor1.getCurrentPosition());
        telemetry.addData("position2", extendMotor2.getCurrentPosition());
        telemetry.addData("position3", extendMotor3.getCurrentPosition());
        telemetry.addData("distance", extendMotor1.getCurrentPosition() / ticks_per_inch);
        //telemetry.addData("ff", ff);
        telemetry.addData("power", extendPower);
        telemetry.addData("current", extendMotor1.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}