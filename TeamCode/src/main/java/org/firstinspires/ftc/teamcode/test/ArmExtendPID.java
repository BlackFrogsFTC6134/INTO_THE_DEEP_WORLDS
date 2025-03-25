package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;
import java.util.List;

//@Disabled
@Config
@TeleOp
public class ArmExtendPID extends OpMode {
    private PIDFController pidfController;

    //public static double p = 0.0, i = 0.0, d = 0.0;

    //public static double kP = 0.009, kI = 0, kD = 0.0002, kF = 0;
    public static double kP = 0.015, kI = 0.1, kD = 0.0005, kF = 0;

    public static int target = 0;
    public static int tolerance = 10;
    public static int lowerPowerCutoffLimit = 200;

    private double ticks_per_rev = 537.7; //goBilda 312 RPM
    private double gear_ratio = 1;
    private double pulley_diameter_inch = 38.2 / 25.4; //goBilda GT2 60T 38.2mm diameter
    private double ticks_per_inch = (ticks_per_rev * gear_ratio) / (pulley_diameter_inch * Math.PI);

    private DcMotorEx extendMotor1;
    private DcMotorEx extendMotor2;
    private DcMotorEx extendMotor3;

    //private double ff = 0;
    private double pid = 0;
    private double power = 0;

    @Override
    public void init() {
        pidfController = new PIDFController(kP, kI, kD, kF);

        extendMotor1 = hardwareMap.get(DcMotorEx.class, "motorExtend1");
        extendMotor2 = hardwareMap.get(DcMotorEx.class, "motorExtend2");
        extendMotor3 = hardwareMap.get(DcMotorEx.class, "motorExtend3");

        extendMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extendMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendMotor1.setDirection(DcMotor.Direction.FORWARD);
        extendMotor2.setDirection(DcMotor.Direction.FORWARD);
        extendMotor3.setDirection(DcMotor.Direction.FORWARD);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }


    @Override
    public void loop() {

        int extendPosition1 = extendMotor1.getCurrentPosition();
        int extendPosition2 = extendMotor2.getCurrentPosition();
        int extendPosition3 = extendMotor3.getCurrentPosition();

        if (target > 3100) {
            target = 3100;
        }

        pidfController.setPIDF(kP, kI, kD, kF);

        pid = pidfController.calculate(extendPosition1, target);
        //power = pid + ff;
        //power = pid;


        /*
        if ((target < lowerPowerCutoffLimit) && (Math.abs(target - extendPosition1) < tolerance)) {
            telemetry.addLine("Power Off");
            extendMotor1.setPower(0);
            extendMotor2.setPower(0);
            extendMotor3.setPower(0);
        } else {
            extendMotor1.setPower(pid);
            extendMotor2.setPower(pid);
            extendMotor3.setPower(pid);
        }*/


        /*
        if (Math.abs(target - extendPosition1) > tolerance) {
            telemetry.addLine("Power ON");
            extendMotor1.setPower(pid);
            extendMotor2.setPower(pid);
            extendMotor3.setPower(pid);
        } else {
            telemetry.addLine("Power Off");
            extendMotor1.setPower(0);
            extendMotor2.setPower(0);
            extendMotor3.setPower(0);
        }*/

        if (target < lowerPowerCutoffLimit && (Math.abs(target - extendPosition1) < tolerance)) {
            telemetry.addLine("Power Off");
            extendMotor1.setPower(0);
            extendMotor2.setPower(0);
            extendMotor3.setPower(0);
        } else {
            telemetry.addLine("Power ON");
            extendMotor1.setPower(pid);
            extendMotor2.setPower(pid);
            extendMotor3.setPower(pid);
        }




        /*
        if (target != extendPosition && (Math.abs(target - extendPosition) > tolerance)) {
            telemetry.addLine("Moving to Position");
            extendMotor1.setPower(power);
        } else {
            telemetry.addLine("No Power");
            extendMotor1.setPower(0.0);
        }
        */

        /*if ((target >= pivotPosition) && (Math.abs(target - pivotPosition) > tolerance)) {
            telemetry.addLine("Moving Up");
            pivotMotor.setPower(power);
            //rightRotationMotor.setPower(power);

        } else if ((target < pivotPosition) && (Math.abs(target - pivotPosition) > tolerance) && (!switchPivotHomePosition.isPressed())) {
            telemetry.addLine("Moving Down");
            pivotMotor.setPower(power);
            //rightRotationMotor.setPower(power);

        } else {
            telemetry.addLine("No Power");
            pivotMotor.setPower(0.0);
            //rightRotationMotor.setPower(0.0);
        }*/

        telemetry.addData("target", target);
        telemetry.addData("position1", extendPosition1);
        telemetry.addData("position2", extendPosition2);
        telemetry.addData("position3", extendPosition3);
        telemetry.addData("distance", extendPosition1 / ticks_per_inch);
        //telemetry.addData("ff", ff);
        telemetry.addData("power", pid);
        telemetry.addData("current", extendMotor1.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}

