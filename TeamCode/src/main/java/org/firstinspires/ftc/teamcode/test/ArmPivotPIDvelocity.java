package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
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
public class ArmPivotPIDvelocity extends OpMode {
    private PIDController pidController;

    //public static double p = 0.0, i = 0.0, d = 0.0;

    public static double pUpLow = 0.0038, iUpLow = 0.6, dUpLow = 0.0005;
    public static double pUpHigh = 0.004, iUpHigh = 0.6, dUpHigh = 0.0005;
    public static double fUpLow = 0.6;
    public static double fUpHigh = 0.4;

    public static double pDownLow = 0.002, iDownLow = 0.2, dDownLow = 0.00089;
    public static double pDownHigh = 0.003, iDownHigh = 0.2, dDownHigh = 0.002;
    public static double fDownLow = 0.0;
    public static double fDownHigh = 0.1;

    public static int target = 0;
    public static int tolerance = 7;

    private double ticks_per_rev = 537.7; //goBilda 312 RPM
    private double worm_gear_ratio = 1.0 / 28.0;
    private double gear_stack_ratio = 3.125;
    private double ticks_per_deg = ticks_per_rev / worm_gear_ratio / gear_stack_ratio / 360;

    private DcMotorEx pivotMotor;
    //private DcMotorEx rightRotationMotor;

    private TouchSensor switchPivotHomePosition;

    //private TouchSensor switchArmRotationDown;

    private double ff = 0;
    private double pid = 0;
    private double power = 0;

    @Override
    public void init() {
        pidController = new PIDController(pUpLow, iUpLow, dUpLow);

        pivotMotor = hardwareMap.get(DcMotorEx.class, "motorPivot");
        //rightRotationMotor = hardwareMap.get(DcMotorEx.class, "rightRotationMotor");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightRotationMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightRotationMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //leftRotationMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pivotMotor.setDirection(DcMotor.Direction.FORWARD);
        //rightRotationMotor.setDirection(DcMotor.Direction.REVERSE);

        switchPivotHomePosition = hardwareMap.get(TouchSensor.class, "switchPivotHome");
        //switchArmRotationUp = hardwareMap.get(TouchSensor.class, "switchArmRotationUp");

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }


    @Override
    public void loop() {

        int pivotPosition = pivotMotor.getCurrentPosition();
        //int armPositionR = rightRotationMotor.getCurrentPosition();
        //int armPosition = (armPositionL + armPositionR) / 2;

        if (target > 1950) {
            target = 1950;
        }

        if (target >= pivotPosition) {
            if (pivotPosition <= 650 && target >= 1400) {
                pidController.setPID(pUpHigh, iUpHigh, dUpHigh);
                ff = Math.cos(Math.toRadians((target / ticks_per_deg)) - 50.2) * fUpHigh;
            } else {
                pidController.setPID(pUpLow, iUpLow, dUpLow);
                ff = Math.cos(Math.toRadians((target / ticks_per_deg)) - 50.2) * fUpLow;
            }
        } else {
            if (pivotPosition >= 1500) {
                pidController.setPID(pDownHigh, iDownHigh, dDownHigh);
                ff = Math.cos(Math.toRadians((target / ticks_per_deg)) - 50.2) * fDownHigh;
            } else {
                pidController.setPID(pDownLow, iDownLow, dDownLow);
                ff = Math.cos(Math.toRadians((target / ticks_per_deg)) - 50.2) * fDownLow;
            }
        }

        /*if (target >= pivotPosition && target >= 1400) {
            pidController.setPID(pUpHigh, iUpHigh, dUpHigh);
            ff = Math.cos(Math.toRadians((target / ticks_per_deg)) - 50.2) * fUpHigh;

        } else if (target >= pivotPosition && target < 1400) {
            pidController.setPID(pUpLow, iUpLow, dUpLow);
            ff = Math.cos(Math.toRadians((target / ticks_per_deg)) - 50.2) * fUpLow;
        } else if (target < pivotPosition && target ) {
            pidController.setPID(pDownHigh, iDownHigh, dDownHigh);
            ff = Math.cos(Math.toRadians((target / ticks_per_deg)) - 50.2) * fDownHigh;
        } else {
            pidController.setPID(pDown, iDown, dDown);
            ff = Math.cos(Math.toRadians((target / ticks_per_deg)) - 50.2) * fDown;
        }*/
        //pidController.setPID(p, i, d);

        pid = pidController.calculate(pivotPosition, target);
        //power = pid + ff;
        power = pid;

        if (target != pivotPosition && (Math.abs(target - pivotPosition) > tolerance)) {
            telemetry.addLine("Moving to Position");
            pivotMotor.setPower(power);
        } else {
            telemetry.addLine("No Power");
            pivotMotor.setPower(0.0);
        }

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
        telemetry.addData("position", pivotPosition);
        telemetry.addData("angle", (pivotPosition / ticks_per_deg) - 50.2);
        //telemetry.addData("ff", ff);
        telemetry.addData("power", power);
        telemetry.addData("current", pivotMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}

