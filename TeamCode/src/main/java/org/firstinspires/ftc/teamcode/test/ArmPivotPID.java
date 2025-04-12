package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

//@Disabled
@Disabled
@Config
@TeleOp
public class ArmPivotPID extends OpMode {
    private PIDController pidController;
    public static double kP = 0.0035, kI = 0.35, kD = 0.0002
            ;

    public static int target = 0;
    public static int tolerance = 5;

    private double ticks_per_rev = 384.5; //goBilda 435 RPM
    private double gear_ratio = 28.0;
    private double ticks_per_deg = (ticks_per_rev * gear_ratio) / 360;

    private DcMotorEx pivotMotor;

    private TouchSensor switchPivotHomePosition;

    private double pid = 0;

    @Override
    public void init() {
        pidController = new PIDController(0, 0, 0);

        pivotMotor = hardwareMap.get(DcMotorEx.class, "motorPivot");
        switchPivotHomePosition = hardwareMap.get(TouchSensor.class, "switchPivotHome");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setDirection(DcMotor.Direction.REVERSE);

        while (!switchPivotHomePosition.isPressed()) {
            pivotMotor.setPower(-0.23);
            telemetry.addData("ArmPivot Pos", pivotMotor.getCurrentPosition());
            telemetry.addData("Home Switch", switchPivotHomePosition.isPressed());
            telemetry.update();
        }
        pivotMotor.setPower(0);
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("ArmPivot Pos", pivotMotor.getCurrentPosition());
        telemetry.addData("Home Switch", switchPivotHomePosition.isPressed());
        telemetry.update();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        telemetry.addLine("Init finished");
    }


    @Override
    public void loop() {

        int pivotPosition = pivotMotor.getCurrentPosition();

        if (target > 6055) {
            target = 6055;
        }

        pidController.setPID(kP, kI, kD);
        pid = pidController.calculate(pivotPosition, target);

        if ((target >= pivotPosition) && (Math.abs(target - pivotPosition) > tolerance)) {
            telemetry.addLine("Moving Up");
            pivotMotor.setPower(pid);
            //rightRotationMotor.setPower(power);

        } else if ((target < pivotPosition) && (Math.abs(target - pivotPosition) > tolerance) && (!switchPivotHomePosition.isPressed())) {
            telemetry.addLine("Moving Down");
            pivotMotor.setPower(pid);
            //rightRotationMotor.setPower(power);

        } else {
            telemetry.addLine("No Power");
            pivotMotor.setPower(0.0);
            //rightRotationMotor.setPower(0.0);
        }

        telemetry.addData("target", target);
        telemetry.addData("position", pivotPosition);
        telemetry.addData("angle", (pivotPosition / ticks_per_deg) - 50.2);
        telemetry.addData("power", pid);
        telemetry.addData("velocity", pivotMotor.getVelocity());
        telemetry.addData("current", pivotMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}

