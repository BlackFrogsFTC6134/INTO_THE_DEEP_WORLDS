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
public class ArmPivotRTP extends OpMode {

    public static int targetPosition = 0;

    public static int tolerance = 10;

    private DcMotorEx pivotMotor;

    private TouchSensor switchPivotHome;

    public static double UP_POWER = 0.5;

    public static double DOWN_POWER = 0.25;

    private double ticks_per_rev = 537.7; //goBilda 312 RPM
    private double worm_gear_ratio = 1.0 / 28.0;
    private double gear_stack_ratio = 3.125;
    private double ticks_per_deg = ticks_per_rev / worm_gear_ratio / gear_stack_ratio / 360;

    private double armPower = 0;
    private double armPosition = 0;
    private double armAngle = 0;


    @Override
    public void init() {

        pivotMotor = hardwareMap.get(DcMotorEx.class, "motorPivot");
        //rightRotationMotor = hardwareMap.get(DcMotorEx.class, "rightRotationMotor");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setDirection(DcMotor.Direction.FORWARD);

        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        switchPivotHome = hardwareMap.get(TouchSensor.class, "pivotHomeSwitch");
        //switchArmRotationUp = hardwareMap.get(TouchSensor.class, "switchArmRotationUp");

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

    }

    @Override
    public void loop() {

        int pivotPosition = pivotMotor.getCurrentPosition();
        boolean switchState = switchPivotHome.isPressed();

        if (targetPosition > 1700) {
            targetPosition = 1700;
        }

        if ((targetPosition >= pivotPosition) && (Math.abs(targetPosition - pivotPosition) > tolerance)) {
            telemetry.addLine("Moving Up");
            pivotMotor.setTargetPosition(targetPosition);
            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivotMotor.setPower(UP_POWER);
        } else if ((targetPosition < pivotPosition) && (Math.abs(targetPosition - pivotPosition) > tolerance) && !switchState) {
            telemetry.addLine("Moving Down");
            pivotMotor.setTargetPosition(targetPosition);
            pivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivotMotor.setPower(DOWN_POWER);
        } else {
            telemetry.addLine("No Power");
            pivotMotor.setPower(0.0);
        }

        telemetry.addData("target", targetPosition);
        telemetry.addData("position", pivotPosition);
        telemetry.addData("angle", ((pivotPosition / ticks_per_deg)) - 50.2);
        telemetry.addData("current", pivotMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("switch", switchState);
        telemetry.addData("velocity", pivotMotor.getVelocity());
        telemetry.update();
    }
}
