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

@Disabled
@Config
@TeleOp
public class ArmPivotPIDext extends OpMode {
    public static double kP_pos = 0.008, kI_pos = 0.0, kD_pos = 0.00025;
    public static double kP_vel = 0.005, kI_vel = 0.0, kD_vel = 0.0002;

    // Outer-loop PID for position (tuning parameters for position error)
    private PIDController positionPID = new PIDController(kP_pos, kI_pos, kD_pos);
    // Inner-loop PID for velocity (tuning parameters for velocity error)
    private PIDController velocityPID = new PIDController(kP_vel, kI_vel, kD_vel);

    // Feed-forward gain to help with battery voltage variation.
    public static double feedForwardGain = 0; // Set this based on system characterization.

    // Maximum allowed velocity (ticks per second) and acceleration limits, as needed.
    public static double maxVelocity = 500;

    public static int targetPosition = 0;
    public static int tolerance = 5;

    private double ticks_per_rev = 384.5; //goBilda 435 RPM
    private double gear_ratio = 28.0;
    private double ticks_per_deg = (ticks_per_rev * gear_ratio) / 360;

    private DcMotorEx pivotMotor;

    private TouchSensor switchPivotHomePosition;

    @Override
    public void init() {

        positionPID.setPID(kP_pos, kI_pos, kD_pos);
        velocityPID.setPID(kP_vel, kI_vel, kD_vel);

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

        int currentPos = pivotMotor.getCurrentPosition();
        double currentVel = pivotMotor.getVelocity();

        if (targetPosition > 4400) {
            targetPosition = 4400;
        }

        positionPID.setPID(kP_pos, kI_pos, kD_pos);
        velocityPID.setPID(kP_vel, kI_vel, kD_vel);

        //pid = pidController.calculate(pivotPosition, target);

        // Outer loop: compute desired velocity from the position error.
        //double posError = targetPosition - currentPos;
        double desiredVel = positionPID.calculate(currentPos, targetPosition);
        // Optionally, clamp the desired velocity.
        desiredVel = Math.max(-maxVelocity, Math.min(maxVelocity, desiredVel));

        // Inner loop: compute correction to achieve the desired velocity.
        //double velError = desiredVel - currentVel;
        double velCorrection = velocityPID.calculate(currentVel, desiredVel);

        // Add a feed-forward term based on desired velocity (and possibly battery voltage compensation).
        double feedForward = feedForwardGain * desiredVel;

        // Compute final motor command.
        double motorCommand = velCorrection + feedForward;

        // Optionally, scale motorCommand based on current battery voltage if you have that measurement.
        //motorCommand = motorCommand * (nominalVoltage / currentBatteryVoltage);

        // Send command to motor.
        //pivotMotor.setPower(motorCommand);

        if ((targetPosition >= currentPos) && (Math.abs(targetPosition - currentPos) > tolerance)) {
            telemetry.addLine("Moving Up");
            pivotMotor.setPower(motorCommand);
            //rightRotationMotor.setPower(power);

        } else if ((targetPosition < currentPos) && (Math.abs(targetPosition - currentPos) > tolerance) && (!switchPivotHomePosition.isPressed())) {
            telemetry.addLine("Moving Down");
            pivotMotor.setPower(motorCommand);
            //rightRotationMotor.setPower(power);

        } else {
            telemetry.addLine("At Target; no power");
            pivotMotor.setPower(0.0);
            //rightRotationMotor.setPower(0.0);
        }

        telemetry.addData("target", targetPosition);
        telemetry.addData("position", currentPos);
        telemetry.addData("angle", (currentPos / ticks_per_deg) - 50.2);
        telemetry.addData("velocity (ticks/sec", currentVel);
        telemetry.addData("power", motorCommand);
        telemetry.addData("current", pivotMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}

