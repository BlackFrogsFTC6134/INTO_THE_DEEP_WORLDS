package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

//@Disabled
@Config
@TeleOp
public class WristManual extends OpMode {

    public static double WRIST_POSITION_PICKUP = 0.5;
    public static double WRIST_POSITION_CARRY = 0.4;
    public static double WRIST_POSITION_DROP = 0.3;
    public static double WRIST_POSITION_HANG = 0.2;

    private double wristTarget = 0;

    private Servo wristServoLeft;
    private Servo wristServoRight;

    @Override
    public void init() {

        wristServoLeft = hardwareMap.get(Servo.class, "servoWristLeft");
        wristServoRight = hardwareMap.get(Servo.class, "servoWristRight");
        wristServoLeft.setDirection(Servo.Direction.FORWARD);
        wristServoRight.setDirection(Servo.Direction.REVERSE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void loop() {

        if (gamepad1.a) {
            wristServoLeft.setPosition(WRIST_POSITION_PICKUP);
            wristServoRight.setPosition(WRIST_POSITION_PICKUP);
        } else if (gamepad1.b) {
            wristServoLeft.setPosition(WRIST_POSITION_CARRY);
            wristServoRight.setPosition(WRIST_POSITION_CARRY);
        } else if (gamepad1.x) {
            wristServoLeft.setPosition(WRIST_POSITION_DROP);
            wristServoRight.setPosition(WRIST_POSITION_DROP);
        } else if (gamepad1.y) {
            wristServoLeft.setPosition(WRIST_POSITION_HANG);
            wristServoRight.setPosition(WRIST_POSITION_HANG);
        }

        telemetry.addLine("Press each button to turn on its respective motor");
        telemetry.addLine();
        telemetry.addLine("<font face=\"monospace\">Xbox/PS4 Button - Position</font>");
        telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;X / ▢&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Drop</font>");
        telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;Y / Δ&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Hang</font>");
        telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;B / O&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Carry</font>");
        telemetry.addLine("<font face=\"monospace\">&nbsp;&nbsp;A / X&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;- Pickup</font>");
        telemetry.addLine();
        //telemetry.addData("position", armPosition);
        //telemetry.addData("positionR", armPositionR);
        //telemetry.addData("position", pivotPosition);
        //telemetry.addData("angle", armAngle);
        //telemetry.addData("power", armPower);
        //telemetry.addData("current", pivotMotor.getCurrent(CurrentUnit.AMPS));
        //telemetry.addData("currentR", rightRotationMotor.getCurrent(CurrentUnit.AMPS));
        //telemetry.update();
    }
}