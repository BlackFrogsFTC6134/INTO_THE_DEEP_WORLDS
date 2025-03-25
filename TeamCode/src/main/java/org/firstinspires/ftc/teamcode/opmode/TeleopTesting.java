package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Pivot;
import org.firstinspires.ftc.teamcode.subsystem.RobotSuperDwarka;
import org.firstinspires.ftc.teamcode.subsystem.Slider2;

import java.util.List;

//@Disabled
@TeleOp
@Config
public class TeleopTesting extends LinearOpMode {

    // Initialize ArmPivot
    RobotSuperDwarka robot;
    //ArmPivot armPivot;

    // Initialize ArmExtend
    //ArmExtend armExtend;

    // Initialize SampleMecanumDrive
    //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    // Initialize Gamepad
    Gamepad lastGamePad1 = new Gamepad();

    boolean usePIDArmPivot = false;
    boolean usePIDArmExtend = false;

    public static double WRIST_HANG = 0.23;
    public static double WRIST_PICKUP = 0.4;
    public static double WRIST_WALL_PICKUP = 0.85;
    public static double WRIST_DROP = 0.8;
    public static double CLAW_OPEN = 0.7;
    public static double CLAW_CLOSE = 0.27;
    public static int PIVOT_HOME_POSITION = 0;
    public static int PIVOT_PICKUP = 112;
    public static int PIVOT_HANG_SPECIMEN_HIGH = 2770;
    public static int PIVOT_HIGH_BASKET = 4150;
    public static int PIVOT_WALL_PICKUP = 750;
    public static int PIVOT_WALL_PICKUP_RELEASE = 1200;
    public static int PIVOT_MAX = 4400;

    public static final int EXTEND_HANG_SPECIMEN_HIGH_RELEASE = 210; //230
    public static final int EXTEND_HANG_SPECIMEN_HIGH = 733;



    @Override
    public void runOpMode() {

        robot = new RobotSuperDwarka(this);
        robot.initialize();

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();

        while (opModeIsActive()) {
            int armPivotPosition = robot.pivot.getPosition();
            int armExtendPosition = robot.slider.getPosition();

            // This is a rising-edge detector that runs if and only if "a" was pressed this loop.
            if (gamepad1.a && !lastGamePad1.a) {
                robot.slider.setPosition(Slider2.EXTEND_HOME, false);
                usePIDArmExtend = true;
            } else if (gamepad1.x && !lastGamePad1.x) {
                robot.pivot.setPosition(PIVOT_WALL_PICKUP);
                usePIDArmPivot = true;
            } else if (gamepad1.y && !lastGamePad1.y) {
                robot.pivot.setPosition(PIVOT_WALL_PICKUP_RELEASE);
                usePIDArmPivot = true;
            } else if (gamepad1.b && !lastGamePad1.b) {
                robot.slider.setPosition(Slider2.EXTEND_BASKET_DROP_HIGH, true);
                usePIDArmExtend = true;
            }

            if (gamepad1.left_stick_y < -0.1) {
                if (armPivotPosition < PIVOT_MAX) {
                    robot.pivot.setPower(-gamepad1.left_stick_y);
                } else {
                    robot.pivot.setPower(0);
                }

                // If we get any sort of manual input, turn PID off.
                usePIDArmPivot = false;
            } else if (gamepad1.left_stick_y > 0.1) {
                if (robot.pivot.isPivotHomeSwitchPressed()) {
                    robot.pivot.setPower(0);
                } else {
                    robot.pivot.setPower(-gamepad1.left_stick_y);
                }

                // If we get any sort of manual input, turn PID off.
                usePIDArmPivot = false;
            } else if (usePIDArmPivot) {
                // Sets the arm pivot motor power according to the PID output.
                robot.pivot.update();
            } else {
                robot.pivot.setPower(0);
            }

            if (gamepad1.right_stick_y < -0.1) {
                robot.slider.setPower(-gamepad1.right_stick_y);

                // If we get any sort of manual input, turn PID off.
                usePIDArmExtend = false;
            } else if (gamepad1.right_stick_y > 0.1) {
                robot.slider.setPower(-gamepad1.right_stick_y);

                // If we get any sort of manual input, turn PID off.
                usePIDArmExtend = false;
            } else if (usePIDArmExtend) {
                // Sets the arm extend motor power according to the PID output.
                robot.slider.update();
            } else {
                robot.slider.setPower(0);
            }

            if (gamepad1.dpad_left) {
                robot.wrist.setPosition(WRIST_WALL_PICKUP);
            } else if (gamepad1.dpad_right) {
                robot.wrist.setPosition(WRIST_PICKUP);
            }

            if (gamepad1.left_bumper) {
                robot.claw.setPosition(CLAW_OPEN);
            } else if (gamepad1.right_bumper) {
                robot.claw.setPosition(CLAW_CLOSE);
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad2.left_stick_y,
                            -gamepad2.left_stick_x,
                            -gamepad2.right_stick_x
                    )
            );

            drive.update();

            //int armPivotPosition = robot.pivot.getPosition();
            //int armExtendPosition = robot.slider.getPosition();

            telemetry.addData("Arm Pos (ticks)", armPivotPosition);
            telemetry.addData("Arm Angle (deg)", armPivotPosition / robot.pivot.ticksPerDeg);
            telemetry.addData("Arm Extend Pos (ticks)", armExtendPosition);
            telemetry.addData("Arm Extend Pos (in)", armExtendPosition / robot.slider.ticksPerInch);
            telemetry.addData("Arm Pivot Mtr Cur", robot.pivot.getCurrent());
            telemetry.addData("Arm Extend Mtr Cur", robot.slider.getCurrent());
            telemetry.update();

        }
    }
}
