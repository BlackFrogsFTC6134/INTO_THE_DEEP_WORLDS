package org.firstinspires.ftc.teamcode.test;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.FullStateFeedback;
import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;

@Config
@TeleOp(name = "ArmPivotFullState", group = "Linear OpMode")
public class ArmPivotFullState extends LinearOpMode {

    private DcMotorEx pivotMotor;

    private double ticks_per_rev = 537.7; //goBilda 312 RPM
    private double worm_gear_ratio = 1.0 / 28.0;
    private double gear_stack_ratio = 3.125;
    private double ticks_per_deg = ticks_per_rev / worm_gear_ratio / gear_stack_ratio / 360;

    public static double x = 0.0; //position
    public static double v = 0.0; //velocity

    //public static double a = 0.0; //accel

    public static double refPosition = 0.0;
    public static double refVelocity = 0.0;

   final double command = 0.0;


    @Override
    public void runOpMode() {

        pivotMotor = hardwareMap.get(DcMotorEx.class, "motorPivot");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setDirection(DcMotor.Direction.FORWARD);

        Vector K = new Vector(new double[] {x, v});
        FullStateFeedback controller = new FullStateFeedback(K);

        double position = 0.0;
        double velocity = 0.0;
        //double command = 0.0;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();

        while (opModeIsActive()) {

            Vector state = new Vector(new double[] {position, velocity});
            Vector reference = new Vector(new double[] {refPosition, refVelocity});

            //command = controller.calculate(reference, state);
            //double command = 0;
            try {
                double command = controller.calculate(reference, state);
                telemetry.addLine("I'm here");
;            } catch (Exception e) {
                throw new RuntimeException(e);
            }

            telemetry.addData("target", x);
            telemetry.addData("command", command);
            //telemetry.addData("position", motorPosition);
            //telemetry.addData("velocity", motorVelocity);
            //telemetry.addData("angle", (motorPosition / ticks_per_deg) - 50.2);
            //telemetry.addData("ff", ff);
            //telemetry.addData("power", power);
            telemetry.addData("current", pivotMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}

