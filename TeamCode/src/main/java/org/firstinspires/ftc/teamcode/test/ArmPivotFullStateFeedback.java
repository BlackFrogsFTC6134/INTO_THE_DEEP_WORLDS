package org.firstinspires.ftc.teamcode.test;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.KalmanEstimator;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Systems.PositionVelocitySystem;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.List;
import java.util.function.DoubleSupplier;

@Disabled
@Config
@TeleOp
public class ArmPivotFullStateFeedback extends LinearOpMode {

    private DcMotorEx pivotMotor;

    private double ticks_per_rev = 537.7; //goBilda 312 RPM
    private double worm_gear_ratio = 1.0 / 28.0;
    private double gear_stack_ratio = 3.125;
    private double ticks_per_deg = ticks_per_rev / worm_gear_ratio / gear_stack_ratio / 360;

    public static double Q = 0.3;
    public static double R = 3;
    public static int N = 3;

    public static double x = 0.0; //position
    public static double v = 0.0; //velocity

    public static double a = 0.0; //accel

    public static double tolerance = 0.0;

    public static double posKp = 0.001;
    public static double posKi = 0.0;
    public static double posKd = 0.0;

    public static double veloKp = 0.001;
    public static double veloKi = 0.0;
    public static double veloKd = 0.0;

    public static double ffKv = 0.1;
    public static double ffKa = 0.3;
    public static double ffKs = 0.0;

    @Override
    public void runOpMode() {

        pivotMotor = hardwareMap.get(DcMotorEx.class, "motorPivot");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setDirection(DcMotor.Direction.FORWARD);

        PIDCoefficients posCoefficients = new PIDCoefficients(posKp, posKi, posKd);
        PIDCoefficients veloCoefficients = new PIDCoefficients(veloKp, veloKi, veloKd);
        BasicPID posControl = new BasicPID(posCoefficients);
        BasicPID veloControl = new BasicPID(veloCoefficients);

        DoubleSupplier motorPosition = new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return pivotMotor.getCurrentPosition();
            }
        };

        DoubleSupplier motorVelocity = new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return pivotMotor.getVelocity();
            }
        };

        KalmanEstimator positionFilter = new KalmanEstimator(motorPosition, Q, R, N);
        KalmanEstimator velocityFilter = new KalmanEstimator(motorVelocity, Q, R, N);

        FeedforwardCoefficients coefficientsFF = new FeedforwardCoefficients(ffKv, ffKa, ffKs);
        BasicFeedforward feedforward = new BasicFeedforward(coefficientsFF);

        PositionVelocitySystem system = new PositionVelocitySystem(positionFilter, velocityFilter, feedforward, posControl, veloControl);

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        waitForStart();

        while (opModeIsActive()) {

            for (LynxModule module : allHubs) {
                module.clearBulkCache();
            }

            double command = system.update(x, v, a);

            if ((x != motorPosition.getAsDouble()) && (Math.abs(x - motorPosition.getAsDouble()) > tolerance)) {
                telemetry.addLine("Moving to Position");
                pivotMotor.setPower(command);
                //rightRotationMotor.setPower(power);

            } else {
                telemetry.addLine("No Power");
                pivotMotor.setPower(0.0);
                //rightRotationMotor.setPower(0.0);
            }


            telemetry.addData("target", x);
            telemetry.addData("command", command);
            telemetry.addData("position", motorPosition.getAsDouble());
            telemetry.addData("velocity", motorVelocity.getAsDouble());
            //telemetry.addData("angle", (motorPosition / ticks_per_deg) - 50.2);
            //telemetry.addData("ff", ff);
            //telemetry.addData("power", power);
            telemetry.addData("current", pivotMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }
    }
}

