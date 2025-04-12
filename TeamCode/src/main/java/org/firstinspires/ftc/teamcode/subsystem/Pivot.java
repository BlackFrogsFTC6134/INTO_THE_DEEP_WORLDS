package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystem.util.AntiWindupPIDController;

public class Pivot extends RobotModuleEx {
    private DcMotorEx pivotMotor;
    private TouchSensor pivotHomeSwitch;
    private AntiWindupPIDController armPIDController;
    private int target = 0;
    private double pid = 0;
    private boolean atTarget = false;
    private VoltageSensor batteryVoltage;
    private final static double nominalVoltage = 12.6;

    public Pivot(RobotSuperDwarka robot) {
        super(robot);
    }

    public void initialize(boolean preserveEncoders) {
        pivotMotor = localHardwareMap.get(DcMotorEx.class, "motorPivot");
        if (!preserveEncoders) {
            pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setDirection(DcMotor.Direction.REVERSE);

        pivotHomeSwitch = localHardwareMap.get(TouchSensor.class, "switchPivotHome");

        batteryVoltage = localHardwareMap.get(VoltageSensor.class, "Control Hub");

        //armPIDController = new PIDController(Constants.PIVOT_kP, Constants.PIVOT_kI, Constants.PIVOT_kD);

        // Initialize our custom PID controller.
        armPIDController = new AntiWindupPIDController(
                Constants.PIVOT_kP,
                Constants.PIVOT_kI,
                Constants.PIVOT_kD,
                Constants.PIVOT_INTEGRAL_MAX,
                Constants.PIVOT_OUTPUT_MAX
        );
    }

    @Override
    public void initialize() {
        initialize(false);
    }

    public boolean isAtTarget() {
        return atTarget;
    }

    public void setAtTarget(boolean atTarget) {
        this.atTarget = atTarget;
    }

    public boolean isPivotHomeSwitchPressed() {
        return pivotHomeSwitch.isPressed();
    }

    public int getPosition() {
        return pivotMotor.getCurrentPosition();
    }

    public double getPower() {
        return pid;
    }

    public void setPower(double power) {
        pivotMotor.setPower(power);
    }

    public double getCurrent() {
        return pivotMotor.getCurrent(CurrentUnit.AMPS);
    }

    public void homePivot() {
        // Move pivot to starting position if home switch is not pressed, and reset encoders
        while (!isPivotHomeSwitchPressed()) {
            setPower(-0.25);
            localTelemetry.addData("ArmPivot Pos", getPosition());
            localTelemetry.addData("Home Switch", isPivotHomeSwitchPressed());
            localTelemetry.update();
        }
        // Set power to 0 when switch is pressed
        setPower(0);
        // Wait to stabilize
        sleep(200);
        // Move pivot up so that it's under tension and first tick is always in direction of movement
        while (isPivotHomeSwitchPressed()) {
            setPower(.23);
            localTelemetry.addData("ArmPivot Pos", getPosition());
            localTelemetry.addData("Home Switch", isPivotHomeSwitchPressed());
            localTelemetry.update();
        }
        // Set power to 0 when switch is no longer depressed
        setPower(0);
        // Reset pivot motor encoders
        resetEncoder();
    }

    public void resetEncoder() {
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Give time for the motor controller to reset the encoders
        sleep(300);
    }
    public void setPosition(int position) {
        target = position;
        setAtTarget(false);

        if (target > Constants.PIVOT_MAX) {
            target = Constants.PIVOT_MAX;
        }
        armPIDController.reset();
        armPIDController.setPID(Constants.PIVOT_kP, Constants.PIVOT_kI, Constants.PIVOT_kD);
    }

    public void update() {
        int pivotPosition = getPosition();
        //double currentBatteryVoltage = batteryVoltage.getVoltage();

        // Assign pid values (power) to variable
        pid = armPIDController.calculate(pivotPosition, target);

        //pid = pid * (nominalVoltage / currentBatteryVoltage);

        // If target position >= current position, and not within tolerance, set power
        if ((target >= pivotPosition) && (Math.abs(target - pivotPosition) > Constants.PIVOT_TOLERANCE)) {
            setPower(pid);
        }
        // If target position < current position, and not within tolerance, set power
        else if ((target < pivotPosition) && (Math.abs(target - pivotPosition) > Constants.PIVOT_TOLERANCE) && !isPivotHomeSwitchPressed()) {
            setPower(pid);
        }
        // Arm pivot is at target position within tolerance
        else {
            setAtTarget(true);
            setPower(0);
        }
    }
}
