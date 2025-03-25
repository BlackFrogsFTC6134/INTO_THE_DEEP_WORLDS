package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Pivot extends RobotModuleEx {
    private DcMotorEx pivotMotor;
    private TouchSensor pivotHomeSwitch;
    public static final double TICKS_PER_REV = 384.5;
    public static final double GEAR_RATIO = 28.0;

    public static final int PIVOT_PICKUP = 112; //220 //400
    public static final int PIVOT_FLOOR_PUSH = 120;
    public static final int PIVOT_PICKUP_WALL_SPECIMEN = 750;
    public static final int PIVOT_PICKUP_WALl_SPECIMEN_RELEASE = 1200;
    public static final int PIVOT_PICKUP_3RD_YELLOW = 300;
    public static final int PIVOT_INTERMEDIATE = 500;
    public static final int PIVOT_PUSH_POSITION = 250;
    public static final int PIVOT_SUBMERSIBLE_PARK = 2650;
    public static final int PIVOT_HANG_SPECIMEN_HIGH_APPROACH = 3055;
    public static final int PIVOT_HANG_SPECIMEN_HIGH = 2770; //2885
    public static final int PIVOT_DROP_HIGH_BASKET = 4150; //4220; //4475
    public static final int PIVOT_MAX = 4400;
    public double ticksPerDeg;
    private PIDController armPIDController;
    double kP = 0.0075, kI = 0.15, kD = 0.00025;

    public static int tolerance = 10;

    private int armPivotTarget = 0;

    private boolean atTarget = false;

    public Pivot(RobotSuperDwarka robot) {
        super(robot);
    }

    public void initialize() {
        pivotMotor = localHardwareMap.get(DcMotorEx.class, "motorPivot");
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivotMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivotMotor.setDirection(DcMotor.Direction.REVERSE);

        pivotHomeSwitch = localHardwareMap.get(TouchSensor.class, "switchPivotHome");

        ticksPerDeg = (TICKS_PER_REV * GEAR_RATIO) / 360.0;
        armPIDController = new PIDController(kP, kI, kD);
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

    public void setPower(double power) {
        pivotMotor.setPower(power);
    }

    public double getCurrent() {
        return pivotMotor.getCurrent(CurrentUnit.AMPS);
    }

    public void resetEncoders() {
        pivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivotMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setPosition(int position) {
        armPivotTarget = position;
        setAtTarget(false);

        if (armPivotTarget > PIVOT_MAX) {
            armPivotTarget = PIVOT_MAX;
        }
        armPIDController.reset();
        //armPIDController.setPID(kP, kI, kD);
    }

    public void update() {
        int pivotPosition = getPosition();

        // Assign pid values (power) to variable
        double pid = armPIDController.calculate(pivotPosition, armPivotTarget);

        // If target position >= current position, and not within tolerance, set power
        if ((armPivotTarget >= pivotPosition) && (Math.abs(armPivotTarget - pivotPosition) > tolerance)) {
            setPower(pid);
        }
        // If target position < current position, and not within tolerance, set power
        else if ((armPivotTarget < pivotPosition) && (Math.abs(armPivotTarget - pivotPosition) > tolerance) && !isPivotHomeSwitchPressed()) {
            setPower(pid);
        }
        // Arm pivot is at target position within tolerance
        else {
            setAtTarget(true);
            setPower(0);
        }
    }
}
