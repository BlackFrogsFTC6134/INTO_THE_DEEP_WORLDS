package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Slider extends RobotModuleEx {

    private DcMotorEx extendMotor1;
    private DcMotorEx extendMotor2;
    private DcMotorEx extendMotor3;
    private PIDController extendPIDController;
    private int target = 0;
    // pivotRangeHigh is used to indicate if the current pivot position is in the high or low range, as defined by PIVOT_PID_THRESHOLD
    private boolean pivotRangeHigh = false;
    private boolean atTarget = false;
    private VoltageSensor batteryVoltage;
    private final static double nominalVoltage = 12.6;

    public Slider(RobotSuperDwarka robot) {
        super(robot);
    }

    public void initialize(boolean preserveEncoders) {
        extendMotor1 = localHardwareMap.get(DcMotorEx.class, "motorExtend1");
        extendMotor2 = localHardwareMap.get(DcMotorEx.class, "motorExtend2");
        extendMotor3 = localHardwareMap.get(DcMotorEx.class, "motorExtend3");

        if (!preserveEncoders) {
            extendMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extendMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            extendMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        extendMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor1.setDirection(DcMotor.Direction.FORWARD);

        extendMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor2.setDirection(DcMotor.Direction.FORWARD);

        extendMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor3.setDirection(DcMotor.Direction.FORWARD);

        extendPIDController = new PIDController(Constants.SLIDER_kP_LOW, Constants.SLIDER_kI_LOW, Constants.SLIDER_kD_LOW);

        batteryVoltage = localHardwareMap.get(VoltageSensor.class, "Control Hub");
    }

    @Override
    public void initialize() {
        initialize(false);
    }

    public int getPosition() {
        return extendMotor1.getCurrentPosition();
    }

    public boolean isAtTarget() {
        return atTarget;
    }

    public void setAtTarget(boolean atTarget) {
        this.atTarget = atTarget;
    }

    public void setPower(double power) {
        extendMotor1.setPower(power);
        extendMotor2.setPower(power);
        extendMotor3.setPower(power);
    }

    public double getCurrent() {
        return extendMotor1.getCurrent(CurrentUnit.AMPS);
    }

    public void setPosition(int position, boolean pivotRangeHigh) {
        target = position;
        setAtTarget(false);

        if (target > Constants.SLIDER_MAX) {
            target = Constants.SLIDER_MAX;
        }
        extendPIDController.reset();

        // Select correct pid coefficients depending on position of pivot
        if (!pivotRangeHigh) {
            this.pivotRangeHigh = false; // pivot is in low range
            extendPIDController.setPID(Constants.SLIDER_kP_LOW, Constants.SLIDER_kI_LOW, Constants.SLIDER_kD_LOW);
        } else {
            this.pivotRangeHigh = true; // pivot is in high range
            extendPIDController.setPID(Constants.SLIDER_kP_HIGH, Constants.SLIDER_kI_HIGH, Constants.SLIDER_kD_HIGH);
        }
    }

    public int getTarget() {
        return target;
    }

    public void update() {
        int extendPosition = getPosition();
        //double currentBatteryVoltage = batteryVoltage.getVoltage();
        //extendPIDController.setPID(kP, kI, kD);
        double pid = extendPIDController.calculate(extendPosition, target);

        //pid = pid * (nominalVoltage / currentBatteryVoltage);

        // Select correct control loop depending on position of pivot
        if (!pivotRangeHigh) {
            if (Math.abs(target - extendPosition) > Constants.SLIDER_TOLERANCE) {
                setPower(pid);
            } else {
                setAtTarget(true);
                setPower(0);
            }
        } else {
            if (Math.abs(target - extendPosition) < Constants.SLIDER_TOLERANCE) {
                setAtTarget(true);
            }
            setPower(pid);
        }
    }
}
