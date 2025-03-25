package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Slider extends RobotModuleEx {

    public static final int POWER_CUTOFF_LIMIT = 300;
    public static final double TICKS_PER_REV = 537.7;
    public static final double GEAR_RATIO = 1;
    public static final double PULLEY_SIZE_INCH = 120.0 / 25.4;

    public static final int EXTEND_HOME = 0;
    public static final int EXTEND_HANG_SPECIMEN_HIGH_RELEASE = 220; //230
    public static final int EXTEND_HANG_SPECIMEN_HIGH = 733;
    public static final int EXTEND_BASKET_DROP_HIGH = 2835;
    public static final int EXTEND_MAX = 3100;
    public double ticksPerInch;
    private PIDController extendPIDController;
    double kP = 0.015, kI = 0.1, kD = 0.0005;
    public static int armExtendTolerance = 10;
    private int armExtendTarget = 0;

    private boolean atTarget = false;

    private DcMotorEx extendMotor1;
    private DcMotorEx extendMotor2;
    private DcMotorEx extendMotor3;

    public Slider(RobotSuperDwarka robot) {
        super(robot);
    }
    public void initialize() {
        extendMotor1 = localHardwareMap.get(DcMotorEx.class, "motorExtend1");
        extendMotor2 = localHardwareMap.get(DcMotorEx.class, "motorExtend2");
        extendMotor3 = localHardwareMap.get(DcMotorEx.class, "motorExtend3");

        extendMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor1.setDirection(DcMotor.Direction.FORWARD);

        extendMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor2.setDirection(DcMotor.Direction.FORWARD);

        extendMotor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendMotor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extendMotor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendMotor3.setDirection(DcMotor.Direction.FORWARD);

        ticksPerInch = (TICKS_PER_REV * GEAR_RATIO) / PULLEY_SIZE_INCH;
        extendPIDController = new PIDController(kP, kI, kD);
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

    public void setPosition(int position) {
        armExtendTarget = position;
        setAtTarget(false);

        if (armExtendTarget > EXTEND_MAX) {
            armExtendTarget = EXTEND_MAX;
        }
        extendPIDController.reset();
        extendPIDController.setPID(kP, kI, kD);
    }

    public int getTarget() {
        return armExtendTarget;
    }

    public void update() {
        int extendPosition = getPosition();
        //extendPIDController.setPID(kP, kI, kD);
        double pid = extendPIDController.calculate(extendPosition, armExtendTarget);

        /*
        if (Math.abs(armExtendTarget - extendPosition) > armExtendTolerance) {
            setPower(pid);
        } else {
            setAtTarget(true);
            setPower(0);
        }*/

        if (Math.abs(armExtendTarget - extendPosition) < armExtendTolerance) {
            setAtTarget(true);
            if (armExtendTarget < POWER_CUTOFF_LIMIT) {
                //setAtTarget(true);
                setPower(0);
            } else {
                setPower(pid);
            }
        } else {
            setPower(pid);
        }

    }
}
