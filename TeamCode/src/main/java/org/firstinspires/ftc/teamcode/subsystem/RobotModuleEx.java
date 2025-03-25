package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class RobotModuleEx {
    public HardwareMap localHardwareMap;
    public Telemetry localTelemetry;
    protected RobotSuperDwarka robotSuperDwarka;
    protected final LinearOpMode opMode;

    public RobotModuleEx(@NonNull RobotSuperDwarka robot) {
        robotSuperDwarka = robot;
        localHardwareMap = robot.getHardwareMap();
        opMode = robot.opMode;
        localTelemetry = opMode.telemetry;
    }
    protected boolean opModeIsActive() {
        return opMode.opModeIsActive();
    }

    protected boolean isStopRequested() {
        return opMode.isStopRequested();
    }

    protected void sleep(long ms) {
        opMode.sleep(ms);
    }

    protected void idle() {
        opMode.idle();
    }

    public HardwareMap getLocalHardwareMap() {
        return localHardwareMap;
    }

    public Telemetry getLocalTelemetry() {
        return localTelemetry;
    }

    abstract public void initialize();
}
