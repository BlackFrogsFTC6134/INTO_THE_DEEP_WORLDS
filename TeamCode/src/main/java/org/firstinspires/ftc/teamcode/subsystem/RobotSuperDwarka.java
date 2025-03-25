package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

public class RobotSuperDwarka extends SampleMecanumDrive {
    protected LinearOpMode opMode;
    HardwareMap robotHardwareMap;
    public Telemetry localTelemetry;
    public Pivot pivot;
    public Slider2 slider;
    public Wrist wrist;
    public Claw claw;
    public ArmMech armMech;
    public DistanceSensor frontRightDistanceSensor;
    public DistanceSensor frontLeftDistanceSensor;

    public HardwareMap getHardwareMap() {
        return robotHardwareMap;
    }

    public RobotSuperDwarka(@NonNull LinearOpMode opMode) {
        super(opMode.hardwareMap);
        this.opMode = opMode;
        robotHardwareMap = opMode.hardwareMap;
        localTelemetry = opMode.telemetry;

        List<LynxModule> allHubs = robotHardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    public void initialize() {
        //Log.d(TAG, "Initializing RobotStyles");

        pivot = new Pivot(this);
        pivot.initialize();

        slider = new Slider2(this);
        slider.initialize();

        wrist = new Wrist(this);
        wrist.initialize();

        claw = new Claw(this);
        claw.initialize();

        armMech = new ArmMech(this);
        armMech.initialize();

        frontLeftDistanceSensor = robotHardwareMap.get(DistanceSensor.class, "sensorDistanceFrontLeft");
        frontRightDistanceSensor = robotHardwareMap.get(DistanceSensor.class, "sensorDistanceFrontRight");

    }
    public double getFrontLeftDistance() {
        return frontLeftDistanceSensor.getDistance(DistanceUnit.INCH);
    }
    public double getFrontRightDistance() {
        return frontRightDistanceSensor.getDistance(DistanceUnit.INCH);
    }
}
