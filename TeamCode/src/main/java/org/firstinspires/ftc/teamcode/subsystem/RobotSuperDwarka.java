package org.firstinspires.ftc.teamcode.subsystem;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;

public class RobotSuperDwarka extends SampleMecanumDrive {
    protected LinearOpMode opMode;
    HardwareMap robotHardwareMap;
    public Telemetry localTelemetry;
    public Pivot pivot;
    public Slider slider;
    public Wrist wrist;
    public Claw claw;
    public ArmMech armMech;
    public DistanceSensor frontRightDistanceSensor;
    public DistanceSensor frontLeftDistanceSensor;
    public HuskyLens huskyLens;
    public LEDController ledController;

    // Hang servo
    public Servo hangServo;

    public HardwareMap getHardwareMap() {
        return robotHardwareMap;
    }

    public RobotSuperDwarka(@NonNull LinearOpMode opMode) {
        super(opMode.hardwareMap);
        this.opMode = opMode;
        robotHardwareMap = opMode.hardwareMap;
        localTelemetry = opMode.telemetry;
    }

    public void initialize(boolean preserveEncoders) {
        //Log.d(TAG, "Initializing RobotStyles");

        // Initialize the HuskyLens from hardware configuration.
        huskyLens = robotHardwareMap.get(HuskyLens.class, "huskylens");
        // Check communication (optional):
        if (!huskyLens.knock()) {
            localTelemetry.addData("HuskyLens", "Communication FAILED!");
        } else {
            localTelemetry.addData("HuskyLens", "Communication OK");
        }
        // Set the algorithm to AprilTag (or TAG_RECOGNITION if that is the name)
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        pivot = new Pivot(this);
        pivot.initialize(preserveEncoders);

        slider = new Slider(this);
        slider.initialize(preserveEncoders);

        wrist = new Wrist(this);
        wrist.initialize();

        claw = new Claw(this);
        claw.initialize();

        armMech = new ArmMech(this);
        armMech.initialize();

        frontLeftDistanceSensor = robotHardwareMap.get(DistanceSensor.class, "sensorDistanceFrontLeft");
        frontRightDistanceSensor = robotHardwareMap.get(DistanceSensor.class, "sensorDistanceFrontRight");

        // Initialize the frog tongue
        hangServo = robotHardwareMap.get(Servo.class, "servoClimberHook");

        // Initialize the LED controller
        ledController = new LEDController(
                robotHardwareMap.get(Servo.class, "ledFront"),
                robotHardwareMap.get(RevBlinkinLedDriver.class, "ledLeft"),
                robotHardwareMap.get(RevBlinkinLedDriver.class, "ledRight")
        );
    }
    public double getFrontLeftDistance() {
        return frontLeftDistanceSensor.getDistance(DistanceUnit.INCH);
    }
    public double getFrontRightDistance() {
        return frontRightDistanceSensor.getDistance(DistanceUnit.INCH);
    }
    public void setHangServoPosition(double position) {
        hangServo.setPosition(position);
    }
}
