package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends RobotModuleEx {

    public static final double CLAW_OPEN = 0.7;
    public static final double CLAW_CLOSE = 0.25;

    private double clawTarget = 0;

    private Servo clawServo1, clawServo2;

    public Claw(RobotSuperDwarka robot) {
        super(robot);
    }

    public void initialize() {
        clawServo1 = localHardwareMap.get(Servo.class, "servoClawLeft");
        clawServo2 = localHardwareMap.get(Servo.class, "servoClawRight");
        clawServo1.setDirection(Servo.Direction.FORWARD);
        clawServo2.setDirection(Servo.Direction.FORWARD);
    }

    public double getPosition() {
        return clawServo1.getPosition();
    }

    public void setPosition(double position) {
        clawTarget = position;
        clawServo1.setPosition(clawTarget);
        clawServo2.setPosition(clawTarget);
    }
}
