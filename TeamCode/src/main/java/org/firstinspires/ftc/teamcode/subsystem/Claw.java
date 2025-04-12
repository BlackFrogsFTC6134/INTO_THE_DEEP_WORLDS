package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends RobotModuleEx {

    private Servo clawServo1;

    public Claw(RobotSuperDwarka robot) {
        super(robot);
    }

    public void initialize() {
        clawServo1 = localHardwareMap.get(Servo.class, "servoClawLeft");
        clawServo1.setDirection(Servo.Direction.FORWARD);
    }

    public double getPosition() {
        return clawServo1.getPosition();
    }

    public void setPosition(double position) {
        clawServo1.setPosition(position);
    }
}
