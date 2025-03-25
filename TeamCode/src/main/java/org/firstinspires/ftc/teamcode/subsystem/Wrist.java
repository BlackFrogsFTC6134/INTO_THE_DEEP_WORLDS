package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Servo;

public class Wrist extends RobotModuleEx {

    public static final double WRIST_START_POSITION = 0.94;
    public static final double WRIST_POSITION_WALL_SPECIMEN_PICK = 0.8;
    public static final double WRIST_POSITION_PICKUP = 0.35; //0.4
    public static final double WRIST_POSITION_FLOOR_PUSH = 0.29;
    public static final double WRIST_POSITION_ZONE_DROP = 0.4; //0.4
    public static final double WRIST_POSITION_CARRY = 0.55;
            ;
    public static final double WRIST_POSITION_HIGH_BASKET_DROP = 0.80; //0.94
    public static final double WRIST_POSITION_HANG = 0.14; //0.23

    private double wristTarget = 0;

    private Servo wristServo1;
    private Servo wristServo2;

    public Wrist(RobotSuperDwarka robot) {
        super(robot);
    }

    public void initialize() {
        wristServo1 = localHardwareMap.get(Servo.class, "servoWristRight");
        wristServo2 = localHardwareMap.get(Servo.class, "servoWristLeft");
        wristServo1.setDirection(Servo.Direction.FORWARD);
        wristServo2.setDirection(Servo.Direction.REVERSE);
    }

    public double getPosition() {
        return wristServo1.getPosition();
    }

    public void setPosition(double position) {
        wristTarget = position;
        wristServo1.setPosition(wristTarget);
        wristServo2.setPosition(wristTarget);
    }
}
