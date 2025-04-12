package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Wrist extends RobotModuleEx {

    // This variable stores our current target position.
    private double target = Constants.WRIST_START_POSITION;

    // Timer to measure elapsed time between updates.
    private ElapsedTime timer = new ElapsedTime();

    private Servo wristServo1;
    private Servo wristServo2;

    public Wrist(RobotSuperDwarka robot) {
        super(robot);
    }

    /**
     * Initialization sequence for the wrist module.
     * Two servos are used to rotate the wrist, mounted opposite to each other.
     * One servo is set to forward, one is set to reverse so that both rotate the same direction.
     */
    public void initialize() {
        wristServo1 = localHardwareMap.get(Servo.class, "servoWristRight");
        wristServo2 = localHardwareMap.get(Servo.class, "servoWristLeft");
        wristServo1.setDirection(Servo.Direction.FORWARD);
        wristServo2.setDirection(Servo.Direction.REVERSE);
    }

    /**
     * Returns the current servo position.
     * Note that this only returns the last set position; there is no guarantee (no feedback) of actual position.
     *
     * @return current servo position
     */
    public double getPosition() {
        return wristServo1.getPosition();
    }

    /**
     * When switching to manual control in TeleOp, reset the target to the actual current servo position.
     * This enables the manual control to begin smoothly from the current position at the time manual control is initiated.
     */
    public void resetManualControl() {
        target = wristServo1.getPosition();
    }

    /**
     * Set the servo position
     * The new position is clamped to [WRIST_MIN, WRIST_MAX].
     *
     * @param position The position from [0, 1] to set the servo.
     */
    public void setPosition(double position) {
        target = Math.max(Constants.WRIST_MIN, Math.min(Constants.WRIST_MAX, position));
        wristServo1.setPosition(target);
        wristServo2.setPosition(target);
    }

    /**
     * Incrementally update the wrist position by a given delta.
     * Delta can be positive (rotate one direction) or negative (rotate the opposite).
     */
    public void incrementTarget(double delta) {
        target += delta;
        setPosition(target);
    }

    /**
     * Update the wrist target position based on a rate command.
     *
     * @param rate The rate of change in "position units per second" (e.g., 0.5 means a change of 0.5 per second).
     */
    public void updateByRate(double rate) {
        double dt = timer.seconds();
        timer.reset();
        target += rate * dt;
        setPosition(target);
    }
}
