/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Motor Velocity Test", group="drive")
@Disabled
public class MotorVelocityTest extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotorEx leftFront   = null;
    private DcMotorEx rightFront  = null;
    private DcMotorEx leftRear  = null;
    private DcMotorEx rightRear  = null;

    private ElapsedTime runtime = new ElapsedTime();

    static double POWER_MAX = 1.0;
    static double POWER_MID = 0.5;

    static double POWER_MIN = 0.2;

    static double power = 0.0;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Note: The settings here assume direct drive on all wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);

        //leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        //telemetry.addData("Starting at",  "%7d :%7d",
        //        leftFront.getCurrentPosition(),
        //       rightFront.getCurrentPosition(),
        //        leftRear.getCurrentPosition(),
        //        rightRear.getCurrentPosition());
        //telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                power = POWER_MIN;
            } else if (gamepad1.x) {
                power = POWER_MID;
            } else if (gamepad1.y) {
                power = POWER_MAX;
            } else if (gamepad1.b) {
                power = 0.0;
            }

            leftFront.setPower(power);
            rightFront.setPower(power);
            leftRear.setPower(power);
            rightRear.setPower(power);

            // Display it for the driver.
            telemetry.addData("Power", power);
            telemetry.addData("VelocityLeft", leftFront.getVelocity());
            telemetry.addData("VelocityLeft", rightFront.getVelocity());
            telemetry.addData("VelocityLeft", leftRear.getVelocity());
            telemetry.addData("VelocityLeft", rightRear.getVelocity());
            telemetry.update();
        }
    }
}
