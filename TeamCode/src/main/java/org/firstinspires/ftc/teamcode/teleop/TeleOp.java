package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.PI;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.opmode.BaseOp;
import org.firstinspires.ftc.teamcode.subsystem.Constants;
import org.firstinspires.ftc.teamcode.subsystem.util.AprilTagLocalizer;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Pivot;
import org.firstinspires.ftc.teamcode.subsystem.util.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystem.Wrist;

import java.util.List;

//@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TeleOp extends BaseOp {

    public enum ArmStateMachine {
        NONE,
        PREPARE_PICK_SAMPLE,
        PICK_SAMPLE,
        PREPARE_PICK_SAMPLE_TELEOP,
        PICK_SAMPLE_TELEOP,
        PREPARE_DROP_SAMPLE_HIGH_BASKET,
        DROP_SAMPLE_HIGH_BASKET,
        PREPARE_PICK_WALL_SPECIMEN,
        PICK_WALL_SPECIMEN,
        PREPARE_HANG_SPECIMEN,
        HANG_SPECIMEN
    }

    ArmStateMachine activeArmStateMachine = ArmStateMachine.NONE;

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();
    Gamepad.Type gamepad2Type;

    boolean runStateMachine = false;

    private int allianceMultiplier = -1;
    protected Pose2d basketDrop;

    double manualPivotPower = 0;
    double manualSliderPower = 0;
    boolean manualPivotActive = false;
    boolean manualSliderActive = false;
    boolean manualWristActive = false;
    boolean manualClawActive = false;

    boolean endGameAlertTriggered = false;
    boolean endGamePrepareAscentAlertTriggered = false;
    boolean endGameAscentAlertTriggered = false;
    private final ElapsedTime loopTime = new ElapsedTime();

    private final ElapsedTime aprilTagTimer = new ElapsedTime();
    private double wristRotationRate = Constants.WRIST_ROTATION_RATE_DEFAULT;

    AprilTagLocalizer aprilTagLocalizer = new AprilTagLocalizer(
            Constants.cameraMountAngleRadians,
            Constants.cameraOffsetInches,
            Constants.cameraHeightInches,
            Constants.APRILTAG_TAG_SIZE_INCHES,
            Constants.APRILTAG_TAG_HEIGHT_INCHES,
            Constants.myFilterAlpha, telemetry);


    protected void initialize() {
        // Call the BaseOp initialization to create the robot
        super.initialize();

        // Transfer Auton pose from PoseStorage to TeleOp if Auton was previous OpMode.
        if (mPrevOpModeIsAuto) {
            robot.setPoseEstimate(PoseStorage.currentPose);
            // Set allianceMultiplier to ensure proper calculation of any poses used in TeleOp.
            allianceMultiplier = getAlliance(mStartingPosition) == AllianceSelection.BLUE ? -1 : 1;

        } else {
            // Set LED to indicate init starting (only for TeleOp practice).
            robot.ledController.setLeftLED(Constants.LED_BLINKIN_PATTERN_ROBOT_INIT_START, Constants.LED_INIT_START_TIMER);
            robot.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_ROBOT_INIT_START, Constants.LED_INIT_START_TIMER);

            // Set TeleOp starting pose for practice/testing (not after Auton)
            robot.wrist.setPosition(Constants.WRIST_START_POSITION);
            sleep(1500);
            robot.claw.setPosition(Constants.CLAW_OPEN);
            robot.pivot.homePivot();

            // If accurate odometry is desired, set the robot on the field as indicated.
            telemetry.addLine("TeleOp Practice/Testing Mode");
            telemetry.addLine("Align robot along field wall on BLUE side, facing right");
            // Below is commented out becuase a method to determine if red/blue in TeleOp practice not implemented
            //telemetry.addLine("Driver Practice: Align robot along field wall on RED side, facing left");
            telemetry.update();

            // Set robot starting pose to BLUE for TeleOp practice/testing.
            robot.setPoseEstimate(new Pose2d(Constants.TELEOP_START_X, Constants.TELEOP_START_Y, Constants.TELEOP_START_HEADING));

            // Set LED to indicate TeleOp init finished.
            robot.ledController.setLeftLED(Constants.LED_BLINKIN_PATTERN_ROBOT_INIT_END, Constants.LED_INIT_END_TIMER);
            robot.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_ROBOT_INIT_END, Constants.LED_INIT_END_TIMER);
        }
        basketDrop = new Pose2d(-allianceMultiplier * Constants.BASKET_DROP_X,-allianceMultiplier * Constants.BASKET_DROP_Y, (Constants.BASKET_DROP_HEADING_TELEOP) + (getAlliance(mStartingPosition)==AllianceSelection.BLUE ? 0 : Math.toRadians(180)));

        // Adjust wrist rotation rate based on controller type.
        gamepad2Type = gamepad2.type();
        if (gamepad2Type == Gamepad.Type.SONY_PS4) {
            wristRotationRate = Constants.WRIST_ROTATION_RATE_PS4;
        } else {
            wristRotationRate = Constants.WRIST_ROTATION_RATE_DEFAULT;
        }
        telemetry.addData("Controller Detected", gamepad2Type.toString());
        telemetry.addLine("Press Start to begin.");
        telemetry.update();
    }

    protected void updateArmMechPosition() {
        robot.pivot.setPosition(robot.pivot.getPosition());
        robot.slider.setPosition(robot.slider.getPosition(), false);
    }

    @Override
    public void runOpMode() {

        initialize();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        waitForStart();

        // Reset AprilTag timer
        aprilTagTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            loopTime.reset();

            int armPivotPosition = robot.pivot.getPosition();
            int armExtendPosition = robot.slider.getPosition();

            // Store gamepad values from previous loop iteration in previousGamepad1/2 to be used in this loop iteration.
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            // Store gamepad values from this loop iteration in currentGamepad1/2 to be used for the entirety of this loop iteration.
            // This prevents gamepad values from changing between being used and stored in previousGamepad1/2.
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            // Check if it's time to update the pose from AprilTag
            if (aprilTagTimer.milliseconds() >= Constants.APRILTAG_UPDATE_INTERVAL_MS) {
                HuskyLens.Block[] blocks = robot.huskyLens.blocks();
                if (blocks != null && blocks.length > 0) {
                    // Use the first detected block (or add logic to choose the best)
                    HuskyLens.Block detection = blocks[0];
                    int detectedID = detection.id;
                    Pose2d aprilTagPose = aprilTagLocalizer.updatePoseFromAprilTag(detection, detectedID);
                    if (aprilTagPose != null) {
                        // Fuse the AprilTag measurement with the current odometry-based estimate if desired.
                        Pose2d odomPose = robot.getPoseEstimate();
                        Pose2d fusedPose = aprilTagLocalizer.fusePose(odomPose, aprilTagPose);
                        //Pose2d fusedPose = aprilTagLocalizer.fusePose(odomPose, aprilTagPose, 0.5);
                        robot.setPoseEstimate(fusedPose);

                        // If desired, indicate pose updated by using led (left for driver)
                        robot.ledController.setLeftLED(Constants.LED_BLINKIN_PATTERN_APRILTAG_DETECT, Constants.LED_APRILTAG_DETECT_TIMER);
                    }
                }
                // Reset the timer so the update occurs again after the interval.
                aprilTagTimer.reset();
            }

            // Check if alignment mode is active.
            boolean alignmentActive = (gamepad1.right_trigger > 0.2);
            boolean lockToBasketActive = (gamepad1.left_trigger > 0.2);

            Pose2d driveCommand = new Pose2d(0, 0, 0); // The drive power to apply this loop

            if (alignmentActive) {
                // Read distances (values returned in inches)
                double leftDistance  = robot.getFrontLeftDistance();
                double rightDistance = robot.getFrontRightDistance();
                double avgDistance   = (leftDistance + rightDistance) / 2.0;

                // Send some telemetry.
                robot.localTelemetry.addData("Left Dist (in)", leftDistance);
                robot.localTelemetry.addData("Right Dist (in)", rightDistance);
                robot.localTelemetry.addData("Avg Dist (in)", avgDistance);

                // Only execute alignment if each sensor is within range
                if (leftDistance <= Constants.ALIGN_TO_SUBMERSIBLE_THRESHOLD_INCHES && rightDistance <= Constants.ALIGN_TO_SUBMERSIBLE_THRESHOLD_INCHES) {
                    // Set target distance from submersible.
                    double targetDistance = Constants.ALIGN_TO_SUBMERSIBLE_TARGET_DISTANCE_INCHES;
                    // Compute errors.
                    double forwardError = avgDistance - targetDistance;
                    double rotationError = leftDistance - rightDistance;  // positive if left is farther.

                    // Gains for forward and rotational correction movements.
                    double kForward = Constants.ALIGN_TO_SUBMERSIBLE_kForward;
                    double kRotation = Constants.ALIGN_TO_SUBMERSIBLE_kRotation;

                    // Compute corrections and clamp them.
                    double forwardCorrection = kForward * forwardError;
                    forwardCorrection = Math.max(-Constants.ALIGN_TO_SUBMERSIBLE_kForward_MAX, Math.min(Constants.ALIGN_TO_SUBMERSIBLE_kForward_MAX, forwardCorrection));

                    double rotationCorrection = -kRotation * rotationError;
                    rotationCorrection = Math.max(-Constants.ALIGN_TO_SUBMERSIBLE_kRotation_MAX, Math.min(Constants.ALIGN_TO_SUBMERSIBLE_kRotation_MAX, rotationCorrection));

                    // Build a Pose2d drive command from these corrections.
                    driveCommand = new Pose2d(forwardCorrection, 0, rotationCorrection);

                    // Update LED and telemetry.
                    robot.ledController.setLeftLED(Constants.LED_BLINKIN_PATTERN_ALIGN_SUBMERSIBLE_ACTIVE, Constants.LED_ALIGN_SUBMERSIBLE_TIMER);
                    robot.localTelemetry.addLine("ALIGN MODE ACTIVE");

                }

            } else if (lockToBasketActive) {
                Pose2d currPose = robot.getPoseEstimate();
                Pose2d diff = basketDrop.minus(currPose);
                Vector2d xy = diff.vec().rotated(-currPose.getHeading());

                double heading = Angle.normDelta(basketDrop.getHeading()) - Angle.normDelta(currPose.getHeading());
                driveCommand = new Pose2d(xy.times(Constants.LOCK_TO_BASKET_xyP), heading + Constants.LOCK_TO_BASKET_headingP);

                // Update LED and telemetry.
                robot.ledController.setLeftLED(Constants.LED_BLINKIN_PATTERN_LOCK_TO_BASKET_ACTIVE, Constants.LED_LOCK_TO_BASKET_TIMER);
                robot.localTelemetry.clearAll();
                robot.localTelemetry.addLine("BASKET DROP LOCK POS ACTIVE");
                robot.localTelemetry.update();
            } else {
                // Otherwise, use your normal driver input.
                double speedMultiplier = gamepad1.left_bumper ? Constants.SPEED_LIMIT : 1.0;
                driveCommand = new Pose2d(
                        -gamepad1.left_stick_y * speedMultiplier,
                        -gamepad1.left_stick_x * speedMultiplier,
                        -gamepad1.right_stick_x * speedMultiplier);
            }

            // Always update the drive power.
            robot.setWeightedDrivePower(driveCommand);

            if (gamepad1.left_stick_button && !previousGamepad1.left_stick_button) {
                robot.turnAsync(Math.toRadians(Constants.TELEOP_TURN_LEFT_RADIANS));
                //robot.turn(Math.toRadians(Constants.TELEOP_TURN_LEFT_RADIANS));
            }

            if (gamepad1.right_stick_button && !previousGamepad1.right_stick_button) {
                robot.turnAsync(Math.toRadians(Constants.TELEOP_TURN_RIGHT_RADIANS));
                //robot.turn(Math.toRadians(Constants.TELEOP_TURN_RIGHT_RADIANS));
            }

            // End-game alerts and controls
            // Start with initial end-game activation timer
            if (mRunTime.seconds() >= Constants.ENDGAME_INITIAL_ALERT_TIMER_SECONDS) {
                // LED alert that end-game is approaching (boolean so we only trigger the alert once)
                if (!endGameAlertTriggered) {
                    robot.ledController.setLeftLED(Constants.LED_BLINKIN_PATTERN_ENDGAME_WARNING, Constants.LED_ENDGAME_WARNING_TIMER);
                    robot.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_ENDGAME_WARNING, Constants.LED_ENDGAME_WARNING_TIMER);
                    endGameAlertTriggered = true;
                }
                // LED alert level 1 - time to drive to submersible and prepare for ascent/hang
                if (mRunTime.seconds() >= Constants.ENDGAME_ASCENT_PREPARE_ALERT_SECONDS) {
                    if (!endGamePrepareAscentAlertTriggered) {
                        robot.ledController.setLeftLED(Constants.LED_BLINKIN_PATTERN_ENDGAME_PREPARE_ASCENT_WARNING, Constants.LED_ENDGAME_PREPARE_ASCENT_WARNING_TIMER);
                        robot.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_ENDGAME_PREPARE_ASCENT_WARNING, Constants.LED_ENDGAME_PREPARE_ASCENT_WARNING_TIMER);
                        endGamePrepareAscentAlertTriggered = true;
                    }

                }
                // LED alert level 2 - time to ascend/hang
                if (mRunTime.seconds() >= Constants.ENDGAME_ASCENT_ALERT_SECONDS) {
                    if (!endGameAscentAlertTriggered) {
                        robot.ledController.setLeftLED(Constants.LED_BLINKIN_PATTERN_ENDGAME_ASCENT_WARNING, Constants.LED_ENDGAME_ASCENT_TIMER);
                        robot.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_ENDGAME_ASCENT_WARNING, Constants.LED_ENDGAME_ASCENT_TIMER);
                        endGameAscentAlertTriggered = true;
                    }

                }
                // Enable ascent/hang servo at end-game
                if (mRunTime.seconds() >= Constants.ENDGAME_CONTROL_ENABLE_TIMER_SECONDS) {
                    if (gamepad1.y) {
                        robot.setHangServoPosition(Constants.ASCENT_SERVO_LATCH);
                    } else {
                        robot.setHangServoPosition(Constants.ASCENT_SERVO_UNLATCH);
                    }
                }
            }

            // This is a rising-edge detector that runs if and only if "dpad down" and "a" was pressed this loop.
            if ((gamepad2.dpad_down) && (currentGamepad2.a && !previousGamepad2.a)) {
                updateArmMechPosition();
                robot.armMech.resetPreparePickSampleState();
                activeArmStateMachine = ArmStateMachine.PREPARE_PICK_SAMPLE;
                runStateMachine = true;
            }

            if (currentGamepad2.a && !previousGamepad2.a) {
                // Only trigger pickSample if preparePickSample is done
                if (robot.armMech.isPreparePickSampleComplete() && robot.armMech.isPickSampleReady()) {
                    robot.armMech.resetPickSampleState(false);
                    activeArmStateMachine = ArmStateMachine.PICK_SAMPLE;
                    runStateMachine = true;
                } else {
                    telemetry.addLine("Arm not ready for floor pickup.");
                }
            }

            // This is a rising-edge detector that runs if and only if "dpad right" and "b" was pressed this loop.
            if ((gamepad2.dpad_right) && (currentGamepad2.b && !previousGamepad2.b)) {
                updateArmMechPosition();
                robot.armMech.resetPreparePickSampleTeleOpState();
                activeArmStateMachine = ArmStateMachine.PREPARE_PICK_SAMPLE_TELEOP;
                runStateMachine = true;
            }

            if (currentGamepad2.b && !previousGamepad2.b) {
                // Only trigger pickSampleTeleOp if PickSampleTeleOpReady is true
                if (robot.armMech.isPreparePickSampleTeleOpComplete() && robot.armMech.isPickSampleTeleOpReady()) {
                    updateArmMechPosition();
                    robot.armMech.resetPickSampleTeleOpState(); //originally false to only allow pick after prepare state
                    activeArmStateMachine = ArmStateMachine.PICK_SAMPLE_TELEOP;
                    runStateMachine = true;
                } else {
                    telemetry.addLine("Arm not ready for pit pickup.");
                }
            }

            // This is a rising-edge detector that runs if and only if "dpad down" and "a" was pressed this loop.
            if ((gamepad2.dpad_left) && (currentGamepad2.x && !previousGamepad2.x)) {
                updateArmMechPosition();
                robot.armMech.resetPrepareDropSampleHighBasketState();
                activeArmStateMachine = ArmStateMachine.PREPARE_DROP_SAMPLE_HIGH_BASKET;
                runStateMachine = true;
            }

            if (currentGamepad2.x && !previousGamepad2.x) {
                // Only trigger pickSample if preparePickSample is done
                if (robot.armMech.isPrepareDropSampleHighBasketComplete() && robot.armMech.isDropHighBasketReady()) {
                    robot.armMech.resetDropSampleHighBasketState(false);
                    activeArmStateMachine = ArmStateMachine.DROP_SAMPLE_HIGH_BASKET;
                    runStateMachine = true;
                } else {
                    telemetry.addLine("Arm not ready for high basket drop.");
                }
            }

            // This is a rising-edge detector that runs if and only if "dpad down" and "a" was pressed this loop.
            if (currentGamepad1.a && !previousGamepad1.a) {
                updateArmMechPosition();
                robot.armMech.resetPreparePickWallSpecimenState();
                activeArmStateMachine = ArmStateMachine.PREPARE_PICK_WALL_SPECIMEN;
                runStateMachine = true;
            }

            if (currentGamepad1.b && !previousGamepad1.b) {
                // Only trigger pickSample if preparePickSample is done
                if (robot.armMech.isPreparePickWallSpecimenComplete() && robot.armMech.isPickWallSpecimenReady()) {
                    robot.armMech.resetPickWallSpecimenState();
                    activeArmStateMachine = ArmStateMachine.PICK_WALL_SPECIMEN;
                    runStateMachine = true;
                } else {
                    telemetry.addLine("Arm not ready for wall specimen pick.");
                }
            }

            // This is a rising-edge detector that runs if and only if "dpad down" and "a" was pressed this loop.
            if ((gamepad2.dpad_up) && (currentGamepad2.y && !previousGamepad2.y)) {
                updateArmMechPosition();
                robot.armMech.resetPrepareHangSpecimenState();
                activeArmStateMachine = ArmStateMachine.PREPARE_HANG_SPECIMEN;
                runStateMachine = true;
            }

            if (currentGamepad2.y && !previousGamepad2.y) {
                // Only trigger pickSample if preparePickSample is done
                if (robot.armMech.isPrepareHangSpecimenComplete() && robot.armMech.isHangSpecimenReady()) {
                    robot.armMech.resetHangSpecimenState();
                    activeArmStateMachine = ArmStateMachine.HANG_SPECIMEN;
                    runStateMachine = true;
                } else {
                    telemetry.addLine("Arm not ready for specimen hang.");
                }
            }

            // Process state machine status
            switch(activeArmStateMachine) {
                case PREPARE_PICK_SAMPLE:
                    if (robot.armMech.preparePickSample()) {
                        activeArmStateMachine = ArmStateMachine.NONE;
                    }
                    break;
                case PICK_SAMPLE:
                    if (robot.armMech.pickSample()) {
                        activeArmStateMachine = ArmStateMachine.NONE;
                    }
                    break;
                case PREPARE_PICK_SAMPLE_TELEOP:
                    if (robot.armMech.preparePickSampleTeleOp()) {
                        activeArmStateMachine = ArmStateMachine.NONE;
                    }
                    break;
                case PICK_SAMPLE_TELEOP:
                    if (robot.armMech.pickSampleTeleOp()) {
                        activeArmStateMachine = ArmStateMachine.NONE;
                    }
                    break;
                case PREPARE_DROP_SAMPLE_HIGH_BASKET:
                    if (robot.armMech.prepareDropSampleHighBasket()) {
                        activeArmStateMachine = ArmStateMachine.NONE;
                    }
                    break;
                case DROP_SAMPLE_HIGH_BASKET:
                    if (robot.armMech.dropSampleHighBasket()) {
                        activeArmStateMachine = ArmStateMachine.NONE;
                    }
                    break;
                case PREPARE_PICK_WALL_SPECIMEN:
                    if (robot.armMech.preparePickWallSpecimen()) {
                        activeArmStateMachine = ArmStateMachine.NONE;
                    }
                    break;
                case PICK_WALL_SPECIMEN:
                    if (robot.armMech.pickWallSpecimen()) {
                        activeArmStateMachine = ArmStateMachine.NONE;
                    }
                    break;
                case PREPARE_HANG_SPECIMEN:
                    if (robot.armMech.prepareHangSpecimen()) {
                        activeArmStateMachine = ArmStateMachine.NONE;
                    }
                    break;
                case HANG_SPECIMEN:
                    if (robot.armMech.hangSpecimenUpdate()) {
                        activeArmStateMachine = ArmStateMachine.NONE;
                    }
                    break;
                // etc.
                case NONE:
                default:
                    // No active state machine
                    break;
            }

            // Process left stick for pivot
            if (Math.abs(gamepad2.left_stick_y) > 0.1) {
                manualPivotActive = true;
                // For upward movement:
                if (gamepad2.left_stick_y < -0.1) {
                    if (robot.pivot.getPosition() < Constants.PIVOT_MAX) {
                        manualPivotPower = -gamepad2.left_stick_y;
                    }
                // downward movement
                } else {
                    if (robot.pivot.isPivotHomeSwitchPressed()) {
                        manualPivotPower = 0;
                    } else {
                        manualPivotPower = -gamepad2.left_stick_y;
                    }
                }
            } else {
                manualPivotActive = false;
            }

            // Process right stick for slider
            if (Math.abs(gamepad2.right_stick_y) > 0.1) {
                manualSliderActive = true;

                // Get manual slider power from gamepad
                double desiredSliderPower = -gamepad2.right_stick_y;

                // Check current pivot position
                int currentPivotPos = robot.pivot.getPosition();

                // Check whether the pivot is below the threshold.
                if (currentPivotPos < Constants.PIVOT_THRESHOLD_FOR_SLIDER_EXTENSION) {
                    // When the pivot is too low, limit the maximum extension.
                    // If the command is to extend (desiredSliderPower > 0) and the slider is already beyond the allowed extension, then limit.
                    if (desiredSliderPower > 0 && robot.slider.getPosition() >= Constants.SLIDER_EXTENSION_LIMIT_WHEN_PIVOT_LOW) {
                        manualSliderPower = 0;
                        telemetry.addLine("Slider extension at MAX: raise pivot to extend further");
                    } else {
                        manualSliderPower = desiredSliderPower;
                    }
                } else {
                    manualSliderPower = desiredSliderPower;
                }
            } else {
                manualSliderActive = false;
            }

            // Use the left/right triggers to update the wrist gradually.
            double triggerInput = gamepad2.left_trigger - gamepad2.right_trigger;
            // This variable holds the rate that we will rotate the wrist; different rates based on gamepad/controller type
            double delta = triggerInput * wristRotationRate;
            // Only update if trigger input is significant (avoid noise near center/default position):
            if (Math.abs(triggerInput) > 0.1) {
                // When manual control starts, reset the wrist's target to its current position.
                if (!manualWristActive) {
                    robot.wrist.resetManualControl(); // This resets the internal target to the current position.
                }
                // Set manual control active
                manualWristActive = true;
                // Update the wrist position by an increment proportional to the trigger input.
                robot.wrist.incrementTarget(delta);
            } else {
                manualWristActive = false;
            }

            // Process left and right bumper for claw
            if (gamepad2.left_bumper) {
                manualClawActive = true;
                robot.claw.setPosition(Constants.CLAW_OPEN);
            } else if (gamepad2.right_bumper) {
                manualClawActive = true;
                robot.claw.setPosition(Constants.CLAW_CLOSE);
            } else {
                manualClawActive = false;
            }

            // If manual input detected, abort and reset all state machines
            if(manualPivotActive || manualSliderActive || manualWristActive || manualClawActive) {
                robot.armMech.abortAllStateMachines();
                activeArmStateMachine = ArmStateMachine.NONE;
                runStateMachine = false;
            }

            // Choose manual control or state machine for pivot, or if none, set 0 power
            if (manualPivotActive) {
                // Use manual values:
                robot.pivot.setPower(manualPivotPower);
            } else if (runStateMachine) {
                // Run the state machine
                robot.pivot.update();
            } else {
                // Otherwise, set power to zero:
                robot.pivot.setPower(0);
            }

            // Choose manual control or state machine for slider, or if none, set 0 power
            if (manualSliderActive) {
                // Use manual values:
                robot.slider.setPower(manualSliderPower);
            } else if (runStateMachine) {
                // Run the state machine (PID updates) only once per loop:
                robot.slider.update();
            } else {
                // Otherwise, set power to zero:
                robot.slider.setPower(0);
            }

            // Update LED timers
            robot.ledController.update();
            // Update robot drive
            robot.update();

            // Print pose to telemetry
            Pose2d poseEstimate = robot.getPoseEstimate();
            robot.localTelemetry.addData("x", poseEstimate.getX());
            robot.localTelemetry.addData("y", poseEstimate.getY());
            robot.localTelemetry.addData("heading", poseEstimate.getHeading());

            robot.localTelemetry.addData("Arm Pos (ticks)", armPivotPosition);
            robot.localTelemetry.addData("Arm Angle (deg)", armPivotPosition / Constants.PIVOT_TICKS_PER_DEGREE);
            robot.localTelemetry.addData("Arm Pivot Mtr Cur", robot.pivot.getCurrent());
            robot.localTelemetry.addData("Arm Pivot Pwr", robot.pivot.getPower());
            robot.localTelemetry.addData("Arm Extend Pos (ticks)", armExtendPosition);
            robot.localTelemetry.addData("Arm Extend Pos (in)", armExtendPosition / Constants.SLIDER_TICKS_PER_INCH);
            robot.localTelemetry.addData("Arm Extend Mtr Cur", robot.slider.getCurrent());

            robot.localTelemetry.addData("Loop time (ms): ", loopTime.milliseconds());
            robot.localTelemetry.addData("Run time", mRunTime.seconds());
            robot.localTelemetry.addData("Wrist pos", robot.wrist.getPosition());
            robot.localTelemetry.update();

        }
        if (isStopRequested()) {
            robot.armMech.abortAllStateMachines();
            robot.ledController.allOff();
        }
    }
}
