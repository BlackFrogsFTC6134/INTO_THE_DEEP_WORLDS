package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@Config
public class Constants {
    // Prevent instantiation.
    private Constants() {}

    // ========== Drivetrain / Roadrunner Constants ==========
    // All drivetrain and Roadrunner constants, values, parameters can be found at:
    // \teamcode\drive\DriveConstants
    // \teamcode\drive\SampleMecanumDrive
    // \teamcode\drive\TwoWheelTrackingLocalizer




    // ========== Pivot Constants ==========
    // Pivot Positions
    public static final int PIVOT_PICKUP = 112; //220 //400
    public static final int PIVOT_PICKUP_WALL_SPECIMEN = 700;
    public static final int PIVOT_PICKUP_WALl_SPECIMEN_RELEASE = 1000;
    public static final int PIVOT_PICKUP_3RD_YELLOW = 300;
    public static final int PIVOT_PICKUP_PREPARE_PIT_PICK = 395;
    public static final int PIVOT_PIT_PICKUP = 500;
    public static final int PIVOT_INTERMEDIATE = 500;
    public static final int PIVOT_SUBMERSIBLE_PARK = 2650;
    public static final int PIVOT_HANG_SPECIMEN_SLIDER_START = 2200;
    public static final int PIVOT_HANG_SPECIMEN_HIGH_APPROACH = 3055;
    public static final int PIVOT_HANG_SPECIMEN_HIGH = 2730; // Josh: 2820, Matt: 2750 (after adjustment)
    public static final int PIVOT_THRESHOLD_FOR_SLIDER_EXTENSION = 3200;
    public static final int PIVOT_DROP_HIGH_BASKET = 4240; //4220; //4475 //Josh robot: 4235, Matt: 4235 (after adjustment)
    public static final int PIVOT_MAX = 4400;
    public static final int PIVOT_TOLERANCE = 12;

    // Pivot Mechanism details
    public static final double PIVOT_TICKS_PER_REV = 384.5; //goBilda 435 RPM
    public static final double PIVOT_GEAR_RATIO = 28.0; //goBilda pan drive kit
    public static final double PIVOT_TICKS_PER_DEGREE = (PIVOT_TICKS_PER_REV * PIVOT_GEAR_RATIO) / 360.0;

    // Pivot PID
    //public static final double PIVOT_kP = 0.003, PIVOT_kI = 0.35, PIVOT_kD = 0.0003;
    // testing values
    //public static final double PIVOT_kP = 0.005, PIVOT_kI = 0.25, PIVOT_kD = 0.0003;
    //public static final double PIVOT_kP = 0.005, PIVOT_kI = 0.4, PIVOT_kD = 0.0002;
    //public static double PIVOT_kP = 0.0048, PIVOT_kI = 0.5, PIVOT_kD = 0.0002; // Josh robot
    public static double PIVOT_kP = 0.0045, PIVOT_kI = 0.4, PIVOT_kD = 0.0001; // Hotrod3 robot
    public static double PIVOT_INTEGRAL_MAX = 0.32;
    public static double PIVOT_OUTPUT_MAX = 1.0;




    // ========== Wrist Constants ==========
    public static final double WRIST_START_POSITION = 0.94;
    public static final double WRIST_POSITION_WALL_SPECIMEN_PICK = 0.84;
    public static final double WRIST_POSITION_PICKUP = 0.35;
    public static final double WRIST_POSITION_CARRY = 0.65;
    public static final double WRIST_POSITION_HIGH_BASKET_DROP = 0.80;
    public static final double WRIST_POSITION_HANG = 0.14;
    public static final double WRIST_MIN = 0.1;
    public static final double WRIST_MAX = 0.94;




    // ========== Slider Constants ==========
    public static final int SLIDER_HOME = 0;
    public static final int SLIDER_HANG_SPECIMEN_HIGH_RELEASE = 205; //230
    public static final int SLIDER_PIT_PICK = 250;
    public static final int SLIDER_HANG_SPECIMEN_HIGH = 735;
    public static final int SLIDER_EXTENSION_LIMIT_WHEN_PIVOT_LOW = 1400;
    public static final int SLIDER_BASKET_DROP_HIGH = 2870; // Josh robot: 2850
    public static final int SLIDER_MAX = 3100;
    public static final int SLIDER_TOLERANCE = 11;

    // Slider Mechanism details
    public static final double SLIDER_TICKS_PER_REV = 537.7; //goBilda 312 RPM
    public static final double SLIDER_GEAR_RATIO = 1.0; // direct drive 1:1
    public static final double SLIDER_PULLEY_SIZE_INCH = 120.0 / 25.4;
    public static final double SLIDER_TICKS_PER_INCH = (SLIDER_TICKS_PER_REV * SLIDER_GEAR_RATIO) / SLIDER_PULLEY_SIZE_INCH;

    // Slider PID
    public static final double SLIDER_kP_LOW = 0.0085, SLIDER_kI_LOW = 0.0, SLIDER_kD_LOW = 0.00015; // low pivot angle
    //public static final double SLIDER_kP_HIGH = 0.011, SLIDER_kI_HIGH = 0.1, SLIDER_kD_HIGH = 0.00002; // high pivot angle // Josh robot
    public static final double SLIDER_kP_HIGH = 0.0085, SLIDER_kI_HIGH = 0.1, SLIDER_kD_HIGH = 0.000012; // high pivot angle // Hotrod3 robot
    // testing values
    //public static final double SLIDER_kP_LOW = 0.015, SLIDER_kI_LOW = 0.1, SLIDER_kD_LOW = 0.0005; // original
    //public static final double SLIDER_kP_LOW = 0.009, SLIDER_kI_LOW = 0.0, SLIDER_kD_LOW = 0.0002; // low pivot angle
    //public static final double SLIDER_kP_HIGH = 0.014, SLIDER_kI_HIGH = 0.4, SLIDER_kD_HIGH = 0.0008; // high pivot angle




    // ========== Claw Constants ==========
    public static final double CLAW_OPEN = 0.7;
    public static final double CLAW_CLOSE = 0.25;




    // ========== TeleOp Constants ==========

    // End-game servo hook
    public static final double ASCENT_SERVO_LATCH = 0.78;
    public static final double ASCENT_SERVO_UNLATCH = 0.85;

    // Speed limit control factor for driver (multiplied by joystick input)
    public static final double SPEED_LIMIT = 0.3;

    // Wrist rotation speed control for operator for both default (Logitech) and PS4-style controllers; PS4 has more sensitive triggers
    public static final double WRIST_ROTATION_RATE_DEFAULT = 0.075;  // Logitech; adjust this value as needed
    public static final double WRIST_ROTATION_RATE_PS4 = 0.045;  // PS4-style; adjust this value as needed

    // Align to Submersible (xyP: forward gain, headingP: rotation gain)
    public static final double ALIGN_TO_SUBMERSIBLE_kForward = 0.1;
    public static final double ALIGN_TO_SUBMERSIBLE_kForward_MAX = 0.25;
    public static final double ALIGN_TO_SUBMERSIBLE_kRotation = 0.1;
    public static final double ALIGN_TO_SUBMERSIBLE_kRotation_MAX = 0.3;
    public static final double ALIGN_TO_SUBMERSIBLE_THRESHOLD_INCHES = 24.0; // won't attempt to align if distance is greater than this value
    public static final double ALIGN_TO_SUBMERSIBLE_TARGET_DISTANCE_INCHES = 10.5; // distance from submersible for hanging specimen on high chamber

    // Align to Submersible (xyP: forward gain, headingP: rotation gain)
    public static final double LOCK_TO_BASKET_xyP = 0.04;
    public static final double LOCK_TO_BASKET_headingP = 0.04;

    // TeleOp manual position starting coordinates (inches/radians)
    public static final double TELEOP_START_X = 0.0;
    public static final double TELEOP_START_Y = 64.5;
    public static final double TELEOP_START_HEADING = Math.toRadians(180); // converts degrees to radians

    // TeleOp auto turn 180 deg value (not 180 deg because of robot dynamics, so need to adjust to achieve 180 deg)
    public static final double TELEOP_TURN_LEFT_RADIANS = Math.toRadians(205);
    public static final double TELEOP_TURN_RIGHT_RADIANS = Math.toRadians(-205);

    // End-game timers
    public static final double ENDGAME_INITIAL_ALERT_TIMER_SECONDS = 85.0;
    public static final double ENDGAME_CONTROL_ENABLE_TIMER_SECONDS = 90.0;
    public static final double ENDGAME_ASCENT_PREPARE_ALERT_SECONDS = 100.0;
    public static final double ENDGAME_ASCENT_ALERT_SECONDS = 105.0;




    // ========== LED Constants ==========
    // Default time (in seconds) the LED stays on when activated (if not, set to indefinite).
    public static final double LED_DEFAULT_TIMEOUT = 3.0;
    public static final RevBlinkinLedDriver.BlinkinPattern LED_BLINKIN_PATTERN_ROBOT_INIT_START = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
    public static final double LED_INIT_START_TIMER = 0; // 0 = indefinitely; will update when init finished
    public static final RevBlinkinLedDriver.BlinkinPattern LED_BLINKIN_PATTERN_ROBOT_INIT_END = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE;
    public static final double LED_INIT_END_TIMER = 5.0;
    public static final RevBlinkinLedDriver.BlinkinPattern LED_BLINKIN_PATTERN_APRILTAG_DETECT = RevBlinkinLedDriver.BlinkinPattern.SHOT_WHITE;
    public static final double LED_APRILTAG_DETECT_TIMER = 1.0;
    public static final RevBlinkinLedDriver.BlinkinPattern LED_BLINKIN_PATTERN_ALIGN_SUBMERSIBLE_ACTIVE = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    public static final double LED_ALIGN_SUBMERSIBLE_TIMER = 0;
    public static final RevBlinkinLedDriver.BlinkinPattern LED_BLINKIN_PATTERN_LOCK_TO_BASKET_ACTIVE = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    public static final double LED_LOCK_TO_BASKET_TIMER = 0;
    public static final LEDController.LEDColor LED_FRONT_COLOR_STATE_MACHINE_START = LEDController.LEDColor.RED;
    public static final RevBlinkinLedDriver.BlinkinPattern LED_BLINKIN_PATTERN_STATE_MACHINE_START = RevBlinkinLedDriver.BlinkinPattern.RED;
    public static final double LED_STATE_MACHINE_START_TIMER = 0;
    public static final LEDController.LEDColor LED_FRONT_COLOR_STATE_MACHINE_END = LEDController.LEDColor.GREEN;
    public static final RevBlinkinLedDriver.BlinkinPattern LED_BLINKIN_PATTERN_STATE_MACHINE_END = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    public static final double LED_STATE_MACHINE_END_TIMER = 0;
    public static final RevBlinkinLedDriver.BlinkinPattern LED_BLINKIN_PATTERN_ENDGAME_WARNING = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
    public static final double LED_ENDGAME_WARNING_TIMER = 0;
    public static final RevBlinkinLedDriver.BlinkinPattern LED_BLINKIN_PATTERN_ENDGAME_PREPARE_ASCENT_WARNING = RevBlinkinLedDriver.BlinkinPattern.ORANGE;
    public static final double LED_ENDGAME_PREPARE_ASCENT_WARNING_TIMER = 0;
    public static final RevBlinkinLedDriver.BlinkinPattern LED_BLINKIN_PATTERN_ENDGAME_ASCENT_WARNING = RevBlinkinLedDriver.BlinkinPattern.FIRE_LARGE;
    public static final double LED_ENDGAME_ASCENT_TIMER = 0;




    // ========== Field & Vision constants ==========

    // HuskyLens mounting data.
    public static final double cameraMountAngleRadians = Math.toRadians(90);  // rotation relative to the robot's forward direction; left rotation is positive
    public static final Vector2d cameraOffsetInches = new Vector2d(-5.5, 0); // offset from center, x: left/right, y: forward/back, z: height
    public static final double cameraHeightInches = 6.5; // offset from center, x: left/right, y: forward/back, z: height

    // HuskyLens camera/image properties.
    public static final double CAMERA_IMAGE_WIDTH_PIXELS = 320.0;
    public static final double CAMERA_IMAGE_HEIGHT_PIXELS = 240.0;
    public static final double CAMERA_FOCAL_LENGTH_PIXELS = 312.0; // originally 312, which equals experimental calculation

    // AprilTag tag info (FTC std is 36h11 family with 4.0" square tag)
    // Tag size represents length of one side of tag in inches (assumes square tag).
    // Height represents distance above playing surface to tag center in inches (std is 6" for FTC)
    public static final double APRILTAG_TAG_SIZE_INCHES = 4.0;
    public static final double APRILTAG_TAG_HEIGHT_INCHES = 6.0;

    // Localizer properties
    public static final double APRILTAG_UPDATE_INTERVAL_MS = 200; // used for TeleOp localization; controls how often to attempt a tag reading
    public static final double APRILTAG_ANGLE_DETECTION_THRESHOLD_RADIANS = Math.toRadians(20.0); // ignore pose estimates outside of this range

    // Filter properties for pose readings
    // myFilterAlpha is exponential smoothing from [0, 1] for AprilTag pose readings; lower=smoother/slower, higher=more responsive
    // Kalman filter for fusing odometry and AprilTag pose estimates
    public static final double myFilterAlpha = 0.6; // exponential smoothing for AprilTag pose readings; lower values = smoother/slower; higher values = less smooth/more responsive
    public static final double[] KALMAN_FILTER_Q_DIAGONAL = {1.0, 1.0, 0.009}; // Process noise (Odometry)
    public static final double[] KALMAN_FILTER_R_DIAGONAL = {3.0, 3.0, 0.05}; // Measurement noise (AprilTag)



    //------------------------------------------------------------------------------
    // Auton Coordinates
    // All poses and coordinates based on BLUE-Basket quadrant (+x,+y).
    // Values not listed are derived in BaseAutoOp.
    //------------------------------------------------------------------------------

    // Auton start pose
    public static final double AUTON_START_X = 17.0;
    public static final double AUTON_START_Y = 64.2;
    public static final double AUTON_START_HEADING = Math.toRadians(-90);

    // Basket drop pose
    public static final double BASKET_DROP_X = 52.5; // Josh: 52.0, Matt HotRod3: 51.0
    public static final double BASKET_DROP_Y = 52.5; // Josh: 52.0, Matt HotRod3: 51.0
    public static final double BASKET_DROP_HEADING_AUTON = Math.toRadians(-135) + Math.toRadians(180); //-135
    public static final double BASKET_DROP_HEADING_TELEOP = Math.toRadians(-135); //-135

    // Initial specimen hang on high chamber
    public static final double INITIAL_SPECIMEN_HANG_X = 7.75; //5.75
    public static final double INITIAL_SPECIMEN_HANG_Y = 40.45; //39.25
    public static final double INITIAL_SPECIMEN_HANG_HEADING = Math.toRadians(-90);
    public static final double SPECIMEN_HANG_OFFSET = 3.0; // offset between Specimens when hanging

    // Neutral (yellow) sample pick 1 (closest to center of field)
    public static final double NEUTRAL_SAMPLE_1_PICK_X = 49.5; //47.5
    public static final double NEUTRAL_SAMPLE_1_PICK_Y = 39.1; //38.4
    public static final double NEUTRAL_SAMPLE_1_PICK_HEADING = Math.toRadians(-90);

    // Neutral (yellow) sample pick 2 (middle)
    public static final double NEUTRAL_SAMPLE_2_PICK_X = 59.5; //57.5
    public static final double NEUTRAL_SAMPLE_2_PICK_Y = 39.1; //38.3
    public static final double NEUTRAL_SAMPLE_2_PICK_HEADING = Math.toRadians(-90);

    // Human player side offset for middle sample x position
    public static final double ALLIANCE_SAMPLE_2_OFFSET_X = 1.5; // adjustment for human side auton for robot dynamics

    // Neutral (yellow) sample pick 3 (edge of field)
    public static final double NEUTRAL_SAMPLE_3_ALIGN_X = 50.5; //48.5
    public static final double NEUTRAL_SAMPLE_3_PICK_X = 54.1; //52.1
    public static final double NEUTRAL_SAMPLE_3_ALIGN_Y = 26.4; //25.2
    public static final double NEUTRAL_SAMPLE_3_ALIGN_HEADING = Math.toRadians(0);

    // Human player side offset for edge sample y position
    public static final double ALLIANCE_SAMPLE_3_OFFSET_Y = 1.0; // adjustment for human side auton for robot dynamics

    // Submersible parking pose for basket side auton
    public static final double SUBMERSIBLE_PARK_X = 23.5; //21.5
    public static final double SUBMERSIBLE_ALIGN_X = 37.0; //35.0
    public static final double SUBMERSIBLE_PARK_Y = 14.0; // Josh robot: 15.2
    public static final double SUBMERSIBLE_ALIGN_Y = 14.0; //Josh robot: 15.2
    public static final double SUBMERSIBLE_ALIGN_HEADING = Math.toRadians(180);

    // Human specimen side auton wall pick pose
    public static final double WALL_ALIGN_X = 38.0; //36.0
    public static final double WALL_ALIGN_Y = 49.2; //48.0
    public static final double WALL_PICK_Y = 57.8; //56.6
    public static final double WALL_ALIGN_HEADING = Math.toRadians(90);
    public static final double WALl_ALIGN_Y_OFFSET = 6.0; // offset away from field wall where robot backs up before turning and heading to Specimen hang.
    public static final double WALl_ALIGN_HEADING_CHECKPOINT = Math.toRadians(0);

    // Human specimen side auton observation zone pose (where the alliance samples are dropped to make specimens)
    public static final double ZONE_DROP_X = 56.0; //54.0
    public static final double ZONE_DROP_Y = 49.2; //48.0
    public static final double ZONE_DROP_HEADING = Math.toRadians(90);

}
