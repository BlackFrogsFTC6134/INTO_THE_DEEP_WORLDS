package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RATE_LIMITER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RATE_LIMITER_ACCEL_NAV_FORWARD_SAMPLE_PUSH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RATE_LIMITER_ACCEL_NAV_REVERSE_SAMPLE_PUSH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RATE_LIMITER_ACCEL_NAV_TO_BASKET;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RATE_LIMITER_ACCEL_NAV_TO_PRESET;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RATE_LIMITER_ACCEL_NAV_TO_PUSH_CHECKPOINT;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RATE_LIMITER_ACCEL_NAV_TO_SUBMERSIBLE;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RATE_LIMITER_ACCEL_NAV_TO_YELLOW;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RATE_LIMITER_AUTON;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RATE_LIMITER_VEL_NAV_FORWARD_SAMPLE_PUSH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RATE_LIMITER_VEL_NAV_REVERSE_SAMPLE_PUSH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RATE_LIMITER_VEL_NAV_TO_BASKET;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RATE_LIMITER_VEL_NAV_TO_PRESET;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RATE_LIMITER_VEL_NAV_TO_PUSH_CHECKPOINT;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RATE_LIMITER_VEL_NAV_TO_YELLOW;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Slider2;
import org.firstinspires.ftc.teamcode.subsystem.Pivot;
import org.firstinspires.ftc.teamcode.subsystem.Claw;
import org.firstinspires.ftc.teamcode.subsystem.Wrist;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

public abstract class BaseAutoOp extends BaseOp {
    enum BasketSpecimenState {
        STATE_IDLE,
        STATE_NAV_TO_SPECIMEN_HANG,
        STATE_HANG_SPECIMEN_ON_HIGH_BAR,
        STATE_NAV_FROM_1ST_YELLOW_SAMPLE_TO_BASKET_AND_DROP,
        STATE_NAV_TO_2ND_YELLOW_SAMPLE_AND_PICK,
        STATE_NAV_FROM_2ND_YELLOW_SAMPLE_TO_BASKET_AND_DROP,
        STATE_NAV_TO_3RD_YELLOW_SAMPLE_ALIGNMENT_POINT,
        STATE_NAV_TO_3RD_YELLOW_SAMPLE_AND_PICK,
        STATE_NAV_TO_1ST_YELLOW_SAMPLE,
        STATE_PICK_1ST_YELLOW_SAMPLE,
        STATE_NAV_FROM_BASKET_TO_SUBMERSIBLE_PARK
    }

    enum HumanSpecimenState {
        STATE_IDLE,
        STATE_NAV_TO_SPECIMEN_HANG,
        STATE_NAV_TO_2ND_SPECIMEN_HANG,
        STATE_HANG_SPECIMEN_ON_HIGH_BAR,
        STATE_PUSH_SEGMENT1_ORIG,
        STATE_PUSH_SEGMENT2_ORIG,
        STATE_PUSH_SEGMENT3_ORIG,
        STATE_PUSH_SEGMENT_FINAL,
        STATE_PUSH_SEGMENT1_NEW,
        STATE_PUSH_SEGMENT2_NEW,
        STATE_PUSH_SEGMENT3_NEW,
        STATE_PUSH_TRAJECTORY_SEQUENCE_FULL,
        STATE_NAV_TO_1ST_ALLIANCE_SAMPLE,
        STATE_NAV_TO_2ND_ALLIANCE_SAMPLE,
        STATE_PICK_1ST_ALLIANCE_SAMPLE,
        STATE_NAV_TO_3RD_ALLIANCE_SAMPLE_PICK,
        STATE_NAV_TO_1ST_ZONE_DROP_AND_RELEASE,
        STATE_NAV_TO_2ND_ZONE_DROP_AND_RELEASE,
        STATE_PICK_2ND_ALLIANCE_SAMPLE,
        STATE_NAV_TO_WALL_ALIGN_AND_DROP_SAMPLE,
        STATE_NAV_TO_3RD_ALLIANCE_SAMPLE_ALIGN_POINT,
        STATE_NAV_TO_WALL_PICK,
        STATE_HANG_2ND_SPECIMEN_ON_HIGH_BAR,
        STATE_NAV_FROM_1ST_YELLOW_SAMPLE_TO_BASKET_AND_DROP,
        STATE_NAV_TO_2ND_YELLOW_SAMPLE_AND_PICK,
        STATE_NAV_FROM_2ND_YELLOW_SAMPLE_TO_BASKET_AND_DROP,
        STATE_NAV_TO_3RD_YELLOW_SAMPLE_ALIGNMENT_POINT,
        STATE_NAV_TO_3RD_YELLOW_SAMPLE_AND_PICK,
        STATE_NAV_TO_1ST_YELLOW_SAMPLE,
        STATE_PICK_1ST_YELLOW_SAMPLE,
        STATE_NAV_FROM_BASKET_TO_SUBMERSIBLE_PARK
    }

    BasketSpecimenState currentBasketSpecimenState = BasketSpecimenState.STATE_IDLE;
    HumanSpecimenState currentHumanSpecimenState = HumanSpecimenState.STATE_IDLE;

    protected Trajectory navToSpecimenHang;
    public static volatile double INITIAL_SPECIMEN_HANG_X = 5.75; //5.0;
    public static volatile double INITIAL_SPECIMEN_HANG_Y = 39.25; //40.0;
    public static volatile double INITIAL_SPECIMEN_HANG_HEADING = Math.toRadians(-90);
    public static double SPECIMEN_HANG_OFFSET = 2.5;
    protected Pose2d submersiblePark;
    protected Pose2d submersibleAlign;
    protected Trajectory submersibleSegment1, submersibleSegment2;
    protected final static double SUBMERSIBLE_PARK_X = 21.5;
    protected final static double SUBMERSIBLE_ALIGN_X = 35.0;
    protected final static double SUBMERSIBLE_PARK_Y = 14.0;
    protected final static double SUBMERSIBLE_ALIGN_Y = 14.0;
    protected Pose2d startPose;
    protected Pose2d initialSpecimenHang;
    protected Pose2d intermediatePosition1;
    protected Pose2d intermediatePosition2;
    protected Pose2d firstPushPosition;
    protected Trajectory navToFirstNeutral, navToSecondNeutral, navToThirdNeutralAlign, navToThirdNeutralPick, navToWallAlign, navToWallPick;
    protected final static double NEUTRAL_SAMPLE_1_PICK_X = 47.5; //45.0
    protected final static double NEUTRAL_SAMPLE_1_PICK_Y = 38.4;
    protected final static double NEUTRAL_SAMPLE_1_PICK_HEADING = Math.toRadians(-90);
    protected final static double NEUTRAL_SAMPLE_2_PICK_X = 57.5;
    protected final static double NEUTRAL_SAMPLE_2_PICK_Y = 38.3;
    protected final static double NEUTRAL_SAMPLE_2_PICK_HEADING = Math.toRadians(-90);
    protected final static double NEUTRAL_SAMPLE_3_ALIGN_X = 48.5; //52
    protected final static double NEUTRAL_SAMPLE_3_PICK_X = 52.1; //52
    protected final static double NEUTRAL_SAMPLE_3_ALIGN_Y = 25.2; //25.7
    protected final static double NEUTRAL_SAMPLE_3_ALIGN_HEADING = Math.toRadians(0);
    protected final static double WALL_ALIGN_X = 48.0;
    protected final static double WALL_ALIGN_Y = 54.0;
    protected final static double WALL_PICK_Y = 62.0;
    protected final static double WALL_ALIGN_HEADING = Math.toRadians(90);
    protected final static double ZONE_DROP_X = 54.0;
    protected final static double ZONE_DROP_Y = 48.0;
    protected final static double ZONE_DROP_HEADING = Math.toRadians(90);


    protected Pose2d firstNeutralSamplePick;
    protected Pose2d secondNeutralSamplePick;
    protected Pose2d thirdNeutralSampleAlign;
    protected Pose2d thirdNeutralSamplePick;
    protected Pose2d basketDrop;
    protected Trajectory navToThirdSampleAlign, navToThirdSamplePick, navTo1stSpecimenHang;
    protected Pose2d zoneDrop, wallPick, wallAlign;
    protected Trajectory navToFirstSample, navToSecondSample, navToZoneDrop, navToZoneDrop2;
    protected Trajectory navToBasket1, navToBasket2, navToBasket3;
    protected final static double BASKET_DROP_X = 50.0;
    protected final static double BASKET_DROP_Y = 50.0;
    protected final static double BASKET_DROP_HEADING = Math.toRadians(-135); //-135
    public static double PRELOADED_SAMPLE_START_MARKER = 10.0;
    public static double THIRD_SAMPLE_START_MARKER = 1.0;
    public static double SAMPLE_TEST_DELAY = 0.5;
    protected Pose2d observationZone;
    protected Pose2d specimenWallPick;

    // Three Samples Push Sequence
    Trajectory navToPushPresetStart, navToPushThreeSamples;
    Trajectory navToPushStartPoint, navToPushSegment1, navToPushSegment2, navToPushSegment3, navToPushSegmentFull;
    TrajectorySequence navToPushFullSequence;
    static final double PRESET_START_X = 36;
    public static double PRESET_START_Y = 35; // need to push it out a little, so it does not catch on submersible
    public static double PRESET_START_HEADING = Math.toRadians(90);
    static double LAST_SPECIMEN_MARGIN = 4.0;
    static final double PRESET_LOW_POINT = 8.0;
    static double PRESET_HIGH_POINT = 42;
    static final double PRESET_SPACING = 10;

    static final double OBSERVATION_ZONE_PICK_X = -36.5;
    static final double OBSERVATION_ZONE_PICK_Y = 60;
    static final double OBSERVATION_ZONE_PICK_HEADING = PI / 2;


    int allianceMultiplier;

    protected BaseAutoOp(StartingPosition position) {
        mStartingPosition = position;
        msGlobalStartingPosition = position;
        allianceMultiplier = getAlliance() == AllianceSelection.BLUE ? -1 : 1;

        initCheckpoints();

        RATE_LIMITER = RATE_LIMITER_AUTON;
    }

    public AllianceSelection getAlliance() {
        return getAlliance(mStartingPosition);
    }

    protected void initCheckpoints() {
        switch (mStartingPosition) {
            case BLUE_BASKET:
                startPose = new Pose2d(15.0, 63.0, Math.toRadians(-90));
                initialSpecimenHang = new Pose2d(INITIAL_SPECIMEN_HANG_X, INITIAL_SPECIMEN_HANG_Y, INITIAL_SPECIMEN_HANG_HEADING);
                firstNeutralSamplePick = new Pose2d(NEUTRAL_SAMPLE_1_PICK_X, NEUTRAL_SAMPLE_1_PICK_Y, NEUTRAL_SAMPLE_1_PICK_HEADING);
                secondNeutralSamplePick = new Pose2d(NEUTRAL_SAMPLE_2_PICK_X, NEUTRAL_SAMPLE_2_PICK_Y, NEUTRAL_SAMPLE_2_PICK_HEADING);
                thirdNeutralSampleAlign = new Pose2d(NEUTRAL_SAMPLE_3_ALIGN_X, NEUTRAL_SAMPLE_3_ALIGN_Y, NEUTRAL_SAMPLE_3_ALIGN_HEADING + (allianceMultiplier == -1 ? 0 : Math.toRadians(180)));
                thirdNeutralSamplePick = new Pose2d(NEUTRAL_SAMPLE_3_PICK_X, thirdNeutralSampleAlign.getY(), thirdNeutralSampleAlign.getHeading());
                basketDrop = new Pose2d(BASKET_DROP_X, BASKET_DROP_Y, BASKET_DROP_HEADING + Math.toRadians(180));
                //observationZone = new Pose2d(-36, 60, Math.toRadians(170.0));
                submersibleAlign = new Pose2d(-allianceMultiplier * SUBMERSIBLE_ALIGN_X, -allianceMultiplier * SUBMERSIBLE_ALIGN_Y, allianceMultiplier == -1 ? PI : 0);
                submersiblePark = new Pose2d(-allianceMultiplier * SUBMERSIBLE_PARK_X, -allianceMultiplier * SUBMERSIBLE_PARK_Y, allianceMultiplier == -1 ? PI : 0);
                break;

            case RED_BASKET:
                startPose = new Pose2d(-15.0, -63.0, Math.toRadians(90));
                initialSpecimenHang = new Pose2d(-INITIAL_SPECIMEN_HANG_X, -INITIAL_SPECIMEN_HANG_Y, -INITIAL_SPECIMEN_HANG_HEADING); // changed y to 37.0 from original 37.5
                firstNeutralSamplePick = new Pose2d(-NEUTRAL_SAMPLE_1_PICK_X, -NEUTRAL_SAMPLE_1_PICK_Y, -NEUTRAL_SAMPLE_1_PICK_HEADING);
                secondNeutralSamplePick = new Pose2d(-NEUTRAL_SAMPLE_2_PICK_X, -NEUTRAL_SAMPLE_2_PICK_Y, -NEUTRAL_SAMPLE_2_PICK_HEADING);
                thirdNeutralSampleAlign = new Pose2d(-NEUTRAL_SAMPLE_3_ALIGN_X, -NEUTRAL_SAMPLE_3_ALIGN_Y, NEUTRAL_SAMPLE_3_ALIGN_HEADING + (allianceMultiplier == -1 ? 0 : Math.toRadians(180)));
                thirdNeutralSamplePick = new Pose2d(-NEUTRAL_SAMPLE_3_PICK_X, thirdNeutralSampleAlign.getY(), thirdNeutralSampleAlign.getHeading());
                basketDrop = new Pose2d(-BASKET_DROP_X, -BASKET_DROP_Y, -BASKET_DROP_HEADING + Math.toRadians(90));
                //observationZone = new Pose2d(36, -60, Math.toRadians(-10.0));
                submersiblePark = new Pose2d(-allianceMultiplier * SUBMERSIBLE_PARK_X, -allianceMultiplier * SUBMERSIBLE_PARK_Y, allianceMultiplier == -1 ? PI : 0);
                break;

            case BLUE_HUMAN:
                startPose = new Pose2d(-15.0, 63.0, Math.toRadians(-90));
                initialSpecimenHang = new Pose2d(-INITIAL_SPECIMEN_HANG_X, INITIAL_SPECIMEN_HANG_Y, INITIAL_SPECIMEN_HANG_HEADING);
                firstNeutralSamplePick = new Pose2d(-NEUTRAL_SAMPLE_1_PICK_X, NEUTRAL_SAMPLE_1_PICK_Y, NEUTRAL_SAMPLE_1_PICK_HEADING);
                secondNeutralSamplePick = new Pose2d(-NEUTRAL_SAMPLE_2_PICK_X, NEUTRAL_SAMPLE_2_PICK_Y, NEUTRAL_SAMPLE_2_PICK_HEADING);
                thirdNeutralSampleAlign = new Pose2d(-NEUTRAL_SAMPLE_3_ALIGN_X, NEUTRAL_SAMPLE_3_ALIGN_Y, NEUTRAL_SAMPLE_3_ALIGN_HEADING + (allianceMultiplier == -1 ? Math.toRadians(180) : 0));
                thirdNeutralSamplePick = new Pose2d(-NEUTRAL_SAMPLE_3_PICK_X, thirdNeutralSampleAlign.getY(), thirdNeutralSampleAlign.getHeading());
                zoneDrop = new Pose2d(-ZONE_DROP_X, ZONE_DROP_Y, ZONE_DROP_HEADING);
                wallAlign = new Pose2d(-WALL_ALIGN_X, WALL_ALIGN_Y, WALL_ALIGN_HEADING);
                wallPick = new Pose2d(-WALL_ALIGN_X, WALL_PICK_Y, WALL_ALIGN_HEADING);
                break;

            case RED_HUMAN:
                startPose = new Pose2d(15.0, -63.0, PI / 2);
                initialSpecimenHang = new Pose2d(INITIAL_SPECIMEN_HANG_X, -INITIAL_SPECIMEN_HANG_Y, -PI / 2);
                intermediatePosition1 = new Pose2d(34.0, -34.0, Math.toRadians(-90.0));
                intermediatePosition2 = new Pose2d(34.0, -14.0, Math.toRadians(-90.0));
                firstPushPosition = new Pose2d(47.0, -14.0, Math.toRadians(-90.0));
                specimenWallPick = new Pose2d(-OBSERVATION_ZONE_PICK_X, -OBSERVATION_ZONE_PICK_Y, -OBSERVATION_ZONE_PICK_HEADING);
                break;
        }
    }

    protected void autonInitialize() {
        super.initialize();
        robot.setPoseEstimate(startPose);
        // Move wrist and claw to starting positions
        robot.wrist.setPosition(Wrist.WRIST_START_POSITION);
        sleep(1500);
        robot.claw.setPosition(Claw.CLAW_CLOSE);

        // Move pivot to starting position if home switch is not pressed, and reset encoders
        while (!robot.pivot.isPivotHomeSwitchPressed()) {
            robot.pivot.setPower(-0.25);
            robot.localTelemetry.addData("ArmPivot Pos", robot.pivot.getPosition());
            robot.localTelemetry.addData("Home Switch", robot.pivot.isPivotHomeSwitchPressed());
            robot.localTelemetry.update();
        }
        // Set power to 0 when switch is pressed
        robot.pivot.setPower(0);
        // Wait to stabilize
        sleep(200);
        // Move pivot up so that it's under tension and first tick is always in direction of movement
        while (robot.pivot.isPivotHomeSwitchPressed()) {
            robot.pivot.setPower(.23);
            robot.localTelemetry.addData("ArmPivot Pos", robot.pivot.getPosition());
            robot.localTelemetry.addData("Home Switch", robot.pivot.isPivotHomeSwitchPressed());
            robot.localTelemetry.update();
        }
        // Set power to 0 when switch is no longer depressed
        robot.pivot.setPower(0);
        // Reset pivot motor encoders
        robot.pivot.resetEncoders();
        // Give time for the motor controller to reset the encoders
        sleep(300);
        // Print final telemetry values
        robot.localTelemetry.clear();
        robot.localTelemetry.addData("ArmPivot Pos", robot.pivot.getPosition());
        robot.localTelemetry.addData("Home Switch", robot.pivot.isPivotHomeSwitchPressed());
        robot.localTelemetry.addLine("Arm Pivot Reset");
        robot.localTelemetry.update();
    }

    protected void runBasketSpecimenOpMode() {

        navToSpecimenHang = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineTo(new Vector2d(initialSpecimenHang.getX(), initialSpecimenHang.getY()),
                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_TO_SUBMERSIBLE))
                .addDisplacementMarker(0, () -> robot.pivot.setPosition(Pivot.PIVOT_HANG_SPECIMEN_HIGH_APPROACH)) // 0" or immediately when starting
                .addDisplacementMarker(19.0, () -> robot.slider.setPosition(Slider2.EXTEND_HANG_SPECIMEN_HIGH, false))
                .addDisplacementMarker(20.0, () -> robot.wrist.setPosition(Wrist.WRIST_POSITION_HANG))
                .build();


        // Add telemetry for distance sensors in init loop
        /*
        while (!isStarted() && !isStopRequested()) {
            double leftDistance = robot.getFrontLeftDistance();
            double rightDistance = robot.getFrontRightDistance();
            telemetry.addData("Front Left Distance (in)", leftDistance);
            telemetry.addData("Front Right Distance (in)", rightDistance);
            telemetry.update();
            sleep(50); // Small delay to avoid spamming the telemetry
        }*/

        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to NAV_TO_SPECIMEN_HANG our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentBasketSpecimenState = BasketSpecimenState.STATE_NAV_TO_SPECIMEN_HANG;
        robot.followTrajectoryAsync(navToSpecimenHang);

        while (opModeIsActive() && !isStopRequested()) {

            // Our state machine logic
            // We define the flow of the state machine through this switch statement
            switch (currentBasketSpecimenState) {
                case STATE_NAV_TO_SPECIMEN_HANG:
                    // Check if the drive class is busy by calling 'isBusy()` while it's following the trajectory.
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished. Ensure we use the async follow function
                    // We also need to wait until the pivot and slider are both at target before hanging the specimen
                    if (!robot.isBusy() && robot.pivot.isAtTarget() && robot.slider.isAtTarget()) {
                        robot.armMech.resetHangSpecimenState(false);
                        robot.armMech.hangSpecimenUpdate();
                        currentBasketSpecimenState = BasketSpecimenState.STATE_HANG_SPECIMEN_ON_HIGH_BAR;
                    }
                    break;
                case STATE_HANG_SPECIMEN_ON_HIGH_BAR:
                    // Check if the state machine (hang specimen operation) is finished
                    // 'hangSpecimenUpdate() = false' while the state machine is still executing
                    // 'hangSpecimenUpdate() = true' when the hang operation is complete
                    // When finished, begin trajectory to pick the first neutral sample
                    if (robot.armMech.hangSpecimenUpdate()) {
                        // Set trajectory based on where robot stopped
                        navToFirstNeutral = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineTo(new Vector2d(firstNeutralSamplePick.getX(), firstNeutralSamplePick.getY()))
                                .addDisplacementMarker(4.0, () -> robot.pivot.setPosition(Pivot.PIVOT_INTERMEDIATE))
                                .build();
                        robot.followTrajectoryAsync(navToFirstNeutral);
                        currentBasketSpecimenState = BasketSpecimenState.STATE_NAV_TO_1ST_YELLOW_SAMPLE;
                    }
                    break;
                case STATE_NAV_TO_1ST_YELLOW_SAMPLE:
                    // Wait for trajectory to finish and pivot to reach pre-pickup position, and then begin pickSample() sequence
                    if (!robot.isBusy() && robot.pivot.isAtTarget()) {
                        // Pickup sample from floor by calling pickSample() method in ArmMech
                        robot.armMech.resetPickSampleState(); // reset pickSample() to START state
                        robot.armMech.pickSample();
                        currentBasketSpecimenState = BasketSpecimenState.STATE_PICK_1ST_YELLOW_SAMPLE;
                    }
                    break;
                case STATE_PICK_1ST_YELLOW_SAMPLE:
                    // When sample pick is complete, begin trajectory to basket drop and start basket drop shortly
                    // into trajectory, with the assumption that the basket drop sequence will take much longer to
                    // complete than the trajectory.
                    if (robot.armMech.pickSample()) {
                        robot.armMech.resetDropSampleHighBasketState(); // reset dropSampleHighBasket to START state
                        navToBasket1 = robot.trajectoryBuilder(robot.getPoseEstimate(), true)
                                .splineTo(new Vector2d(basketDrop.getX(), basketDrop.getY()), basketDrop.getHeading())
                                .addDisplacementMarker(0.2, () -> robot.armMech.dropSampleHighBasket())
                                .build();
                        robot.followTrajectoryAsync(navToBasket1);
                        currentBasketSpecimenState = BasketSpecimenState.STATE_NAV_FROM_1ST_YELLOW_SAMPLE_TO_BASKET_AND_DROP;
                    }
                    break;
                case STATE_NAV_FROM_1ST_YELLOW_SAMPLE_TO_BASKET_AND_DROP:
                    // Wait for the basket drop to finish (also the trajectory to finish, but that is expected
                    // to finish long before the sample drop is finished).
                    // When finished, start trajectory to pick 2nd yellow sample; pickSample() must be timed using displacement marker
                    // such that the wrist does not contact and push the sample; wrist should come down over the top of sample as trajectory ends.
                    if (!robot.isBusy() && robot.armMech.dropSampleHighBasket()) {
                        robot.armMech.resetPickSampleState(); // reset pickSample() to START state
                        navToSecondNeutral = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineToLinearHeading(new Pose2d(secondNeutralSamplePick.getX(), secondNeutralSamplePick.getY(), secondNeutralSamplePick.getHeading()))
                                .addDisplacementMarker(5.0, () -> robot.armMech.pickSample())
                                .build();
                        robot.followTrajectoryAsync(navToSecondNeutral);
                        currentBasketSpecimenState = BasketSpecimenState.STATE_NAV_TO_2ND_YELLOW_SAMPLE_AND_PICK;
                    }
                    break;
                case STATE_NAV_TO_2ND_YELLOW_SAMPLE_AND_PICK:
                    // Wait for trajectory and yellow sample pick to finish; when finished, begin trajectory
                    // to basket drop and drop sample in high basket.
                    if (!robot.isBusy() && robot.armMech.pickSample()) {
                        robot.armMech.resetDropSampleHighBasketState(); // reset dropSampleHighBasket to START state
                        navToBasket2 = robot.trajectoryBuilder(robot.getPoseEstimate(), true)
                                .lineToLinearHeading(new Pose2d(basketDrop.getX(), basketDrop.getY(), basketDrop.getHeading() + Math.toRadians(180)))
                                .addDisplacementMarker(0.2, () -> robot.armMech.dropSampleHighBasket())
                                .build();
                        robot.followTrajectoryAsync(navToBasket2);
                        currentBasketSpecimenState = BasketSpecimenState.STATE_NAV_FROM_2ND_YELLOW_SAMPLE_TO_BASKET_AND_DROP;
                    }
                    break;
                case STATE_NAV_FROM_2ND_YELLOW_SAMPLE_TO_BASKET_AND_DROP:
                    // Wait for the basket drop to finish (also the trajectory to finish, but that is expected
                    // to finish long before the sample drop is finished).
                    // When finished, start trajectory to align to 3rd yellow sample; and move pivot to prep position (above 3rd sample)
                    if (!robot.isBusy() && robot.armMech.dropSampleHighBasket()) {
                        navToThirdNeutralAlign = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineToLinearHeading(new Pose2d(thirdNeutralSampleAlign.getX(), thirdNeutralSampleAlign.getY(), thirdNeutralSampleAlign.getHeading()))
                                .addDisplacementMarker(5.0, () -> robot.pivot.setPosition(Pivot.PIVOT_PICKUP_3RD_YELLOW))
                                .build();
                        robot.followTrajectoryAsync(navToThirdNeutralAlign);
                        currentBasketSpecimenState = BasketSpecimenState.STATE_NAV_TO_3RD_YELLOW_SAMPLE_ALIGNMENT_POINT;
                    }
                    break;
                case STATE_NAV_TO_3RD_YELLOW_SAMPLE_ALIGNMENT_POINT:
                    // Wait for trajectory to 3rd yellow sample alignment point to finish and for
                    // pivot to be at target (high enough to clear sample), and then begin movement to
                    // 3rd yellow sample pick point and pick sample
                    if (!robot.isBusy() && robot.pivot.isAtTarget()) {
                        robot.armMech.resetPickSampleState(); // reset pickSample() to START state
                        navToThirdNeutralPick = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineTo(new Vector2d(thirdNeutralSamplePick.getX(), thirdNeutralSamplePick.getY()))
                                .addDisplacementMarker(3.5, () -> robot.armMech.pickSample())
                                .build();
                        robot.followTrajectoryAsync(navToThirdNeutralPick);
                        currentBasketSpecimenState = BasketSpecimenState.STATE_NAV_TO_3RD_YELLOW_SAMPLE_AND_PICK;
                    }
                    break;
                case STATE_NAV_TO_3RD_YELLOW_SAMPLE_AND_PICK:
                    // Wait for trajectory and 3rd yellow sample pick to finish; when finished, begin trajectory
                    // to basket drop and drop sample in high basket.
                    if (!robot.isBusy() && robot.armMech.pickSample()) {
                        robot.armMech.resetDropSampleHighBasketState(); // reset dropSampleHighBasket to START state
                        navToBasket3 = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineToLinearHeading(new Pose2d(basketDrop.getX(), basketDrop.getY(), basketDrop.getHeading() + Math.toRadians(180)))
                                .addDisplacementMarker(0.2, () -> robot.armMech.dropSampleHighBasket())
                                .build();
                        robot.followTrajectoryAsync(navToBasket3);
                        currentBasketSpecimenState = BasketSpecimenState.STATE_NAV_FROM_BASKET_TO_SUBMERSIBLE_PARK;
                    }
                    break;
                case STATE_NAV_FROM_BASKET_TO_SUBMERSIBLE_PARK:
                    // When basket drop is finished, begin submersible parking trajectory, and move the pivot, wrist, and slider
                    // mechanisms to prepare to rest on the high rung
                    // Note: the submersible parking sequence is two trajectories chained together
                    if (!robot.isBusy() && robot.armMech.dropSampleHighBasket()) {
                        submersibleSegment1 = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineToLinearHeading(new Pose2d(submersibleAlign.getX(), submersibleAlign.getY(), submersibleAlign.getHeading()))
                                .addDisplacementMarker(6.0, () -> robot.pivot.setPosition(Pivot.PIVOT_SUBMERSIBLE_PARK))
                                .addDisplacementMarker(10.0, () -> robot.wrist.setPosition(Wrist.WRIST_POSITION_HIGH_BASKET_DROP))
                                .addDisplacementMarker(16.0, () -> robot.slider.setPosition(Slider2.EXTEND_HANG_SPECIMEN_HIGH, false))
                                .addDisplacementMarker(() -> robot.followTrajectoryAsync(submersibleSegment2))
                                .build();
                        submersibleSegment2 = robot.trajectoryBuilder(submersibleSegment1.end(), false)
                                .lineTo(new Vector2d(submersiblePark.getX(), submersiblePark.getY()))
                                .build();
                        robot.followTrajectoryAsync(submersibleSegment1);
                        currentBasketSpecimenState = BasketSpecimenState.STATE_IDLE;
                        break;
                    }
                case STATE_IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }
            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            robot.update();
            // We update our pivot and slider PID controllers continuously in the background, regardless of state
            robot.pivot.update();
            robot.slider.update();

            // Read pose
            Pose2d poseEstimate = robot.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            //PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            robot.localTelemetry.addData("x", poseEstimate.getX());
            robot.localTelemetry.addData("y", poseEstimate.getY());
            robot.localTelemetry.addData("heading", poseEstimate.getHeading());
            robot.localTelemetry.addData("Pivot Pos", robot.pivot.getPosition());
            robot.localTelemetry.addData("Slider Pos", robot.slider.getPosition());
            robot.localTelemetry.addData("Slider at Target", robot.slider.isAtTarget());
            robot.localTelemetry.addData("Pivot at Target", robot.pivot.isAtTarget());
            robot.localTelemetry.update();
        }

    }

    protected void runHumanSpecimenOpMode2() {
        navToSpecimenHang = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineTo(new Vector2d(initialSpecimenHang.getX(), initialSpecimenHang.getY()),
                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_TO_SUBMERSIBLE))
                .addDisplacementMarker(0, () -> robot.pivot.setPosition(Pivot.PIVOT_HANG_SPECIMEN_HIGH_APPROACH)) // 0" or immediately when starting
                //.addDisplacementMarker(22.0, () -> robot.slider.setPosition(Slider2.EXTEND_HANG_SPECIMEN_HIGH, false))
                .addDisplacementMarker(() -> robot.wrist.setPosition(Wrist.WRIST_POSITION_HANG))
                .addDisplacementMarker(() -> robot.slider.setPosition(Slider2.EXTEND_HANG_SPECIMEN_HIGH, false))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to NAV_TO_SPECIMEN_HANG our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentHumanSpecimenState = HumanSpecimenState.STATE_NAV_TO_SPECIMEN_HANG;
        robot.followTrajectoryAsync(navToSpecimenHang);

        while (opModeIsActive() && !isStopRequested()) {

            // Our state machine logic
            // We define the flow of the state machine through this switch statement
            switch (currentHumanSpecimenState) {
                case STATE_NAV_TO_SPECIMEN_HANG:
                    // Check if the drive class is busy by calling 'isBusy()` while it's following the trajectory.
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished. Ensure we use the async follow function
                    // We also need to wait until the pivot and slider are both at target before hanging the specimen
                    if (!robot.isBusy() && robot.pivot.isAtTarget() && robot.slider.isAtTarget()) {
                        robot.armMech.resetHangSpecimenState(false);
                        robot.armMech.hangSpecimenUpdate();
                        currentHumanSpecimenState = HumanSpecimenState.STATE_HANG_SPECIMEN_ON_HIGH_BAR;
                        //currentHumanSpecimenState = HumanSpecimenState.STATE_PUSH_SEGMENT1_ORIG;
                    }
                    break;
                case STATE_HANG_SPECIMEN_ON_HIGH_BAR:
                    // Check if the state machine (hang specimen operation) is finished
                    // 'hangSpecimenUpdate() = false' while the state machine is still executing
                    // 'hangSpecimenUpdate() = true' when the hang operation is complete
                    // When finished, begin trajectory to push the first alliance-specific sample
                    if (robot.armMech.hangSpecimenUpdate()) {
                        navToFirstSample = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineTo(new Vector2d(firstNeutralSamplePick.getX(), firstNeutralSamplePick.getY()))
                                .addDisplacementMarker(4.0, () -> robot.pivot.setPosition(Pivot.PIVOT_INTERMEDIATE))
                                .build();
                        robot.followTrajectoryAsync(navToFirstSample);
                        currentHumanSpecimenState = HumanSpecimenState.STATE_NAV_TO_1ST_ALLIANCE_SAMPLE;
                    }
                    break;
                case STATE_NAV_TO_1ST_ALLIANCE_SAMPLE:
                    // Wait for trajectory to finish and pivot to reach pre-pickup position, and then begin pickSample() sequence
                    if (!robot.isBusy() && robot.pivot.isAtTarget()) {
                        // Pickup sample from floor by calling pickSample() method in ArmMech
                        robot.armMech.resetPickSampleState(); // reset pickSample() to START state
                        robot.armMech.pickSample();
                        currentHumanSpecimenState = HumanSpecimenState.STATE_PICK_1ST_ALLIANCE_SAMPLE;
                    }
                    break;
                case STATE_PICK_1ST_ALLIANCE_SAMPLE:
                    // When sample pick is complete, begin trajectory to basket drop and start basket drop shortly
                    // into trajectory, with the assumption that the basket drop sequence will take much longer to
                    // complete than the trajectory.
                    if (robot.armMech.pickSample()) {
                        robot.armMech.resetDropSampleZone();
                        navToZoneDrop = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineToLinearHeading(new Pose2d(zoneDrop.getX(), zoneDrop.getY(), zoneDrop.getHeading()))
                                .addDisplacementMarker(() -> robot.armMech.dropSampleZone())
                                .build();
                        robot.followTrajectoryAsync(navToZoneDrop);
                        currentHumanSpecimenState = HumanSpecimenState.STATE_NAV_TO_1ST_ZONE_DROP_AND_RELEASE;
                    }
                    break;
                case STATE_NAV_TO_1ST_ZONE_DROP_AND_RELEASE:
                    if (!robot.isBusy() && robot.armMech.dropSampleZone()) {
                        navToSecondSample = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineToLinearHeading(new Pose2d(secondNeutralSamplePick.getX(), secondNeutralSamplePick.getY(), secondNeutralSamplePick.getHeading()))
                                .addDisplacementMarker(4.0, () -> robot.pivot.setPosition(Pivot.PIVOT_INTERMEDIATE))
                                .build();
                        robot.followTrajectoryAsync(navToSecondSample);
                        currentHumanSpecimenState = HumanSpecimenState.STATE_NAV_TO_2ND_ALLIANCE_SAMPLE;
                    }
                    break;
                case STATE_NAV_TO_2ND_ALLIANCE_SAMPLE:
                    if (!robot.isBusy() && robot.pivot.isAtTarget()) {
                        // Pickup sample from floor by calling pickSample() method in ArmMech
                        robot.armMech.resetPickSampleState(); // reset pickSample() to START state
                        robot.armMech.pickSample();
                        currentHumanSpecimenState = HumanSpecimenState.STATE_PICK_2ND_ALLIANCE_SAMPLE;
                    }
                    break;
                case STATE_PICK_2ND_ALLIANCE_SAMPLE:
                    // When sample pick is complete, begin trajectory to basket drop and start basket drop shortly
                    // into trajectory, with the assumption that the basket drop sequence will take much longer to
                    // complete than the trajectory.
                    if (robot.armMech.pickSample()) {
                        robot.armMech.resetDropSampleZone();
                        navToZoneDrop2 = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineToLinearHeading(new Pose2d(zoneDrop.getX(), zoneDrop.getY(), zoneDrop.getHeading()))
                                .addDisplacementMarker(() -> robot.armMech.dropSampleZone())
                                .build();
                        robot.followTrajectoryAsync(navToZoneDrop2);
                        currentHumanSpecimenState = HumanSpecimenState.STATE_NAV_TO_2ND_ZONE_DROP_AND_RELEASE;
                    }
                    break;
                case STATE_NAV_TO_2ND_ZONE_DROP_AND_RELEASE:
                    if (!robot.isBusy() && robot.armMech.dropSampleZone()) {
                        navToThirdSampleAlign = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineToLinearHeading(new Pose2d(thirdNeutralSampleAlign.getX(), thirdNeutralSampleAlign.getY(), thirdNeutralSampleAlign.getHeading()))
                                .addDisplacementMarker(1.0, () -> robot.pivot.setPosition(Pivot.PIVOT_PICKUP_3RD_YELLOW))
                                .build();
                        robot.followTrajectoryAsync(navToThirdSampleAlign);
                        currentHumanSpecimenState = HumanSpecimenState.STATE_NAV_TO_3RD_ALLIANCE_SAMPLE_ALIGN_POINT;
                    }
                    break;
                case STATE_NAV_TO_3RD_ALLIANCE_SAMPLE_ALIGN_POINT:
                    if (!robot.isBusy() && robot.pivot.isAtTarget()) {
                        robot.armMech.resetPickSampleState();
                        navToThirdSamplePick = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineTo(new Vector2d(thirdNeutralSamplePick.getX(), thirdNeutralSamplePick.getY()))
                                .addDisplacementMarker(() -> robot.armMech.pickSample())
                                .build();
                        robot.followTrajectoryAsync(navToThirdSamplePick);
                        currentHumanSpecimenState = HumanSpecimenState.STATE_NAV_TO_3RD_ALLIANCE_SAMPLE_PICK;
                    }
                    break;
                case STATE_NAV_TO_3RD_ALLIANCE_SAMPLE_PICK:
                    if (!robot.isBusy() && robot.armMech.pickSample()) {
                        robot.armMech.resetDropSampleZone(); // reset dropSampleHighBasket to START state
                        navToWallAlign = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineToLinearHeading(new Pose2d(wallAlign.getX(), wallAlign.getY(), wallAlign.getHeading()))
                                .addDisplacementMarker(() -> robot.armMech.dropSampleZone())
                                .build();
                        robot.followTrajectoryAsync(navToWallAlign);
                        currentHumanSpecimenState = HumanSpecimenState.STATE_NAV_TO_WALL_ALIGN_AND_DROP_SAMPLE;
                    }
                    break;
                case STATE_NAV_TO_WALL_ALIGN_AND_DROP_SAMPLE:
                    if (!robot.isBusy() && robot.armMech.dropSampleZone()) {
                        robot.armMech.resetPickWallSpecimenState();
                        navToWallPick = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineTo(new Vector2d(wallPick.getX(), wallPick.getY()))
                                .addDisplacementMarker(() -> robot.armMech.pickWallSpecimen())
                                .build();
                        robot.followTrajectoryAsync(navToWallPick);
                        currentHumanSpecimenState = HumanSpecimenState.STATE_NAV_TO_WALL_PICK;
                    }
                    break;
                case STATE_NAV_TO_WALL_PICK:
                    if (!robot.isBusy() && robot.armMech.pickWallSpecimen()) {
                        navTo1stSpecimenHang = robot.trajectoryBuilder(robot.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(initialSpecimenHang.getX() + 5, initialSpecimenHang.getY(), initialSpecimenHang.getHeading()),
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_TO_SUBMERSIBLE))
                                .addDisplacementMarker(0, () -> robot.pivot.setPosition(Pivot.PIVOT_HANG_SPECIMEN_HIGH_APPROACH)) // 0" or immediately when starting
                                //.addDisplacementMarker(22.0, () -> robot.slider.setPosition(Slider2.EXTEND_HANG_SPECIMEN_HIGH, false))
                                .addDisplacementMarker(() -> robot.wrist.setPosition(Wrist.WRIST_POSITION_HANG))
                                .addDisplacementMarker(() -> robot.slider.setPosition(Slider2.EXTEND_HANG_SPECIMEN_HIGH, false))
                                .build();
                        robot.followTrajectoryAsync(navTo1stSpecimenHang);
                        currentHumanSpecimenState = HumanSpecimenState.STATE_NAV_TO_2ND_SPECIMEN_HANG;
                    }
                    break;
                case STATE_NAV_TO_2ND_SPECIMEN_HANG:
                    // Check if the drive class is busy by calling 'isBusy()` while it's following the trajectory.
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished. Ensure we use the async follow function
                    // We also need to wait until the pivot and slider are both at target before hanging the specimen
                    if (!robot.isBusy() && robot.pivot.isAtTarget() && robot.slider.isAtTarget()) {
                        robot.armMech.resetHangSpecimenState(false);
                        robot.armMech.hangSpecimenUpdate();
                        currentHumanSpecimenState = HumanSpecimenState.STATE_HANG_2ND_SPECIMEN_ON_HIGH_BAR;
                    }
                    break;
                case STATE_HANG_2ND_SPECIMEN_ON_HIGH_BAR:
                    // Check if the state machine (hang specimen operation) is finished
                    // 'hangSpecimenUpdate() = false' while the state machine is still executing
                    // 'hangSpecimenUpdate() = true' when the hang operation is complete
                    // When finished, begin trajectory to push the first alliance-specific sample
                    if (robot.armMech.hangSpecimenUpdate()) {
                        currentHumanSpecimenState = HumanSpecimenState.STATE_IDLE;
                    }
                    break;
                case STATE_IDLE:
                    break;


            }
            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            robot.update();
            // We update our pivot and slider PID controllers continuously in the background, regardless of state
            robot.pivot.update();
            robot.slider.update();

            // Read pose
            Pose2d poseEstimate = robot.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            //PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            robot.localTelemetry.addData("x", poseEstimate.getX());
            robot.localTelemetry.addData("y", poseEstimate.getY());
            robot.localTelemetry.addData("heading", poseEstimate.getHeading());
            robot.localTelemetry.addData("Pivot Pos", robot.pivot.getPosition());
            robot.localTelemetry.addData("Slider Pos", robot.slider.getPosition());
            robot.localTelemetry.addData("Slider at Target", robot.slider.isAtTarget());
            robot.localTelemetry.addData("Pivot at Target", robot.pivot.isAtTarget());
            robot.localTelemetry.update();
        }
    }

    protected void runHumanSpecimenOpMode() {

        navToSpecimenHang = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineTo(new Vector2d(initialSpecimenHang.getX(), initialSpecimenHang.getY()),
                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_TO_SUBMERSIBLE))
                .addDisplacementMarker(0, () -> robot.pivot.setPosition(Pivot.PIVOT_HANG_SPECIMEN_HIGH_APPROACH)) // 0" or immediately when starting
                //.addDisplacementMarker(22.0, () -> robot.slider.setPosition(Slider2.EXTEND_HANG_SPECIMEN_HIGH, false))
                .addDisplacementMarker(() -> robot.wrist.setPosition(Wrist.WRIST_POSITION_HANG))
                .addDisplacementMarker(() -> robot.slider.setPosition(Slider2.EXTEND_HANG_SPECIMEN_HIGH, false))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to NAV_TO_SPECIMEN_HANG our first step
        // Then have it follow that trajectory
        // Make sure you use the async version of the commands
        // Otherwise it will be blocking and pause the program here until the trajectory finishes
        currentHumanSpecimenState = HumanSpecimenState.STATE_NAV_TO_SPECIMEN_HANG;
        robot.followTrajectoryAsync(navToSpecimenHang);

        while (opModeIsActive() && !isStopRequested()) {

            // Our state machine logic
            // We define the flow of the state machine through this switch statement
            switch (currentHumanSpecimenState) {
                case STATE_NAV_TO_SPECIMEN_HANG:
                    // Check if the drive class is busy by calling 'isBusy()` while it's following the trajectory.
                    // Once `isBusy() == false`, the trajectory follower signals that it is finished. Ensure we use the async follow function
                    // We also need to wait until the pivot and slider are both at target before hanging the specimen
                    if (!robot.isBusy() && robot.pivot.isAtTarget() && robot.slider.isAtTarget()) {
                        robot.armMech.resetHangSpecimenState(false);
                        robot.armMech.hangSpecimenUpdate();
                        currentHumanSpecimenState = HumanSpecimenState.STATE_PUSH_TRAJECTORY_SEQUENCE_FULL;
                        //currentHumanSpecimenState = HumanSpecimenState.STATE_PUSH_SEGMENT1_ORIG;
                    }
                    break;
                case STATE_HANG_SPECIMEN_ON_HIGH_BAR:
                    // Check if the state machine (hang specimen operation) is finished
                    // 'hangSpecimenUpdate() = false' while the state machine is still executing
                    // 'hangSpecimenUpdate() = true' when the hang operation is complete
                    // When finished, begin trajectory to push the first alliance-specific sample
                    if (robot.armMech.hangSpecimenUpdate()) {
                        robot.armMech.resetPickWallSpecimenState();
                        navToPushStartPoint = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                // Enter push sequence
                                //.lineTo(new Vector2d(PRESET_START_X * allianceMultiplier, PRESET_START_Y * -allianceMultiplier),
                                .lineToLinearHeading(new Pose2d(PRESET_START_X * allianceMultiplier, -allianceMultiplier*PRESET_START_Y, -allianceMultiplier*PRESET_START_HEADING), // enter push sequence
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL*RATE_LIMITER_VEL_NAV_TO_PUSH_CHECKPOINT, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_TO_PUSH_CHECKPOINT))
                                .addDisplacementMarker(1.0, () -> robot.pivot.setPosition(Pivot.PIVOT_PICKUP))
                                //.addDisplacementMarker(1.0, () -> robot.wrist.setPosition(Wrist.WRIST_POSITION_PICKUP)
                                //.addDisplacementMarker(() -> robot.wrist.setPosition(Wrist.WRIST_POSITION_PICKUP))
                                .addDisplacementMarker(() -> robot.followTrajectoryAsync(navToPushSegment1))
                                .build();

                        navToPushSegment1 = robot.trajectoryBuilder(navToPushStartPoint.end(), false)
                                // Segment 1
                                .splineToConstantHeading(new Vector2d(allianceMultiplier*PRESET_START_X , -allianceMultiplier*(PRESET_LOW_POINT + PRESET_SPACING/2)), allianceMultiplier*PI/2,
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_REVERSE_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_REVERSE_SAMPLE_PUSH)) // 1.54 - S
                                .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + PRESET_SPACING), -allianceMultiplier*PRESET_LOW_POINT), allianceMultiplier*PI/2,
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_REVERSE_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_REVERSE_SAMPLE_PUSH)) // 1.78 - SW
                                .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + PRESET_SPACING), -allianceMultiplier*PRESET_HIGH_POINT), allianceMultiplier*PI/2,
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_FORWARD_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_FORWARD_SAMPLE_PUSH))// 2.65 - N
                                .build();
                        robot.followTrajectoryAsync(navToPushStartPoint);
                        currentHumanSpecimenState = HumanSpecimenState.STATE_PUSH_SEGMENT2_NEW;
                    }
                    break;
                case STATE_PUSH_SEGMENT1_ORIG:
                    if (!robot.isBusy()) {
                        navToPushSegment2 = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .splineToConstantHeading(new Vector2d(allianceMultiplier * (PRESET_START_X + PRESET_SPACING), -allianceMultiplier * (PRESET_LOW_POINT + PRESET_SPACING / 2)), allianceMultiplier * PI / 2,
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_REVERSE_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_REVERSE_SAMPLE_PUSH)) // 3.33 - S
                                .splineToConstantHeading(new Vector2d(allianceMultiplier * (PRESET_START_X + 2 * PRESET_SPACING), -allianceMultiplier * PRESET_LOW_POINT), allianceMultiplier * PI / 2,
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_REVERSE_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_REVERSE_SAMPLE_PUSH)) // 3.38 - SW
                                .splineToConstantHeading(new Vector2d(allianceMultiplier * (PRESET_START_X + 2 * PRESET_SPACING), -allianceMultiplier * PRESET_HIGH_POINT), allianceMultiplier * PI / 2,
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_FORWARD_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_FORWARD_SAMPLE_PUSH)) // 4.18 - N
                                .build();
                        robot.followTrajectoryAsync(navToPushSegment2);
                        currentHumanSpecimenState = HumanSpecimenState.STATE_PUSH_SEGMENT2_ORIG;
                    }
                    break;
                case STATE_PUSH_SEGMENT2_ORIG:
                    if (!robot.isBusy()) {
                        navToPushSegment3 = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                // Segment 3
                                .splineToConstantHeading(new Vector2d(allianceMultiplier * (PRESET_START_X + 2 * PRESET_SPACING), -allianceMultiplier * (PRESET_LOW_POINT + PRESET_SPACING / 2)), allianceMultiplier * PI / 2,
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_REVERSE_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_REVERSE_SAMPLE_PUSH)) // 3.33 - S
                                .splineToConstantHeading(new Vector2d(allianceMultiplier * (PRESET_START_X + 3 * PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier * PRESET_LOW_POINT), allianceMultiplier * PI / 2,
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_REVERSE_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_REVERSE_SAMPLE_PUSH)) // 3.38 - SW
                                .splineToConstantHeading(new Vector2d(allianceMultiplier * (PRESET_START_X + 3 * PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier * PRESET_HIGH_POINT), allianceMultiplier * PI / 2,
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_FORWARD_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_FORWARD_SAMPLE_PUSH)) // 4.18 - N
                                .addDisplacementMarker(() -> robot.armMech.pickWallSpecimen())
                                .build();
                        robot.followTrajectoryAsync(navToPushSegment3);
                        currentHumanSpecimenState = HumanSpecimenState.STATE_PUSH_SEGMENT_FINAL;
                    }
                    break;

                case STATE_PUSH_SEGMENT1_NEW:
                    if (robot.armMech.hangSpecimenUpdate()) {
                        robot.armMech.resetPickWallSpecimenState();
                        navToPushStartPoint = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                // Enter push sequence
                                //.lineTo(new Vector2d(PRESET_START_X * allianceMultiplier, PRESET_START_Y * -allianceMultiplier),
                                .lineToLinearHeading(new Pose2d(PRESET_START_X * allianceMultiplier, -allianceMultiplier * PRESET_START_Y, -allianceMultiplier * PRESET_START_HEADING), // enter push sequence
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_TO_PUSH_CHECKPOINT, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_TO_PUSH_CHECKPOINT))
                                .addDisplacementMarker(1.0, () -> robot.pivot.setPosition(Pivot.PIVOT_PICKUP))
                                //.addDisplacementMarker(1.0, () -> robot.wrist.setPosition(Wrist.WRIST_POSITION_PICKUP)
                                //.addDisplacementMarker(() -> robot.wrist.setPosition(Wrist.WRIST_POSITION_PICKUP))
                                .addDisplacementMarker(() -> robot.followTrajectoryAsync(navToPushSegment1))
                                .build();

                        navToPushSegment1 = robot.trajectoryBuilder(navToPushStartPoint.end(), false)
                                // Segment 1
                                .lineToSplineHeading(new Pose2d(allianceMultiplier * PRESET_START_X, -allianceMultiplier * (PRESET_LOW_POINT + PRESET_SPACING / 2), -allianceMultiplier * PI / 2),
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_REVERSE_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_REVERSE_SAMPLE_PUSH))
                                .splineToConstantHeading(new Vector2d(allianceMultiplier * (PRESET_START_X + PRESET_SPACING), -allianceMultiplier * PRESET_LOW_POINT), allianceMultiplier * PI / 2,
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_REVERSE_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_REVERSE_SAMPLE_PUSH)) // 1.78 - SW
                                .lineToLinearHeading(new Pose2d(allianceMultiplier * (PRESET_START_X + PRESET_SPACING), -allianceMultiplier * PRESET_HIGH_POINT, -allianceMultiplier * PI / 2),
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_FORWARD_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_FORWARD_SAMPLE_PUSH))// 2.65 - N)
                                .build();
                        robot.followTrajectoryAsync(navToPushStartPoint);
                        currentHumanSpecimenState = HumanSpecimenState.STATE_PUSH_SEGMENT2_NEW;
                    }
                    break;
                case STATE_PUSH_SEGMENT2_NEW:
                    if (!robot.isBusy()) {
                        navToPushSegment2 = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                // Segment 2
                                .lineToSplineHeading(new Pose2d(allianceMultiplier * (PRESET_START_X + PRESET_SPACING), -allianceMultiplier * (PRESET_LOW_POINT + PRESET_SPACING / 2), -allianceMultiplier * PI / 2),
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_REVERSE_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_REVERSE_SAMPLE_PUSH))
                                .splineToConstantHeading(new Vector2d(allianceMultiplier * (PRESET_START_X + 2 * PRESET_SPACING), -allianceMultiplier * PRESET_LOW_POINT), allianceMultiplier * PI / 2,
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_REVERSE_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_REVERSE_SAMPLE_PUSH)) // 3.38 - SW
                                .lineToLinearHeading(new Pose2d(allianceMultiplier * (PRESET_START_X + 2 * PRESET_SPACING), -allianceMultiplier * PRESET_HIGH_POINT, -allianceMultiplier * PI / 2),
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_FORWARD_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_FORWARD_SAMPLE_PUSH))// 2.65 - N))
                                .build();
                        robot.followTrajectoryAsync(navToPushSegment2);
                        currentHumanSpecimenState = HumanSpecimenState.STATE_PUSH_SEGMENT3_NEW;
                    }
                    break;
                case STATE_PUSH_SEGMENT3_NEW:
                    if (!robot.isBusy()) {
                        navToPushSegment3 = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                // Segment 3
                                .lineToSplineHeading(new Pose2d(allianceMultiplier*(PRESET_START_X + 2*PRESET_SPACING), -allianceMultiplier*(PRESET_LOW_POINT + PRESET_SPACING/2), -allianceMultiplier*PI/2),
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_REVERSE_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_REVERSE_SAMPLE_PUSH))
                                .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 3*PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier*PRESET_LOW_POINT), allianceMultiplier*PI/2,
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_REVERSE_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_REVERSE_SAMPLE_PUSH))
                                .lineToLinearHeading(new Pose2d(allianceMultiplier*(PRESET_START_X + 3*PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier*PRESET_HIGH_POINT, -allianceMultiplier*PI/2),
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_FORWARD_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_FORWARD_SAMPLE_PUSH))
                                .addDisplacementMarker(() -> robot.armMech.pickWallSpecimen())
                                .build();
                        robot.followTrajectoryAsync(navToPushSegment3);
                        currentHumanSpecimenState = HumanSpecimenState.STATE_PUSH_SEGMENT_FINAL;
                    }
                    break;
                case STATE_PUSH_TRAJECTORY_SEQUENCE_FULL:
                    if (robot.armMech.hangSpecimenUpdate()) {
                        robot.armMech.resetPickWallSpecimenState();
                        navToPushFullSequence = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(PRESET_START_X * allianceMultiplier, -allianceMultiplier * PRESET_START_Y, -allianceMultiplier * PRESET_START_HEADING), // enter push sequence
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_TO_PUSH_CHECKPOINT, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_TO_PUSH_CHECKPOINT))
                                .addDisplacementMarker(1.0, () -> robot.pivot.setPosition(Pivot.PIVOT_FLOOR_PUSH))
                                .lineToSplineHeading(new Pose2d(allianceMultiplier * PRESET_START_X, -allianceMultiplier * (PRESET_LOW_POINT + PRESET_SPACING / 2), -allianceMultiplier * PI / 2),
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_REVERSE_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_REVERSE_SAMPLE_PUSH))
                                .splineToConstantHeading(new Vector2d(allianceMultiplier * (PRESET_START_X + PRESET_SPACING), -allianceMultiplier * PRESET_LOW_POINT), allianceMultiplier * PI / 2,
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_REVERSE_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_REVERSE_SAMPLE_PUSH)) // 1.78 - SW
                                .addDisplacementMarker(() -> robot.wrist.setPosition(Wrist.WRIST_POSITION_FLOOR_PUSH))
                                .lineToLinearHeading(new Pose2d(allianceMultiplier * (PRESET_START_X + PRESET_SPACING), -allianceMultiplier * PRESET_HIGH_POINT, -allianceMultiplier * PI / 2),
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_FORWARD_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_FORWARD_SAMPLE_PUSH))// 2.65 - N)
                                .addDisplacementMarker(() -> robot.wrist.setPosition(Wrist.WRIST_POSITION_CARRY))
                                .lineToSplineHeading(new Pose2d(allianceMultiplier * (PRESET_START_X + PRESET_SPACING), -allianceMultiplier * (PRESET_LOW_POINT + PRESET_SPACING / 2), -allianceMultiplier * PI / 2),
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_REVERSE_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_REVERSE_SAMPLE_PUSH))
                                .splineToConstantHeading(new Vector2d(allianceMultiplier * (PRESET_START_X + 2 * PRESET_SPACING), -allianceMultiplier * PRESET_LOW_POINT), allianceMultiplier * PI / 2,
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_REVERSE_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_REVERSE_SAMPLE_PUSH)) // 3.38 - SW
                                .addDisplacementMarker(() -> robot.wrist.setPosition(Wrist.WRIST_POSITION_FLOOR_PUSH))
                                .lineToLinearHeading(new Pose2d(allianceMultiplier * (PRESET_START_X + 2 * PRESET_SPACING), -allianceMultiplier * PRESET_HIGH_POINT, -allianceMultiplier * PI / 2),
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_FORWARD_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_FORWARD_SAMPLE_PUSH))// 2.65 - N))
                                .addDisplacementMarker(() -> robot.wrist.setPosition(Wrist.WRIST_POSITION_CARRY))
                                .lineToSplineHeading(new Pose2d(allianceMultiplier*(PRESET_START_X + 2*PRESET_SPACING), -allianceMultiplier*(PRESET_LOW_POINT + PRESET_SPACING/2), -allianceMultiplier*PI/2),
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_REVERSE_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_REVERSE_SAMPLE_PUSH))
                                .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 3*PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier*PRESET_LOW_POINT), allianceMultiplier*PI/2,
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_REVERSE_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_REVERSE_SAMPLE_PUSH))
                                .addDisplacementMarker(() -> robot.wrist.setPosition(Wrist.WRIST_POSITION_FLOOR_PUSH))
                                .lineToLinearHeading(new Pose2d(allianceMultiplier*(PRESET_START_X + 3*PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier*PRESET_HIGH_POINT, -allianceMultiplier*PI/2),
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL * RATE_LIMITER_VEL_NAV_FORWARD_SAMPLE_PUSH, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_FORWARD_SAMPLE_PUSH))
                                .addDisplacementMarker(() -> robot.armMech.pickWallSpecimen())
                                .build();
                        robot.followTrajectorySequenceAsync(navToPushFullSequence);
                        currentHumanSpecimenState = HumanSpecimenState.STATE_PUSH_SEGMENT_FINAL;
                    }
                    break;

                case STATE_PUSH_SEGMENT_FINAL:
                    // Wait for trajectory to finish and pivot to reach pre-pickup position, and then begin pickSample() sequence
                    if (!robot.isBusy() && robot.armMech.pickWallSpecimen()) {
                        // Pickup sample from floor by calling pickSample() method in ArmMech
                        //robot.armMech.resetPickSampleState(); // reset pickSample() to START state
                        //robot.armMech.pickSample();
                        currentHumanSpecimenState = HumanSpecimenState.STATE_IDLE;
                        //currentHumanSpecimenState = HumanSpecimenState.STATE_PICK_1ST_YELLOW_SAMPLE;
                    }
                    break;

                case STATE_IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }
            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            robot.update();
            // We update our pivot and slider PID controllers continuously in the background, regardless of state
            robot.pivot.update();
            robot.slider.update();

            // Read pose
            Pose2d poseEstimate = robot.getPoseEstimate();

            // Continually write pose to `PoseStorage`
            //PoseStorage.currentPose = poseEstimate;

            // Print pose to telemetry
            robot.localTelemetry.addData("x", poseEstimate.getX());
            robot.localTelemetry.addData("y", poseEstimate.getY());
            robot.localTelemetry.addData("heading", poseEstimate.getHeading());
            robot.localTelemetry.addData("Pivot Pos", robot.pivot.getPosition());
            robot.localTelemetry.addData("Slider Pos", robot.slider.getPosition());
            robot.localTelemetry.addData("Slider at Target", robot.slider.isAtTarget());
            robot.localTelemetry.addData("Pivot at Target", robot.pivot.isAtTarget());
            robot.localTelemetry.update();
        }

    }



}
