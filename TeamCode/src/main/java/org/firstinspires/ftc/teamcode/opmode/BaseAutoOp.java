package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RATE_LIMITER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RATE_LIMITER_ACCEL_NAV_TO_SUBMERSIBLE;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RATE_LIMITER_AUTON;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;

import org.firstinspires.ftc.teamcode.auton.AutonMenuSelector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystem.Constants;
import org.firstinspires.ftc.teamcode.subsystem.util.PoseStorage;

import java.util.List;

public abstract class BaseAutoOp extends BaseOp {
    // Basket specimen state machine states
    enum BasketSpecimenState {
        IDLE,
        NAV_TO_SUBMERSIBLE_AND_PREPARE_HANG,
        HANG_SPECIMEN_ON_HIGH_BAR,
        NAV_TO_1ST_YELLOW_SAMPLE,
        PICK_1ST_YELLOW_SAMPLE,
        NAV_FROM_1ST_YELLOW_SAMPLE_TO_BASKET_AND_DROP,
        NAV_TO_2ND_YELLOW_SAMPLE_AND_PICK,
        NAV_FROM_2ND_YELLOW_SAMPLE_TO_BASKET_AND_DROP,
        NAV_TO_3RD_YELLOW_SAMPLE_ALIGNMENT_POINT,
        NAV_TO_3RD_YELLOW_SAMPLE_AND_PICK,
        DROP_3RD_YELLOW_TO_BASKET_AND_NAV_TO_SUBMERSIBLE_PARK
    }
    BasketSpecimenState currentBasketSpecimenState;

    enum HumanSpecimenState {
        IDLE,
        NAV_TO_SUBMERSIBLE_AND_PREPARE_HANG,
        HANG_SPECIMEN_ON_HIGH_BAR,
        PICK_1ST_ALLIANCE_SAMPLE,
        NAV_TO_1ST_ZONE_DROP_AND_RELEASE,
        PICK_2ND_ALLIANCE_SAMPLE,
        NAV_TO_2ND_ZONE_DROP_AND_RELEASE,
        NAV_TO_3RD_ALLIANCE_SAMPLE_PICK,
        NAV_TO_WALL_ALIGN_AND_DROP_SAMPLE,
        PREPARE_PICK_WALL_SPECIMEN,
        NAV_TO_WALL_PICK,
        PREPARE_HANG_2,
        HANG_2ND_SPECIMEN_ON_HIGH_BAR,
        PREPARE_PICK_WALL_SPECIMEN_2,
        NAV_TO_WALL_PICK_2,
        PREPARE_HANG_3,
        HANG_3RD_SPECIMEN_ON_HIGH_BAR,
    }
    HumanSpecimenState currentHumanSpecimenState;


    //------------------------------------------------------------------------------
    // Start pose
    //------------------------------------------------------------------------------
    protected Pose2d startPose;


    //------------------------------------------------------------------------------
    // Common Auton Trajectories and Poses (shared with all Auton sequences)
    //------------------------------------------------------------------------------

    // Sample pick poses
    protected Pose2d poseFirstSamplePick, poseSecondSamplePick, poseThirdSampleAlign, poseThirdSamplePick;

    // Specimen Hang trajectory and pose
    protected Trajectory navToSpecimenHang;
    protected Pose2d poseInitialSpecimenHang;


    //------------------------------------------------------------------------------
    // Basket Side Auton Specific Trajectories and Poses
    //------------------------------------------------------------------------------

    // Neutral (yellow) Sample pick trajectories
    protected Trajectory navToFirstNeutral, navToSecondNeutral, navToThirdNeutralAlign, navToThirdNeutralPick;

    // Basket trajectories and pose
    protected Trajectory navToBasket1, navToBasket2, navToBasket3;
    protected Pose2d poseBasketDrop;

    // Submersible Park trajectories and pose
    protected Trajectory navToSubmersibleSegment1, navToSubmersibleSegment2;
    protected Pose2d poseSubmersiblePark, poseSubmersibleAlign;


    //------------------------------------------------------------------------------
    // Human Side Auton Specific Trajectories and Poses
    //------------------------------------------------------------------------------

    // Specimen hang trajectories and poses (initial Basket side Auton pose is used)
    protected Trajectory navTo1stWallAlignCheckPoint, navTo1stSpecimenHang, navTo2ndWallAlignCheckPoint, navTo2ndSpecimenHang;
    protected Pose2d poseSecondSpecimenHang, poseThirdSpecimenHang;

    // Alliance (red/blue) Sample pick trajectories (Basket side Auton poses are used)
    protected Trajectory navToFirstSample, navToSecondSample, navToThirdSampleAlign, navToThirdSamplePick;

    // Wall pick trajectories and poses
    protected Trajectory navToWallAlign, navToWallPick, navToWallPick2Align, navToWallPick2;
    protected Pose2d poseWallPick, poseWallAlign, poseWallAlignCheckpoint;

    // Observation Zone drop trajectories and poses
    protected Trajectory navToZoneDrop, navToZoneDrop2;
    protected Pose2d poseZoneDrop;

    //------------------------------------------------------------------------------

    // Variable to hold a 1 or -1 as a multiplier that reflects which Alliance color is being played
    int allianceMultiplier;

    //AutonMenuSelector autonMenuSelector;

    protected BaseAutoOp(StartingPosition position) {
        //mStartingPosition = position;
        //msGlobalStartingPosition = position;
        //allianceMultiplier = getAlliance(mStartingPosition) == AllianceSelection.BLUE ? -1 : 1;

        //initCheckpoints();

        RATE_LIMITER = RATE_LIMITER_AUTON;
    }

    protected void initCheckpoints() {
        switch (mStartingPosition) {
            case BLUE_BASKET:
                startPose = new Pose2d(Constants.AUTON_START_X, Constants.AUTON_START_Y, Constants.AUTON_START_HEADING);
                poseInitialSpecimenHang = new Pose2d(Constants.INITIAL_SPECIMEN_HANG_X, Constants.INITIAL_SPECIMEN_HANG_Y, Constants.INITIAL_SPECIMEN_HANG_HEADING);
                poseFirstSamplePick = new Pose2d(Constants.NEUTRAL_SAMPLE_1_PICK_X, Constants.NEUTRAL_SAMPLE_1_PICK_Y, Constants.NEUTRAL_SAMPLE_1_PICK_HEADING);
                poseSecondSamplePick = new Pose2d(Constants.NEUTRAL_SAMPLE_2_PICK_X, Constants.NEUTRAL_SAMPLE_2_PICK_Y, Constants.NEUTRAL_SAMPLE_2_PICK_HEADING);
                poseThirdSampleAlign = new Pose2d(Constants.NEUTRAL_SAMPLE_3_ALIGN_X, Constants.NEUTRAL_SAMPLE_3_ALIGN_Y, Constants.NEUTRAL_SAMPLE_3_ALIGN_HEADING);
                poseThirdSamplePick = new Pose2d(Constants.NEUTRAL_SAMPLE_3_PICK_X, poseThirdSampleAlign.getY(), poseThirdSampleAlign.getHeading());
                poseBasketDrop = new Pose2d(Constants.BASKET_DROP_X, Constants.BASKET_DROP_Y, Constants.BASKET_DROP_HEADING_AUTON);
                poseSubmersibleAlign = new Pose2d(Constants.SUBMERSIBLE_ALIGN_X, Constants.SUBMERSIBLE_ALIGN_Y, Constants.SUBMERSIBLE_ALIGN_HEADING);
                poseSubmersiblePark = new Pose2d(Constants.SUBMERSIBLE_PARK_X, Constants.SUBMERSIBLE_PARK_Y, poseSubmersibleAlign.getHeading());
                break;

            case RED_BASKET:
                startPose = new Pose2d(-Constants.AUTON_START_X, -Constants.AUTON_START_Y, -Constants.AUTON_START_HEADING);
                poseInitialSpecimenHang = new Pose2d(-Constants.INITIAL_SPECIMEN_HANG_X, -Constants.INITIAL_SPECIMEN_HANG_Y, -Constants.INITIAL_SPECIMEN_HANG_HEADING);
                poseFirstSamplePick = new Pose2d(-Constants.NEUTRAL_SAMPLE_1_PICK_X, -Constants.NEUTRAL_SAMPLE_1_PICK_Y, -Constants.NEUTRAL_SAMPLE_1_PICK_HEADING);
                poseSecondSamplePick = new Pose2d(-Constants.NEUTRAL_SAMPLE_2_PICK_X, -Constants.NEUTRAL_SAMPLE_2_PICK_Y, -Constants.NEUTRAL_SAMPLE_2_PICK_HEADING);
                poseThirdSampleAlign = new Pose2d(-Constants.NEUTRAL_SAMPLE_3_ALIGN_X, -Constants.NEUTRAL_SAMPLE_3_ALIGN_Y, Constants.NEUTRAL_SAMPLE_3_ALIGN_HEADING + Math.toRadians(180));
                poseThirdSamplePick = new Pose2d(-Constants.NEUTRAL_SAMPLE_3_PICK_X, poseThirdSampleAlign.getY(), poseThirdSampleAlign.getHeading());
                poseBasketDrop = new Pose2d(-Constants.BASKET_DROP_X, -Constants.BASKET_DROP_Y, Constants.BASKET_DROP_HEADING_AUTON - Math.toRadians(180));
                poseSubmersibleAlign = new Pose2d(-Constants.SUBMERSIBLE_ALIGN_X, -Constants.SUBMERSIBLE_ALIGN_Y, Constants.SUBMERSIBLE_ALIGN_HEADING - Math.toRadians(180));
                poseSubmersiblePark = new Pose2d(-Constants.SUBMERSIBLE_PARK_X, -Constants.SUBMERSIBLE_PARK_Y, poseSubmersibleAlign.getHeading());
                break;

            case BLUE_HUMAN:
                startPose = new Pose2d(-Constants.AUTON_START_X, Constants.AUTON_START_Y, Constants.AUTON_START_HEADING);
                poseInitialSpecimenHang = new Pose2d(-Constants.INITIAL_SPECIMEN_HANG_X, Constants.INITIAL_SPECIMEN_HANG_Y, Constants.INITIAL_SPECIMEN_HANG_HEADING);
                poseSecondSpecimenHang = new Pose2d(poseInitialSpecimenHang.getX() + Constants.SPECIMEN_HANG_OFFSET, poseInitialSpecimenHang.getY(), poseInitialSpecimenHang.getHeading());
                poseThirdSpecimenHang = new Pose2d(poseInitialSpecimenHang.getX() + (2 * Constants.SPECIMEN_HANG_OFFSET), poseInitialSpecimenHang.getY(), poseInitialSpecimenHang.getHeading());
                poseFirstSamplePick = new Pose2d(-Constants.NEUTRAL_SAMPLE_1_PICK_X, Constants.NEUTRAL_SAMPLE_1_PICK_Y, Constants.NEUTRAL_SAMPLE_1_PICK_HEADING);
                poseSecondSamplePick = new Pose2d(-Constants.NEUTRAL_SAMPLE_2_PICK_X + (-Constants.ALLIANCE_SAMPLE_2_OFFSET_X), Constants.NEUTRAL_SAMPLE_2_PICK_Y, Constants.NEUTRAL_SAMPLE_2_PICK_HEADING);
                poseThirdSampleAlign = new Pose2d(-Constants.NEUTRAL_SAMPLE_3_ALIGN_X, Constants.NEUTRAL_SAMPLE_3_ALIGN_Y + (-Constants.ALLIANCE_SAMPLE_3_OFFSET_Y), Constants.NEUTRAL_SAMPLE_3_ALIGN_HEADING + Math.toRadians(180));
                poseThirdSamplePick = new Pose2d(-Constants.NEUTRAL_SAMPLE_3_PICK_X, poseThirdSampleAlign.getY(), poseThirdSampleAlign.getHeading());
                poseZoneDrop = new Pose2d(-Constants.ZONE_DROP_X, Constants.ZONE_DROP_Y, Constants.ZONE_DROP_HEADING);
                poseWallAlign = new Pose2d(-Constants.WALL_ALIGN_X, Constants.WALL_ALIGN_Y, Constants.WALL_ALIGN_HEADING);
                poseWallPick = new Pose2d(-Constants.WALL_ALIGN_X, Constants.WALL_PICK_Y, poseWallAlign.getHeading());
                poseWallAlignCheckpoint = new Pose2d(poseWallAlign.getX(), poseWallAlign.getY() - Constants.WALl_ALIGN_Y_OFFSET, Constants.WALl_ALIGN_HEADING_CHECKPOINT);
                break;

            case RED_HUMAN:
                startPose = new Pose2d(Constants.AUTON_START_X, -Constants.AUTON_START_Y, -Constants.AUTON_START_HEADING);
                poseInitialSpecimenHang = new Pose2d(Constants.INITIAL_SPECIMEN_HANG_X, -Constants.INITIAL_SPECIMEN_HANG_Y, -Constants.INITIAL_SPECIMEN_HANG_HEADING);
                poseSecondSpecimenHang = new Pose2d(poseInitialSpecimenHang.getX() - Constants.SPECIMEN_HANG_OFFSET, poseInitialSpecimenHang.getY(), poseInitialSpecimenHang.getHeading());
                poseThirdSpecimenHang = new Pose2d(poseInitialSpecimenHang.getX() - (2 * Constants.SPECIMEN_HANG_OFFSET), poseInitialSpecimenHang.getY(), poseInitialSpecimenHang.getHeading());
                poseFirstSamplePick = new Pose2d(Constants.NEUTRAL_SAMPLE_1_PICK_X, -Constants.NEUTRAL_SAMPLE_1_PICK_Y, -Constants.NEUTRAL_SAMPLE_1_PICK_HEADING);
                poseSecondSamplePick = new Pose2d(Constants.NEUTRAL_SAMPLE_2_PICK_X + (Constants.ALLIANCE_SAMPLE_2_OFFSET_X), -Constants.NEUTRAL_SAMPLE_2_PICK_Y, -Constants.NEUTRAL_SAMPLE_2_PICK_HEADING);
                poseThirdSampleAlign = new Pose2d(Constants.NEUTRAL_SAMPLE_3_ALIGN_X, -Constants.NEUTRAL_SAMPLE_3_ALIGN_Y + (Constants.ALLIANCE_SAMPLE_3_OFFSET_Y), Constants.NEUTRAL_SAMPLE_3_ALIGN_HEADING);
                poseThirdSamplePick = new Pose2d(Constants.NEUTRAL_SAMPLE_3_PICK_X, poseThirdSampleAlign.getY(), poseThirdSampleAlign.getHeading());
                poseZoneDrop = new Pose2d(Constants.ZONE_DROP_X, -Constants.ZONE_DROP_Y, -Constants.ZONE_DROP_HEADING);
                poseWallAlign = new Pose2d(Constants.WALL_ALIGN_X, -Constants.WALL_ALIGN_Y, -Constants.WALL_ALIGN_HEADING);
                poseWallPick = new Pose2d(Constants.WALL_ALIGN_X, -Constants.WALL_PICK_Y, poseWallAlign.getHeading());
                poseWallAlignCheckpoint = new Pose2d(poseWallAlign.getX(), poseWallAlign.getY() + Constants.WALl_ALIGN_Y_OFFSET, Constants.WALl_ALIGN_HEADING_CHECKPOINT + Math.toRadians(180));
                break;
        }
    }

    protected void autonInitialize() {
        super.initialize();

        // Set LED to indicate init starting.
        robot.ledController.setLeftLED(Constants.LED_BLINKIN_PATTERN_ROBOT_INIT_START, Constants.LED_INIT_START_TIMER);
        robot.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_ROBOT_INIT_START, Constants.LED_INIT_START_TIMER);

        // Enable bulk caching mode to read all encoder and analog sensors simultaneously in one cycle (I2C takes longer).
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Move wrist and claw to starting positions with a sleep between,
        // to allow time for the wrist to reach target position prior to closing the claw.
        robot.wrist.setPosition(Constants.WRIST_START_POSITION);
        sleep(1500);
        robot.claw.setPosition(Constants.CLAW_CLOSE);

        // Home the pivot.
        robot.pivot.homePivot();

        // Set a default value for mStartingPosition so it's not null
        mStartingPosition = StartingPosition.BLUE_BASKET;

        // Use AutonMenuSelector to select the starting position.
        mStartingPosition = AutonMenuSelector.selectStartingPosition(gamepad1, telemetry, mStartingPosition);

        allianceMultiplier = getAlliance(mStartingPosition) == AllianceSelection.BLUE ? -1 : 1;

        initCheckpoints();

        // Set robot starting pose.
        robot.setPoseEstimate(startPose);

        // Print final telemetry values.
        robot.localTelemetry.clear();
        robot.localTelemetry.addData("ArmPivot Pos", robot.pivot.getPosition());
        robot.localTelemetry.addData("Home Switch", robot.pivot.isPivotHomeSwitchPressed());
        robot.localTelemetry.addLine("Arm pivot reset. Init complete.");
        robot.localTelemetry.addData("Start pose", mStartingPosition.toString());
        robot.localTelemetry.addLine("Press start to begin");
        robot.localTelemetry.update();

        // Set LED to indicate init finished.
        robot.ledController.setLeftLED(Constants.LED_BLINKIN_PATTERN_ROBOT_INIT_END, Constants.LED_INIT_END_TIMER);
        robot.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_ROBOT_INIT_END, Constants.LED_INIT_END_TIMER);
    }

    protected void runBasketSpecimenOpMode() {

        // Reset state machine states for upcoming actions
        robot.armMech.resetPrepareHangSpecimenState();
        robot.armMech.resetHangSpecimenState();
        // Load initial trajectory to submersible hang (to avoid small load time if loaded after start button is pressed)
        navToSpecimenHang = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineTo(new Vector2d(poseInitialSpecimenHang.getX(), poseInitialSpecimenHang.getY()),
                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_TO_SUBMERSIBLE))
                .addDisplacementMarker(0, () -> robot.armMech.prepareHangSpecimen())
                .build();

        waitForStart();

        // Set the current state to STATE_NAV_TO_SUBMERSIBLE_AND_PREPARE_HANG, our first step, then have it follow that trajectory.
        // Make sure you use the async version of the trajectory commands, otherwise it will be blocking.
        currentBasketSpecimenState = BasketSpecimenState.NAV_TO_SUBMERSIBLE_AND_PREPARE_HANG;
        robot.followTrajectoryAsync(navToSpecimenHang);

        // Begin auton while loop; this is essentially a large switch statement, which functions as our master state machine
        while (opModeIsActive() && !isStopRequested()) {

            // Our state machine logic
            // We define the flow of the state machine through this switch statement
            switch (currentBasketSpecimenState) {
                case NAV_TO_SUBMERSIBLE_AND_PREPARE_HANG:
                    if (robot.armMech.prepareHangSpecimen()) {
                        robot.armMech.hangSpecimenUpdate();
                        currentBasketSpecimenState = BasketSpecimenState.HANG_SPECIMEN_ON_HIGH_BAR;
                    }
                    break;
                case HANG_SPECIMEN_ON_HIGH_BAR:
                    // Check if the state machine (hang specimen operation) is finished
                    // 'hangSpecimenUpdate() = false' while the state machine is still executing
                    // 'hangSpecimenUpdate() = true' when the hang operation is complete
                    // When finished, begin trajectory to pick the first neutral sample
                    if (!robot.isBusy() && robot.armMech.hangSpecimenUpdate()) {
                        // Set trajectory based on where robot stopped
                        navToFirstNeutral = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineTo(new Vector2d(poseFirstSamplePick.getX(), poseFirstSamplePick.getY()))
                                .addDisplacementMarker(4.0, () -> robot.pivot.setPosition(Constants.PIVOT_INTERMEDIATE))
                                .build();
                        robot.followTrajectoryAsync(navToFirstNeutral);
                        currentBasketSpecimenState = BasketSpecimenState.NAV_TO_1ST_YELLOW_SAMPLE;
                    }
                    break;
                case NAV_TO_1ST_YELLOW_SAMPLE:
                    // Wait for trajectory to finish and pivot to reach pre-pickup position, and then begin pickSample() sequence
                    if (!robot.isBusy() && robot.pivot.isAtTarget()) {
                        // Reset state machine, and then pick sample from floor by calling pickSample() method in ArmMech()
                        // Set auton=true for reset so that we bypass ready check because we assume robot has set pivot and is already prepared
                        robot.armMech.resetPickSampleState(true);
                        robot.armMech.pickSample();
                        currentBasketSpecimenState = BasketSpecimenState.PICK_1ST_YELLOW_SAMPLE;
                    }
                    break;
                case PICK_1ST_YELLOW_SAMPLE:
                    // When sample pick is complete, begin trajectory to basket drop and start basket drop shortly
                    // into trajectory, with the assumption that the basket drop sequence will take much longer to
                    // complete than the trajectory.
                    if (robot.armMech.pickSample()) {
                        robot.armMech.resetDropSampleHighBasketState(true); // reset dropSampleHighBasket to START state
                        navToBasket1 = robot.trajectoryBuilder(robot.getPoseEstimate(), true)
                                .splineTo(new Vector2d(poseBasketDrop.getX(), poseBasketDrop.getY()), poseBasketDrop.getHeading())
                                .addDisplacementMarker(0.2, () -> robot.armMech.dropSampleHighBasket())
                                .build();
                        robot.followTrajectoryAsync(navToBasket1);
                        currentBasketSpecimenState = BasketSpecimenState.NAV_FROM_1ST_YELLOW_SAMPLE_TO_BASKET_AND_DROP;
                    }
                    break;
                case NAV_FROM_1ST_YELLOW_SAMPLE_TO_BASKET_AND_DROP:
                    // Wait for the basket drop to finish (also the trajectory to finish, but that is expected
                    // to finish long before the sample drop is finished).
                    // When finished, start trajectory to pick 2nd yellow sample; pickSample() must be timed using displacement marker
                    // such that the wrist does not contact and push the sample; wrist should come down over the top of sample as trajectory ends.
                    if (!robot.isBusy() && robot.armMech.dropSampleHighBasket()) {
                        robot.armMech.resetPickSampleState(true); // reset pickSample() to START state
                        navToSecondNeutral = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineToLinearHeading(new Pose2d(poseSecondSamplePick.getX(), poseSecondSamplePick.getY(), poseSecondSamplePick.getHeading()))
                                .addDisplacementMarker(5.0, () -> robot.armMech.pickSample())
                                .build();
                        robot.followTrajectoryAsync(navToSecondNeutral);
                        currentBasketSpecimenState = BasketSpecimenState.NAV_TO_2ND_YELLOW_SAMPLE_AND_PICK;
                    }
                    break;
                case NAV_TO_2ND_YELLOW_SAMPLE_AND_PICK:
                    // Wait for trajectory and yellow sample pick to finish; when finished, begin trajectory
                    // to basket drop and drop sample in high basket.
                    if (!robot.isBusy() && robot.armMech.pickSample()) {
                        robot.armMech.resetDropSampleHighBasketState(true); // reset dropSampleHighBasket to START state
                        navToBasket2 = robot.trajectoryBuilder(robot.getPoseEstimate(), true)
                                .lineToLinearHeading(new Pose2d(poseBasketDrop.getX(), poseBasketDrop.getY(), poseBasketDrop.getHeading() + Math.toRadians(180)))
                                .addDisplacementMarker(0.2, () -> robot.armMech.dropSampleHighBasket())
                                .build();
                        robot.followTrajectoryAsync(navToBasket2);
                        currentBasketSpecimenState = BasketSpecimenState.NAV_FROM_2ND_YELLOW_SAMPLE_TO_BASKET_AND_DROP;
                    }
                    break;
                case NAV_FROM_2ND_YELLOW_SAMPLE_TO_BASKET_AND_DROP:
                    // Wait for the basket drop to finish (also the trajectory to finish, but that is expected
                    // to finish long before the sample drop is finished).
                    // When finished, start trajectory to align to 3rd yellow sample; and move pivot to prep position (above 3rd sample)
                    if (!robot.isBusy() && robot.armMech.dropSampleHighBasket()) {
                        navToThirdNeutralAlign = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineToLinearHeading(new Pose2d(poseThirdSampleAlign.getX(), poseThirdSampleAlign.getY(), poseThirdSampleAlign.getHeading()))
                                .addDisplacementMarker(5.0, () -> robot.pivot.setPosition(Constants.PIVOT_PICKUP_3RD_YELLOW))
                                .build();
                        robot.followTrajectoryAsync(navToThirdNeutralAlign);
                        currentBasketSpecimenState = BasketSpecimenState.NAV_TO_3RD_YELLOW_SAMPLE_ALIGNMENT_POINT;
                    }
                    break;
                case NAV_TO_3RD_YELLOW_SAMPLE_ALIGNMENT_POINT:
                    // Wait for trajectory to 3rd yellow sample alignment point to finish and for
                    // pivot to be at target (high enough to clear sample), and then begin movement to
                    // 3rd yellow sample pick point and pick sample
                    if (!robot.isBusy() && robot.pivot.isAtTarget()) {
                        robot.armMech.resetPickSampleState(true); // reset pickSample() to START state
                        navToThirdNeutralPick = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineTo(new Vector2d(poseThirdSamplePick.getX(), poseThirdSamplePick.getY()))
                                .addDisplacementMarker(3.5, () -> robot.armMech.pickSample())
                                .build();
                        robot.followTrajectoryAsync(navToThirdNeutralPick);
                        currentBasketSpecimenState = BasketSpecimenState.NAV_TO_3RD_YELLOW_SAMPLE_AND_PICK;
                    }
                    break;
                case NAV_TO_3RD_YELLOW_SAMPLE_AND_PICK:
                    // Wait for trajectory and 3rd yellow sample pick to finish; when finished, begin trajectory
                    // to basket drop and drop sample in high basket.
                    if (!robot.isBusy() && robot.armMech.pickSample()) {
                        robot.armMech.resetDropSampleHighBasketState(true); // reset dropSampleHighBasket to START state
                        navToBasket3 = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineToLinearHeading(new Pose2d(poseBasketDrop.getX(), poseBasketDrop.getY(), poseBasketDrop.getHeading() + Math.toRadians(180)))
                                .addDisplacementMarker(0.2, () -> robot.armMech.dropSampleHighBasket())
                                .build();
                        robot.followTrajectoryAsync(navToBasket3);
                        currentBasketSpecimenState = BasketSpecimenState.DROP_3RD_YELLOW_TO_BASKET_AND_NAV_TO_SUBMERSIBLE_PARK;
                    }
                    break;
                case DROP_3RD_YELLOW_TO_BASKET_AND_NAV_TO_SUBMERSIBLE_PARK:
                    // When basket drop is finished, begin submersible parking trajectory, and move the pivot, wrist, and slider
                    // mechanisms to prepare to rest on the high rung
                    // Note: the submersible parking sequence is two trajectories chained together
                    if (!robot.isBusy() && robot.armMech.dropSampleHighBasket()) {
                        navToSubmersibleSegment1 = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineToLinearHeading(new Pose2d(poseSubmersibleAlign.getX(), poseSubmersibleAlign.getY(), poseSubmersibleAlign.getHeading()))
                                .addDisplacementMarker(6.0, () -> robot.pivot.setPosition(Constants.PIVOT_SUBMERSIBLE_PARK))
                                .addDisplacementMarker(10.0, () -> robot.wrist.setPosition(Constants.WRIST_POSITION_HIGH_BASKET_DROP))
                                .addDisplacementMarker(18.0, () -> robot.slider.setPosition(Constants.SLIDER_HANG_SPECIMEN_HIGH, false))
                                .addDisplacementMarker(() -> robot.followTrajectoryAsync(navToSubmersibleSegment2))
                                .build();
                        navToSubmersibleSegment2 = robot.trajectoryBuilder(navToSubmersibleSegment1.end(), false)
                                .lineTo(new Vector2d(poseSubmersiblePark.getX(), poseSubmersiblePark.getY()))
                                .build();
                        robot.followTrajectoryAsync(navToSubmersibleSegment1);
                        currentBasketSpecimenState = BasketSpecimenState.IDLE;
                        break;
                    }
                case IDLE:
                    // Do nothing in IDLE
                    // currentState does not change once in IDLE
                    // This concludes the autonomous program
                    break;
            }
            // Anything outside of the switch statement will run independent of the currentState.

            // We update drive continuously in the background, regardless of state.
            robot.update();

            // We update our pivot and slider PID controllers continuously in the background, regardless of state.
            robot.pivot.update();
            robot.slider.update();

            // Continually write pose to `PoseStorage` for later retrieval in TeleOp.
            PoseStorage.currentPose = robot.getPoseEstimate();

            // Print pose to telemetry.
            robot.localTelemetry.addData("Pivot Pos", robot.pivot.getPosition());
            robot.localTelemetry.addData("Slider Pos", robot.slider.getPosition());
            robot.localTelemetry.addData("Slider at Target", robot.slider.isAtTarget());
            robot.localTelemetry.addData("Pivot at Target", robot.pivot.isAtTarget());
            robot.localTelemetry.update();
        }

    }

    protected void runHumanSpecimenOpMode() {
        // Reset state machine states for upcoming actions
        robot.armMech.resetPrepareHangSpecimenState();
        robot.armMech.resetHangSpecimenState();
        // Load initial trajectory to submersible hang (to avoid small load time if loaded after start button is pressed)
        navToSpecimenHang = robot.trajectoryBuilder(robot.getPoseEstimate())
                .lineTo(new Vector2d(poseInitialSpecimenHang.getX(), poseInitialSpecimenHang.getY()),
                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_TO_SUBMERSIBLE))
                .addDisplacementMarker(0, () -> robot.armMech.prepareHangSpecimen())
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // Set the current state to STATE_NAV_TO_SUBMERSIBLE_AND_PREPARE_HANG, our first step, then have it follow that trajectory.
        // Make sure you use the async version of the trajectory commands, otherwise it will be blocking.
        currentHumanSpecimenState = HumanSpecimenState.NAV_TO_SUBMERSIBLE_AND_PREPARE_HANG;
        robot.followTrajectoryAsync(navToSpecimenHang);

        // Begin auton while loop; this is essentially a large switch statement, which functions as our master state machine
        while (opModeIsActive() && !isStopRequested()) {

            // Our state machine logic
            // We define the flow of the state machine through this switch statement
            switch (currentHumanSpecimenState) {
                case NAV_TO_SUBMERSIBLE_AND_PREPARE_HANG:
                    if (robot.armMech.prepareHangSpecimen()) {
                        robot.armMech.hangSpecimenUpdate();
                        currentHumanSpecimenState = HumanSpecimenState.HANG_SPECIMEN_ON_HIGH_BAR;
                    }
                    break;
                case HANG_SPECIMEN_ON_HIGH_BAR:
                    // Check if the state machine (hang specimen operation) is finished
                    // 'hangSpecimenUpdate() = false' while the state machine is still executing
                    // 'hangSpecimenUpdate() = true' when the hang operation is complete
                    // When finished, begin trajectory to push the first alliance-specific sample
                    if (!robot.isBusy() && robot.armMech.hangSpecimenUpdate()) {
                        robot.armMech.resetPickSampleState(true);
                        navToFirstSample = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineTo(new Vector2d(poseFirstSamplePick.getX(), poseFirstSamplePick.getY()))
                                //.addDisplacementMarker(4.0, () -> robot.pivot.setPosition(Pivot.PIVOT_INTERMEDIATE))
                                .addDisplacementMarker(2.0, () -> robot.armMech.pickSample())
                                .build();
                        robot.followTrajectoryAsync(navToFirstSample);
                        currentHumanSpecimenState = HumanSpecimenState.PICK_1ST_ALLIANCE_SAMPLE;
                    }
                    break;
                case PICK_1ST_ALLIANCE_SAMPLE:
                    // When sample pick is complete, begin trajectory to basket drop and start basket drop shortly
                    // into trajectory, with the assumption that the basket drop sequence will take much longer to
                    // complete than the trajectory.
                    if (!robot.isBusy() && robot.armMech.pickSample()) {
                        robot.armMech.resetDropSampleZone();
                        navToZoneDrop = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineToLinearHeading(new Pose2d(poseZoneDrop.getX(), poseZoneDrop.getY(), poseZoneDrop.getHeading()))
                                .addDisplacementMarker(() -> robot.armMech.dropSampleZone())
                                .build();
                        robot.followTrajectoryAsync(navToZoneDrop);
                        currentHumanSpecimenState = HumanSpecimenState.NAV_TO_1ST_ZONE_DROP_AND_RELEASE;
                    }
                    break;
                case NAV_TO_1ST_ZONE_DROP_AND_RELEASE:
                    if (!robot.isBusy() && robot.armMech.dropSampleZone()) {
                        robot.armMech.resetPickSampleState(true);
                        navToSecondSample = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineToLinearHeading(new Pose2d(poseSecondSamplePick.getX() - 1.0, poseSecondSamplePick.getY(), poseSecondSamplePick.getHeading()))
                                //.addDisplacementMarker(2.0, () -> robot.pivot.setPosition(Pivot.PIVOT_INTERMEDIATE))
                                .addDisplacementMarker(3.0, () -> robot.wrist.setPosition(Constants.WRIST_POSITION_PICKUP))
                                .addDisplacementMarker(() -> robot.armMech.pickSample())
                                .build();
                        robot.followTrajectoryAsync(navToSecondSample);
                        currentHumanSpecimenState = HumanSpecimenState.PICK_2ND_ALLIANCE_SAMPLE;
                    }
                    break;
                case PICK_2ND_ALLIANCE_SAMPLE:
                    // When sample pick is complete, begin trajectory to basket drop and start basket drop shortly
                    // into trajectory, with the assumption that the basket drop sequence will take much longer to
                    // complete than the trajectory.
                    if (!robot.isBusy() && robot.armMech.pickSample()) {
                        robot.armMech.resetDropSampleZone();
                        navToZoneDrop2 = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineToLinearHeading(new Pose2d(poseZoneDrop.getX(), poseZoneDrop.getY(), poseZoneDrop.getHeading()))
                                .addDisplacementMarker(() -> robot.armMech.dropSampleZone())
                                .build();
                        robot.followTrajectoryAsync(navToZoneDrop2);
                        currentHumanSpecimenState = HumanSpecimenState.NAV_TO_2ND_ZONE_DROP_AND_RELEASE;
                    }
                    break;
                case NAV_TO_2ND_ZONE_DROP_AND_RELEASE:
                    if (!robot.isBusy() && robot.armMech.dropSampleZone()) {
                        robot.armMech.resetPickSampleState(true);
                        navToThirdSampleAlign = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineToLinearHeading(new Pose2d(poseThirdSampleAlign.getX(), poseThirdSampleAlign.getY(), poseThirdSampleAlign.getHeading()))
                                .addDisplacementMarker(1.0, () -> robot.pivot.setPosition(Constants.PIVOT_PICKUP_3RD_YELLOW))
                                .addDisplacementMarker(6.0, () -> robot.wrist.setPosition(Constants.WRIST_POSITION_PICKUP))
                                .addDisplacementMarker(() -> robot.followTrajectoryAsync(navToThirdSamplePick))
                                .build();
                        navToThirdSamplePick = robot.trajectoryBuilder(navToThirdSampleAlign.end(), false)
                                .lineTo(new Vector2d(poseThirdSamplePick.getX(), poseThirdSamplePick.getY()))
                                .addDisplacementMarker(() -> robot.armMech.pickSample())
                                .build();
                        robot.followTrajectoryAsync(navToThirdSampleAlign);
                        currentHumanSpecimenState = HumanSpecimenState.NAV_TO_3RD_ALLIANCE_SAMPLE_PICK;
                    }
                    break;
                case NAV_TO_3RD_ALLIANCE_SAMPLE_PICK:
                    if (!robot.isBusy() && robot.armMech.pickSample()) {
                        robot.armMech.resetDropSampleZone(); // reset dropSampleHighBasket to START state
                        navToWallAlign = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineToLinearHeading(new Pose2d(poseWallAlign.getX(), poseWallAlign.getY(), poseWallAlign.getHeading()))
                                .addDisplacementMarker(() -> robot.armMech.dropSampleZone())
                                .build();
                        robot.followTrajectoryAsync(navToWallAlign);
                        currentHumanSpecimenState = HumanSpecimenState.NAV_TO_WALL_ALIGN_AND_DROP_SAMPLE;
                    }
                    break;
                case NAV_TO_WALL_ALIGN_AND_DROP_SAMPLE:
                    if (!robot.isBusy() && robot.armMech.dropSampleZone()) {
                        //robot.pivot.setPosition(Pivot.PIVOT_PICKUP_WALL_SPECIMEN);
                        robot.armMech.resetPreparePickWallSpecimenState();
                        robot.armMech.resetPickWallSpecimenState();
                        navToWallPick = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineTo(new Vector2d(poseWallPick.getX(), poseWallPick.getY()))
                                .addDisplacementMarker(0.1, () -> robot.armMech.preparePickWallSpecimen())
                                //.addDisplacementMarker(() -> robot.armMech.pickWallSpecimen())
                                .build();
                        robot.followTrajectoryAsync(navToWallPick);
                        currentHumanSpecimenState = HumanSpecimenState.PREPARE_PICK_WALL_SPECIMEN;
                    }
                    break;
                case PREPARE_PICK_WALL_SPECIMEN:
                    if (!robot.isBusy() && robot.armMech.preparePickWallSpecimen()) {
                        robot.armMech.pickWallSpecimen();
                        currentHumanSpecimenState = HumanSpecimenState.NAV_TO_WALL_PICK;
                    }
                    break;
                case NAV_TO_WALL_PICK:
                    if (robot.armMech.pickWallSpecimen()) {
                        robot.armMech.resetPrepareHangSpecimenState();
                        navTo1stWallAlignCheckPoint = robot.trajectoryBuilder(robot.getPoseEstimate())
                                //.lineToSplineHeading(new Pose2d(poseWallPick.getX(), poseWallPick.getY() - 6, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(poseWallAlignCheckpoint.getX(), poseWallAlignCheckpoint.getY(), poseWallAlignCheckpoint.getHeading()))
                                .addDisplacementMarker(() -> robot.followTrajectoryAsync(navTo1stSpecimenHang))
                                .build();
                        navTo1stSpecimenHang = robot.trajectoryBuilder(navTo1stWallAlignCheckPoint.end())
                                //.splineToLinearHeading(new Pose2d(poseInitialSpecimenHang.getX() + Constants.SPECIMEN_HANG_OFFSET, poseInitialSpecimenHang.getY(), poseInitialSpecimenHang.getHeading()), poseInitialSpecimenHang.getHeading(),
                                .splineToLinearHeading(new Pose2d(poseSecondSpecimenHang.getX(), poseSecondSpecimenHang.getY(), poseSecondSpecimenHang.getHeading()), poseSecondSpecimenHang.getHeading(),
                                SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                                SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_TO_SUBMERSIBLE))
                                .addDisplacementMarker(16.0, () -> robot.armMech.prepareHangSpecimen()) // 0" or immediately when starting

                                .build();
                        robot.followTrajectoryAsync(navTo1stWallAlignCheckPoint);
                        currentHumanSpecimenState = HumanSpecimenState.PREPARE_HANG_2;
                    }
                    break;
                case PREPARE_HANG_2:
                    if (!robot.isBusy() && robot.armMech.prepareHangSpecimen()) {
                        robot.armMech.resetHangSpecimenState();
                        robot.armMech.hangSpecimenUpdate();
                        currentHumanSpecimenState = HumanSpecimenState.HANG_2ND_SPECIMEN_ON_HIGH_BAR;
                    }
                    break;
                case HANG_2ND_SPECIMEN_ON_HIGH_BAR:
                    // Check if the state machine (hang specimen operation) is finished
                    // 'hangSpecimenUpdate() = false' while the state machine is still executing
                    // 'hangSpecimenUpdate() = true' when the hang operation is complete
                    // When finished, begin trajectory to push the first alliance-specific sample
                    if (!robot.isBusy() && robot.armMech.hangSpecimenUpdate()) {
                        robot.armMech.resetPreparePickWallSpecimenState();
                        robot.armMech.resetPickWallSpecimenState();
                        navToWallPick2Align = robot.trajectoryBuilder(robot.getPoseEstimate(), false)
                                .lineToLinearHeading(new Pose2d(poseWallAlign.getX(), poseWallAlign.getY(), poseWallAlign.getHeading()))
                                .addDisplacementMarker(0.1, () -> robot.armMech.preparePickWallSpecimen())
                                .addDisplacementMarker(() -> robot.followTrajectoryAsync(navToWallPick2))
                                .build();
                        navToWallPick2 = robot.trajectoryBuilder(navToWallPick2Align.end())
                                .lineTo(new Vector2d(poseWallPick.getX(), poseWallPick.getY()))
                                //.addDisplacementMarker(() -> robot.armMech.pickWallSpecimen())
                                .build();
                        robot.followTrajectoryAsync(navToWallPick2Align);
                        currentHumanSpecimenState = HumanSpecimenState.PREPARE_PICK_WALL_SPECIMEN_2;
                    }
                    break;
                case PREPARE_PICK_WALL_SPECIMEN_2:
                    if (!robot.isBusy() && robot.armMech.preparePickWallSpecimen()) {
                        robot.armMech.pickWallSpecimen();
                        currentHumanSpecimenState = HumanSpecimenState.NAV_TO_WALL_PICK_2;
                    }
                    break;
                case NAV_TO_WALL_PICK_2:
                    if (!robot.isBusy() && robot.armMech.pickWallSpecimen()) {
                        robot.armMech.resetPrepareHangSpecimenState();
                        navTo2ndWallAlignCheckPoint = robot.trajectoryBuilder(robot.getPoseEstimate())
                                //.lineToSplineHeading(new Pose2d(poseWallPick.getX(), poseWallPick.getY() - 6, Math.toRadians(0)))
                                .lineToSplineHeading(new Pose2d(poseWallAlignCheckpoint.getX(), poseWallAlignCheckpoint.getY(), poseWallAlignCheckpoint.getHeading()))
                                .addDisplacementMarker(() -> robot.followTrajectoryAsync(navTo2ndSpecimenHang))
                                .build();
                        navTo2ndSpecimenHang = robot.trajectoryBuilder(navTo2ndWallAlignCheckPoint.end())
                                .splineToLinearHeading(new Pose2d(poseThirdSpecimenHang.getX(), poseThirdSpecimenHang.getY(), poseThirdSpecimenHang.getHeading()), poseThirdSpecimenHang.getHeading(),
                                        SampleMecanumDrive.getVelocityConstraint(MAX_VEL, MAX_ANG_VEL, TRACK_WIDTH),
                                        SampleMecanumDrive.getAccelerationConstraint(MAX_ACCEL * RATE_LIMITER_ACCEL_NAV_TO_SUBMERSIBLE))
                                .addDisplacementMarker(18.0, () -> robot.armMech.prepareHangSpecimen())
                                .build();
                        robot.followTrajectoryAsync(navTo2ndWallAlignCheckPoint);
                        currentHumanSpecimenState = HumanSpecimenState.PREPARE_HANG_3;
                    }
                    break;
                case PREPARE_HANG_3:
                    if (!robot.isBusy() && robot.armMech.prepareHangSpecimen()) {
                        robot.armMech.resetHangSpecimenState();
                        robot.armMech.hangSpecimenUpdate();
                        currentHumanSpecimenState = HumanSpecimenState.HANG_3RD_SPECIMEN_ON_HIGH_BAR;
                    }
                    break;
                case HANG_3RD_SPECIMEN_ON_HIGH_BAR:
                    if (robot.armMech.hangSpecimenUpdate()) {
                        currentHumanSpecimenState = HumanSpecimenState.IDLE;
                    }
                    break;
                case IDLE:
                    break;
            }
            // Anything outside of the switch statement will run independent of the currentState

            // We update drive continuously in the background, regardless of state
            robot.update();

            // We update our pivot and slider PID controllers continuously in the background, regardless of state
            robot.pivot.update();
            robot.slider.update();

            // Continually write pose to `PoseStorage`
            PoseStorage.currentPose = robot.getPoseEstimate();

            // Print pose to telemetry
            robot.localTelemetry.addData("Pivot Pos", robot.pivot.getPosition());
            robot.localTelemetry.addData("Slider Pos", robot.slider.getPosition());
            robot.localTelemetry.addData("Slider at Target", robot.slider.isAtTarget());
            robot.localTelemetry.addData("Pivot at Target", robot.pivot.isAtTarget());
            robot.localTelemetry.update();
        }
    }
}
