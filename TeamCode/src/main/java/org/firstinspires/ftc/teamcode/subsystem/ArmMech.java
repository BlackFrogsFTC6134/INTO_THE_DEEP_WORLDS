package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmMech extends RobotModuleEx {
    private Pivot pivot;
    private Slider slider;
    private Wrist wrist;
    private Claw claw;

    protected enum PrepareHangSpecimenState {
        IDLE,
        START,
        RAISING_PIVOT,
        EXTENDING_SLIDER,
        ROTATING_WRIST,
        EXIT
    }
    protected PrepareHangSpecimenState currentPrepareHangSpecimenState;

    protected enum HangSpecimenOnChamberState {
        IDLE,
        START,
        SETTING_WRIST,
        SETTING_PIVOT,
        PIVOT_STABILIZING,
        RETRACTING_SLIDER_TO_CLAW_RELEASE_POSITION,
        OPENING_CLAW,
        SETTING_SLIDER_AND_WRIST_TO_FINISH_POSITION,
        EXIT_HANG_SPECIMEN
    }
    protected HangSpecimenOnChamberState currentHangSpecimenOnChamberState;

    protected enum PreparePickSampleState {
        IDLE,
        START,
        SETTING_WRIST,
        SETTING_PIVOT,
        CLOSING_CLAW,
        ROTATING_WRIST,
        EXIT
    }
    protected PreparePickSampleState currentPreparePickSampleState;

    protected enum PickSampleState {
        IDLE,
        START,
        SETTING_PIVOT,
        CLOSING_CLAW,
        ROTATING_WRIST,
        EXIT_PICK_SAMPLE
    }
    protected PickSampleState currentPickSampleState;

    protected enum PreparePickSampleTeleOpState {
        IDLE,
        START,
        SETTING_PIVOT,
        SETTING_SLIDER,
        EXIT
    }
    protected PreparePickSampleTeleOpState currentPreparePickSampleTeleOpState;

    protected enum PickSampleTeleOpState {
        IDLE,
        START,
        CLOSING_CLAW,
        ROTATING_WRIST,
        EXIT_PICK_SAMPLE
    }
    protected PickSampleTeleOpState currentPickSampleTeleOpState;

    protected enum PreparePickWallSpecimenState {
        IDLE,
        START,
        RAISING_PIVOT,
        EXIT,
    }
    protected PreparePickWallSpecimenState currentPreparePickWallSpecimenState;

    protected enum PickWallSpecimenState {
        IDLE,
        START,
        SETTING_PIVOT_AND_WRIST,
        CLOSING_CLAW,
        RAISING_PIVOT,
        EXIT_PICK_WALL_SPECIMEN
    }
    protected PickWallSpecimenState currentPickWallSpecimenState;

    protected enum PrepareDropSampleHighBasketState {
        IDLE,
        START,
        ROTATING_PIVOT,
        EXIT
    }
    protected PrepareDropSampleHighBasketState currentPrepareDropSampleHighBasketState;

    protected enum DropSampleHighBasketState {
        IDLE,
        START,
        ROTATING_PIVOT,
        MOVING_SLIDER_TO_HIGH_BASKET,
        OPENING_CLAW,
        MOVING_SLIDER_TO_HOME,
        EXIT_DROP_SAMPLE_HIGH_BASKET
    }
    protected DropSampleHighBasketState currentDropSampleHighBasketState;

    protected enum DropSampleZoneState {
        IDLE,
        START,
        RAISING_WRIST,
        OPENING_CLAW,
        EXIT_DROP_SAMPLE_ZONE
    }
    protected DropSampleZoneState currentDropSampleZoneState;

    private boolean hangingComplete = false;
    private boolean prepareHangSpecimenComplete = false;
    private boolean hangSpecimenReady = false;
    private boolean pickSampleComplete = false;
    private boolean preparePickSampleComplete = false;
    private boolean pickSampleReady = false;
    private boolean preparePickWallSpecimenComplete = false;
    private boolean pickWallSpecimenComplete = false;
    private boolean pickWallSpecimenReady = false;
    private boolean prepareDropSampleHighBasketComplete = false;
    private boolean dropSampleHighBasketComplete = false;
    private boolean dropHighBasketReady = false;
    private boolean dropSampleZoneComplete = false;
    private boolean preparePickSampleTeleOpComplete = false;
    private boolean pickSampleTeleOpComplete = false;
    private boolean pickSampleTeleOpReady = false;

    ElapsedTime prepareHangSpecimenTimer = new ElapsedTime();
    ElapsedTime hangSpecimenTimer = new ElapsedTime();
    ElapsedTime preparePickSampleTimer = new ElapsedTime();
    ElapsedTime pickSampleTimer = new ElapsedTime();
    ElapsedTime preparePickWallSpecimenTimer = new ElapsedTime();
    ElapsedTime pickWallSpecimenTimer = new ElapsedTime();
    ElapsedTime prepareDropSampleHighBasketTimer = new ElapsedTime();
    ElapsedTime dropSampleHighBasketTimer = new ElapsedTime();
    ElapsedTime dropSampleZoneTimer = new ElapsedTime();
    ElapsedTime pickSampleTeleOpTimer = new ElapsedTime();
    ElapsedTime preparePickSampleTeleOpTimer = new ElapsedTime();


    public ArmMech(RobotSuperDwarka robot) {
        super(robot);
    }

    public void initialize() {
        pivot = robotSuperDwarka.pivot;
        slider = robotSuperDwarka.slider;
        wrist = robotSuperDwarka.wrist;
        claw = robotSuperDwarka.claw;
    }

    public void resetPrepareDropSampleHighBasketState() {
        setPrepareDropSampleHighBasketState(PrepareDropSampleHighBasketState.START);
        prepareDropSampleHighBasketComplete = false;
        pickSampleReady = false;
        pickWallSpecimenReady = false;
        hangSpecimenReady = false;
        pickSampleTeleOpReady = false;
    }

    private void setPrepareDropSampleHighBasketState(PrepareDropSampleHighBasketState newState) {
        prepareDropSampleHighBasketTimer.reset();
        currentPrepareDropSampleHighBasketState = newState;
    }

    public boolean prepareDropSampleHighBasket() {
        if (dropHighBasketReady) {
            setPrepareDropSampleHighBasketState(PrepareDropSampleHighBasketState.EXIT);
        }
        switch (currentPrepareDropSampleHighBasketState) {
            case IDLE:
                // Do nothing
                break;
            case START:
                // Set LED to indicate state machine starting
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_START, Constants.LED_STATE_MACHINE_START_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_START, Constants. LED_STATE_MACHINE_START_TIMER);
                // Pull slider back to home position
                slider.setPosition(Constants.SLIDER_HOME, false);
                // Rotate pivot to high basket drop position
                pivot.setPosition(Constants.PIVOT_DROP_HIGH_BASKET);
                setPrepareDropSampleHighBasketState(PrepareDropSampleHighBasketState.ROTATING_PIVOT);
                break;
            case ROTATING_PIVOT:
                // When pivot is at target position, rotate wrist and begin moving slider up to drop position
                if (pivot.isAtTarget() && slider.isAtTarget()) {
                    setPrepareDropSampleHighBasketState(PrepareDropSampleHighBasketState.EXIT);
                }
                break;
            case EXIT:
                dropHighBasketReady = true;
                prepareDropSampleHighBasketComplete = true;
                // Set LED to indicate state machine stopping
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                break;
        }
        return prepareDropSampleHighBasketComplete;
    }

    public boolean isPrepareDropSampleHighBasketComplete() {
        return prepareDropSampleHighBasketComplete;
    }

    public boolean isDropHighBasketReady() {
        return dropHighBasketReady;
    }

    public void resetDropSampleHighBasketState(boolean auton) {
        setDropSampleHighBasketState(DropSampleHighBasketState.START);
        dropSampleHighBasketComplete = false;
        if (auton) {
            dropHighBasketReady = true;
        }

        // Reset the readiness of the other state machines if this state machine is called
        // don't think I need these reset here
        //pickSampleReady = false;
        //pickWallSpecimenReady = false;
        //hangSpecimenReady = false;
        //pickSampleTeleOpReady = false;

    }

    private void setDropSampleHighBasketState(DropSampleHighBasketState newState) {
        dropSampleHighBasketTimer.reset();
        currentDropSampleHighBasketState = newState;
    }

    public boolean dropSampleHighBasket() {
        // Only allow dropSampleHighBasket to run if the prepare stage was completed and nothing has disturbed it.
        if (!dropHighBasketReady) {
            // Optionally log a message or set telemetry to indicate not ready.
            return false;
        }
        switch (currentDropSampleHighBasketState) {
            case IDLE:
                //Do nothing
                break;
            case START:
                // Set LED to indicate state machine starting
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_START, Constants.LED_STATE_MACHINE_START_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_START, Constants. LED_STATE_MACHINE_START_TIMER);
                // Rotate pivot to high basket drop position
                pivot.setPosition(Constants.PIVOT_DROP_HIGH_BASKET);
                setDropSampleHighBasketState(DropSampleHighBasketState.ROTATING_PIVOT);
                break;
            case ROTATING_PIVOT:
                // When pivot is at target position, rotate wrist and begin moving slider up to drop position
                if (pivot.isAtTarget()) {
                    wrist.setPosition(Constants.WRIST_POSITION_HIGH_BASKET_DROP);
                    slider.setPosition(Constants.SLIDER_BASKET_DROP_HIGH, true);
                    setDropSampleHighBasketState(DropSampleHighBasketState.MOVING_SLIDER_TO_HIGH_BASKET);
                }
                break;
            case MOVING_SLIDER_TO_HIGH_BASKET:
                // When slider is at target position, open claw to drop sample in basket (we assume wrist completes movement prior to slider, thus no timer)
                if (slider.isAtTarget()) {
                    claw.setPosition(Constants.CLAW_OPEN);
                    setDropSampleHighBasketState(DropSampleHighBasketState.OPENING_CLAW);
                }
                break;
            case OPENING_CLAW:
                //Timer to ensure time for claw to open and specimen to drop clear; when timer is finished,
                // rotate wrist and move slider to home position
                if (dropSampleHighBasketTimer.milliseconds() > 245) {
                    wrist.setPosition(Constants.WRIST_POSITION_PICKUP); //originally WRIST_POSITION_HANG
                    slider.setPosition(Constants.SLIDER_HOME, false);
                    setDropSampleHighBasketState(DropSampleHighBasketState.MOVING_SLIDER_TO_HOME);
                }
                break;
            case MOVING_SLIDER_TO_HOME:
                if (slider.isAtTarget()) {
                    setDropSampleHighBasketState(DropSampleHighBasketState.EXIT_DROP_SAMPLE_HIGH_BASKET);
                }
                break;
            case EXIT_DROP_SAMPLE_HIGH_BASKET:
                dropSampleHighBasketComplete = true;
                dropHighBasketReady = false;
                // Set LED to indicate state machine stopping
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                break;
        }
        return dropSampleHighBasketComplete;
    }

    public void resetDropSampleZone() {
        setDropSampleZoneState(DropSampleZoneState.START);
        dropSampleZoneComplete = false;
    }

    private void setDropSampleZoneState(DropSampleZoneState newState) {
        dropSampleZoneTimer.reset();
        currentDropSampleZoneState = newState;
    }

    public boolean dropSampleZone() {
        switch (currentDropSampleZoneState) {
            case IDLE:
                //Do nothing
                break;
            case START:
                // Set LED to indicate state machine starting
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_START, Constants.LED_STATE_MACHINE_START_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_START, Constants. LED_STATE_MACHINE_START_TIMER);
                // Rotate wrist to zone drop position
                //pivot.setPosition(Pivot.PIVOT_DROP_HIGH_BASKET);
                // Open claw to drop sample
                claw.setPosition(Constants.CLAW_OPEN);
                //wrist.setPosition(Wrist.WRIST_POSITION_ZONE_DROP);
                setDropSampleZoneState(DropSampleZoneState.OPENING_CLAW);
                break;
            case OPENING_CLAW:
                // When slider is at target position, open claw to drop sample in basket (we assume wrist completes movement prior to slider, thus no timer)
                if (dropSampleZoneTimer.milliseconds() >= 20) {
                    //wrist.setPosition(Wrist.WRIST_POSITION_CARRY);
                    //pivot.setPosition(Pivot.PIVOT_INTERMEDIATE);
                    pivot.setPosition(Constants.PIVOT_PICKUP_3RD_YELLOW);
                    //claw.setPosition(Claw.CLAW_OPEN);
                    setDropSampleZoneState(DropSampleZoneState.EXIT_DROP_SAMPLE_ZONE);
                    //setDropSampleZoneState(DropSampleZoneState.RAISING_WRIST);
                }
                break;
            case RAISING_WRIST:
                //Timer to ensure time for claw to open and specimen to drop clear; when timer is finished,
                // rotate wrist and move slider to home position
                if (dropSampleZoneTimer.milliseconds() > 20) {
                    //wrist.setPosition(Wrist.WRIST_POSITION_CARRY); //originally WRIST_POSITION_HANG
                    //slider.setPosition(Slider2.EXTEND_HOME, false);
                    setDropSampleZoneState(DropSampleZoneState.EXIT_DROP_SAMPLE_ZONE);
                }
                break;
            case EXIT_DROP_SAMPLE_ZONE:
                dropSampleZoneComplete = true;
                // Set LED to indicate state machine stopping
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                break;
        }
        return dropSampleZoneComplete;
    }

    public void resetPreparePickWallSpecimenState() {
        setPreparePickWallSpecimenState(PreparePickWallSpecimenState.START);
        preparePickWallSpecimenComplete = false;
        pickSampleReady = false;
        hangSpecimenReady = false;
        dropHighBasketReady = false;
        pickSampleTeleOpReady = false;
    }

    private void setPreparePickWallSpecimenState(PreparePickWallSpecimenState newState) {
        preparePickWallSpecimenTimer.reset();
        currentPreparePickWallSpecimenState = newState;
    }

    public boolean preparePickWallSpecimen() {
        if (pickWallSpecimenReady) {
            setPreparePickWallSpecimenState(PreparePickWallSpecimenState.EXIT);
        }
        switch (currentPreparePickWallSpecimenState) {
            case IDLE:
                //Do nothing
                break;
            case START:
                // Set LED to indicate state machine starting
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_START, Constants.LED_STATE_MACHINE_START_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_START, Constants. LED_STATE_MACHINE_START_TIMER);
                // Pull slider back to home position
                slider.setPosition(Constants.SLIDER_HOME, false);
                // Set pivot and wrist to wall pickup position
                wrist.setPosition(Constants.WRIST_POSITION_WALL_SPECIMEN_PICK);
                claw.setPosition(Constants.CLAW_OPEN);
                pivot.setPosition(Constants.PIVOT_PICKUP_WALL_SPECIMEN);
                setPreparePickWallSpecimenState(PreparePickWallSpecimenState.RAISING_PIVOT);
                break;
            case RAISING_PIVOT:
                // When pivot is at position and enough time has elapsed for the wrist and claw to move into position, exit state machine
                if (pivot.isAtTarget() && slider.isAtTarget() && preparePickWallSpecimenTimer.milliseconds() > 100) {
                    setPreparePickWallSpecimenState(PreparePickWallSpecimenState.EXIT);
                }
                break;
            case EXIT:
                pickWallSpecimenReady = true;
                preparePickWallSpecimenComplete= true;
                // Set LED to indicate state machine stopping
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                break;
        }
        return preparePickWallSpecimenComplete;
    }

    public boolean isPreparePickWallSpecimenComplete() {
        return preparePickWallSpecimenComplete;
    }

    public boolean isPickWallSpecimenReady() {
        return pickWallSpecimenReady;
    }


    public void resetPickWallSpecimenState() {
        setPickWallSpecimenState(PickWallSpecimenState.START);
        pickWallSpecimenComplete = false;

        // maybe don't need to reset here
        //pickSampleReady = false;
        //hangSpecimenReady = false;
        //dropHighBasketReady = false;
        //pickSampleTeleOpReady = false;
    }

    private void setPickWallSpecimenState(PickWallSpecimenState newState) {
        pickWallSpecimenTimer.reset();
        currentPickWallSpecimenState = newState;
    }

    public boolean pickWallSpecimen() {
        // Only allow pickWallSample to run if the prepare stage was completed and nothing has disturbed it.
        if (!pickWallSpecimenReady) {
            // Optionally log a message or set telemetry to indicate not ready.
            return false;
        }
        switch (currentPickWallSpecimenState) {
            case IDLE:
                //Do nothing
                break;
            case START:
                // Set LED to indicate state machine starting
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_START, Constants.LED_STATE_MACHINE_START_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_START, Constants. LED_STATE_MACHINE_START_TIMER);
                // Set pivot and wrist to wall pickup position
                //wrist.setPosition(Wrist.WRIST_POSITION_WALL_SPECIMEN_PICK);
                //claw.setPosition(Claw.CLAW_OPEN);
                //pivot.setPosition(Pivot.PIVOT_PICKUP_WALL_SPECIMEN);
                //setPickWallSpecimenState(PickWallSpecimenState.SETTING_PIVOT_AND_WRIST);
                claw.setPosition(Constants.CLAW_CLOSE);
                setPickWallSpecimenState(PickWallSpecimenState.CLOSING_CLAW);
                break;
            case CLOSING_CLAW:
                // Give time for the claw to grab the sample and then rotate wrist to clear the floor
                if (pickWallSpecimenTimer.milliseconds() > 150) {
                    //wrist.setPosition(Wrist.WRIST_POSITION_CARRY);
                    pivot.setPosition(Constants.PIVOT_PICKUP_WALl_SPECIMEN_RELEASE);
                    setPickWallSpecimenState(PickWallSpecimenState.RAISING_PIVOT);
                }
                break;
            case RAISING_PIVOT:
                // Wait until pivot is raised enough to release the specimen from the wall
                if (pivot.isAtTarget()) {
                    setPickWallSpecimenState(PickWallSpecimenState.EXIT_PICK_WALL_SPECIMEN);
                }
                break;
            case EXIT_PICK_WALL_SPECIMEN:
                pickWallSpecimenComplete = true;
                pickWallSpecimenReady = false;
                // Set LED to indicate state machine stopping
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                break;
        }
        return pickWallSpecimenComplete;
    }


    public void resetPreparePickSampleState() {
        setPreparePickSampleState(PreparePickSampleState.START);
        preparePickSampleComplete = false;
        // Reset ready states
        pickWallSpecimenReady = false;
        hangSpecimenReady = false;
        dropHighBasketReady = false;
    }

    private void setPreparePickSampleState(PreparePickSampleState newState) {
        preparePickSampleTimer.reset();
        currentPreparePickSampleState = newState;
    }

    public boolean preparePickSample() {
        if (pickSampleReady) {
            setPreparePickSampleState(PreparePickSampleState.EXIT);
        }
        switch (currentPreparePickSampleState) {
            case IDLE:
                //Do nothing
                break;
            case START:
                // Set LED to indicate state machine starting
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_START, Constants.LED_STATE_MACHINE_START_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_START, Constants. LED_STATE_MACHINE_START_TIMER);
                // Pull slider back to home position
                slider.setPosition(Constants.SLIDER_HOME, false);
                // Set pivot to pickup position
                wrist.setPosition(Constants.WRIST_POSITION_PICKUP);
                claw.setPosition(Constants.CLAW_OPEN);
                pivot.setPosition(Constants.PIVOT_PICKUP_PREPARE_PIT_PICK);
                setPreparePickSampleState(PreparePickSampleState.SETTING_PIVOT);
                break;
            case SETTING_PIVOT:
                // Prepare pick sample is finished when pivot is at target and enough time has elapsed that wrist and claw are in position
                if (pivot.isAtTarget() && slider.isAtTarget() && preparePickSampleTimer.milliseconds() >= 200) {
                    setPreparePickSampleState(PreparePickSampleState.EXIT);
                }
                break;
            case EXIT:
                pickSampleReady = true;
                // also allow teleop pick
                // setting pickSampleTeleOpReady is not allowing preparePickSampleTeleOp to run
                pickSampleTeleOpReady = true;
                preparePickSampleComplete = true;
                // Set LED to indicate state machine stopping
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                break;
        }
        return preparePickSampleComplete;
    }

    public boolean isPreparePickSampleComplete() {
        return preparePickSampleComplete;
    }

    public boolean isPickSampleReady() {
        return pickSampleReady;
    }

    public void resetPickSampleState(boolean auton) {
        setPickSampleState(PickSampleState.START);
        pickSampleComplete = false;
        if (auton) {
            pickSampleReady = true;
        }

        // maybe don't need to reset here
        //pickWallSpecimenReady = false;
        //hangSpecimenReady = false;
        //dropHighBasketReady = false;
        //pickSampleTeleOpReady = false;
    }

    private void setPickSampleState(PickSampleState newState) {
        pickSampleTimer.reset();
        currentPickSampleState = newState;
    }

    public boolean pickSample() {
        // Only allow pickSample to run if the prepare stage was completed and nothing has disturbed it.
        if (!pickSampleReady) {
            // Optionally log a message or set telemetry to indicate not ready.
            return false;
        }
        switch (currentPickSampleState) {
            case IDLE:
                //Do nothing
                break;
            case START:
                // Set LED to indicate state machine starting
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_START, Constants.LED_STATE_MACHINE_START_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_START, Constants. LED_STATE_MACHINE_START_TIMER);
                // Set pivot to pickup position
                pivot.setPosition(Constants.PIVOT_PICKUP);
                setPickSampleState(PickSampleState.SETTING_PIVOT);
                break;
            case SETTING_PIVOT:
                // When pivot is at position, close claw, and move to next state
                if (pivot.isAtTarget()) {
                    claw.setPosition(Constants.CLAW_CLOSE);
                    setPickSampleState(PickSampleState.CLOSING_CLAW);
                }
                break;
            case CLOSING_CLAW:
                // Give time for the claw to grab the sample and then rotate wrist to clear the floor
                if (pickSampleTimer.milliseconds() >= 200) {
                    pivot.setPosition(Constants.PIVOT_PICKUP_PREPARE_PIT_PICK);
                    wrist.setPosition(Constants.WRIST_POSITION_CARRY);
                    setPickSampleState(PickSampleState.ROTATING_WRIST);
                }
                break;
            case ROTATING_WRIST:
                // Give time for the wrist to rotate backward with the sample to clear the floor; it may be desirable for this to be low or
                // zero (0) if there are no issues with the sample coming loose or breaking free with robot movement
                if (pickSampleTimer.milliseconds() > 70) {
                    // Pull slider back after pick for teleop case in case slider was extended for a sample pick from the pit;
                    // In auton, the slider should already be at home position, so this command should cause no issue
                    slider.setPosition(Constants.SLIDER_HOME, false);
                    setPickSampleState(PickSampleState.EXIT_PICK_SAMPLE);
                }
                break;
            case EXIT_PICK_SAMPLE:
                pickSampleComplete = true;
                pickSampleReady = false;
                pickSampleTeleOpReady = false;
                // Set LED to indicate state machine stopping
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                break;
        }
        return pickSampleComplete;
    }




    public void resetPreparePickSampleTeleOpState() {
        setPreparePickSampleTeleOpState(PreparePickSampleTeleOpState.START);
        preparePickSampleTeleOpComplete = false;

        // Reset ready states
        pickWallSpecimenReady = false;
        hangSpecimenReady = false;
        dropHighBasketReady = false;
        pickSampleReady = false;
    }

    private void setPreparePickSampleTeleOpState(PreparePickSampleTeleOpState newState) {
        preparePickSampleTeleOpTimer.reset();
        currentPreparePickSampleTeleOpState = newState;
    }

    public boolean preparePickSampleTeleOp() {
        //if (pickSampleTeleOpReady) {
        //    setPreparePickSampleTeleOpState(PreparePickSampleTeleOpState.EXIT);
        //}
        switch (currentPreparePickSampleTeleOpState) {
            case IDLE:
                //Do nothing
                break;
            case START:
                // Set LED to indicate state machine starting
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_START, Constants.LED_STATE_MACHINE_START_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_START, Constants. LED_STATE_MACHINE_START_TIMER);
                // Pull slider back to home position
                //slider.setPosition(Slider.EXTEND_HOME, false);
                // Set pivot to pit pickup position
                //wrist.setPosition(Wrist.WRIST_POSITION_PICKUP);
                //claw.setPosition(Claw.CLAW_OPEN);
                slider.setPosition(Constants.SLIDER_PIT_PICK, false);
                pivot.setPosition(Constants.PIVOT_PIT_PICKUP);
                claw.setPosition(Constants.CLAW_OPEN);
                setPreparePickSampleTeleOpState(PreparePickSampleTeleOpState.SETTING_PIVOT);
                break;
            case SETTING_PIVOT:
                // Prepare pick sample is finished when pivot is at target and enough time has elapsed that wrist and claw are in position
                if (pivot.isAtTarget() && slider.isAtTarget()) {
                    wrist.setPosition(Constants.WRIST_POSITION_PICKUP);
                    claw.setPosition(Constants.CLAW_OPEN);
                    //slider.setPosition(Slider.EXTEND_PIT_PICK, false);
                    setPreparePickSampleTeleOpState(PreparePickSampleTeleOpState.SETTING_SLIDER);
                }
                break;
            case SETTING_SLIDER:
                // Prepare pick sample is finished when pivot is at target and enough time has elapsed that wrist and claw are in position
                if (preparePickSampleTeleOpTimer.milliseconds() >= 90) {
                    //slider.setPosition(Slider.EXTEND_PIT_PICK);
                    setPreparePickSampleTeleOpState(PreparePickSampleTeleOpState.EXIT);
                }
                break;
            case EXIT:
                pickSampleTeleOpReady = true;
                preparePickSampleTeleOpComplete = true;
                // Set LED to indicate state machine stopping
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                break;
        }
        return preparePickSampleTeleOpComplete;
    }

    public boolean isPreparePickSampleTeleOpComplete() {
        return preparePickSampleTeleOpComplete;
    }








    public void resetPickSampleTeleOpState() {
        setPickSampleTeleOpState(PickSampleTeleOpState.START);
        pickSampleTeleOpComplete = false;
        pickSampleReady = false;
        pickSampleTeleOpReady = true;

        // maybe don't need to reset here
        //pickSampleReady = false;
        //pickWallSpecimenReady = false;
        //hangSpecimenReady = false;
        //dropHighBasketReady = false;
    }

    private void setPickSampleTeleOpState(PickSampleTeleOpState newState) {
        pickSampleTeleOpTimer.reset();
        currentPickSampleTeleOpState = newState;
    }

    public boolean isPickSampleTeleOpReady() {
        return pickSampleTeleOpReady;
    }


    // Special pick sample state machine for teleop pit pick
    public boolean pickSampleTeleOp() {
        // Only allow pickSample to run if the prepare stage was completed and nothing has disturbed it.
        if (!pickSampleTeleOpReady && !pickSampleReady) {
            // Optionally log a message or set telemetry to indicate not ready.
            return false;
        }
        switch (currentPickSampleTeleOpState) {
            case IDLE:
                //Do nothing
                break;
            case START:
                // Set LED to indicate state machine starting
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_START, Constants.LED_STATE_MACHINE_START_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_START, Constants. LED_STATE_MACHINE_START_TIMER);
                // Close claw
                claw.setPosition(Constants.CLAW_CLOSE);
                setPickSampleTeleOpState(PickSampleTeleOpState.CLOSING_CLAW);
                break;
            case CLOSING_CLAW:
                // Give time for the claw to grab the sample and then rotate wrist to clear the floor
                if (pickSampleTeleOpTimer.milliseconds() >= 190) {
                    pivot.setPosition(Constants.PIVOT_PIT_PICKUP);
                    wrist.setPosition(Constants.WRIST_POSITION_CARRY);
                    setPickSampleTeleOpState(PickSampleTeleOpState.ROTATING_WRIST);
                }
                break;
            case ROTATING_WRIST:
                // Give time for the wrist to rotate backward with the sample to clear the floor; it may be desirable for this to be low or
                // zero (0) if there are no issues with the sample coming loose or breaking free with robot movement
                if (pivot.isAtTarget() && pickSampleTeleOpTimer.milliseconds() > 20) {
                    // Pull slider back after pick for teleop case in case slider was extended for a sample pick from the pit;
                    // In auton, the slider should already be at home position, so this command should cause no issue
                    slider.setPosition(Constants.SLIDER_HOME, false);
                    setPickSampleTeleOpState(PickSampleTeleOpState.EXIT_PICK_SAMPLE);
                }
                break;
            case EXIT_PICK_SAMPLE:
                pickSampleTeleOpComplete = true;
                pickSampleTeleOpReady = false;
                pickSampleReady = false;
                // Set LED to indicate state machine stopping
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                break;
        }
        return pickSampleTeleOpComplete;
    }

    // Helper method for prepareHangSpecimen; resets the state machine
    public void resetPrepareHangSpecimenState() {
        prepareHangSpecimenComplete = false;
        setPrepareHangSpecimenState(PrepareHangSpecimenState.START);
        pickWallSpecimenReady = false;
        pickSampleReady = false;
        dropHighBasketReady = false;
        pickSampleTeleOpReady = false;
    }

    // Helper method for prepareHangSpecimen; sets current state and resets timer
    private void setPrepareHangSpecimenState(PrepareHangSpecimenState newState) {
        currentPrepareHangSpecimenState = newState;
        prepareHangSpecimenTimer.reset();
    }

    public boolean prepareHangSpecimen() {
        if (hangSpecimenReady) {
            setPrepareHangSpecimenState(PrepareHangSpecimenState.EXIT);
        }
        switch (currentPrepareHangSpecimenState) {
            case IDLE:
                //Do Nothing
                break;
            case START:
                // Set LED to indicate state machine starting
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_START, Constants.LED_STATE_MACHINE_START_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_START, Constants. LED_STATE_MACHINE_START_TIMER);
                // Pull slider back to home position
                slider.setPosition(Constants.SLIDER_HOME, false);
                // Start by raising the pivot to the approach position
                wrist.setPosition(Constants.WRIST_POSITION_HIGH_BASKET_DROP);
                pivot.setPosition(Constants.PIVOT_HANG_SPECIMEN_HIGH);
                //pivot.setPosition(Pivot.PIVOT_HANG_SPECIMEN_HIGH_APPROACH);
                setPrepareHangSpecimenState(PrepareHangSpecimenState.RAISING_PIVOT);
                break;
            case RAISING_PIVOT:
                // When pivot reaches the approach position, extend the slider to prepare for hang
                if (pivot.getPosition() >= Constants.PIVOT_HANG_SPECIMEN_SLIDER_START) {
                    slider.setPosition(Constants.SLIDER_HANG_SPECIMEN_HIGH, false);
                    wrist.setPosition(Constants.WRIST_POSITION_HANG);
                    setPrepareHangSpecimenState(PrepareHangSpecimenState.EXTENDING_SLIDER);
                }
                break;
            case EXTENDING_SLIDER:
                // When slider reaches the target position, rotate the wrist to the hang position
                if (pivot.isAtTarget() && slider.isAtTarget()) {
                    //wrist.setPosition(Wrist.WRIST_POSITION_HANG);
                    setPrepareHangSpecimenState(PrepareHangSpecimenState.ROTATING_WRIST);
                }
                break;
            case ROTATING_WRIST:
                if (prepareHangSpecimenTimer.milliseconds() >= 120) {
                    setPrepareHangSpecimenState(PrepareHangSpecimenState.EXIT);
                }
            case EXIT:
                hangSpecimenReady = true;
                prepareHangSpecimenComplete = true;
                // Set LED to indicate state machine stopping
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                break;
        }
        return prepareHangSpecimenComplete;
    }

    public boolean isPrepareHangSpecimenComplete() {
        return prepareHangSpecimenComplete;
    }

    public boolean isHangSpecimenReady() {
        return hangSpecimenReady;
    }

    public void resetHangSpecimenState() {
        setHangSpecimenState(HangSpecimenOnChamberState.START);
        hangingComplete = false;

        // maybe don't need to reset here
        //pickWallSpecimenReady = false;
        //pickSampleReady = false;
        //dropHighBasketReady = false;
        //pickSampleTeleOpReady = false;
    }

    private void setHangSpecimenState(HangSpecimenOnChamberState newState) {
        currentHangSpecimenOnChamberState = newState;
        hangSpecimenTimer.reset();
    }

    public boolean hangSpecimenUpdate() {
        //localTelemetry.addData("ArmMech2 State", currentHangSpecimenOnChamberState.toString());
        //localTelemetry.update();

        // Only allow hangSpecimenUpdate to run if the prepare stage was completed and nothing has disturbed it.
        if (!hangSpecimenReady) {
            // Optionally log a message or set telemetry to indicate not ready.
            return false;
        }

        switch (currentHangSpecimenOnChamberState) {
            case IDLE:
                //Do Nothing
                break;
            case START:
                // Set LED to indicate state machine starting
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_START, Constants.LED_STATE_MACHINE_START_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_START, Constants. LED_STATE_MACHINE_START_TIMER);
                // Start by setting the wrist position the hang specimen position and start a timer
                // to give the wrist time to rotate before moving the pivot, else it will catch on the bar;
                // it may be desirable to set the wrist prior to running this state machine, in that case,
                // setWristTimer should be set to false when calling resetHangSpecimenState, and the pivot will begin
                // moving down immediately.
                pivot.setPosition(Constants.PIVOT_HANG_SPECIMEN_HIGH);
                setHangSpecimenState(HangSpecimenOnChamberState.SETTING_PIVOT);
                break;
            case SETTING_PIVOT:
                // if setWristTimer is true, then the state machine will give time for the wrist to move
                // into position and stabilize; else, the pivot will begin moving down immediately
                if (pivot.isAtTarget()) {
                    setHangSpecimenState(HangSpecimenOnChamberState.PIVOT_STABILIZING);
                }
                break;
            case PIVOT_STABILIZING:
                // Give pivot time to stabilize before retracting slider
                if (hangSpecimenTimer.milliseconds() >= 80) {
                    slider.setPosition(Constants.SLIDER_HOME, false);
                    //slider.setPosition(Slider.EXTEND_HANG_SPECIMEN_HIGH_RELEASE, false);
                    setHangSpecimenState(HangSpecimenOnChamberState.RETRACTING_SLIDER_TO_CLAW_RELEASE_POSITION);
                }
                break;
            case RETRACTING_SLIDER_TO_CLAW_RELEASE_POSITION:
                // When slider reaches the specimen release position, open the claw to release the specimen
                // and start a timer to ensure enough time for the claw to open and the specimen to swing clear
                if (/*slider.isAtTarget() && */slider.getPosition() <= Constants.SLIDER_HANG_SPECIMEN_HIGH_RELEASE) {
                    claw.setPosition(Constants.CLAW_OPEN);
                    setHangSpecimenState(HangSpecimenOnChamberState.SETTING_SLIDER_AND_WRIST_TO_FINISH_POSITION);
                    //setHangSpecimenState(HangSpecimenOnChamberState.OPENING_CLAW);
                }
                break;
            case OPENING_CLAW:
                // Timer to ensure time for claw to open and specimen to swing clear; when timer is finished,
                // move slider to home position and rotate the wrist to clear the submersible
                if (hangSpecimenTimer.milliseconds() >= 10) {
                    slider.setPosition(Constants.SLIDER_HOME, false);
                    //wrist.setPosition(Wrist.WRIST_POSITION_PICKUP);
                    setHangSpecimenState(HangSpecimenOnChamberState.SETTING_SLIDER_AND_WRIST_TO_FINISH_POSITION);
                }
                break;
            case SETTING_SLIDER_AND_WRIST_TO_FINISH_POSITION:
                // Wait until slider is in home position and enough time has elapsed such that the wrist is finished moving; this is to
                // ensure the wrist is pulled back quickly in auton to prevent it from hitting the submersible structure when moving to pick
                // the first yellow sample
                if (slider.isAtTarget() /*&& hangSpecimenTimer.milliseconds() > 20*/) {
                    wrist.setPosition(Constants.WRIST_POSITION_PICKUP);
                    setHangSpecimenState(HangSpecimenOnChamberState.EXIT_HANG_SPECIMEN);
                }
                break;
            case EXIT_HANG_SPECIMEN:
                hangingComplete = true;
                hangSpecimenReady = false;
                // Set LED to indicate state machine stopping
                robotSuperDwarka.ledController.setFrontLED(Constants.LED_FRONT_COLOR_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
                break;
        }
        return hangingComplete;
    }

    // A global abort method to reset all arm state machines.
    public void abortAllStateMachines() {
        // Reset all state machines to starting state
        // may not need all of these, might only need the 'ready variables below

        // try removing these
        //resetPreparePickSampleState();
        resetPickSampleState(false);
        //resetPrepareHangSpecimenState();

        resetHangSpecimenState();

        //resetPrepareDropSampleHighBasketState();

        resetDropSampleHighBasketState(false);

        //resetDropSampleZone();
        //resetPreparePickWallSpecimenState();

        resetPickWallSpecimenState();


        // Reset all boolean ready variables
        // maybe don't need these set here; managed through reset methods

        pickSampleReady = false;
        pickWallSpecimenReady = false;
        hangSpecimenReady = false;
        dropHighBasketReady = false;
        //pickSampleTeleOpReady = true;


        // Set LED to indicate all state machines reset
        robotSuperDwarka.ledController.setRightLED(Constants.LED_BLINKIN_PATTERN_STATE_MACHINE_END, Constants.LED_STATE_MACHINE_END_TIMER);
    }
}
