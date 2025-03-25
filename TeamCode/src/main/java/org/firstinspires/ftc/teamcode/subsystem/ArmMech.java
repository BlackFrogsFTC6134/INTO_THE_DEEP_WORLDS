package org.firstinspires.ftc.teamcode.subsystem;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmMech extends RobotModuleEx {
    //private LinearOpMode opMode;
    private Pivot pivot;
    private Slider2 slider;
    private Wrist wrist;
    private Claw claw;

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

    protected enum PickSampleState {
        IDLE,
        START,
        SETTING_PIVOT,
        CLOSING_CLAW,
        ROTATING_WRIST,
        EXIT_PICK_SAMPLE
    }
    protected PickSampleState currentPickSampleState;

    protected enum PickWallSpecimenState {
        IDLE,
        START,
        SETTING_PIVOT_AND_WRIST,
        CLOSING_CLAW,
        ROTATING_WRIST,
        EXIT_PICK_WALL_SPECIMEN
    }
    protected PickWallSpecimenState currentPickWallSpecimenState;

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
        LOWERING_WRIST,
        RAISING_WRIST,
        OPENING_CLAW,
        EXIT_DROP_SAMPLE_ZONE
    }
    protected DropSampleZoneState currentDropSampleZoneState;

    private boolean hangingComplete = false;
    private boolean pickSampleComplete = false;
    private boolean dropSampleComplete = false;
    private boolean pickWallSpecimenComplete = false;
    private boolean dropSampleZoneComplete = false;

    private boolean setWristTimer = false;

    ElapsedTime hangSpecimenTimer = new ElapsedTime();
    ElapsedTime pickSampleTimer = new ElapsedTime();
    ElapsedTime pickWallSpecimenTimer = new ElapsedTime();
    ElapsedTime dropSampleHighBasketTimer = new ElapsedTime();
    ElapsedTime dropSampleZoneTimer = new ElapsedTime();

    public ArmMech(RobotSuperDwarka robot) {
        super(robot);
    }

    public void initialize() {
        pivot = robotSuperDwarka.pivot;
        slider = robotSuperDwarka.slider;
        wrist = robotSuperDwarka.wrist;
        claw = robotSuperDwarka.claw;
    }

    public void resetDropSampleHighBasketState() {
        setDropSampleHighBasketState(DropSampleHighBasketState.START);
        dropSampleComplete = false;
    }

    private void setDropSampleHighBasketState(DropSampleHighBasketState newState) {
        dropSampleHighBasketTimer.reset();
        currentDropSampleHighBasketState = newState;
    }

    public boolean dropSampleHighBasket() {
        switch (currentDropSampleHighBasketState) {
            case IDLE:
                //Do nothing
                break;
            case START:
                // Rotate pivot to high basket drop position
                pivot.setPosition(Pivot.PIVOT_DROP_HIGH_BASKET);
                setDropSampleHighBasketState(DropSampleHighBasketState.ROTATING_PIVOT);
                break;
            case ROTATING_PIVOT:
                // When pivot is at target position, rotate wrist and begin moving slider up to drop position
                if (pivot.isAtTarget()) {
                    wrist.setPosition(Wrist.WRIST_POSITION_HIGH_BASKET_DROP);
                    slider.setPosition(Slider2.EXTEND_BASKET_DROP_HIGH, true);
                    setDropSampleHighBasketState(DropSampleHighBasketState.MOVING_SLIDER_TO_HIGH_BASKET);
                }
                break;
            case MOVING_SLIDER_TO_HIGH_BASKET:
                // When slider is at target position, open claw to drop sample in basket (we assume wrist completes movement prior to slider, thus no timer)
                if (slider.isAtTarget()) {
                    claw.setPosition(Claw.CLAW_OPEN);
                    setDropSampleHighBasketState(DropSampleHighBasketState.OPENING_CLAW);
                }
                break;
            case OPENING_CLAW:
                //Timer to ensure time for claw to open and specimen to drop clear; when timer is finished,
                // rotate wrist and move slider to home position
                if (dropSampleHighBasketTimer.milliseconds() > 225) {
                    wrist.setPosition(Wrist.WRIST_POSITION_PICKUP); //originally WRIST_POSITION_HANG
                    slider.setPosition(Slider2.EXTEND_HOME, false);
                    setDropSampleHighBasketState(DropSampleHighBasketState.MOVING_SLIDER_TO_HOME);
                }
                break;
            case MOVING_SLIDER_TO_HOME:
                if (slider.isAtTarget()) {
                    setDropSampleHighBasketState(DropSampleHighBasketState.EXIT_DROP_SAMPLE_HIGH_BASKET);
                }
                break;
            case EXIT_DROP_SAMPLE_HIGH_BASKET:
                dropSampleComplete = true;
                break;
        }
        return dropSampleComplete;
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
                // Rotate wrist to zone drop position
                //pivot.setPosition(Pivot.PIVOT_DROP_HIGH_BASKET);
                wrist.setPosition(Wrist.WRIST_POSITION_ZONE_DROP);
                setDropSampleZoneState(DropSampleZoneState.LOWERING_WRIST);
                break;
            case LOWERING_WRIST:
                // When pivot is at target position, rotate wrist and begin moving slider up to drop position
                if (dropSampleZoneTimer.milliseconds() > 50) {
                    //wrist.setPosition(Wrist.WRIST_POSITION_HIGH_BASKET_DROP);
                    //slider.setPosition(Slider2.EXTEND_BASKET_DROP_HIGH, true);
                    claw.setPosition(Claw.CLAW_OPEN);
                    setDropSampleZoneState(DropSampleZoneState.OPENING_CLAW);
                }
                break;
            case OPENING_CLAW:
                // When slider is at target position, open claw to drop sample in basket (we assume wrist completes movement prior to slider, thus no timer)
                if (dropSampleZoneTimer.milliseconds() > 100) {
                    wrist.setPosition(Wrist.WRIST_POSITION_CARRY);
                    //claw.setPosition(Claw.CLAW_OPEN);
                    setDropSampleZoneState(DropSampleZoneState.RAISING_WRIST);
                }
                break;
            case RAISING_WRIST:
                //Timer to ensure time for claw to open and specimen to drop clear; when timer is finished,
                // rotate wrist and move slider to home position
                if (dropSampleZoneTimer.milliseconds() > 50) {
                    //wrist.setPosition(Wrist.WRIST_POSITION_CARRY); //originally WRIST_POSITION_HANG
                    //slider.setPosition(Slider2.EXTEND_HOME, false);
                    setDropSampleZoneState(DropSampleZoneState.EXIT_DROP_SAMPLE_ZONE);
                }
                break;
            case EXIT_DROP_SAMPLE_ZONE:
                dropSampleZoneComplete = true;
                break;
        }
        return dropSampleZoneComplete;
    }


    public void resetPickWallSpecimenState() {
        setPickWallSpecimenState(PickWallSpecimenState.START);
        pickWallSpecimenComplete = false;
    }

    private void setPickWallSpecimenState(PickWallSpecimenState newState) {
        pickWallSpecimenTimer.reset();
        currentPickWallSpecimenState = newState;
    }

    public boolean pickWallSpecimen() {
        switch (currentPickWallSpecimenState) {
            case IDLE:
                //Do nothing
                break;
            case START:
                // Set pivot and wrist to wall pickup position
                pivot.setPosition(Pivot.PIVOT_PICKUP_WALL_SPECIMEN);
                claw.setPosition(Claw.CLAW_OPEN);
                wrist.setPosition(Wrist.WRIST_POSITION_WALL_SPECIMEN_PICK);
                setPickWallSpecimenState(PickWallSpecimenState.SETTING_PIVOT_AND_WRIST);
                break;
            case SETTING_PIVOT_AND_WRIST:
                // When pivot is at position, close claw, and move to next state
                if (pivot.isAtTarget() && pickWallSpecimenTimer.milliseconds() > 50) {
                    claw.setPosition(Claw.CLAW_CLOSE);
                    setPickWallSpecimenState(PickWallSpecimenState.CLOSING_CLAW);
                }
                break;
            case CLOSING_CLAW:
                // Give time for the claw to grab the sample and then rotate wrist to clear the floor
                if (pickWallSpecimenTimer.milliseconds() > 225) {
                    //wrist.setPosition(Wrist.WRIST_POSITION_CARRY);
                    pivot.setPosition(Pivot.PIVOT_PICKUP_WALl_SPECIMEN_RELEASE);
                    setPickWallSpecimenState(PickWallSpecimenState.ROTATING_WRIST);
                }
                break;
            case ROTATING_WRIST:
                // Give time for the wrist to rotate backward with the sample to clear the floor; it may be desirable for this to be low or
                // zero (0) if there are no issues with the sample coming loose or breaking free with robot movement
                if (pickWallSpecimenTimer.milliseconds() > 200) {
                    setPickWallSpecimenState(PickWallSpecimenState.EXIT_PICK_WALL_SPECIMEN);
                }
                break;
            case EXIT_PICK_WALL_SPECIMEN:
                pickWallSpecimenComplete = true;
                break;
        }
        return pickWallSpecimenComplete;
    }






    public void resetPickSampleState() {
        setPickSampleState(PickSampleState.START);
        pickSampleComplete = false;
    }

    private void setPickSampleState(PickSampleState newState) {
        pickSampleTimer.reset();
        currentPickSampleState = newState;
    }

    public boolean pickSample() {
        switch (currentPickSampleState) {
            case IDLE:
                //Do nothing
                break;
            case START:
                // Set pivot to pickup position
                pivot.setPosition(Pivot.PIVOT_PICKUP);
                setPickSampleState(PickSampleState.SETTING_PIVOT);
                break;
            case SETTING_PIVOT:
                // When pivot is at position, close claw, and move to next state
                if (pivot.isAtTarget()) {
                    claw.setPosition(Claw.CLAW_CLOSE);
                    setPickSampleState(PickSampleState.CLOSING_CLAW);
                }
                break;
            case CLOSING_CLAW:
                // Give time for the claw to grab the sample and then rotate wrist to clear the floor
                if (pickSampleTimer.milliseconds() > 225) {
                    wrist.setPosition(Wrist.WRIST_POSITION_CARRY);
                    setPickSampleState(PickSampleState.ROTATING_WRIST);
                }
                break;
            case ROTATING_WRIST:
                // Give time for the wrist to rotate backward with the sample to clear the floor; it may be desirable for this to be low or
                // zero (0) if there are no issues with the sample coming loose or breaking free with robot movement
                if (pickSampleTimer.milliseconds() > 10) {
                    setPickSampleState(PickSampleState.EXIT_PICK_SAMPLE);
                }
                break;
            case EXIT_PICK_SAMPLE:
                pickSampleComplete = true;
                break;
        }
        return pickSampleComplete;
    }

    public void resetHangSpecimenState(boolean setWristTimer) {
        hangingComplete = false;
        this.setWristTimer = setWristTimer;
        setHangSpecimenState(HangSpecimenOnChamberState.START);
    }

    private void setHangSpecimenState(HangSpecimenOnChamberState newState) {
        currentHangSpecimenOnChamberState = newState;
        hangSpecimenTimer.reset();
    }

    public boolean hangSpecimenUpdate() {
        localTelemetry.addData("ArmMech2 State", currentHangSpecimenOnChamberState.toString());
        localTelemetry.update();

        switch (currentHangSpecimenOnChamberState) {
            case IDLE:
                //Do Nothing
                break;
            case START:
                // Start by setting the wrist position the hang specimen position and start a timer
                // to give the wrist time to rotate before moving the pivot, else it will catch on the bar;
                // it may be desirable to set the wrist prior to running this state machine, in that case,
                // setWristTimer should be set to false when calling resetHangSpecimenState, and the pivot will begin
                // moving down immediately.
                if (setWristTimer) {
                    wrist.setPosition(Wrist.WRIST_POSITION_HANG);
                }
                setHangSpecimenState(HangSpecimenOnChamberState.SETTING_PIVOT);
                break;
            case SETTING_PIVOT:
                // if setWristTimer is true, then the state machine will give time for the wrist to move
                // into position and stabilize; else, the pivot will begin moving down immediately
                if (setWristTimer) {
                    if (hangSpecimenTimer.milliseconds() > 150) {
                        pivot.setPosition(Pivot.PIVOT_HANG_SPECIMEN_HIGH);
                        setHangSpecimenState(HangSpecimenOnChamberState.SETTING_WRIST);
                    }
                } else {
                    pivot.setPosition(Pivot.PIVOT_HANG_SPECIMEN_HIGH);
                    setHangSpecimenState(HangSpecimenOnChamberState.SETTING_WRIST);
                }
                break;
            case SETTING_WRIST:
                // Wait for pivot to reach set position and then give pivot time to stabilize
                if (pivot.isAtTarget()) {
                    setHangSpecimenState(HangSpecimenOnChamberState.PIVOT_STABILIZING);
                }
                break;
            case PIVOT_STABILIZING:
                // Give pivot time to stabilize before retracting slider
                if (hangSpecimenTimer.milliseconds() > 100) {
                    slider.setPosition(Slider2.EXTEND_HANG_SPECIMEN_HIGH_RELEASE, false);
                    setHangSpecimenState(HangSpecimenOnChamberState.RETRACTING_SLIDER_TO_CLAW_RELEASE_POSITION);
                }
                break;
            case RETRACTING_SLIDER_TO_CLAW_RELEASE_POSITION:
                // When slider reaches the specimen release position, open the claw to release the specimen
                // and start a timer to ensure enough time for the claw to open and the specimen to swing clear
                if (slider.isAtTarget()) {
                    claw.setPosition(Claw.CLAW_OPEN);
                    setHangSpecimenState(HangSpecimenOnChamberState.OPENING_CLAW);
                }
                break;
            case OPENING_CLAW:
                // Timer to ensure time for claw to open and specimen to swing clear; when timer is finished,
                // move slider to home position and rotate the wrist to clear the submersible
                if (hangSpecimenTimer.milliseconds() > 10) {
                    slider.setPosition(Slider2.EXTEND_HOME, false);
                    wrist.setPosition(Wrist.WRIST_POSITION_PICKUP);
                    setHangSpecimenState(HangSpecimenOnChamberState.SETTING_SLIDER_AND_WRIST_TO_FINISH_POSITION);
                }
                break;
            case SETTING_SLIDER_AND_WRIST_TO_FINISH_POSITION:
                // Wait until slider is in home position and enough time has elapsed such that the wrist is finished moving; this is to
                // ensure the wrist is pulled back quickly in auton to prevent it from hitting the submersible structure when moving to pick
                // the first yellow sample
                if (slider.isAtTarget() && hangSpecimenTimer.milliseconds() > 20) {
                    //wrist.setPosition(Wrist.WRIST_POSITION_PICKUP);
                    setHangSpecimenState(HangSpecimenOnChamberState.EXIT_HANG_SPECIMEN);
                }
                break;
            case EXIT_HANG_SPECIMEN:
                hangingComplete = true;
                break;
        }
        return hangingComplete;
    }
}
