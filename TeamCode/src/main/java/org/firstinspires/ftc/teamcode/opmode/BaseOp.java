package org.firstinspires.ftc.teamcode.opmode;

import android.app.Activity;
import android.util.Pair;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.subsystem.util.PoseStorage;
import org.firstinspires.ftc.teamcode.subsystem.RobotSuperDwarka;

import java.util.ArrayList;
import java.util.List;

public abstract class BaseOp extends LinearOpMode {

    public enum AllianceSelection {
        BLUE,
        RED,
        NONE
    }


    public enum StartingPosition {
        BLUE_BASKET,
        BLUE_HUMAN,
        RED_BASKET,
        RED_HUMAN
    }

    protected static StartingPosition mStartingPosition = StartingPosition.BLUE_BASKET;

    protected RobotSuperDwarka robot;

    // lifecycle management
    protected AppUtil appUtil = AppUtil.getInstance();
    protected final Activity activity;
    protected OpModeManagerImpl opModeManager = null;
    protected OpModeNotifications opModeNotifications = new OpModeNotifications();

    // Provides OpMode RunTime after pressing start button, will return null if not started
    protected ElapsedTime mRunTime;

    // Stores list of check points to be used in auto to measure performance
    protected List<Pair<String, Double>> mCheckPoints = new ArrayList<>();

    // these two variables are used to keep track of opmode sequence to allow for proper localizer
    // initialization - first we need to set global heading in auto and then use it in teleop
    // global heading should be reset if teleops are run sequentially during driver practice
    protected static boolean mPrevOpModeIsAuto = false;
    protected static StartingPosition mPrevStartingPosition = null;
    protected static boolean mCurrentOpModeIsAuto = false;

    protected BaseOp() {
        activity = appUtil.getActivity();
        registerLifeCycleCallbacks();
    }

    protected void initialize() {
        // Preserve pivot and slider encoder values if coming from Auton
        boolean preserveEncoders = mPrevOpModeIsAuto;
        robot = new RobotSuperDwarka(this);
        robot.initialize(preserveEncoders);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    protected static AllianceSelection getAlliance(@NonNull StartingPosition position) {
        switch (position) {
            case BLUE_BASKET:
            case BLUE_HUMAN:
                return AllianceSelection.BLUE;
            case RED_BASKET:
            case RED_HUMAN:
                return AllianceSelection.RED;
            default:
                return AllianceSelection.NONE;
        }
    }

    //----------------------------------------------------------------------------------------------
    // Activity life cycles
    //----------------------------------------------------------------------------------------------
    protected void close() {  // Must be idempotent. Callable from ANY thread
        //Log.i(TAG, "OpMode is closed");

        //sendCheckpointsToDashboard();

        if (robot != null) {
            PoseStorage.currentPose = robot.getPoseEstimate();
        }
        unregisterLifeCycleCallbacks();
    }

    protected void registerLifeCycleCallbacks() {
        this.opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(this.activity);
        if (this.opModeManager != null) {
            this.opModeManager.registerListener(this.opModeNotifications);
        }
    }

    protected void unregisterLifeCycleCallbacks() {
        if (this.opModeManager != null) {
            try {
                this.opModeManager.unregisterListener(this.opModeNotifications);
            } catch (Exception e) {
                telemetry.addData("Warning", "Failed to unregister OpModeListener: " + e.getMessage());
                telemetry.update();
            }
        }
    }

    protected class OpModeNotifications implements OpModeManagerNotifier.Notifications {
        @Override
        public void onOpModePreInit(OpMode opMode) {
            mPrevOpModeIsAuto = mCurrentOpModeIsAuto;
            mCurrentOpModeIsAuto = (opMode instanceof BaseAutoOp);
        }

        @Override
        public void onOpModePreStart(OpMode opMode) {
            // Start runtime timer
            mRunTime = new ElapsedTime();
        }

        @Override
        public void onOpModePostStop(OpMode opMode) {
            // We automatically shut down after the opmode (in which we are started) stops.
            close();
        }
    }
    //----------------------------------------------------------------------------------------------
    // End activity life cycles
    //----------------------------------------------------------------------------------------------
}
