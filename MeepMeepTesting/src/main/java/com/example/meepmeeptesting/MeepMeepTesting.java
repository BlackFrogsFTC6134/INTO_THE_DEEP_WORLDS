package com.example.meepmeeptesting;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
/*
public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(53, 65, Math.toRadians(180), Math.toRadians(180), 9.6)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(16.75, -64.75, Math.toRadians(90)))
                        .splineTo(new Vector2d(10.0, -27.0), Math.toRadians(90))
                        //.forward(30)
                        //.turn(Math.toRadians(90))
                        //.forward(30)
                        //.turn(Math.toRadians(90))
                        //.forward(30)
                        //.turn(Math.toRadians(90))
                        //.forward(30)
                        //.turn(Math.toRadians(90))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_OFFICIAL)//FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
*/
public class MeepMeepTesting {
    public static double BOT_WIDTH = 12.25;
    public static double BOT_LENGTH = 18.0;
    public static double TRACK_WIDTH = 8.75; // inches
    public static final double MAX_RPM = 435;
    public static double GEAR_RATIO = 1;
    public static double WHEEL_RADIUS = (104.0 / 2.0) / 25.4; // 104mm diameter wheel
    public static double RATE_LIMITER = 0.80;
    public static double MAX_VEL =  ((MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * 2 * PI) * RATE_LIMITER;
    public static double MAX_ACCEL = ((MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * 2 * PI) * RATE_LIMITER;
    public static double MAX_ANG_VEL = 6 * RATE_LIMITER;
    public static double MAX_ANG_ACCEL = Math.toRadians(180);
    public static volatile double INITIAL_SPECIMEN_HANG_X = 5.75; //5.0;
    public static volatile double INITIAL_SPECIMEN_HANG_Y = 39.25; //40.0;
    public static volatile double INITIAL_SPECIMEN_HANG_HEADING = Math.toRadians(-90);
    static final double PRESET_SPACING = 10;
    static final double LAST_SPECIMEN_MARGIN = 4.0;
    static final double PRESET_START_X = 34.5;
    static final double PRESET_LOW_POINT = 14.0;
    static final double PRESET_HIGH_POINT = 48;
    static final double PRESET_START_Y = 38; // need to push it out a little, so it does not catch on submersible

    static final double OBSERVATION_ZONE_PICK_X = -36.5;
    static final double OBSERVATION_ZONE_PICK_Y = 62;
    static final double OBSERVATION_ZONE_PICK_HEADING = Math.toRadians(-90);

    protected final static double NEUTRAL_SAMPLE_1_PICK_X = 47.5;
    protected final static double NEUTRAL_SAMPLE_1_PICK_Y = 38.4;
    protected final static double NEUTRAL_SAMPLE_1_PICK_HEADING = Math.toRadians(-90);
    protected final static double NEUTRAL_SAMPLE_2_PICK_X = 57.5;
    protected final static double NEUTRAL_SAMPLE_2_PICK_Y = 38.3;
    protected final static double NEUTRAL_SAMPLE_2_PICK_HEADING = Math.toRadians(-90);
    protected final static double NEUTRAL_SAMPLE_3_ALIGN_X = 48.5; //52
    protected final static double NEUTRAL_SAMPLE_3_PICK_X = 52.1; //52
    protected final static double NEUTRAL_SAMPLE_3_ALIGN_Y = 25.2; //25.7
    protected final static double NEUTRAL_SAMPLE_3_PICK_Y = 52.1; //52
    protected final static double NEUTRAL_SAMPLE_3_ALIGN_HEADING = Math.toRadians(0);
    protected final static double WALL_ALIGN_X = 37.0;
    protected final static double WALL_ALIGN_Y = 48.0;
    protected final static double WALL_PICK_Y = 56.6;
    protected final static double WALL_ALIGN_HEADING = Math.toRadians(90);
    protected final static double ZONE_DROP_X = 54.0;
    protected final static double ZONE_DROP_Y = 48.0;
    protected final static double ZONE_DROP_HEADING = Math.toRadians(90);
    protected final static double ALIGN_DELTA_SUBMERSIBLE_PARK = 3.5;

    protected final static int START_POSE_BASKET_X = 15;
    protected final static int START_POSE_BASKET_Y = 63;
    protected final static double START_POSE_HEADING = Math.toRadians(-90);

    protected final static double BASKET_DROP_X = 50.0;
    protected final static double BASKET_DROP_Y = 50.0;
    protected final static double BASKET_DROP_HEADING = Math.toRadians(-135);
    protected final static double SUBMERSIBLE_PARK_X = 35; //24;
    protected final static double SUBMERSIBLE_PARK_Y = 14; //13;
    //protected final static double SUBMERSIBLE_PARK_HEADING = PI;

    static final int allianceMultiplier = -1; // -1 for blue, 1 for red
    static Pose2d specimenPick = new Pose2d(-allianceMultiplier*OBSERVATION_ZONE_PICK_X, -allianceMultiplier*OBSERVATION_ZONE_PICK_Y, -allianceMultiplier*OBSERVATION_ZONE_PICK_HEADING);
    static Pose2d submersiblePark = new Pose2d(-allianceMultiplier*SUBMERSIBLE_PARK_X, -allianceMultiplier*SUBMERSIBLE_PARK_Y, allianceMultiplier == -1 ? PI : 0);

    //static Pose2d thirdNeutralSampleAlign = new Pose2d(NEUTRAL_SAMPLE_3_ALIGN_X, NEUTRAL_SAMPLE_3_ALIGN_Y, NEUTRAL_SAMPLE_3_ALIGN_HEADING + (allianceMultiplier == -1 ? 0 : Math.toRadians(180)));
    //static Pose2d thirdNeutralSamplePick = new Pose2d(NEUTRAL_SAMPLE_3_PICK_X, thirdNeutralSampleAlign.getY(), thirdNeutralSampleAlign.getHeading());
    //static Pose2d thirdNeutralSampleAlign = new Pose2d(-allianceMultiplier*59.0, -allianceMultiplier*26.5, allianceMultiplier == -1 ? 0 : PI); // with slight offset to rotate the sample

    public static double SPECIMEN_HANG_OFFSET = 6.0;
    public static int hangIndex = 0;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot;


        //myBot = hangInitialSpecimenHuman(meepMeep, 0);
        //myBot = pickFirstAllianceSample(meepMeep);
        //myBot = dropFirstAllianceSampleInZone(meepMeep);
        //myBot = pickSecondAllianceSample(meepMeep);
        //myBot = dropSecondAllianceSampleInZone(meepMeep);
        //myBot = pickThirdAllianceSample(meepMeep);
        //myBot = dropThirdAllianceSampleInZone(meepMeep);
        //myBot = hangSecondSpecimenHuman(meepMeep);
        myBot = specimenToWallPick((meepMeep));
        //myBot = pushThreeSamplesSpline(meepMeep);

        //myBot = hangInitialSpecimenBasket(meepMeep, 0);
        //myBot = pickFirstNeutral(meepMeep);
        //myBot = dropFirstYellowInBasket(meepMeep);
        //myBot = pickSecondNeutral(meepMeep);
        //myBot = dropSecondYellowInBasket(meepMeep);
        //myBot = pickThirdNeutral(meepMeep);
        //myBot = dropThirdYellowInBasket(meepMeep);
        //myBot = parkSubmersible(meepMeep);

        // original
        //myBot = pushThreeSamples(meepMeep, false);


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.85f)
                .addEntity(myBot)
                .start();
    }

    public static RoadRunnerBotEntity hangSecondSpecimenHuman(MeepMeep meepMeep) {
        // hang pose spline
        //Pose2d hangPose = new Pose2d(-INITIAL_SPECIMEN_HANG_X - allianceMultiplier*SPECIMEN_HANG_OFFSET*hangIndex, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y, allianceMultiplier*INITIAL_SPECIMEN_HANG_HEADING);

        // hang pose line
        Pose2d hangPose = new Pose2d(allianceMultiplier*INITIAL_SPECIMEN_HANG_X - allianceMultiplier*SPECIMEN_HANG_OFFSET, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y, -allianceMultiplier*INITIAL_SPECIMEN_HANG_HEADING);
        //Vector2d hangPose = new Vector2d(allianceMultiplier*INITIAL_SPECIMEN_HANG_X - allianceMultiplier*SPECIMEN_HANG_OFFSET, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y, -allianceMultiplier*INITIAL_SPECIMEN_HANG_HEADING)
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(allianceMultiplier*WALL_ALIGN_X, -allianceMultiplier*WALL_PICK_Y, -allianceMultiplier*WALL_ALIGN_HEADING))

                        .setReversed(false)
                        //.back(6)
                        //.turn(-PI/2)
                        //.splineTo(new Vector2d(allianceMultiplier * WALL_ALIGN_X, -allianceMultiplier * WALL_PICK_Y - 6), PI)
                        //.splineToConstantHeading(new Vector2d(allianceMultiplier*WALL_ALIGN_X, -allianceMultiplier*WALL_PICK_Y - 6), -allianceMultiplier*WALL_ALIGN_HEADING)
                        //.lineToSplineHeading(new Pose2d(allianceMultiplier*WALL_ALIGN_X, -allianceMultiplier*WALL_PICK_Y - 6, -allianceMultiplier*WALL_ALIGN_HEADING))
                        .lineToSplineHeading(new Pose2d(allianceMultiplier*WALL_ALIGN_X, -allianceMultiplier*WALL_PICK_Y - 6, allianceMultiplier == -1 ? 0 : PI))
                        //.splineTo(new Vector2d(hangPose.getX(), hangPose.getY()), hangPose.getHeading()) // 1.40, 1.47, 1.55
                        .splineToLinearHeading(new Pose2d(hangPose.getX(), hangPose.getY(), hangPose.getHeading()), hangPose.getHeading())

                        //.lineToLinearHeading(hangPose) // 1.28, 1.32, 1.38, 1.44, 1.49

                        .build());
        return myBot;
    }

    public static RoadRunnerBotEntity specimenToWallPick(MeepMeep meepMeep) {
        // hang pose spline
        //Pose2d hangPose = new Pose2d(-INITIAL_SPECIMEN_HANG_X - allianceMultiplier*SPECIMEN_HANG_OFFSET*hangIndex, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y, allianceMultiplier*INITIAL_SPECIMEN_HANG_HEADING);

        // hang pose line
        Pose2d hangPose = new Pose2d(allianceMultiplier*INITIAL_SPECIMEN_HANG_X - allianceMultiplier*SPECIMEN_HANG_OFFSET, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y, -allianceMultiplier*INITIAL_SPECIMEN_HANG_HEADING);
        //Vector2d hangPose = new Vector2d(allianceMultiplier*INITIAL_SPECIMEN_HANG_X - allianceMultiplier*SPECIMEN_HANG_OFFSET, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y, -allianceMultiplier*INITIAL_SPECIMEN_HANG_HEADING)
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(allianceMultiplier*INITIAL_SPECIMEN_HANG_X - allianceMultiplier*SPECIMEN_HANG_OFFSET, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y, -allianceMultiplier*INITIAL_SPECIMEN_HANG_HEADING))

                        .setReversed(false)
                        //.turn(-PI/2)
                        //.splineTo(new Vector2d(allianceMultiplier*INITIAL_SPECIMEN_HANG_X + 8, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y + 10), Math.toRadians(180))
                        //.lineToLinearHeading(new Pose2d(allianceMultiplier*WALL_ALIGN_X, -allianceMultiplier*WALL_PICK_Y, -allianceMultiplier*WALL_ALIGN_HEADING))
                        .lineToLinearHeading(new Pose2d(allianceMultiplier*WALL_ALIGN_X, -allianceMultiplier*WALL_ALIGN_Y, -allianceMultiplier*WALL_ALIGN_HEADING))
                        //.splineTo(new Vector2d(allianceMultiplier*WALL_ALIGN_X, -allianceMultiplier*WALL_ALIGN_Y), -allianceMultiplier*WALL_ALIGN_HEADING)
                        //.splineTo(new Vector2d(allianceMultiplier*WALL_ALIGN_X, -allianceMultiplier*WALL_PICK_Y), -allianceMultiplier*WALL_ALIGN_HEADING)
                        .lineTo(new Vector2d(allianceMultiplier*WALL_ALIGN_X, -allianceMultiplier*WALL_PICK_Y))
                        //.back(6)
                        //.turn(-PI/2)
                        //.splineTo(new Vector2d(allianceMultiplier * WALL_ALIGN_X, -allianceMultiplier * WALL_PICK_Y - 6), PI)
                        //.splineToConstantHeading(new Vector2d(allianceMultiplier*WALL_ALIGN_X, -allianceMultiplier*WALL_PICK_Y - 6), -allianceMultiplier*WALL_ALIGN_HEADING)
                        //.lineToSplineHeading(new Pose2d(allianceMultiplier*WALL_ALIGN_X, -allianceMultiplier*WALL_PICK_Y - 6, -allianceMultiplier*WALL_ALIGN_HEADING))
                        //.lineToSplineHeading(new Pose2d(allianceMultiplier*WALL_ALIGN_X, -allianceMultiplier*WALL_PICK_Y - 6, allianceMultiplier == -1 ? 0 : PI))
                        //.splineTo(new Vector2d(hangPose.getX(), hangPose.getY()), hangPose.getHeading()) // 1.40, 1.47, 1.55
                        //.splineToLinearHeading(new Pose2d(hangPose.getX(), hangPose.getY(), hangPose.getHeading()), hangPose.getHeading())

                        //.lineToLinearHeading(hangPose) // 1.28, 1.32, 1.38, 1.44, 1.49

                        .build());
        return myBot;
    }


    public static RoadRunnerBotEntity parkSubmersible(MeepMeep meepMeep) {
        Pose2d basketDrop = new Pose2d(-allianceMultiplier*BASKET_DROP_X, -allianceMultiplier*BASKET_DROP_Y, allianceMultiplier == -1 ? -PI*3/4 : PI/4);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(basketDrop)
                        .lineToLinearHeading(new Pose2d(submersiblePark.getX(), submersiblePark.getY(), submersiblePark.getHeading()))
                        //.splineTo(new Vector2d(submersiblePark.getX() - allianceMultiplier*ALIGN_DELTA_SUBMERSIBLE_PARK , submersiblePark.getY()), submersiblePark.getHeading())
                        //.splineTo(new Vector2d(submersiblePark.getX(), submersiblePark.getY()), submersiblePark.getHeading())
                        .build());

        return myBot;
    }

    public static RoadRunnerBotEntity hangInitialSpecimenBasket(MeepMeep meepMeep, int hangIndex) {
        // hang pose spline
        //Pose2d hangPose = new Pose2d(-INITIAL_SPECIMEN_HANG_X - allianceMultiplier*SPECIMEN_HANG_OFFSET*hangIndex, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y, allianceMultiplier*INITIAL_SPECIMEN_HANG_HEADING);

        // hang pose line
        Vector2d hangPose = new Vector2d(-allianceMultiplier*INITIAL_SPECIMEN_HANG_X - allianceMultiplier*SPECIMEN_HANG_OFFSET*hangIndex, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-allianceMultiplier*START_POSE_BASKET_X, -allianceMultiplier*START_POSE_BASKET_Y, -allianceMultiplier*START_POSE_HEADING))

                        //.setReversed(true)
                        //.splineTo(new Vector2d(hangPose.getX(), hangPose.getY()), hangPose.getHeading()) // 1.40, 1.47, 1.55

                        .lineTo(hangPose) // 1.28, 1.32, 1.38, 1.44, 1.49

                        .build());
        return myBot;
    }

    public static RoadRunnerBotEntity hangInitialSpecimenHuman(MeepMeep meepMeep, int hangIndex) {
        // hang pose spline
        //Pose2d hangPose = new Pose2d(-INITIAL_SPECIMEN_HANG_X - allianceMultiplier*SPECIMEN_HANG_OFFSET*hangIndex, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y, allianceMultiplier*INITIAL_SPECIMEN_HANG_HEADING);

        // hang pose line
        Vector2d hangPose = new Vector2d(allianceMultiplier*INITIAL_SPECIMEN_HANG_X - allianceMultiplier*SPECIMEN_HANG_OFFSET*hangIndex, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(allianceMultiplier*START_POSE_BASKET_X, -allianceMultiplier*START_POSE_BASKET_Y, -allianceMultiplier*START_POSE_HEADING))

                        //.setReversed(true)
                        //.splineTo(new Vector2d(hangPose.getX(), hangPose.getY()), hangPose.getHeading()) // 1.40, 1.47, 1.55

                        .lineTo(hangPose) // 1.28, 1.32, 1.38, 1.44, 1.49

                        .build());
        return myBot;
    }



    public static RoadRunnerBotEntity pickFirstNeutral(MeepMeep meepMeep) {
        Pose2d initialSpecimenHang = new Pose2d(-allianceMultiplier*INITIAL_SPECIMEN_HANG_X, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y, -allianceMultiplier*INITIAL_SPECIMEN_HANG_HEADING);
        Pose2d basketDrop = new Pose2d(-allianceMultiplier*BASKET_DROP_X, -allianceMultiplier*BASKET_DROP_Y, -allianceMultiplier*BASKET_DROP_HEADING);
        Pose2d firstNeutralSamplePick = new Pose2d(-allianceMultiplier*NEUTRAL_SAMPLE_1_PICK_X, -allianceMultiplier*NEUTRAL_SAMPLE_1_PICK_Y, -allianceMultiplier*NEUTRAL_SAMPLE_1_PICK_HEADING);
        Pose2d firstNeutralSampleAlign = new Pose2d(firstNeutralSamplePick.getX(), firstNeutralSamplePick.getY() + ALIGN_DELTA_SUBMERSIBLE_PARK, firstNeutralSamplePick.getHeading());

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(initialSpecimenHang)

                        .lineTo(new Vector2d(firstNeutralSamplePick.getX(), firstNeutralSamplePick.getY()))
                        //.splineTo(new Vector2d(firstNeutralSampleAlign.getX(), firstNeutralSampleAlign.getY()), firstNeutralSampleAlign.getHeading())
                        //.splineTo(new Vector2d(firstNeutralSamplePick.getX(), firstNeutralSamplePick.getY()), firstNeutralSamplePick.getHeading())
                        .build());

        return myBot;
    }

    public static RoadRunnerBotEntity pickFirstAllianceSample(MeepMeep meepMeep) {
        Pose2d initialSpecimenHang = new Pose2d(allianceMultiplier*INITIAL_SPECIMEN_HANG_X, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y, -allianceMultiplier*INITIAL_SPECIMEN_HANG_HEADING);
        Pose2d firstNeutralSamplePick = new Pose2d(allianceMultiplier*NEUTRAL_SAMPLE_1_PICK_X, -allianceMultiplier*NEUTRAL_SAMPLE_1_PICK_Y, -allianceMultiplier*NEUTRAL_SAMPLE_1_PICK_HEADING);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(initialSpecimenHang)

                        .lineTo(new Vector2d(firstNeutralSamplePick.getX(), firstNeutralSamplePick.getY()))
                        //.splineTo(new Vector2d(firstNeutralSampleAlign.getX(), firstNeutralSampleAlign.getY()), firstNeutralSampleAlign.getHeading())
                        //.splineTo(new Vector2d(firstNeutralSamplePick.getX(), firstNeutralSamplePick.getY()), firstNeutralSamplePick.getHeading())
                        .build());

        return myBot;
    }

    public static RoadRunnerBotEntity dropFirstAllianceSampleInZone(MeepMeep meepMeep) {
        Pose2d zoneDrop = new Pose2d(allianceMultiplier*ZONE_DROP_X, -allianceMultiplier*ZONE_DROP_Y, -allianceMultiplier*ZONE_DROP_HEADING);
        Pose2d firstNeutralSamplePick = new Pose2d(allianceMultiplier*NEUTRAL_SAMPLE_1_PICK_X, -allianceMultiplier*NEUTRAL_SAMPLE_1_PICK_Y, -allianceMultiplier*NEUTRAL_SAMPLE_1_PICK_HEADING);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(firstNeutralSamplePick)
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(zoneDrop.getX(), zoneDrop.getY(), zoneDrop.getHeading()))
                        //.splineTo(new Vector2d(zoneDrop.getX(), zoneDrop.getY()), zoneDrop.getHeading())//+ (allianceMultiplier==-1?PI:PI/2))
                        .build());

        return myBot;
    }

    public static RoadRunnerBotEntity pickSecondAllianceSample(MeepMeep meepMeep) {
        Pose2d zoneDrop = new Pose2d(allianceMultiplier*ZONE_DROP_X, -allianceMultiplier*ZONE_DROP_Y, -allianceMultiplier*ZONE_DROP_HEADING);
        Pose2d secondNeutralSamplePick = new Pose2d(allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_X, -allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_Y, -allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_HEADING);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(zoneDrop)
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(secondNeutralSamplePick.getX(), secondNeutralSamplePick.getY(), secondNeutralSamplePick.getHeading()))
                        //.splineTo(new Vector2d(secondNeutralSamplePick.getX(), secondNeutralSamplePick.getY()), secondNeutralSamplePick.getHeading())// + (allianceMultiplier==-1?Math.toRadians(0):Math.toRadians(90)))
                        .build());

        return myBot;
    }

    public static RoadRunnerBotEntity dropSecondAllianceSampleInZone(MeepMeep meepMeep) {
        Pose2d zoneDrop = new Pose2d(allianceMultiplier*ZONE_DROP_X, -allianceMultiplier*ZONE_DROP_Y, -allianceMultiplier*ZONE_DROP_HEADING);
        Pose2d secondNeutralSamplePick = new Pose2d(allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_X, -allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_Y, -allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_HEADING);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(secondNeutralSamplePick)
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(zoneDrop.getX(), zoneDrop.getY(), zoneDrop.getHeading()))
                        //.splineTo(new Vector2d(zoneDrop.getX(), zoneDrop.getY()), zoneDrop.getHeading())//+ (allianceMultiplier==-1?PI:PI/2))
                        .build());

        return myBot;
    }

    public static RoadRunnerBotEntity pickThirdAllianceSample(MeepMeep meepMeep) {
        Pose2d zoneDrop = new Pose2d(allianceMultiplier*ZONE_DROP_X, -allianceMultiplier*ZONE_DROP_Y, -allianceMultiplier*ZONE_DROP_HEADING);
        //Pose2d thirdNeutralSamplePick = new Pose2d(allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_X, -allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_Y, -allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_HEADING+.0000001);
        //Pose2d thirdNeutralSamplePick = new Pose2d(allianceMultiplier*NEUTRAL_SAMPLE_3_PICK_X, -allianceMultiplier*NEUTRAL_SAMPLE_3_PICK_Y, allianceMultiplier*NEUTRAL_SAMPLE_3_PICK_HEADING + (allianceMultiplier == 1 ? 0 : Math.toRadians(180)));

        Pose2d thirdNeutralSampleAlign = new Pose2d(allianceMultiplier*NEUTRAL_SAMPLE_3_ALIGN_X, -allianceMultiplier*NEUTRAL_SAMPLE_3_ALIGN_Y, -allianceMultiplier*NEUTRAL_SAMPLE_3_ALIGN_HEADING + (allianceMultiplier == -1 ? Math.toRadians(180) : 0));
        Pose2d thirdNeutralSamplePick = new Pose2d(allianceMultiplier*NEUTRAL_SAMPLE_3_PICK_X, thirdNeutralSampleAlign.getY(), thirdNeutralSampleAlign.getHeading());

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(zoneDrop)
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(thirdNeutralSampleAlign.getX(), thirdNeutralSampleAlign.getY(), thirdNeutralSampleAlign.getHeading()))
                        //.splineTo(new Vector2d(secondNeutralSamplePick.getX(), secondNeutralSamplePick.getY()), secondNeutralSamplePick.getHeading())// + (allianceMultiplier==-1?Math.toRadians(0):Math.toRadians(90)))
                        .lineTo(new Vector2d(thirdNeutralSamplePick.getX(), thirdNeutralSamplePick.getY()))
                        .build());

        return myBot;
    }

    public static RoadRunnerBotEntity dropThirdAllianceSampleInZone(MeepMeep meepMeep) {
        Pose2d zoneDrop = new Pose2d(allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_X + (allianceMultiplier*2.5), -allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_Y + (-allianceMultiplier*10), -allianceMultiplier*ZONE_DROP_HEADING);
        Pose2d wallPickAlign = new Pose2d(allianceMultiplier*WALL_ALIGN_X, -allianceMultiplier*WALL_PICK_Y, -allianceMultiplier*ZONE_DROP_HEADING);
        Pose2d wallPick = new Pose2d(allianceMultiplier*WALL_ALIGN_X, -allianceMultiplier*WALL_PICK_Y + (-allianceMultiplier*6), -allianceMultiplier*ZONE_DROP_HEADING);

        Pose2d thirdNeutralSampleAlign = new Pose2d(allianceMultiplier*NEUTRAL_SAMPLE_3_ALIGN_X, -allianceMultiplier*NEUTRAL_SAMPLE_3_ALIGN_Y, -allianceMultiplier*NEUTRAL_SAMPLE_3_ALIGN_HEADING + (allianceMultiplier == -1 ? Math.toRadians(180) : 0));
        Pose2d thirdNeutralSamplePick = new Pose2d(allianceMultiplier*NEUTRAL_SAMPLE_3_PICK_X, thirdNeutralSampleAlign.getY(), thirdNeutralSampleAlign.getHeading());
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(thirdNeutralSamplePick)
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(wallPickAlign.getX(), wallPickAlign.getY(), wallPickAlign.getHeading()))
                        //.splineTo(new Vector2d(zoneDrop.getX(), zoneDrop.getY()), zoneDrop.getHeading())//+ (allianceMultiplier==-1?PI:PI/2))
                        .lineTo(new Vector2d(wallPick.getX(), wallPick.getY()))
                        .build());

        return myBot;
    }

    public static RoadRunnerBotEntity dropFirstYellowInBasket(MeepMeep meepMeep) {
        Pose2d basketDrop = new Pose2d(-allianceMultiplier*BASKET_DROP_X, -allianceMultiplier*BASKET_DROP_Y, -allianceMultiplier*BASKET_DROP_HEADING);
        Pose2d firstNeutralSamplePick = new Pose2d(-allianceMultiplier*NEUTRAL_SAMPLE_1_PICK_X, -allianceMultiplier*NEUTRAL_SAMPLE_1_PICK_Y, -allianceMultiplier*NEUTRAL_SAMPLE_1_PICK_HEADING);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(firstNeutralSamplePick)
                        .setReversed(true)
                        .splineTo(new Vector2d(basketDrop.getX(), basketDrop.getY()), basketDrop.getHeading() + (allianceMultiplier==-1?PI:PI/2))
                        .build());

        return myBot;
    }

    public static RoadRunnerBotEntity pickSecondNeutral(MeepMeep meepMeep) {
        Pose2d basketDrop = new Pose2d(-allianceMultiplier*BASKET_DROP_X, -allianceMultiplier*BASKET_DROP_Y, -allianceMultiplier*BASKET_DROP_HEADING + (allianceMultiplier==-1?Math.toRadians(0):Math.toRadians(-90)));
        //Pose2d secondNeutralSampleAlign = new Pose2d(-allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_X, -allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_Y, -allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_HEADING);// + (allianceMultiplier==-1?Math.toRadians(0):Math.toRadians(90)));
        Pose2d secondNeutralSamplePick = new Pose2d(-allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_X, -allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_Y, -allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_HEADING);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(basketDrop)
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(secondNeutralSamplePick.getX(), secondNeutralSamplePick.getY(), secondNeutralSamplePick.getHeading()))
                        //.splineTo(new Vector2d(secondNeutralSamplePick.getX(), secondNeutralSamplePick.getY()), secondNeutralSamplePick.getHeading())// + (allianceMultiplier==-1?Math.toRadians(0):Math.toRadians(90)))
                        .build());

        return myBot;
    }

    public static RoadRunnerBotEntity dropSecondYellowInBasket(MeepMeep meepMeep) {
        Pose2d basketDrop = new Pose2d(-allianceMultiplier*BASKET_DROP_X, -allianceMultiplier*BASKET_DROP_Y, -allianceMultiplier*BASKET_DROP_HEADING);
        Pose2d secondNeutralSamplePick = new Pose2d(-allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_X, -allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_Y, -allianceMultiplier*NEUTRAL_SAMPLE_2_PICK_HEADING);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(secondNeutralSamplePick)
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(basketDrop.getX(), basketDrop.getY(), basketDrop.getHeading() + (allianceMultiplier==-1?Math.toRadians(0):Math.toRadians(-90))))
                        //.splineTo(new Vector2d(basketDrop.getX(), basketDrop.getY()), basketDrop.getHeading()+ (allianceMultiplier==-1?PI:PI/2))
                        .build());

        return myBot;
    }


    public static RoadRunnerBotEntity pickThirdNeutral(MeepMeep meepMeep) {
        Pose2d basketDrop = new Pose2d(-allianceMultiplier*BASKET_DROP_X, -allianceMultiplier*BASKET_DROP_Y, -allianceMultiplier*BASKET_DROP_HEADING + (allianceMultiplier==-1?Math.toRadians(0):Math.toRadians(-90)));
        //Pose2d thirdNeutralSampleAlign = new Pose2d(-allianceMultiplier*NEUTRAL_SAMPLE_3_ALIGN_X, -allianceMultiplier*NEUTRAL_SAMPLE_3_ALIGN_Y, -allianceMultiplier*NEUTRAL_SAMPLE_3_ALIGN_HEADING);
        //Pose2d thirdNeutralSampleAlign = new Pose2d(-allianceMultiplier*59.0, -allianceMultiplier*26.5, allianceMultiplier == -1 ? 0 : PI);
        //Pose2d thirdNeutralSamplePick = new Pose2d(-allianceMultiplier*NEUTRAL_SAMPLE_3_PICK_X, -allianceMultiplier*NEUTRAL_SAMPLE_3_ALIGN_Y, -allianceMultiplier*NEUTRAL_SAMPLE_3_ALIGN_HEADING + (allianceMultiplier == -1 ? 0 : Math.toRadians(180)));

        Pose2d thirdNeutralSampleAlign = new Pose2d(-allianceMultiplier*NEUTRAL_SAMPLE_3_ALIGN_X, -allianceMultiplier*NEUTRAL_SAMPLE_3_ALIGN_Y, -allianceMultiplier*NEUTRAL_SAMPLE_3_ALIGN_HEADING + (allianceMultiplier == -1 ? 0 : Math.toRadians(180)));
        Pose2d thirdNeutralSamplePick = new Pose2d(-allianceMultiplier*NEUTRAL_SAMPLE_3_PICK_X, thirdNeutralSampleAlign.getY(), thirdNeutralSampleAlign.getHeading());


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(basketDrop)
                        .setReversed(false)
                        .lineToLinearHeading(new Pose2d(thirdNeutralSampleAlign.getX(), thirdNeutralSampleAlign.getY(), thirdNeutralSampleAlign.getHeading()))
                        //.splineTo(new Vector2d(secondNeutralSamplePick.getX(), secondNeutralSamplePick.getY()), secondNeutralSamplePick.getHeading())// + (allianceMultiplier==-1?Math.toRadians(0):Math.toRadians(90)))
                        .lineTo(new Vector2d(thirdNeutralSamplePick.getX(), thirdNeutralSamplePick.getY()))
                        .build());

        return myBot;
    }

    public static RoadRunnerBotEntity dropThirdYellowInBasket(MeepMeep meepMeep) {
        Pose2d basketDrop = new Pose2d(-allianceMultiplier*BASKET_DROP_X, -allianceMultiplier*BASKET_DROP_Y, -allianceMultiplier*BASKET_DROP_HEADING);
        //Pose2d thirdNeutralSamplePick = new Pose2d(-allianceMultiplier*NEUTRAL_SAMPLE_3_PICK_X, -allianceMultiplier*NEUTRAL_SAMPLE_3_ALIGN_Y, -allianceMultiplier*NEUTRAL_SAMPLE_3_ALIGN_HEADING);

        Pose2d thirdNeutralSampleAlign = new Pose2d(-allianceMultiplier*NEUTRAL_SAMPLE_3_ALIGN_X, -allianceMultiplier*NEUTRAL_SAMPLE_3_ALIGN_Y, -allianceMultiplier*NEUTRAL_SAMPLE_3_ALIGN_HEADING + (allianceMultiplier == -1 ? 0 : Math.toRadians(180)));
        Pose2d thirdNeutralSamplePick = new Pose2d(-allianceMultiplier*NEUTRAL_SAMPLE_3_PICK_X, thirdNeutralSampleAlign.getY(), thirdNeutralSampleAlign.getHeading());

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(thirdNeutralSamplePick)
                        .setReversed(true)
                        .lineToLinearHeading(new Pose2d(basketDrop.getX(), basketDrop.getY(), basketDrop.getHeading() + (allianceMultiplier==-1?Math.toRadians(0):Math.toRadians(-90))))
                        //.splineTo(new Vector2d(basketDrop.getX(), basketDrop.getY()), basketDrop.getHeading()+ (allianceMultiplier==-1?PI:PI/2))
                        .build());

        return myBot;
    }

    public static RoadRunnerBotEntity pushThreeSamples(MeepMeep meepMeep, boolean original) {
        Pose2d initialSpecimenHang = new Pose2d(allianceMultiplier*INITIAL_SPECIMEN_HANG_X, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y, allianceMultiplier*PI/2);
        //new Vector2d(allianceMultiplier*INITIAL_SPECIMEN_HANG_X - allianceMultiplier*SPECIMEN_HANG_OFFSET*hangIndex, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y);

        if (original) {
            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                    .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(initialSpecimenHang)
                            //.splineToConstantHeading(new Vector2d(PRESET_START_X * allianceMultiplier, -allianceMultiplier * PRESET_START_Y), allianceMultiplier * PI / 2)
                            //.lineToLinearHeading(new Pose2d(PRESET_START_X * allianceMultiplier, -allianceMultiplier * PRESET_START_Y, -allianceMultiplier * PI / 2)) // enter push sequence
                            .lineToSplineHeading(new Pose2d(PRESET_START_X * allianceMultiplier, -allianceMultiplier * PRESET_START_Y, -allianceMultiplier * PI / 2))

                            // Segment 1 - original
                            .splineToConstantHeading(new Vector2d(allianceMultiplier*PRESET_START_X, -allianceMultiplier*(PRESET_LOW_POINT + PRESET_SPACING/2)), allianceMultiplier*PI/2)  // 1.54 - S
                            .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + PRESET_SPACING), -allianceMultiplier*PRESET_LOW_POINT), allianceMultiplier*PI/2) // 1.78 - SW
                            .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + PRESET_SPACING), -allianceMultiplier*PRESET_HIGH_POINT), allianceMultiplier*PI/2)// 2.65 - N

                            //Segment 2 - original
                            .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + PRESET_SPACING), -allianceMultiplier*(PRESET_LOW_POINT + PRESET_SPACING/2)),allianceMultiplier*PI/2) // 3.33 - S
                            .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 2*PRESET_SPACING), -allianceMultiplier*PRESET_LOW_POINT), allianceMultiplier*PI/2) // 3.38 - SW
                            .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 2*PRESET_SPACING), -allianceMultiplier*PRESET_HIGH_POINT), allianceMultiplier*PI/2) // 4.18 - N

                            // Segment 3 - original
                            .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 2*PRESET_SPACING), -allianceMultiplier*(PRESET_LOW_POINT + PRESET_SPACING/2)),allianceMultiplier*PI/2) // 3.33 - S
                            .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 3*PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier*PRESET_LOW_POINT), allianceMultiplier*PI/2) // 3.38 - SW
                            .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 3*PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier*PRESET_HIGH_POINT), allianceMultiplier*PI/2) // 4.18 - N

                            // skip pull back
                            //.splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 3*PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier*(PRESET_HIGH_POINT - PRESET_SPACING)), allianceMultiplier*PI/2) // 4.18 - N
                            // short drive forward
                            //.splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 3*PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier*(PRESET_HIGH_POINT + PRESET_SPACING)), allianceMultiplier*PI/2) // 4.18 - N

                            //.setReversed(true)
                            //.splineToLinearHeading(specimenPick, -allianceMultiplier*PI/2) // 4.18 - N
                            //.splineToLinearHeading(specimenPick, -allianceMultiplier*PI/2) // 4.18 - N
                            //.splineToConstantHeading(new Vector2d(specimenPick.getX(), specimenPick.getY()), -allianceMultiplier*PI/2) // 4.18 - N
                            //.lineToLinearHeading(specimenPick)

                            .build());

            return myBot;
        } else {
            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                    .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(initialSpecimenHang)
                            //.lineToLinearHeading(new Pose2d(PRESET_START_X * allianceMultiplier, -allianceMultiplier * PRESET_START_Y, -allianceMultiplier * PI / 2)) // enter push sequence

                            // Segment 1
                            //.lineToSplineHeading(new Pose2d(allianceMultiplier * PRESET_START_X, -allianceMultiplier * (PRESET_LOW_POINT + PRESET_SPACING / 2), -allianceMultiplier * PI / 2))
                            //.splineToConstantHeading(new Vector2d(allianceMultiplier * (PRESET_START_X + PRESET_SPACING), -allianceMultiplier * PRESET_LOW_POINT), allianceMultiplier * PI / 2) // 1.78 - SW
                            //.lineToLinearHeading(new Pose2d(allianceMultiplier * (PRESET_START_X + PRESET_SPACING), -allianceMultiplier * PRESET_HIGH_POINT, -allianceMultiplier * PI / 2))

                            // Segment 2
                            //.lineToSplineHeading(new Pose2d(allianceMultiplier * (PRESET_START_X + PRESET_SPACING), -allianceMultiplier * (PRESET_LOW_POINT + PRESET_SPACING / 2), -allianceMultiplier * PI / 2))
                            //.splineToConstantHeading(new Vector2d(allianceMultiplier * (PRESET_START_X + 2 * PRESET_SPACING), -allianceMultiplier * PRESET_LOW_POINT), allianceMultiplier * PI / 2) // 3.38 - SW
                            //.lineToLinearHeading(new Pose2d(allianceMultiplier * (PRESET_START_X + 2 * PRESET_SPACING), -allianceMultiplier * PRESET_HIGH_POINT, -allianceMultiplier * PI / 2))

                            // Segment 3
                            //.lineToSplineHeading(new Pose2d(allianceMultiplier * (PRESET_START_X + 2 * PRESET_SPACING), -allianceMultiplier * (PRESET_LOW_POINT + PRESET_SPACING / 2), -allianceMultiplier * PI / 2))
                            //.splineToConstantHeading(new Vector2d(allianceMultiplier * (PRESET_START_X + 3 * PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier * PRESET_LOW_POINT), allianceMultiplier * PI / 2)
                            //.lineToLinearHeading(new Pose2d(allianceMultiplier * (PRESET_START_X + 3 * PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier * PRESET_HIGH_POINT, -allianceMultiplier * PI / 2))

                            // Updated version
                            //.lineToLinearHeading(new Pose2d(PRESET_START_X * allianceMultiplier, -allianceMultiplier * PRESET_START_Y, allianceMultiplier * PI / 2)) // enter push sequence
                            .lineToSplineHeading(new Pose2d(PRESET_START_X * allianceMultiplier, -allianceMultiplier*PRESET_START_Y, allianceMultiplier==-1?0:PI))
                            //.splineToConstantHeading(new Vector2d(PRESET_START_X * allianceMultiplier, -allianceMultiplier * PRESET_START_Y), -allianceMultiplier * PI / 2)

                            // Segment 1
                            //.lineToSplineHeading(new Pose2d(allianceMultiplier * PRESET_START_X, -allianceMultiplier * (PRESET_LOW_POINT + PRESET_SPACING / 2), -allianceMultiplier * PI / 2))
                            .lineToSplineHeading(new Pose2d(allianceMultiplier * (PRESET_START_X + PRESET_SPACING/2.5), -allianceMultiplier * PRESET_LOW_POINT, allianceMultiplier==-1?0:PI))
                            //.splineToConstantHeading(new Vector2d(allianceMultiplier * (PRESET_START_X + PRESET_SPACING), -allianceMultiplier * PRESET_LOW_POINT), allianceMultiplier * PI / 2) // 1.78 - SW
                            .splineToConstantHeading(new Vector2d(allianceMultiplier * (PRESET_START_X + PRESET_SPACING/1.2), -allianceMultiplier * PRESET_HIGH_POINT), allianceMultiplier==-1?0:PI)
                            //.lineToLinearHeading(new Pose2d(allianceMultiplier * (PRESET_START_X + PRESET_SPACING), -allianceMultiplier * PRESET_HIGH_POINT, -allianceMultiplier * PI / 2))

                            // Segment 2
                            //.lineToSplineHeading(new Pose2d(allianceMultiplier * (PRESET_START_X + PRESET_SPACING), -allianceMultiplier * (PRESET_LOW_POINT + PRESET_SPACING / 2), -allianceMultiplier * PI / 2))
                            .lineToSplineHeading(new Pose2d(allianceMultiplier * (PRESET_START_X + 1.5*PRESET_SPACING), -allianceMultiplier * (PRESET_LOW_POINT), allianceMultiplier==-1?0:PI))
                            //.splineToConstantHeading(new Vector2d(allianceMultiplier * (PRESET_START_X + 2 * PRESET_SPACING-5), -allianceMultiplier * PRESET_LOW_POINT), allianceMultiplier * PI / 2)

                            .splineToConstantHeading(new Vector2d(allianceMultiplier * (PRESET_START_X + 1.6*PRESET_SPACING), -allianceMultiplier * PRESET_HIGH_POINT), allianceMultiplier==-1?0:PI) // 3.38 - SW
                            //.lineToLinearHeading(new Pose2d(allianceMultiplier * (PRESET_START_X + 2 * PRESET_SPACING), -allianceMultiplier * PRESET_HIGH_POINT, -allianceMultiplier * PI / 2))

                            // Segment 3
                            .lineToSplineHeading(new Pose2d(allianceMultiplier * (PRESET_START_X + 2.5*PRESET_SPACING), -allianceMultiplier * (PRESET_LOW_POINT), allianceMultiplier==-1?PI/2:PI))
                            //.splineToConstantHeading(new Vector2d(allianceMultiplier * (PRESET_START_X + 2.7*PRESET_SPACING), -allianceMultiplier * PRESET_HIGH_POINT), allianceMultiplier * PI / 2 * 0)
                            //.lineToSplineHeading(new Pose2d(allianceMultiplier * (PRESET_START_X + 2 * PRESET_SPACING), -allianceMultiplier * (PRESET_LOW_POINT + PRESET_SPACING / 2), -allianceMultiplier * PI / 2))
                            //.splineToConstantHeading(new Vector2d(allianceMultiplier * (PRESET_START_X + 3 * PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier * PRESET_LOW_POINT), allianceMultiplier * PI / 2)
                            .lineToLinearHeading(new Pose2d(allianceMultiplier * (PRESET_START_X + 3 * PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier * PRESET_HIGH_POINT, allianceMultiplier==-1?PI/2:PI))




                            .build());

            return myBot;
        }
    }

    public static RoadRunnerBotEntity pushThreeSamplesSpline(MeepMeep meepMeep) {
        Pose2d initialSpecimenHang = new Pose2d(allianceMultiplier*INITIAL_SPECIMEN_HANG_X, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y, allianceMultiplier*PI/2);
        //new Vector2d(allianceMultiplier*INITIAL_SPECIMEN_HANG_X - allianceMultiplier*SPECIMEN_HANG_OFFSET*hangIndex, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(initialSpecimenHang)

                        // enter push sequence
                        //.lineTo(new Vector2d(PRESET_START_X * allianceMultiplier, -allianceMultiplier * PRESET_START_Y))
                        //.splineToConstantHeading(new Vector2d(PRESET_START_X * allianceMultiplier, -allianceMultiplier*PRESET_START_Y), allianceMultiplier*PI/2) // enter push sequence
                        //.lineToLinearHeading(new Pose2d(PRESET_START_X * allianceMultiplier, -allianceMultiplier*PRESET_START_Y, -allianceMultiplier*PI/2)) // enter push sequence
                        //.splineToConstantHeading(new Vector2d(PRESET_START_X * allianceMultiplier, -allianceMultiplier*PRESET_START_Y), -allianceMultiplier*PI/2)
                        //.splineTo(new Vector2d(PRESET_START_X * allianceMultiplier, -allianceMultiplier*PRESET_START_Y), -allianceMultiplier*PI/2)
                        .lineToSplineHeading(new Pose2d(PRESET_START_X * allianceMultiplier, -allianceMultiplier*PRESET_START_Y, allianceMultiplier==-1?0:PI))
                        //.splineTo(new Vector2d(PRESET_START_X * allianceMultiplier, -allianceMultiplier*PRESET_START_Y), -allianceMultiplier*PI/2)

                        //.splineTo(new Vector2d(allianceMultiplier*(PRESET_START_X + PRESET_SPACING), -allianceMultiplier*PRESET_LOW_POINT), -allianceMultiplier*PI/2)
                        //.lineToLinearHeading(new Pose2d(allianceMultiplier*(PRESET_START_X + PRESET_SPACING), -allianceMultiplier*PRESET_LOW_POINT, -allianceMultiplier*PI/2))

                        // Segment 1
                        //.splineTo(new Vector2d(allianceMultiplier*PRESET_START_X, -allianceMultiplier*(PRESET_LOW_POINT + PRESET_SPACING/2)), -allianceMultiplier*PI/2)
                        //.splineToConstantHeading(new Vector2d(allianceMultiplier*PRESET_START_X, -allianceMultiplier*(PRESET_LOW_POINT + PRESET_SPACING/2)), -allianceMultiplier*PI/2) // 1.78 - SW)
                        //.lineToSplineHeading(new Pose2d(allianceMultiplier*PRESET_START_X, -allianceMultiplier*(PRESET_LOW_POINT + PRESET_SPACING/2), -allianceMultiplier*PI/2))
                        .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X), -allianceMultiplier*PRESET_LOW_POINT + PRESET_SPACING/4), allianceMultiplier*PI/2)
                        //.splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + PRESET_SPACING), -allianceMultiplier*PRESET_LOW_POINT + PRESET_SPACING/2), allianceMultiplier*PI/2)
                        //.splineToLinearHeading(new Pose2d(allianceMultiplier*(PRESET_START_X + PRESET_SPACING), -allianceMultiplier*PRESET_LOW_POINT + PRESET_SPACING/2), allianceMultiplier*PI/2)


                        .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + PRESET_SPACING), -allianceMultiplier*PRESET_LOW_POINT), allianceMultiplier*PI/2) // 1.78 - SW
                        .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + PRESET_SPACING), -allianceMultiplier*PRESET_HIGH_POINT), -allianceMultiplier*PI/2)

                        //.lineToLinearHeading(new Pose2d(allianceMultiplier*(PRESET_START_X + PRESET_SPACING), -allianceMultiplier*PRESET_HIGH_POINT, -allianceMultiplier*PI/2))

                        // Segment 1 - original
                        //.splineToConstantHeading(new Vector2d(allianceMultiplier*PRESET_START_X, -allianceMultiplier*(PRESET_LOW_POINT + PRESET_SPACING/2)), allianceMultiplier*PI/2)  // 1.54 - S
                        //.splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + PRESET_SPACING), -allianceMultiplier*PRESET_LOW_POINT), allianceMultiplier*PI/2) // 1.78 - SW
                        //.splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + PRESET_SPACING), -allianceMultiplier*PRESET_HIGH_POINT), allianceMultiplier*PI/2)// 2.65 - N

                        // Segment 2

                        //.lineToSplineHeading(new Pose2d(allianceMultiplier*(PRESET_START_X + PRESET_SPACING), -allianceMultiplier*(PRESET_LOW_POINT + PRESET_SPACING/2), -allianceMultiplier*PI/2))
                        //.splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 2*PRESET_SPACING), -allianceMultiplier*PRESET_LOW_POINT), allianceMultiplier*PI/2) // 3.38 - SW
                        //.lineToLinearHeading(new Pose2d(allianceMultiplier*(PRESET_START_X + 2*PRESET_SPACING), -allianceMultiplier*PRESET_HIGH_POINT, -allianceMultiplier*PI/2))

                        //Segment 2 - original
                        .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + PRESET_SPACING), -allianceMultiplier*(PRESET_LOW_POINT + PRESET_SPACING/4)),allianceMultiplier*PI/2) // 3.33 - S
                        .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 2*PRESET_SPACING), -allianceMultiplier*PRESET_LOW_POINT), allianceMultiplier*PI/2) // 3.38 - SW
                        .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 2*PRESET_SPACING), -allianceMultiplier*PRESET_HIGH_POINT), allianceMultiplier*PI/2) // 4.18 - N

                        // Segment 3
                        //.lineToSplineHeading(new Pose2d(allianceMultiplier*(PRESET_START_X + 2*PRESET_SPACING), -allianceMultiplier*(PRESET_LOW_POINT + PRESET_SPACING/2), -allianceMultiplier*PI/2))
                        //.splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 3*PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier*PRESET_LOW_POINT), allianceMultiplier*PI/2)
                        //.lineToLinearHeading(new Pose2d(allianceMultiplier*(PRESET_START_X + 3*PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier*PRESET_HIGH_POINT, -allianceMultiplier*PI/2))


                        // Segment 3 - original
                        .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 2*PRESET_SPACING), -allianceMultiplier*(PRESET_LOW_POINT + PRESET_SPACING/4)),allianceMultiplier*PI/2) // 3.33 - S
                        .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 3*PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier*PRESET_LOW_POINT), allianceMultiplier*PI/2) // 3.38 - SW
                        .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 3*PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier*PRESET_HIGH_POINT), allianceMultiplier*PI/2) // 4.18 - N

                        // skip pull back
                        //.splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 3*PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier*(PRESET_HIGH_POINT - PRESET_SPACING)), allianceMultiplier*PI/2) // 4.18 - N
                        // short drive forward
                        //.splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 3*PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier*(PRESET_HIGH_POINT + PRESET_SPACING)), allianceMultiplier*PI/2) // 4.18 - N

                        //.setReversed(true)
                        //.splineToLinearHeading(specimenPick, -allianceMultiplier*PI/2) // 4.18 - N
                        //.splineToLinearHeading(specimenPick, -allianceMultiplier*PI/2) // 4.18 - N
                        //.splineToConstantHeading(new Vector2d(specimenPick.getX(), specimenPick.getY()), -allianceMultiplier*PI/2) // 4.18 - N
                        //.lineToLinearHeading(specimenPick)


                        .build());

        return myBot;
    }

}