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
    public static double BOT_WIDTH = 17;
    public static double BOT_LENGTH = 17.75;
    public static double TRACK_WIDTH = 14.28; // inches in Sharks robot
    public static final double MAX_RPM = 435;
    public static double GEAR_RATIO = 1;
    public static double WHEEL_RADIUS = 104.0/2/25.4; // 104mm diameter wheel
    public static double RATE_LIMITER = 0.80;
    public static double MAX_VEL =  ((MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * 2 * PI) * RATE_LIMITER;
    public static double MAX_ACCEL = ((MAX_RPM / 60) * GEAR_RATIO * WHEEL_RADIUS * 2 * PI) * RATE_LIMITER;
    public static double MAX_ANG_VEL = 6 * RATE_LIMITER;
    public static double MAX_ANG_ACCEL = Math.toRadians(180);
    public static volatile double INITIAL_SPECIMEN_HANG_X = 11.0;
    public static volatile double INITIAL_SPECIMEN_HANG_Y = 39.0;
    public static volatile double INITIAL_SPECIMEN_HANG_HEADING = PI/2;
    static final double PRESET_SPACING = 10;
    static final double LAST_SPECIMEN_MARGIN = 4.75;
    static final double PRESET_START_X = 36;
    static final double PRESET_LOW_POINT = 12;
    static final double PRESET_HIGH_POINT = 56;
    static final double PRESET_START_Y = 35; // need to push it out a little, so it does not catch on submersible

    static final double OBSERVATION_ZONE_PICK_X = -36.5;
    static final double OBSERVATION_ZONE_PICK_Y = 62;
    static final double OBSERVATION_ZONE_PICK_HEADING = -PI/2;

    protected final static double NEUTRAL_SAMPLE_1_PICK_X = 48.0;
    protected final static double NEUTRAL_SAMPLE_1_PICK_Y = 35.0;
    protected final static double NEUTRAL_SAMPLE_1_PICK_HEADING = -PI/2;
    protected final static double ALIGN_DELTA_SUBMERSIBLE_PARK = 3.5;

    protected final static int START_POSE_BASKET_X = 15;
    protected final static int START_POSE_BASKET_Y = 63;
    protected final static double START_POSE_HEADING = -PI/2;

    protected final static double BASKET_DROP_X = 54;
    protected final static double BASKET_DROP_Y = 54;
    protected final static double BASKET_DROP_HEADING = -PI*3/4;
    protected final static double SUBMERSIBLE_PARK_X = 24;
    protected final static double SUBMERSIBLE_PARK_Y = 13;
    //protected final static double SUBMERSIBLE_PARK_HEADING = PI;

    static final int allianceMultiplier = 1; // -1 for blue, 1 for red
    static Pose2d specimenPick = new Pose2d(-allianceMultiplier*OBSERVATION_ZONE_PICK_X, -allianceMultiplier*OBSERVATION_ZONE_PICK_Y, -allianceMultiplier*OBSERVATION_ZONE_PICK_HEADING);
    static Pose2d submersiblePark = new Pose2d(-allianceMultiplier*SUBMERSIBLE_PARK_X, -allianceMultiplier*SUBMERSIBLE_PARK_Y, allianceMultiplier == -1 ? PI : 0);

    static Pose2d thirdNeutralSampleAlign = new Pose2d(-allianceMultiplier*59.0, -allianceMultiplier*26.5, allianceMultiplier == -1 ? 0 : PI); // with slight offset to rotate the sample

    public static double SPECIMEN_HANG_OFFSET = 3.0;
    public static int hangIndex = 0;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot;// = pushThreeSamples(meepMeep);
        // RoadRunnerBotEntity myBot; // = pickFirstSpecimen(meepMeep);



        myBot = pushThreeSamples(meepMeep);
        //myBot = deliverInitialSample(meepMeep);
        //myBot = pickThirdNeutral(meepMeep);
        //  myBot = parkBySubmersible(meepMeep);

        //myBot = pickFirstSpecimen(meepMeep);
        //myBot = pickNextSpecimen(meepMeep, 1);
        //myBot = hangInitialSpecimenBasket(meepMeep, 0);
        //myBot = pickFirstNeutral(meepMeep);
        //myBot = hangSpecimen(meepMeep, 0);

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static RoadRunnerBotEntity pickThirdNeutral(MeepMeep meepMeep) {
        Pose2d basketDrop = new Pose2d(-allianceMultiplier*BASKET_DROP_X, -allianceMultiplier*BASKET_DROP_Y, allianceMultiplier == -1 ? -PI*3/4 : PI/4);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(basketDrop)
                        .lineToLinearHeading(new Pose2d(thirdNeutralSampleAlign.getX(), thirdNeutralSampleAlign.getY(), thirdNeutralSampleAlign.getHeading()))
                        .build());

        return myBot;
    }

    public static RoadRunnerBotEntity parkBySubmersible(MeepMeep meepMeep) {
        Pose2d basketDrop = new Pose2d(-allianceMultiplier*BASKET_DROP_X, -allianceMultiplier*BASKET_DROP_Y, allianceMultiplier == -1 ? -PI*3/4 : PI/4);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .setDimensions(BOT_WIDTH, BOT_LENGTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(basketDrop)
                        .splineTo(new Vector2d(submersiblePark.getX() - allianceMultiplier*ALIGN_DELTA_SUBMERSIBLE_PARK , submersiblePark.getY()), submersiblePark.getHeading())
                        .splineTo(new Vector2d(submersiblePark.getX(), submersiblePark.getY()), submersiblePark.getHeading())
                        .build());

        return myBot;
    }

    public static RoadRunnerBotEntity hangInitialSpecimenBasket(MeepMeep meepMeep, int hangIndex) {
        // hang pose spline
        // Pose2d hangPose = new Pose2d(-INITIAL_SPECIMEN_HANG_X - allianceMultiplier*SPECIMEN_HANG_OFFSET*hangIndex, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y, allianceMultiplier*INITIAL_SPECIMEN_HANG_HEADING);

        // hang pose line
        Vector2d hangPose = new Vector2d(-allianceMultiplier*INITIAL_SPECIMEN_HANG_X - allianceMultiplier*SPECIMEN_HANG_OFFSET*hangIndex, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-allianceMultiplier*START_POSE_BASKET_X, -allianceMultiplier*START_POSE_BASKET_Y, -allianceMultiplier*START_POSE_HEADING))

                        // .setReversed(true)
                        // .splineTo(new Vector2d(hangPose.getX(), hangPose.getY()), hangPose.getHeading()) // 1.40, 1.47, 1.55

                        .lineTo(hangPose) // 1.28, 1.32, 1.38, 1.44, 1.49

                        .build());
        return myBot;
    }

    public static RoadRunnerBotEntity pickFirstNeutral(MeepMeep meepMeep) {
        Pose2d initialSpecimenHang = new Pose2d(-INITIAL_SPECIMEN_HANG_X, -INITIAL_SPECIMEN_HANG_Y, INITIAL_SPECIMEN_HANG_HEADING);
        Pose2d basketDrop = new Pose2d(-allianceMultiplier*BASKET_DROP_X, -allianceMultiplier*BASKET_DROP_Y, -allianceMultiplier*BASKET_DROP_HEADING);
        Pose2d firstNeutralSamplePick = new Pose2d(-NEUTRAL_SAMPLE_1_PICK_X, -NEUTRAL_SAMPLE_1_PICK_Y, -NEUTRAL_SAMPLE_1_PICK_HEADING);
        Pose2d firstNeutralSampleAlign = new Pose2d(firstNeutralSamplePick.getX(), firstNeutralSamplePick.getY() + ALIGN_DELTA_SUBMERSIBLE_PARK, firstNeutralSamplePick.getHeading());

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(initialSpecimenHang)

                        .lineTo(new Vector2d(firstNeutralSamplePick.getX(), firstNeutralSamplePick.getY()))
                        //.splineTo(new Vector2d(firstNeutralSampleAlign.getX(), firstNeutralSampleAlign.getY()), firstNeutralSampleAlign.getHeading())
                        //.splineTo(new Vector2d(firstNeutralSamplePick.getX(), firstNeutralSamplePick.getY()), firstNeutralSamplePick.getHeading())
                        .build());

        return myBot;
    }


    public static RoadRunnerBotEntity deliverInitialSample(MeepMeep meepMeep) {
        Pose2d basketDrop = new Pose2d(-allianceMultiplier*BASKET_DROP_X, -allianceMultiplier*BASKET_DROP_Y, -allianceMultiplier*BASKET_DROP_HEADING);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-allianceMultiplier*START_POSE_BASKET_X, -allianceMultiplier*START_POSE_BASKET_Y, -allianceMultiplier*START_POSE_HEADING))

                        .setReversed(true)
                        .splineTo(new Vector2d(basketDrop.getX(), basketDrop.getY()), basketDrop.getHeading()+ (allianceMultiplier==-1?PI:PI/2))
                        .build());

        return myBot;
    }

    public static RoadRunnerBotEntity pickNextSpecimen(MeepMeep meepMeep, int hangIndex) {
        Pose2d hangPose = new Pose2d(-INITIAL_SPECIMEN_HANG_X - allianceMultiplier*SPECIMEN_HANG_OFFSET*hangIndex, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y, -allianceMultiplier*INITIAL_SPECIMEN_HANG_HEADING);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(hangPose)

                        // .splineTo(new Vector2d(specimenPick.getX(), specimenPick.getY()), specimenPick.getHeading()) // 1.40, 1.47, 1.55, 1.62, 1.69
                        .lineToLinearHeading(specimenPick) // 1.26, 1.32, 1.38, 1.44, 1.49

                        .build());

        return myBot;
    }

    public static RoadRunnerBotEntity hangSpecimen(MeepMeep meepMeep, int hangIndex) {
        // hang pose spline
        // Pose2d hangPose = new Pose2d(-INITIAL_SPECIMEN_HANG_X - allianceMultiplier*SPECIMEN_HANG_OFFSET*hangIndex, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y, allianceMultiplier*INITIAL_SPECIMEN_HANG_HEADING);

        // hang pose line
        Pose2d hangPose = new Pose2d(allianceMultiplier*INITIAL_SPECIMEN_HANG_X - allianceMultiplier*SPECIMEN_HANG_OFFSET*hangIndex, -allianceMultiplier*INITIAL_SPECIMEN_HANG_Y, -allianceMultiplier*INITIAL_SPECIMEN_HANG_HEADING);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(specimenPick)

                        // .setReversed(true)
                        // .splineTo(new Vector2d(hangPose.getX(), hangPose.getY()), hangPose.getHeading()) // 1.40, 1.47, 1.55

                        .lineToLinearHeading(hangPose) // 1.28, 1.32, 1.38, 1.44, 1.49

                        .build());
        return myBot;
    }

    public static RoadRunnerBotEntity pickFirstSpecimen(MeepMeep meepMeep) {
        Pose2d pullBack = new Pose2d(allianceMultiplier*(PRESET_START_X + 3*PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier*(PRESET_HIGH_POINT - PRESET_SPACING), -allianceMultiplier*PI/2);

        // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(pullBack)
                        .lineToLinearHeading(specimenPick)
                        .build());

        return myBot;
    }

    public static RoadRunnerBotEntity pushThreeSamples(MeepMeep meepMeep) {
        Pose2d initialSpecimenHang = new Pose2d(allianceMultiplier*INITIAL_SPECIMEN_HANG_X, -allianceMultiplier*40.0, -allianceMultiplier*PI/2);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(initialSpecimenHang)

                        // enter push sequence
                        .splineToConstantHeading(new Vector2d(PRESET_START_X * allianceMultiplier, -allianceMultiplier*PRESET_START_Y), allianceMultiplier*PI/2) // enter push sequence

                        // Segment 1
                        .splineToConstantHeading(new Vector2d(allianceMultiplier*PRESET_START_X , -allianceMultiplier*(PRESET_LOW_POINT + PRESET_SPACING/2)), allianceMultiplier*PI/2)  // 1.54 - S
                        .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + PRESET_SPACING), -allianceMultiplier*PRESET_LOW_POINT), allianceMultiplier*PI/2) // 1.78 - SW
                        .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + PRESET_SPACING), -allianceMultiplier*PRESET_HIGH_POINT), allianceMultiplier*PI/2)// 2.65 - N

                        // Segment 2
                        .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + PRESET_SPACING), -allianceMultiplier*(PRESET_LOW_POINT + PRESET_SPACING/2)),allianceMultiplier*PI/2) // 3.33 - S
                        .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 2*PRESET_SPACING), -allianceMultiplier*PRESET_LOW_POINT), allianceMultiplier*PI/2) // 3.38 - SW
                        .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 2*PRESET_SPACING), -allianceMultiplier*PRESET_HIGH_POINT), allianceMultiplier*PI/2) // 4.18 - N

                        // Segment 3
                        .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 2*PRESET_SPACING), -allianceMultiplier*(PRESET_LOW_POINT + PRESET_SPACING/2)),allianceMultiplier*PI/2) // 3.33 - S
                        .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 3*PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier*PRESET_LOW_POINT), allianceMultiplier*PI/2) // 3.38 - SW
                        .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 3*PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier*PRESET_HIGH_POINT), allianceMultiplier*PI/2) // 4.18 - N

                        // skip pull back
                        // .splineToConstantHeading(new Vector2d(allianceMultiplier*(PRESET_START_X + 3*PRESET_SPACING - LAST_SPECIMEN_MARGIN), -allianceMultiplier*(PRESET_HIGH_POINT - PRESET_SPACING)), allianceMultiplier*PI/2) // 4.18 - N

                        .setReversed(true)
                        //.splineToLinearHeading(specimenPick, -allianceMultiplier*PI/2) // 4.18 - N
                        //.splineToLinearHeading(specimenPick, -allianceMultiplier*PI/2) // 4.18 - N
                        .splineToConstantHeading(new Vector2d(specimenPick.getX(), specimenPick.getY()), -allianceMultiplier*PI/2) // 4.18 - N
                        //.lineToLinearHeading(specimenPick)

                        .build());

        return myBot;
    }
}