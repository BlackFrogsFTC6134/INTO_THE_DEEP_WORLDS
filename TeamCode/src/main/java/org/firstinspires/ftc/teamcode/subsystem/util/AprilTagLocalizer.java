package org.firstinspires.ftc.teamcode.subsystem.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystem.Constants;

import java.util.HashMap;
import java.util.Map;

public class AprilTagLocalizer {
    // Lookup table mapping AprilTag IDs to their known global poses.
    // Field center is (0,0), valid x,y in [-72,72].
    private Map<Integer, Pose2d> tagGlobalPoses = new HashMap<>();

    // Camera rotation relative to the robot's forward direction; left rotation is positive.
    private final double cameraMountAngleRadians;

    // Camera offset (in robot coordinates) from the robot’s center.
    private final Vector2d cameraOffsetInches;

    // Camera height from the playing field surface.
    private final double cameraHeightInches;

    // One side of AprilTag length in inches (assumes square tag).
    private final double tagSize;

    // AprilTag center height above playing field surface in inches.
    private final double tagHeight;

    // Filtering parameter for exponential moving average.
    private final double filterAlpha;

    // Last computed pose for filtering.
    private Pose2d lastPose = null;

    // Telemetry for debugging.
    private final Telemetry telemetry;

    // Kalman filter for fusing odometry and AprilTag measurements.
    private KalmanFilter3D kalmanFilter = null;

    public AprilTagLocalizer(double cameraMountAngleRadians, Vector2d cameraOffsetInches, double cameraHeightInches, double tagSize, double tagHeight, double filterAlpha, Telemetry telemetry) {
        this.cameraMountAngleRadians = cameraMountAngleRadians;
        this.cameraOffsetInches = cameraOffsetInches;
        this.cameraHeightInches = cameraHeightInches;
        this.tagSize = tagSize;
        this.tagHeight = tagHeight;
        this.filterAlpha = filterAlpha;
        this.telemetry = telemetry;

        // Populate lookup table with known AprilTag positions.
        // (Update these values to your field’s actual tag positions, but this is standard for an FTC field as of April 2025)
        tagGlobalPoses.put(1, new Pose2d(-72, 48, Math.toRadians(180)));
        tagGlobalPoses.put(2, new Pose2d(0, 72, Math.toRadians(90)));
        tagGlobalPoses.put(3, new Pose2d(72, 48, Math.toRadians(0)));
        tagGlobalPoses.put(4, new Pose2d(72, -48, Math.toRadians(0)));
        tagGlobalPoses.put(5, new Pose2d(0, -72, Math.toRadians(-90)));
        tagGlobalPoses.put(6, new Pose2d(-72, -48, Math.toRadians(180)));

    }

    /**
     * Updates the robot's global pose based on a HuskyLens detection.
     * @param detection A HuskyLens.Block with detection.center.x, detection.center.y, detection.width.
     * @param tagID The detected tag's ID.
     * @return Filtered global Pose2d or null if tag unknown.
     */
    public Pose2d updatePoseFromAprilTag(HuskyLens.Block detection, int tagID) {
        Pose2d tagGlobalPose = tagGlobalPoses.get(tagID);
        if (tagGlobalPose == null) return null;  // Unknown tag

        // Calculate image center.
        double cx = Constants.CAMERA_IMAGE_WIDTH_PIXELS / 2.0;
        double cy = Constants.CAMERA_IMAGE_HEIGHT_PIXELS / 2.0;

        // Compute the horizontal error (in pixels) between the detected center and the image center.
        double pixelErrorX = detection.x - cx;
        telemetry.addData("pixelErrorX", pixelErrorX);

        // Convert pixel error to an angle (radians) using the pinhole camera model.
        double angleError = Math.atan2(pixelErrorX, Constants.CAMERA_FOCAL_LENGTH_PIXELS);
        telemetry.addData("angleError (rad)", angleError);
        // If angle is greater than threshold, then ignore pose reading
        if (Math.abs(angleError) > Constants.APRILTAG_ANGLE_DETECTION_THRESHOLD_RADIANS) return null;

        // Total relative angle from the tag to the camera is the sum of the mounting angle and the measured offset.
        double totalAngle = cameraMountAngleRadians + angleError;
        telemetry.addData("cameraMountAngle (rad)", cameraMountAngleRadians);
        telemetry.addData("totalAngle (rad)", totalAngle);

        // Estimate the measured distance along the ray from the camera to the tag.
        double avgSize = (detection.width + detection.height) / 2.0;
        double d_measured = (tagSize * Constants.CAMERA_FOCAL_LENGTH_PIXELS) / avgSize;
        //double d_measured = (TAG_REAL_WIDTH * focalLength) / detection.width;
        // Compensate for camera seeing tag at an angle
        double d_corrected = d_measured * Math.cos(angleError);
        telemetry.addData("Detection avgSize", avgSize);
        telemetry.addData("d_measured", d_measured);

        // Now, to account for the fact that the AprilTag is elevated, compute the horizontal distance on the ground:
        double dz = cameraHeightInches - tagHeight;  // For example, 6.5 - 6.0 = 0.5 inches (adjust as needed)
        double d_horiz = Math.sqrt(Math.max(0, d_corrected * d_corrected - dz * dz));
        telemetry.addData("d_horiz", d_horiz);

        // Compute the sideways offset (in inches) in the camera frame.
        double forward_cam = d_horiz * Math.cos(angleError);
        double sideways_cam = d_horiz * Math.sin(angleError);
        telemetry.addData("forward_cam", forward_cam);
        telemetry.addData("sideways_cam", sideways_cam);

        // In the camera frame (with optical axis forward):
        //   forward component = d_horiz,
        //   sideways component = sideways_cam.
        // The vector from the tag to the camera in the camera frame is then:
        double tagToCamForward = -forward_cam;
        double tagToCamSide = -sideways_cam;

        // Rotate the translation vector into the robot frame.
        // (Note: We swap the roles of X and Y here so that the “forward” component maps to the robot’s Y.)
        double relX_robot = tagToCamForward * Math.sin(cameraMountAngleRadians) + tagToCamSide * Math.cos(cameraMountAngleRadians);
        double relY_robot = tagToCamForward * Math.cos(cameraMountAngleRadians) - tagToCamSide * Math.sin(cameraMountAngleRadians);
        telemetry.addData("relX_robot (pre-swap)", relX_robot);
        telemetry.addData("relY_robot (pre-swap)", relY_robot);

        // --- NEW: If the detected tag is one whose distance is along the Y axis (e.g. tag 2 or 5), swap the offsets.
        if (tagID == 2 || tagID == 5) {
            double temp = relX_robot;
            relX_robot = relY_robot;
            relY_robot = temp;
            telemetry.addLine("Swapped relative offsets for tag " + tagID);
        }
        telemetry.addData("relX_robot", relX_robot);
        telemetry.addData("relY_robot", relY_robot);


        // Now determine the robot's global position based on the tag’s global pose.
        // For the x-coordinate:
        double cameraGlobalX;
        double cameraGlobalY;

        if (tagGlobalPose.getX() > 0 && tagGlobalPose.getY() > 0) {
            // Tag is on the right side; the camera is to the left of the tag.
            cameraGlobalX = tagGlobalPose.getX() - Math.abs(relX_robot);
            cameraGlobalY = tagGlobalPose.getY() + Math.abs(relY_robot);
        } else if (tagGlobalPose.getX() < 0 && tagGlobalPose.getY() < 0) {
            // Tag is on the left side; the camera is to the right of the tag.
            cameraGlobalX = tagGlobalPose.getX() + Math.abs(relX_robot);
            cameraGlobalY = tagGlobalPose.getY() - Math.abs(relY_robot);
        } else if (tagGlobalPose.getX() > 0 && tagGlobalPose.getY() < 0) {
            // Tag is on the right side; the camera is to the left of the tag.
            cameraGlobalX = tagGlobalPose.getX() - Math.abs(relX_robot);
            cameraGlobalY = tagGlobalPose.getY() - Math.abs(relY_robot);
        } else if (tagGlobalPose.getX() < 0 && tagGlobalPose.getY() > 0) {
            cameraGlobalX = tagGlobalPose.getX() + Math.abs(relX_robot);
            cameraGlobalY = tagGlobalPose.getY() - Math.abs(relY_robot);
        } else {
            // Tag x is zero; use sign of pixel error.
            cameraGlobalX = tagGlobalPose.getX() - (Math.signum(pixelErrorX) * Math.abs(relX_robot));

            // testing new method
            if (tagGlobalPose.getY() > 0) {
                cameraGlobalY = tagGlobalPose.getY() - Math.abs(relY_robot);
            } else if (tagGlobalPose.getY() < 0) {
                cameraGlobalY = tagGlobalPose.getY() + Math.abs(relY_robot);
            } else {
                double pixelErrorY = detection.y - cy;
                cameraGlobalY = tagGlobalPose.getY() + (Math.signum(pixelErrorY) * Math.abs(relY_robot));
            }

        }

        telemetry.addData("cameraGlobalX", cameraGlobalX);
        telemetry.addData("cameraGlobalY", cameraGlobalY);

        // Now account for the physical offset between the camera and the robot center.
        double cosH = Math.cos(tagGlobalPose.getHeading());
        double sinH = Math.sin(tagGlobalPose.getHeading());
        double offsetX = cameraOffsetInches.getX() * cosH - cameraOffsetInches.getY() * sinH;
        double offsetY = cameraOffsetInches.getX() * sinH + cameraOffsetInches.getY() * cosH;
        telemetry.addData("offsetX", offsetX);
        telemetry.addData("offsetY", offsetY);

        // The robot’s global position is the camera global position minus the offset.
        double robotGlobalX = cameraGlobalX + offsetX;
        double robotGlobalY = cameraGlobalY + offsetY;
        telemetry.addData("robotGlobalX", robotGlobalX);
        telemetry.addData("robotGlobalY", robotGlobalY);

        telemetry.addData("robotGlobalX", robotGlobalX);
        telemetry.addData("robotGlobalY", robotGlobalY);

        // For the robot's heading, assume the robot's heading is the tag's heading minus the measured total angle.
        double robotHeading = tagGlobalPose.getHeading() - totalAngle;
        telemetry.addData("robotHeading (rad)", robotHeading);

        Pose2d newPose = new Pose2d(robotGlobalX, robotGlobalY, robotHeading);

        // (Optional) Apply exponential moving average filtering.
        if (lastPose == null) {
            lastPose = newPose;
        } else {
            double filteredX = filterAlpha * newPose.getX() + (1 - filterAlpha) * lastPose.getX();
            double filteredY = filterAlpha * newPose.getY() + (1 - filterAlpha) * lastPose.getY();
            double deltaHeading = Angle.normDelta(newPose.getHeading() - lastPose.getHeading());
            double filteredHeading = lastPose.getHeading() + filterAlpha * deltaHeading;
            newPose = new Pose2d(filteredX, filteredY, filteredHeading);
            lastPose = newPose;
        }

        telemetry.update();
        return newPose;
    }

    /**
     * Fuses the current odometry pose with the AprilTag measurement using a Kalman filter.
     * This method initializes the filter on the first call.
     *
     * @param odomPose    The current pose from odometry.
     * @param aprilTagPose The measured pose from the AprilTag.
     * @return The fused pose.
     */
    public Pose2d fusePose(Pose2d odomPose, Pose2d aprilTagPose) {
        // Initialize the Kalman filter if needed.
        if (kalmanFilter == null) {
            // Initialize with the odometry pose.
            kalmanFilter = new KalmanFilter3D(odomPose, Constants.KALMAN_FILTER_Q_DIAGONAL, Constants.KALMAN_FILTER_R_DIAGONAL);
        }
        // (Optional) Use the odometry pose to predict the next state.
        kalmanFilter.predict();
        // Then update with the AprilTag measurement.
        kalmanFilter.update(aprilTagPose);
        return kalmanFilter.getState();
    }

}
