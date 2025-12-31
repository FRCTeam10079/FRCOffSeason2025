package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralDetectionConstants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawDetection;
import frc.robot.RobotContainer;

import java.util.ArrayList;

/**
 * Subsystem for detecting coral on the field
 * 
 * This subsystem:
 * - Tracks detected coral positions on the field
 * - Calculates the distance and angle to coral pieces
 */
public class CoralDetectionSubsystem extends SubsystemBase {
    
    // CONFIGURATION - Using constants from Constants.java
    /** Name of the Limelight used for coral detection */
    public static final String LIMELIGHT_NAME = CoralDetectionConstants.LIMELIGHT_NAME;
    
    /** 
     * Camera position relative to robot center
     * Uses values from Constants.CoralDetectionConstants
     */
    public static final Pose3d ROBOT_TO_CAMERA_OFFSET = new Pose3d(
        new Translation3d(
            Units.inchesToMeters(CoralDetectionConstants.CAMERA_OFFSET_X_INCHES),
            Units.inchesToMeters(CoralDetectionConstants.CAMERA_OFFSET_Y_INCHES),
            Units.inchesToMeters(CoralDetectionConstants.CAMERA_OFFSET_Z_INCHES)
        ),
        new Rotation3d(
            Units.degreesToRadians(CoralDetectionConstants.CAMERA_ROLL_DEGREES),
            Units.degreesToRadians(CoralDetectionConstants.CAMERA_PITCH_DEGREES),
            Units.degreesToRadians(CoralDetectionConstants.CAMERA_YAW_DEGREES)
        )
    );
    
    /** Pipeline index for coral detection (neural network detector) */
    public static final int CORAL_DETECTION_PIPELINE = CoralDetectionConstants.CORAL_DETECTION_PIPELINE;
    
    /** Coral radius in meters */
    public static final double CORAL_RADIUS_METERS = Units.inchesToMeters(CoralDetectionConstants.CORAL_DIAMETER_INCHES / 2.0);
    
    /** How long to keep a coral detection in memory before discarding (seconds) */
    public static final double DETECTION_TIMEOUT_SECONDS = CoralDetectionConstants.DETECTION_TIMEOUT_SECONDS;
    
    /** Maximum number of coral detections to track */
    public static final int MAX_TRACKED_CORALS = CoralDetectionConstants.MAX_TRACKED_CORALS;
    
    /** Class ID for coral in the neural network model */
    public static final int CORAL_CLASS_ID = CoralDetectionConstants.CORAL_CLASS_ID;
    
    // INTERNAL STATE
    private final RobotContainer robotContainer;
    private final ArrayList<TrackedCoral> coralTracker = new ArrayList<>();
    
    /**
     * Internal class to track a coral detection with timestamp
     */
    private static class TrackedCoral {
        final Pose2d fieldPose;           // Position on the field
        final Translation2d robotRelative; // Position relative to robot when detected
        final double detectionTime;        // FPGA timestamp when detected
        
        public TrackedCoral(Pose2d fieldPose, Translation2d robotRelative, double detectionTime) {
            this.fieldPose = fieldPose;
            this.robotRelative = robotRelative;
            this.detectionTime = detectionTime;
        }
    }
    
    // CONSTRUCTOR
    public CoralDetectionSubsystem(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        
        // Configure the Limelight
        configureLimelight();
        
        System.out.println("[CoralDetection] Initialized with camera: " + LIMELIGHT_NAME);
    }
    
    /**
     * Configure the Limelight camera settings
     */
    private void configureLimelight() {
        // Set the camera pose in robot space
        LimelightHelpers.setCameraPose_RobotSpace(
            LIMELIGHT_NAME,
            ROBOT_TO_CAMERA_OFFSET.getX(),
            ROBOT_TO_CAMERA_OFFSET.getY(),
            ROBOT_TO_CAMERA_OFFSET.getZ(),
            Units.radiansToDegrees(ROBOT_TO_CAMERA_OFFSET.getRotation().getX()),
            Units.radiansToDegrees(ROBOT_TO_CAMERA_OFFSET.getRotation().getY()),
            Units.radiansToDegrees(ROBOT_TO_CAMERA_OFFSET.getRotation().getZ())
        );
        
        // Set to coral detection pipeline
        LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, CORAL_DETECTION_PIPELINE);
    }
    
    // PERIODIC UPDATE
    @Override
    public void periodic() {
        // Process coral detections
        updateCoralDetections();
        
        // Output telemetry
        outputTelemetry();
    }
    
    /**
     * Process new coral detections from the Limelight
     */
    private void updateCoralDetections() {
        // Get robot pose for transforming detections to field coordinates
        Pose2d robotPose = robotContainer.drivetrain.getState().Pose;
        if (robotPose == null) {
            return;
        }
        
        // Get raw neural network detections
        RawDetection[] detections = LimelightHelpers.getRawDetections(LIMELIGHT_NAME);
        double now = Timer.getFPGATimestamp();
        
        // Remove old detections
        coralTracker.removeIf(coral -> (now - coral.detectionTime) > DETECTION_TIMEOUT_SECONDS);
        
        // Limit tracked coral count
        while (coralTracker.size() > MAX_TRACKED_CORALS) {
            coralTracker.remove(0);
        }
        
        // Process each detection
        for (RawDetection detection : detections) {
            // Skip if not a coral (check class ID)
            if (detection.classId != CORAL_CLASS_ID) {
                continue;
            }
            
            // Calculate distance to coral using camera geometry
            // This returns position relative to CAMERA, not robot
            Translation2d coralRelativeToRobot = calcDistanceToCoral(detection.txnc, detection.tync)
                .minus(ROBOT_TO_CAMERA_OFFSET.getTranslation().toTranslation2d());  // Subtract camera offset to get robot-relative
            
            // Transform to field coordinates
            Pose2d coralFieldPose = robotPose.transformBy(
                new Transform2d(coralRelativeToRobot, new Rotation2d())
            );
            
            // Check if coral is within field bounds (basic sanity check)
            if (isOutsideField(coralFieldPose)) {
                SmartDashboard.putBoolean("CoralDetection/OutsideField", true);
                continue;
            }
            
            // Add to tracker
            coralTracker.add(new TrackedCoral(coralFieldPose, coralRelativeToRobot, now));
        }
    }
    
    // CORAL POSE CALCULATION
    /**
     * Calculate the distance to a coral based on tx/ty angles from the camera.
     * Uses pinhole camera geometry to estimate the 3D position.
     * 
     * NOTE: This returns the position relative to the CAMERA, not the robot center.
     * The camera offset is subtracted separately in updateCoralDetections().
     * 
     * @param tx Horizontal angle to target (degrees)
     * @param ty Vertical angle to target (degrees)
     * @return Translation2d representing the coral position relative to the CAMERA
     */
    public Translation2d calcDistanceToCoral(double tx, double ty) {
        // Get camera pitch angle (how much it's tilted down)
        double cameraPitchRad = ROBOT_TO_CAMERA_OFFSET.getRotation().getY();
        
        // Get camera height above the coral (coral sits on ground + coral radius)
        double cameraHeight = ROBOT_TO_CAMERA_OFFSET.getZ() - CORAL_RADIUS_METERS;
        
        // Calculate the total vertical angle to the coral
        double totalAngleY = Units.degreesToRadians(-ty) - cameraPitchRad;
        
        // Calculate forward distance using trig (camera height / tan(angle))
        double distanceForward = cameraHeight / Math.tan(totalAngleY);
        
        // Calculate the hypotenuse distance from camera to coral (in the vertical plane)
        double distanceHypotenuse = Math.hypot(distanceForward, cameraHeight);
        
        // Get camera yaw offset
        double cameraYawRad = ROBOT_TO_CAMERA_OFFSET.getRotation().getZ();
        
        // Calculate horizontal angle including camera yaw offset
        double totalAngleX = Units.degreesToRadians(-tx) + cameraYawRad;
        
        // Calculate lateral distance
        double distanceLateral = distanceHypotenuse * Math.tan(totalAngleX);
        
        // Log for debugging
        SmartDashboard.putNumber("CoralDetection/tx", tx);
        SmartDashboard.putNumber("CoralDetection/ty", ty);
        SmartDashboard.putNumber("CoralDetection/DistanceForward", distanceForward);
        SmartDashboard.putNumber("CoralDetection/DistanceLateral", distanceLateral);
        SmartDashboard.putNumber("CoralDetection/DistanceHypotenuse", distanceHypotenuse);
        
        // Return position relative to CAMERA (not robot center)
        // X = forward, Y = left
        return new Translation2d(distanceForward, distanceLateral);
    }
    
    /**
     * Check if a pose is outside the field boundaries
     */
    private boolean isOutsideField(Pose2d pose) {
         double fieldLength = 17.548225; // meters
         double fieldWidth = 8.0518;     // meters
         double margin = Units.feetToMeters(0.5); // 0.5 foot margin

         return pose.getX() < -margin || pose.getX() > fieldLength + margin ||
             pose.getY() < -margin || pose.getY() > fieldWidth + margin;
    }
    
    /**
     * Get the field pose of the closest coral to the robot.
     * 
     * @return Pose2d of the closest coral, or null if no coral detected
     */
    public Pose2d getClosestCoralPose() {
        if (robotContainer.drivetrain.getState().Pose == null) {
            return null;
        }
        return getClosestCoralPose(robotContainer.drivetrain.getState().Pose.getTranslation());
    }
    
    /**
     * Get the field pose of the closest coral to a given position.
     * 
     * @param referencePoint The point to measure distance from
     * @return Pose2d of the closest coral, or null if no coral detected
     */
    public Pose2d getClosestCoralPose(Translation2d referencePoint) {
        TrackedCoral closest = null;
        double closestDistance = Double.MAX_VALUE;
        
        for (TrackedCoral coral : coralTracker) {
            double distance = coral.fieldPose.getTranslation().getDistance(referencePoint);
            if (distance < closestDistance) {
                closestDistance = distance;
                closest = coral;
            }
        }
        
        return closest != null ? closest.fieldPose : null;
    }
    
    /**
     * Get a pose with the translation of the closest coral and rotation pointing toward it.
     * Useful for driving toward the coral.
     * 
     * @return Pose2d with coral position and angle to face it, or null if no coral
     */
    public Pose2d getCoralPoseWithHeading() {
        Pose2d robotPose = robotContainer.drivetrain.getState().Pose;
        if (robotPose == null) {
            return null;
        }
        
        Pose2d coralPose = getClosestCoralPose();
        if (coralPose == null) {
            return null;
        }
        
        // Calculate angle from robot to coral
        Translation2d delta = coralPose.getTranslation().minus(robotPose.getTranslation());
        Rotation2d angleToFace = delta.getAngle();
        
        return new Pose2d(coralPose.getTranslation(), angleToFace);
    }
    
    /**
     * Check if any coral is currently detected.
     * 
     * @return true if coral is being tracked
     */
    public boolean hasCoral() {
        return !coralTracker.isEmpty();
    }
    
    /**
     * Get the number of tracked coral pieces.
     * 
     * @return Number of coral currently being tracked
     */
    public int getCoralCount() {
        return coralTracker.size();
    }
    
    /**
     * Get the number of coral detected in the current frame from Limelight.
     * 
     * @return Number of coral detections from Limelight
     */
    public int getLimelightTargetCount() {
        return LimelightHelpers.getTargetCount(LIMELIGHT_NAME);
    }
    
    /**
     * Ensure coral detection pipeline is active.
     */
    public void enableCoralDetection() {
        LimelightHelpers.setPipelineIndex(LIMELIGHT_NAME, CORAL_DETECTION_PIPELINE);
    }
    
    /**
     * Clear all tracked coral (useful when picking up coral).
     */
    public void clearTrackedCoral() {
        coralTracker.clear();
    }
    
    /**
     * Get the horizontal offset (tx) from the Limelight for the primary target.
     * 
     * @return tx in degrees
     */
    public double getTx() {
        return LimelightHelpers.getTX(LIMELIGHT_NAME);
    }
    
    /**
     * Get the vertical offset (ty) from the Limelight for the primary target.
     * 
     * @return ty in degrees
     */
    public double getTy() {
        return LimelightHelpers.getTY(LIMELIGHT_NAME);
    }
    
    /**
     * Check if the Limelight has a valid target.
     * 
     * @return true if target is valid
     */
    public boolean hasTarget() {
        return LimelightHelpers.getTV(LIMELIGHT_NAME);
    }
    
    // TELEMETRY
    private void outputTelemetry() {
        SmartDashboard.putBoolean("CoralDetection/HasCoral", hasCoral());
        SmartDashboard.putNumber("CoralDetection/CoralCount", getCoralCount());
        SmartDashboard.putNumber("CoralDetection/LimelightTargets", getLimelightTargetCount());
        SmartDashboard.putBoolean("CoralDetection/HasTarget", hasTarget());
        
        if (hasCoral()) {
            Pose2d closest = getClosestCoralPose();
            if (closest != null) {
                SmartDashboard.putNumber("CoralDetection/ClosestX", closest.getX());
                SmartDashboard.putNumber("CoralDetection/ClosestY", closest.getY());
                
                // Distance from robot to coral
                Pose2d robotPose = robotContainer.drivetrain.getState().Pose;
                if (robotPose != null) {
                    double distance = robotPose.getTranslation().getDistance(closest.getTranslation());
                    SmartDashboard.putNumber("CoralDetection/DistanceMeters", distance);
                    SmartDashboard.putNumber("CoralDetection/DistanceInches", Units.metersToInches(distance));
                }
            }
        }
        
        SmartDashboard.putNumber("CoralDetection/TX", getTx());
        SmartDashboard.putNumber("CoralDetection/TY", getTy());
    }
}
