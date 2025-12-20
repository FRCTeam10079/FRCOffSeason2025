package frc.robot.subsystems.coraldetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawDetection;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Limelight-based implementation of CoralDetectionIO.
 * 
 * Uses trigonometry-based distance calculation for accurate coral positioning.
 * Maintains a tracking list of detected corals with temporal filtering.
 */
public class CoralDetectionIOLimelight implements CoralDetectionIO {
    
    // Configuration
    private final CoralDetectionConfig config;
    private final String limelightName;
    
    // Cached values for trigonometry calculations
    private final double cameraHeightMeters;
    private final double cameraPitchRadians;
    private final double cameraYawRadians;
    private final double cameraXOffsetMeters;
    private final double cameraYOffsetMeters;
    private final double coralRadiusMeters;
    
    // Coral tracking
    private final ArrayList<TrackedCoral> trackedCorals = new ArrayList<>();
    private static final int MAX_TRACKED_CORALS = 20;
    
    // Robot pose supplier for field-relative calculations
    private Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();
    
    // Best coral cache
    private TrackedCoral bestCoral = null;
    
    /**
     * Internal class for tracking coral detections over time.
     */
    private static class TrackedCoral {
        final Pose2d fieldPose;
        final Translation2d robotRelativePosition;
        final double detectionTime;
        
        TrackedCoral(Pose2d fieldPose, Translation2d robotRelativePosition, double detectionTime) {
            this.fieldPose = fieldPose;
            this.robotRelativePosition = robotRelativePosition;
            this.detectionTime = detectionTime;
        }
    }
    
    /**
     * Create a new CoralDetectionIOLimelight with default configuration.
     */
    public CoralDetectionIOLimelight() {
        this(new CoralDetectionConfig());
    }
    
    /**
     * Create a new CoralDetectionIOLimelight with custom configuration.
     */
    public CoralDetectionIOLimelight(CoralDetectionConfig config) {
        this.config = config;
        this.limelightName = config.limelightName;
        
        // Cache converted values for performance
        this.cameraHeightMeters = config.cameraZOffsetMeters;
        this.cameraPitchRadians = config.cameraPitchRadians;
        this.cameraYawRadians = config.cameraYawRadians;
        this.cameraXOffsetMeters = config.cameraXOffsetMeters;
        this.cameraYOffsetMeters = config.cameraYOffsetMeters;
        this.coralRadiusMeters = config.coralRadiusMeters;
        
        // Set pipeline to neural network detection
        LimelightHelpers.setPipelineIndex(limelightName, 0);
        
        // Enable LEDs for better coral detection
        LimelightHelpers.setLEDMode_PipelineControl(limelightName);
        
        // Sync camera pose to Limelight
        // This enables built-in 3D features and keeps config in sync
        // Parameters: forward, side, up (meters), roll, pitch, yaw (degrees)
        LimelightHelpers.setCameraPose_RobotSpace(
            limelightName,
            cameraXOffsetMeters,                           // Forward offset
            cameraYOffsetMeters,                           // Side offset (left positive)
            cameraHeightMeters,                            // Height from ground
            0.0,                                           // Roll (usually 0)
            Math.toDegrees(cameraPitchRadians),            // Pitch (positive = down)
            Math.toDegrees(cameraYawRadians)               // Yaw (positive = left)
        );
        
        // Debug: Print configuration
        System.out.println("===== CORAL DETECTION INITIALIZED =====");
        System.out.println("Limelight Name: " + limelightName);
        System.out.println("Camera Height:" + cameraHeightMeters + " m");
        System.out.println("Camera Pitch:" + Math.toDegrees(cameraPitchRadians) + " deg");
        System.out.println("Coral Class ID: " + config.coralClassId + " (0 = accept any non-zero)");
        System.out.println("======================================");
    }
    
    /**
     * Set the robot pose supplier for field-relative calculations.
     */
    public void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
        this.robotPoseSupplier = supplier;
    }
    
    @Override
    public void updateInputs(CoralDetectionInputs inputs) {
        double now = Timer.getFPGATimestamp();
        Pose2d robotPose = robotPoseSupplier.get();
        
        // Ensure we're on the correct pipeline for neural detection
        double currentPipeline = LimelightHelpers.getCurrentPipelineIndex(limelightName);
        if (currentPipeline != 0) {
            LimelightHelpers.setPipelineIndex(limelightName, 0);
            SmartDashboard.putString("CoralIO/PipelineStatus", "Switching to pipeline 0");
        } else {
            SmartDashboard.putString("CoralIO/PipelineStatus", "On pipeline 0 (neural)");
        }
        
        // Check camera connection
        inputs.cameraConnected = LimelightHelpers.getTV(limelightName);
        inputs.currentPipeline = (int) currentPipeline;
        
        // Get latency
        double captureLatency = LimelightHelpers.getLatency_Capture(limelightName);
        double pipelineLatency = LimelightHelpers.getLatency_Pipeline(limelightName);
        inputs.latencyMs = captureLatency + pipelineLatency;
        inputs.timestamp = now - (inputs.latencyMs / 1000.0);
        
        // Remove stale detections
        trackedCorals.removeIf(coral -> 
            (now - coral.detectionTime) > config.staleTimeoutSeconds);
        
        // Limit tracker size
        while (trackedCorals.size() > MAX_TRACKED_CORALS) {
            trackedCorals.remove(0);
        }
        
        // Get raw detections from neural network
        RawDetection[] rawDetections = LimelightHelpers.getRawDetections(limelightName);
        
        // Debug output
        SmartDashboard.putBoolean("CoralIO/GotDetections", rawDetections != null && rawDetections.length > 0);
        SmartDashboard.putNumber("CoralIO/DetectionArrayLength", rawDetections != null ? rawDetections.length : 0);
        
        if (rawDetections == null || rawDetections.length == 0) {
            inputs.hasDetection = false;
            SmartDashboard.putString("CoralIO/Status", "No detections from neural network");
            updateBestCoral(robotPose.getTranslation());
            return;
        }
        
        SmartDashboard.putString("CoralIO/Status", "Processing " + rawDetections.length + " detections");
        
        // Debug: log raw detection count
        SmartDashboard.putNumber("CoralIO/RawDetectionCount", rawDetections.length);
        
        // Process each detection
        boolean foundCoral = false;
        for (RawDetection detection : rawDetections) {
            // Debug: log all detections
            SmartDashboard.putNumber("CoralIO/LastClassID", detection.classId);
            SmartDashboard.putNumber("CoralIO/LastConfidence", detection.ta);
            
            // IMPORTANT: Class ID 0 is typically the background/invalid class
            // Skip class 0 detections (like FRC 1678 does)
            if (detection.classId == 0) {
                continue;
            }
            
            // If you have a specific coral class ID configured, filter for it
            // Otherwise, accept any non-zero class
            if (config.coralClassId > 0 && detection.classId != config.coralClassId) {
                continue;
            }
            
            // txnc and tync from RawDetection are already in degrees
            // (horizontal and vertical offset from principal point to target)
            double tx = detection.txnc;
            double ty = detection.tync;
            
            // Calculate camera-relative position, then apply camera offset
            // Using minus to translate from camera frame to robot frame
            Translation2d cameraRelative = calcDistToCoral(tx, ty);
            Translation2d cameraOffset = new Translation2d(cameraXOffsetMeters, cameraYOffsetMeters);
            Translation2d coralRobotRelative = cameraRelative.minus(cameraOffset);
            
            // Skip if too far
            double distance = coralRobotRelative.getNorm();
            if (distance > config.maxTrackingDistanceMeters || distance < 0.05) {
                continue;
            }
            
            // Transform to field-relative pose
            Pose2d coralFieldPose = robotPose.transformBy(
                new Transform2d(coralRobotRelative, new Rotation2d())
            );
            
            // Skip if outside field boundaries
            if (isOutsideField(coralFieldPose)) {
                continue;
            }
            
            // Add to tracker
            trackedCorals.add(new TrackedCoral(coralFieldPose, coralRobotRelative, now));
            
            // Update inputs with first valid detection
            if (!foundCoral) {
                inputs.hasDetection = true;
                inputs.tx = tx;
                inputs.ty = ty;
                inputs.ta = detection.ta;
                inputs.classId = detection.classId;
                inputs.robotRelativePosition = coralRobotRelative;
                inputs.distanceMeters = distance;
                inputs.angleToCoralDeg = coralRobotRelative.getAngle().getDegrees();
                foundCoral = true;
            }
        }
        
        if (!foundCoral) {
            inputs.hasDetection = false;
        }
        
        // Update best coral
        updateBestCoral(robotPose.getTranslation());
    }
    
    /**
     * Find the closest coral to the robot.
     */
    private void updateBestCoral(Translation2d robotPosition) {
        TrackedCoral closest = null;
        double closestDistance = Double.MAX_VALUE;
        
        for (TrackedCoral coral : trackedCorals) {
            double distance = coral.fieldPose.getTranslation().getDistance(robotPosition);
            if (distance < closestDistance) {
                closestDistance = distance;
                closest = coral;
            }
        }
        
        bestCoral = closest;
    }
    
    /**
     * Basic field boundary check.
     */
    private boolean isOutsideField(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        // FRC field is approximately 16.5m x 8.2m
        return x < 0 || x > 16.5 || y < 0 || y > 8.2;
    }
    
    @Override
    public Translation2d calcDistToCoral(double tx, double ty) {
        // Calculate total vertical angle (camera pitch + ty from detection)
        // ty is negative when looking down at the coral
        double totalAngleY = Units.degreesToRadians(-ty) - cameraPitchRadians;
        
        // Calculate forward distance using camera height and vertical angle
        double heightAboveCoral = cameraHeightMeters - coralRadiusMeters;
        
        // Avoid division by zero or very small angles
        if (Math.abs(Math.tan(totalAngleY)) < 0.001) {
            return new Translation2d(0, 0);
        }
        
        double distAwayY = heightAboveCoral / Math.tan(totalAngleY);
        
        // Calculate hypotenuse distance from camera to ground point below coral
        double distHypotenuseToGround = Math.hypot(distAwayY, heightAboveCoral);
        
        // Calculate horizontal angle (camera yaw + tx from detection)
        double totalAngleX = Units.degreesToRadians(-tx) + cameraYawRadians;
        
        // Calculate lateral distance using horizontal angle
        double distAwayX = distHypotenuseToGround * Math.tan(totalAngleX);
        
        // Debug output
        SmartDashboard.putNumber("CoralIO/Raw_TX", tx);
        SmartDashboard.putNumber("CoralIO/Raw_TY", ty);
        SmartDashboard.putNumber("CoralIO/DistForward_m", distAwayY);
        SmartDashboard.putNumber("CoralIO/DistLateral_m", distAwayX);
        
        // Return camera-relative position (X forward, Y lateral)
        // Camera offset will be applied in the tracker loop (matching frc1678 pattern)
        return new Translation2d(distAwayY, distAwayX);
    }
    
    @Override
    public void setPipeline(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(limelightName, pipelineIndex);
    }
    
    @Override
    public Optional<Pose2d> getClosestCoralPose(Translation2d robotPosition) {
        if (bestCoral == null) {
            return Optional.empty();
        }
        
        double now = Timer.getFPGATimestamp();
        if ((now - bestCoral.detectionTime) > config.staleTimeoutSeconds) {
            return Optional.empty();
        }
        
        return Optional.of(bestCoral.fieldPose);
    }
    
    @Override
    public Optional<Translation2d> getClosestCoralRobotRelative() {
        if (bestCoral == null) {
            return Optional.empty();
        }
        
        double now = Timer.getFPGATimestamp();
        if ((now - bestCoral.detectionTime) > config.staleTimeoutSeconds) {
            return Optional.empty();
        }
        
        return Optional.of(bestCoral.robotRelativePosition);
    }
    
    @Override
    public int getTrackedCoralCount() {
        return trackedCorals.size();
    }
    
    @Override
    public boolean hasCoral() {
        if (bestCoral == null) {
            return false;
        }
        double now = Timer.getFPGATimestamp();
        return (now - bestCoral.detectionTime) <= config.staleTimeoutSeconds;
    }
    
    @Override
    public void clearTrackedCorals() {
        trackedCorals.clear();
        bestCoral = null;
    }
}
