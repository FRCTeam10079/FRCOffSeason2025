package frc.robot.subsystems.coraldetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Optional;

/**
 * IO interface for coral detection.
 * 
 * This interface abstracts the hardware layer from the subsystem logic, allowing for easy swapping between real hardware and simulation...
 */
public interface CoralDetectionIO {
    
    /**
     * Inputs from the coral detection system.
     * Updated each periodic cycle.
     */
    class CoralDetectionInputs {
        // Raw detection data
        public boolean hasDetection = false;
        public double tx = 0.0;                          // Horizontal angle to target (degrees)
        public double ty = 0.0;                          // Vertical angle to target (degrees)
        public double ta = 0.0;                          // Target area percentage
        public int classId = -1;                         // Neural network class ID
        public double confidence = 0.0;                  // Detection confidence (0-1)
        public double latencyMs = 0.0;                   // Total latency in milliseconds
        
        // Calculated position data
        public Translation2d robotRelativePosition = new Translation2d();  // Meters
        public double distanceMeters = 0.0;
        public double angleToCoralDeg = 0.0;
        
        // Pipeline status
        public int currentPipeline = 0;
        public boolean cameraConnected = false;
        
        // Timestamp of this detection
        public double timestamp = 0.0;
    }
    
    /**
     * Configuration for the coral detection camera.
     * Passed to IO during construction.
     */
    class CoralDetectionConfig {
        public String limelightName = "limelight-coral";
        
        // Camera mounting (meters and radians)
        public double cameraXOffsetMeters = 0.0;
        public double cameraYOffsetMeters = 0.0;
        public double cameraZOffsetMeters = 0.6;  // ~24 inches
        public double cameraPitchRadians = Math.toRadians(30);
        public double cameraYawRadians = 0.0;
        
        // Coral properties
        public double coralRadiusMeters = 0.057;  // ~2.25 inches
        
        // Detection filtering
        public double minArea = 0.05;
        public double maxArea = 50.0;
        public double minConfidence = 0.6;
        public int coralClassId = 0;
        
        // Tracking
        public double maxTrackingDistanceMeters = 3.0;  // ~10 feet
        public double staleTimeoutSeconds = 0.3;
    }
    
    /**
     * Update inputs from the detection system.
     * Called once per periodic cycle.
     */
    void updateInputs(CoralDetectionInputs inputs);
    
    /**
     * Set the pipeline on the Limelight.
     * @param pipelineIndex The pipeline index to set
     */
    void setPipeline(int pipelineIndex);
    
    /**
     * Get the closest coral pose relative to a given position.
     * @param robotPosition The robot's current position
     * @return The closest coral's field-relative pose, or empty if none
     */
    Optional<Pose2d> getClosestCoralPose(Translation2d robotPosition);
    
    /**
     * Get the robot-relative position of the closest coral.
     * @return Translation2d in meters (X forward, Y left)
     */
    Optional<Translation2d> getClosestCoralRobotRelative();
    
    /**
     * Get the number of currently tracked corals.
     */
    int getTrackedCoralCount();
    
    /**
     * Check if any coral is currently detected.
     */
    boolean hasCoral();
    
    /**
     * Clear all tracked corals.
     */
    void clearTrackedCorals();
    
    /**
     * Calculate distance to coral using camera geometry.
     * 
     * @param tx Horizontal angle (degrees, positive = right)
     * @param ty Vertical angle (degrees, positive = up)
     * @return Robot-relative translation (meters)
     */
    Translation2d calcDistToCoral(double tx, double ty);
}
