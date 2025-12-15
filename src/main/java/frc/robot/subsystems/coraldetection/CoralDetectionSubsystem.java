package frc.robot.subsystems.coraldetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralDetectionConstants;
import frc.robot.Robot;
import frc.robot.subsystems.coraldetection.CoralDetectionIO.CoralDetectionConfig;
import frc.robot.subsystems.coraldetection.CoralDetectionIO.CoralDetectionInputs;

import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

public class CoralDetectionSubsystem extends SubsystemBase {
    
    // IO layer
    private final CoralDetectionIO io;
    private final CoralDetectionInputs inputs = new CoralDetectionInputs();
    
    // Robot pose supplier for field-relative calculations
    private Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();
    
    // Detection state
    private boolean detectionEnabled = true;
    
    /**
     * Create CoralDetectionSubsystem with automatic IO selection.
     * Uses real Limelight IO when running on robot, simulation IO otherwise.
     */
    public CoralDetectionSubsystem() {
        this(createDefaultIO());
    }
    
    /**
     * Create CoralDetectionSubsystem with specified IO.
     */
    public CoralDetectionSubsystem(CoralDetectionIO io) {
        this.io = io;
    }
    
    /**
     * Create the appropriate IO based on robot mode.
     */
    private static CoralDetectionIO createDefaultIO() {
        CoralDetectionConfig config = createConfigFromConstants();
        
        if (Robot.isReal()) {
            return new CoralDetectionIOLimelight(config);
        } else {
            return new CoralDetectionIOSim(config);
        }
    }
    
    /**
     * Create configuration from Constants.
     */
    private static CoralDetectionConfig createConfigFromConstants() {
        CoralDetectionConfig config = new CoralDetectionConfig();
        
        config.limelightName = CoralDetectionConstants.LIMELIGHT_NAME;
        config.cameraXOffsetMeters = Units.inchesToMeters(CoralDetectionConstants.CAMERA_X_OFFSET_INCHES);
        config.cameraYOffsetMeters = Units.inchesToMeters(CoralDetectionConstants.CAMERA_Y_OFFSET_INCHES);
        config.cameraZOffsetMeters = Units.inchesToMeters(CoralDetectionConstants.CAMERA_Z_OFFSET_INCHES);
        config.cameraPitchRadians = Units.degreesToRadians(CoralDetectionConstants.CAMERA_PITCH_DEGREES);
        config.cameraYawRadians = Units.degreesToRadians(CoralDetectionConstants.CAMERA_YAW_DEGREES);
        config.coralRadiusMeters = Units.inchesToMeters(CoralDetectionConstants.CORAL_RADIUS_INCHES);
        config.minArea = CoralDetectionConstants.MIN_TARGET_AREA;
        config.maxArea = CoralDetectionConstants.MAX_TARGET_AREA;
        config.minConfidence = CoralDetectionConstants.MIN_DETECTION_CONFIDENCE;
        config.coralClassId = CoralDetectionConstants.CORAL_CLASS_ID;
        config.maxTrackingDistanceMeters = Units.inchesToMeters(CoralDetectionConstants.MAX_TRACKING_DISTANCE_INCHES);
        config.staleTimeoutSeconds = CoralDetectionConstants.STALE_DETECTION_TIMEOUT_SECONDS;
        
        return config;
    }
    
    /**
     * Set the robot pose supplier for field-relative tracking.
     * Call this from RobotContainer after drivetrain is initialized.
     */
    public void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
        this.robotPoseSupplier = supplier;
        
        // Pass to IO if it supports it
        if (io instanceof CoralDetectionIOLimelight) {
            ((CoralDetectionIOLimelight) io).setRobotPoseSupplier(supplier);
        } else if (io instanceof CoralDetectionIOSim) {
            ((CoralDetectionIOSim) io).setRobotPoseSupplier(supplier);
        }
    }
    
    @Override
    public void periodic() {
        if (detectionEnabled) {
            io.updateInputs(inputs);
        }
        
        updateTelemetry();
    }
    
    /**
     * Update AdvantageKit and SmartDashboard telemetry.
     */
    private void updateTelemetry() {
        // AdvantageKit logging
        Logger.recordOutput("CoralDetection/Enabled", detectionEnabled);
        Logger.recordOutput("CoralDetection/HasDetection", inputs.hasDetection);
        Logger.recordOutput("CoralDetection/CameraConnected", inputs.cameraConnected);
        Logger.recordOutput("CoralDetection/TrackedCount", io.getTrackedCoralCount());
        Logger.recordOutput("CoralDetection/DistanceMeters", inputs.distanceMeters);
        Logger.recordOutput("CoralDetection/AngleDegrees", inputs.angleToCoralDeg);
        Logger.recordOutput("CoralDetection/LatencyMs", inputs.latencyMs);
        
        if (inputs.hasDetection) {
            Logger.recordOutput("CoralDetection/RobotRelative", inputs.robotRelativePosition);
            
            Optional<Pose2d> coralPose = io.getClosestCoralPose(robotPoseSupplier.get().getTranslation());
            coralPose.ifPresent(pose -> Logger.recordOutput("CoralDetection/FieldPose", pose));
        }
        
        // SmartDashboard (legacy)
        SmartDashboard.putBoolean("Coral/Enabled", detectionEnabled);
        SmartDashboard.putBoolean("Coral/HasDetection", inputs.hasDetection);
        SmartDashboard.putBoolean("Coral/CameraConnected", inputs.cameraConnected);
        SmartDashboard.putNumber("Coral/TrackedCount", io.getTrackedCoralCount());
        SmartDashboard.putNumber("Coral/Distance_m", inputs.distanceMeters);
        SmartDashboard.putNumber("Coral/Angle_deg", inputs.angleToCoralDeg);
    }
    
    /**
     * Check if coral is currently detected.
     */
    public boolean hasCoral() {
        return detectionEnabled && io.hasCoral();
    }
    
    /**
     * Get the latest detection inputs.
     */
    public CoralDetectionInputs getInputs() {
        return inputs;
    }
    
    /**
     * Get distance to closest coral in meters.
     * @return Distance, or -1 if no coral
     */
    public double getDistanceToCoralMeters() {
        if (!hasCoral()) {
            return -1;
        }
        return inputs.distanceMeters;
    }
    
    /**
     * Get distance to closest coral in inches.
     * @return Distance, or -1 if no coral
     */
    public double getDistanceToCoralInches() {
        double meters = getDistanceToCoralMeters();
        return meters < 0 ? -1 : Units.metersToInches(meters);
    }
    
    /**
     * Get angle to closest coral in degrees.
     * Positive = left, negative = right
     */
    public double getAngleToCoralDegrees() {
        if (!hasCoral()) {
            return 0;
        }
        return inputs.angleToCoralDeg;
    }
    
    /**
     * Get robot-relative position of closest coral.
     * @return Translation2d (X forward, Y left) in meters, or empty
     */
    public Optional<Translation2d> getCoralRobotRelative() {
        return io.getClosestCoralRobotRelative();
    }
    
    /**
     * Get field-relative pose of closest coral.
     * @return Coral pose, or empty if none
     */
    public Optional<Pose2d> getCoralFieldPose() {
        return io.getClosestCoralPose(robotPoseSupplier.get().getTranslation());
    }
    
    /**
     * Check if robot is aligned to coral within tolerance.
     * @param toleranceDeg Angle tolerance in degrees
     */
    public boolean isAlignedToCoral(double toleranceDeg) {
        if (!hasCoral()) {
            return false;
        }
        return Math.abs(inputs.angleToCoralDeg) < toleranceDeg;
    }
    
    /**
     * Check if robot is within pickup range.
     * @param rangeInches Distance threshold in inches
     */
    public boolean isWithinPickupRange(double rangeInches) {
        if (!hasCoral()) {
            return false;
        }
        return getDistanceToCoralInches() > 0 && 
               getDistanceToCoralInches() <= rangeInches;
    }
    
    /**
     * Enable or disable coral detection.
     */
    public void setDetectionEnabled(boolean enabled) {
        this.detectionEnabled = enabled;
    }
    
    /**
     * Check if detection is currently enabled.
     */
    public boolean isDetectionEnabled() {
        return detectionEnabled;
    }
    
    /**
     * Get number of currently tracked corals.
     */
    public int getTrackedCoralCount() {
        return io.getTrackedCoralCount();
    }
    
    /**
     * Clear all tracked corals.
     */
    public void clearTracking() {
        io.clearTrackedCorals();
    }
    
    // COMMANDS
    
    /**
     * Command that waits until coral is detected.
     */
    public Command waitForCoral() {
        return Commands.waitUntil(this::hasCoral)
            .withName("WaitForCoral");
    }
    
    /**
     * Command that waits until aligned to coral.
     */
    public Command waitForAlignment(double toleranceDeg) {
        return Commands.waitUntil(() -> isAlignedToCoral(toleranceDeg))
            .withName("WaitForAlignment");
    }
    
    /**
     * Command that waits until within pickup range.
     */
    public Command waitForPickupRange(double rangeInches) {
        return Commands.waitUntil(() -> isWithinPickupRange(rangeInches))
            .withName("WaitForPickupRange");
    }
    
    /**
     * Command that enables detection.
     */
    public Command enableDetection() {
        return Commands.runOnce(() -> setDetectionEnabled(true))
            .withName("EnableCoralDetection");
    }
    
    /**
     * Command that disables detection.
     */
    public Command disableDetection() {
        return Commands.runOnce(() -> setDetectionEnabled(false))
            .withName("DisableCoralDetection");
    }
    
    /**
     * Command that clears tracked corals.
     */
    public Command clearTrackedCorals() {
        return Commands.runOnce(this::clearTracking)
            .withName("ClearTrackedCorals");
    }
    
    /**
     * Command that prints detection info to console.
     */
    public Command printDetectionInfo() {
        return Commands.runOnce(() -> {
            if (hasCoral()) {
                System.out.println("=== CORAL DETECTED ===");
                System.out.println(String.format("Distance: %.2f m (%.1f in)", 
                    inputs.distanceMeters, getDistanceToCoralInches()));
                System.out.println(String.format("Angle: %.1f degrees", inputs.angleToCoralDeg));
                System.out.println(String.format("Position: X=%.2f m, Y=%.2f m", 
                    inputs.robotRelativePosition.getX(), inputs.robotRelativePosition.getY()));
                
                getCoralFieldPose().ifPresent(pose -> 
                    System.out.println(String.format("Field: (%.2f, %.2f)", 
                        pose.getX(), pose.getY())));
            } else {
                System.out.println("No coral detected");
            }
        }).withName("PrintCoralInfo");
    }
}
