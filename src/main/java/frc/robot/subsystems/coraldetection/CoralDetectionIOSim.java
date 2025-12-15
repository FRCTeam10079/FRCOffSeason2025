package frc.robot.subsystems.coraldetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * Simulation implementation of CoralDetectionIO.
 * Basically just ripped it from frc1678's DetectionIOLimelightSim.
 * This will be helpful for simulation testing over break...
 * 
 * Allows for testing coral detection logic without physical hardware.
 * Can be fed simulated coral positions via NetworkTables for automated testing.
 */
public class CoralDetectionIOSim implements CoralDetectionIO {
    
    // Configuration
    private final CoralDetectionConfig config;
    
    // NetworkTables for simulation input
    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final NetworkTable simTable = ntInstance.getTable("Simulation/CoralDetection");
    
    // Publishers for visualization
    private final StructPublisher<Pose2d> simCoralPosePub = 
        simTable.getStructTopic("SimulatedCoralPose", Pose2d.struct).publish();
    private final StructPublisher<Translation2d> simCoralTranslationPub =
        simTable.getStructTopic("SimulatedCoralTranslation", Translation2d.struct).publish();
    
    // Simulated coral state
    private Pose2d simulatedCoralPose = null;
    private Translation2d simulatedRobotRelative = null;
    private double simulatedDetectionTime = 0;
    private boolean simulationEnabled = true;
    
    // Robot pose supplier
    private Supplier<Pose2d> robotPoseSupplier = () -> new Pose2d();
    
    // Cached config values
    private final double cameraHeightMeters;
    private final double cameraPitchRadians;
    private final double cameraYawRadians;
    private final double cameraXOffsetMeters;
    private final double cameraYOffsetMeters;
    private final double coralRadiusMeters;
    
    /**
     * Create simulation IO with default config.
     */
    public CoralDetectionIOSim() {
        this(new CoralDetectionConfig());
    }
    
    /**
     * Create simulation IO with custom config.
     */
    public CoralDetectionIOSim(CoralDetectionConfig config) {
        this.config = config;
        
        this.cameraHeightMeters = config.cameraZOffsetMeters;
        this.cameraPitchRadians = config.cameraPitchRadians;
        this.cameraYawRadians = config.cameraYawRadians;
        this.cameraXOffsetMeters = config.cameraXOffsetMeters;
        this.cameraYOffsetMeters = config.cameraYOffsetMeters;
        this.coralRadiusMeters = config.coralRadiusMeters;
        
        // Set up default simulated coral position (can be overridden via NT)
        // This places a coral ~2 meters in front of the robot
        setSimulatedCoralFieldPosition(new Translation2d(14.0, 7.0));
    }
    
    /**
     * Set the robot pose supplier.
     */
    public void setRobotPoseSupplier(Supplier<Pose2d> supplier) {
        this.robotPoseSupplier = supplier;
    }
    
    /**
     * Set a simulated coral at a field-relative position.
     */
    public void setSimulatedCoralFieldPosition(Translation2d fieldPosition) {
        Pose2d robotPose = robotPoseSupplier.get();
        simulatedCoralPose = new Pose2d(fieldPosition, robotPose.getRotation());
        
        // Calculate robot-relative position
        Translation2d relative = fieldPosition.minus(robotPose.getTranslation());
        // Rotate to robot frame
        simulatedRobotRelative = relative.rotateBy(robotPose.getRotation().unaryMinus());
        
        simulatedDetectionTime = Timer.getFPGATimestamp();
        
        // Publish for visualization
        simCoralPosePub.set(simulatedCoralPose);
        simCoralTranslationPub.set(simulatedRobotRelative);
    }
    
    /**
     * Set a simulated coral at a robot-relative position.
     */
    public void setSimulatedCoralRobotRelative(Translation2d robotRelative) {
        Pose2d robotPose = robotPoseSupplier.get();
        
        simulatedRobotRelative = robotRelative;
        
        // Transform to field-relative
        Translation2d fieldRelative = robotRelative.rotateBy(robotPose.getRotation())
            .plus(robotPose.getTranslation());
        simulatedCoralPose = new Pose2d(fieldRelative, robotPose.getRotation());
        
        simulatedDetectionTime = Timer.getFPGATimestamp();
        
        // Publish for visualization
        simCoralPosePub.set(simulatedCoralPose);
        simCoralTranslationPub.set(simulatedRobotRelative);
    }
    
    /**
     * Clear the simulated coral.
     */
    public void clearSimulatedCoral() {
        simulatedCoralPose = null;
        simulatedRobotRelative = null;
    }
    
    /**
     * Enable or disable simulation.
     */
    public void setSimulationEnabled(boolean enabled) {
        this.simulationEnabled = enabled;
    }
    
    @Override
    public void updateInputs(CoralDetectionInputs inputs) {
        double now = Timer.getFPGATimestamp();
        
        // Update simulated robot-relative position based on current robot pose
        if (simulatedCoralPose != null) {
            Pose2d robotPose = robotPoseSupplier.get();
            Translation2d relative = simulatedCoralPose.getTranslation()
                .minus(robotPose.getTranslation());
            simulatedRobotRelative = relative.rotateBy(robotPose.getRotation().unaryMinus());
        }
        
        inputs.cameraConnected = true;
        inputs.currentPipeline = 0;
        inputs.latencyMs = 20.0; // Simulated 20ms latency
        inputs.timestamp = now - 0.02;
        
        // Check if we have a valid simulated coral
        boolean hasValidCoral = simulationEnabled && 
            simulatedCoralPose != null && 
            simulatedRobotRelative != null &&
            (now - simulatedDetectionTime) < config.staleTimeoutSeconds * 5; // Longer timeout for sim
        
        if (!hasValidCoral) {
            inputs.hasDetection = false;
            return;
        }
        
        double distance = simulatedRobotRelative.getNorm();
        
        // Check if within detection range
        if (distance > config.maxTrackingDistanceMeters || distance < 0.1) {
            inputs.hasDetection = false;
            return;
        }
        
        inputs.hasDetection = true;
        inputs.robotRelativePosition = simulatedRobotRelative;
        inputs.distanceMeters = distance;
        inputs.angleToCoralDeg = simulatedRobotRelative.getAngle().getDegrees();
        
        // Reverse-calculate tx/ty for simulation
        // This simulates what the camera would see
        double tx = Math.toDegrees(Math.atan2(simulatedRobotRelative.getY(), simulatedRobotRelative.getX()));
        inputs.tx = tx;
        inputs.ty = 0; // Simplified - would need full geometry to reverse-calculate
        inputs.ta = 100.0 / (distance * distance); // Area decreases with distance squared
        inputs.classId = config.coralClassId;
        inputs.confidence = 0.95;
        
        SmartDashboard.putBoolean("CoralSim/HasDetection", true);
        SmartDashboard.putNumber("CoralSim/Distance_m", distance);
        SmartDashboard.putNumber("CoralSim/Angle_deg", inputs.angleToCoralDeg);
    }
    
    @Override
    public void setPipeline(int pipelineIndex) {
        // No-op in simulation
        SmartDashboard.putNumber("CoralSim/RequestedPipeline", pipelineIndex);
    }
    
    @Override
    public Optional<Pose2d> getClosestCoralPose(Translation2d robotPosition) {
        if (!simulationEnabled || simulatedCoralPose == null) {
            return Optional.empty();
        }
        return Optional.of(simulatedCoralPose);
    }
    
    @Override
    public Optional<Translation2d> getClosestCoralRobotRelative() {
        if (!simulationEnabled || simulatedRobotRelative == null) {
            return Optional.empty();
        }
        return Optional.of(simulatedRobotRelative);
    }
    
    @Override
    public int getTrackedCoralCount() {
        return (simulationEnabled && simulatedCoralPose != null) ? 1 : 0;
    }
    
    @Override
    public boolean hasCoral() {
        return simulationEnabled && simulatedCoralPose != null;
    }
    
    @Override
    public void clearTrackedCorals() {
        clearSimulatedCoral();
    }
    
    @Override
    public Translation2d calcDistToCoral(double tx, double ty) {
        // Same implementation as real IO
        // Returns camera-relative position (offset applied in tracker loop)
        double totalAngleY = Math.toRadians(-ty) - cameraPitchRadians;
        double heightAboveCoral = cameraHeightMeters - coralRadiusMeters;
        
        if (Math.abs(Math.tan(totalAngleY)) < 0.001) {
            return new Translation2d(0, 0);
        }
        
        double distAwayY = heightAboveCoral / Math.tan(totalAngleY);
        double distHypotenuseToGround = Math.hypot(distAwayY, heightAboveCoral);
        double totalAngleX = Math.toRadians(-tx) + cameraYawRadians;
        double distAwayX = distHypotenuseToGround * Math.tan(totalAngleX);
        
        // Return camera-relative position
        return new Translation2d(distAwayY, distAwayX);
    }
}
