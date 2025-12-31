package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.RobotStateMachine;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralDetectionSubsystem;

/**
 * Command to align the robot to a detected coral piece using the Limelight's neural network detection.
 * 
 * This command:
 * 1. Continuously tracks coral position from the CoralDetectionSubsystem
 * 2. Uses PID control to drive toward the coral
 * 3. Rotates the robot to face the coral
 * 4. Stops when within tolerance of the coral position
 */
public class AlignToCoral extends Command {
    
    // CONFIGURATION
    
    /** Distance in front of coral to stop at (meters) - don't want to ram the coral */
    private static final double STOP_DISTANCE_FROM_CORAL = 0.1; // ~4 inches
    
    /** Position tolerance before command completes (meters) */
    // Using 1678 AutoConstants.kAutoLinearEpsilon (4 cm) as a placeholder
    private static final double POSITION_TOLERANCE = 0.04;

    /** Rotation tolerance before command completes (radians) */
    // Using 1678 AutoConstants.kAutoAngleEpsilon (1 deg) as a placeholder
    private static final double ROTATION_TOLERANCE = Math.toRadians(1.0);

    /** Maximum speed when driving to coral (m/s) */
    // Using 1678 GeneratedConstants.kSpeedAt12Volts * 0.5 (detection max speed) as a placeholder
    private static final double MAX_SPEED = 5.91 * 0.5; // ~2.955 m/s

    /** Maximum rotation speed (rad/s) */
    // Using 1678 DriveConstants.kMaxAngularRate = 2.75 * PI (rad/s) as a placeholder
    private static final double MAX_ROTATION_SPEED = 2.75 * Math.PI;
    
    /** Command timeout (seconds) */
    // Using 1678 AutoConstants.kDefaultTrajectoryTimeout placeholder (1.0s) as a placeholder
    private static final double COMMAND_TIMEOUT = 1.0;

    /** How long to hold position before considering complete (seconds) */
    // Using 1678 AutoConstants.kDelayTime placeholder (80 ms) as a placeholder
    private static final double SETTLING_TIME = 0.08;
    
    // SUBSYSTEMS
    private final RobotContainer robotContainer;
    private final CommandSwerveDrivetrain drivetrain;
    private final CoralDetectionSubsystem coralDetection;
    private final RobotStateMachine stateMachine;
    
    // PID CONTROLLERS
    // Position PID (for driving toward coral)
    // Using 1678 auto-align translation controller gains as placeholders
    private final PIDController pidX = new PIDController(3.15, 0.0, 0.0);
    private final PIDController pidY = new PIDController(3.15, 0.0, 0.0);

    // Rotation PID (for facing coral)
    // Using 1678 auto-align heading controller gains as placeholders
    private final PIDController pidRotation = new PIDController(5.0, 0.0, 0.0);
    
    // SWERVE REQUESTS
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity);
    
    private final SwerveRequest.Idle stopRequest = new SwerveRequest.Idle();
    
    // STATE
    private Timer timer = new Timer();
    private Timer settlingTimer = new Timer();
    private Pose2d targetPose = null;
    private boolean coralEverDetected = false;
    private Pose2d lastKnownCoralPose = null;
    
    // Moving average for smoother tracking (reduces jitter)
    private static final int POSE_BUFFER_SIZE = 5;
    private final Pose2d[] poseBuffer = new Pose2d[POSE_BUFFER_SIZE];
    private int poseBufferIndex = 0;
    
    // CONSTRUCTOR
    /**
     * Create a new AlignToCoral command.
     * 
     * @param robotContainer The RobotContainer with all subsystems
     */
    public AlignToCoral(RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
        this.drivetrain = robotContainer.drivetrain;
        this.coralDetection = robotContainer.coralDetection;
        this.stateMachine = RobotStateMachine.getInstance();
        
        // Enable continuous input for rotation (wrap around at 180 degrees)
        pidRotation.enableContinuousInput(-Math.PI, Math.PI);
        
        // Require the drivetrain
        addRequirements(drivetrain);
        
        System.out.println("[AlignToCoral] Command created");
    }
    
    // COMMAND LIFECYCLE
    @Override
    public void initialize() {
        System.out.println("[AlignToCoral] Command started");
        
        // Update state machine
        stateMachine.setDrivetrainMode(RobotStateMachine.DrivetrainMode.VISION_TRACKING);
        stateMachine.setAlignedToTarget(false);
        
        // Reset state
        timer.restart();
        settlingTimer.reset();
        settlingTimer.stop();
        targetPose = null;
        coralEverDetected = false;
        lastKnownCoralPose = null;
        
        // Clear pose buffer
        for (int i = 0; i < POSE_BUFFER_SIZE; i++) {
            poseBuffer[i] = null;
        }
        poseBufferIndex = 0;
        
        // Reset PID controllers
        pidX.reset();
        pidY.reset();
        pidRotation.reset();
        
        // Make sure coral detection is enabled
        coralDetection.enableCoralDetection();
        
        // Reduce speed during coral tracking
        robotContainer.MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.5;
        
        Logger.recordOutput("AlignToCoral/Started", true);
    }
    
    @Override
    public void execute() {
        // Get current robot pose
        Pose2d robotPose = drivetrain.getState().Pose;
        if (robotPose == null) {
            return;
        }
        
        // Try to get coral position from detection
        Pose2d coralPose = coralDetection.getCoralPoseWithHeading();
        
        if (coralPose != null) {
            // Coral detected - update tracking
            coralEverDetected = true;
            lastKnownCoralPose = coralPose;
            
            // Add to moving average buffer
            poseBuffer[poseBufferIndex] = coralPose;
            poseBufferIndex = (poseBufferIndex + 1) % POSE_BUFFER_SIZE;
            
            // Calculate smoothed target pose
            targetPose = calculateSmoothedPose();
            
            Logger.recordOutput("AlignToCoral/CoralDetected", true);
        } else if (lastKnownCoralPose != null) {
            // No current detection but we had one before - use last known position
            targetPose = calculateSmoothedPose();
            Logger.recordOutput("AlignToCoral/UsingLastKnown", true);
        } else {
            // No coral ever detected
            Logger.recordOutput("AlignToCoral/NoCoralFound", true);
            SmartDashboard.putBoolean("AlignToCoral/Searching", true);
            return;
        }
        
        SmartDashboard.putBoolean("AlignToCoral/Searching", false);
        
        if (targetPose == null) {
            return;
        }
        
        // Calculate target position (stop before the coral, not on it)
        Translation2d coralTranslation = targetPose.getTranslation();
        Translation2d robotToCoralVector = coralTranslation.minus(robotPose.getTranslation());
        
        // Normalize and scale to stop short of coral
        double distanceToCoral = robotToCoralVector.getNorm();
        if (distanceToCoral > STOP_DISTANCE_FROM_CORAL) {
            // Move toward coral but stop STOP_DISTANCE_FROM_CORAL meters away
            double targetDistance = distanceToCoral - STOP_DISTANCE_FROM_CORAL;
            Translation2d targetPosition = robotPose.getTranslation().plus(
                robotToCoralVector.times(targetDistance / distanceToCoral)
            );
            
            // Target rotation: face the coral
            Rotation2d targetRotation = robotToCoralVector.getAngle();
            
            // Update PID setpoints
            pidX.setSetpoint(targetPosition.getX());
            pidY.setSetpoint(targetPosition.getY());
            pidRotation.setSetpoint(targetRotation.getRadians());
        } else {
            // Already close enough - just face the coral
            pidX.setSetpoint(robotPose.getX());
            pidY.setSetpoint(robotPose.getY());
            pidRotation.setSetpoint(robotToCoralVector.getAngle().getRadians());
        }
        
        // Calculate velocities using PID
        double velocityX = pidX.calculate(robotPose.getX());
        double velocityY = pidY.calculate(robotPose.getY());
        double velocityRotation = pidRotation.calculate(robotPose.getRotation().getRadians());
        
        // Clamp velocities
        velocityX = MathUtil.clamp(velocityX, -MAX_SPEED, MAX_SPEED);
        velocityY = MathUtil.clamp(velocityY, -MAX_SPEED, MAX_SPEED);
        velocityRotation = MathUtil.clamp(velocityRotation, -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);
        
        // Check if we're at target
        boolean atPosition = Math.abs(pidX.getError()) < POSITION_TOLERANCE &&
                            Math.abs(pidY.getError()) < POSITION_TOLERANCE;
        boolean atRotation = Math.abs(pidRotation.getError()) < ROTATION_TOLERANCE;
        
        if (atPosition && atRotation) {
            settlingTimer.start();
            if (!settlingTimer.hasElapsed(SETTLING_TIME)) {
                // Still settling - apply reduced velocities
                velocityX *= 0.3;
                velocityY *= 0.3;
                velocityRotation *= 0.3;
            }
        } else {
            settlingTimer.reset();
            settlingTimer.stop();
        }
        
        // Apply drive command
        drivetrain.setControl(driveRequest
            .withVelocityX(velocityX)
            .withVelocityY(velocityY)
            .withRotationalRate(velocityRotation)
        );
        
        // Telemetry
        SmartDashboard.putNumber("AlignToCoral/TargetX", targetPose.getX());
        SmartDashboard.putNumber("AlignToCoral/TargetY", targetPose.getY());
        SmartDashboard.putNumber("AlignToCoral/ErrorX", pidX.getError());
        SmartDashboard.putNumber("AlignToCoral/ErrorY", pidY.getError());
        SmartDashboard.putNumber("AlignToCoral/ErrorRotation", Math.toDegrees(pidRotation.getError()));
        SmartDashboard.putNumber("AlignToCoral/DistanceToCoral", distanceToCoral);
        SmartDashboard.putBoolean("AlignToCoral/AtPosition", atPosition);
        SmartDashboard.putBoolean("AlignToCoral/AtRotation", atRotation);
        
        Logger.recordOutput("AlignToCoral/TargetPose", targetPose);
        Logger.recordOutput("AlignToCoral/VelocityX", velocityX);
        Logger.recordOutput("AlignToCoral/VelocityY", velocityY);
        Logger.recordOutput("AlignToCoral/VelocityRotation", velocityRotation);
    }
    
    /**
     * Calculate a smoothed target pose from the buffer of recent detections.
     * This reduces jitter from noisy detections.
     */
    private Pose2d calculateSmoothedPose() {
        int count = 0;
        double sumX = 0;
        double sumY = 0;
        double sumCos = 0;
        double sumSin = 0;
        
        for (Pose2d pose : poseBuffer) {
            if (pose != null) {
                sumX += pose.getX();
                sumY += pose.getY();
                sumCos += Math.cos(pose.getRotation().getRadians());
                sumSin += Math.sin(pose.getRotation().getRadians());
                count++;
            }
        }
        
        if (count == 0) {
            return lastKnownCoralPose;
        }
        
        double avgX = sumX / count;
        double avgY = sumY / count;
        double avgAngle = Math.atan2(sumSin / count, sumCos / count);
        
        return new Pose2d(avgX, avgY, new Rotation2d(avgAngle));
    }
    
    @Override
    public boolean isFinished() {
        // Timeout
        if (timer.hasElapsed(COMMAND_TIMEOUT)) {
            System.out.println("[AlignToCoral] Timed out");
            return true;
        }
        
        // Check if at target with settling time
        if (settlingTimer.hasElapsed(SETTLING_TIME)) {
            System.out.println("[AlignToCoral] Reached target");
            return true;
        }
        
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        drivetrain.setControl(stopRequest);
        
        // Update state machine
        stateMachine.setDrivetrainMode(RobotStateMachine.DrivetrainMode.FIELD_CENTRIC);
        
        if (!interrupted && coralEverDetected) {
            stateMachine.setAlignedToTarget(true);
            System.out.println("[AlignToCoral] Completed successfully - aligned to coral");
        } else {
            stateMachine.setAlignedToTarget(false);
            if (interrupted) {
                System.out.println("[AlignToCoral] Interrupted");
            } else {
                System.out.println("[AlignToCoral] Ended without finding coral");
            }
        }
        
        // Restore normal speed
        robotContainer.MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7;
        
        // Clear tracked coral so we don't accidentally drive to old position
        coralDetection.clearTrackedCoral();
        
        Logger.recordOutput("AlignToCoral/Ended", true);
        Logger.recordOutput("AlignToCoral/Interrupted", interrupted);
        Logger.recordOutput("AlignToCoral/CoralFound", coralEverDetected);
        
        SmartDashboard.putBoolean("AlignToCoral/Active", false);
    }
}
