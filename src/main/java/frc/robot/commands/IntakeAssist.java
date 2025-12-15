package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotIntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.RobotStateMachine;
import frc.robot.RobotStateMachine.CoralState;
import frc.robot.RobotStateMachine.GameState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PivotIntakeSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;
import frc.robot.subsystems.coraldetection.CoralDetectionSubsystem;

/**
 * Vision-assisted coral intake command.
 * 
 * This is a command that:
 * 1. Deploys the intake to ground position
 * 2. Searches for coral using neural network detection
 * 3. Drives toward detected coral using PID control
 * 4. Activates intake wheels when close
 * 5. Stows intake when coral is collected
 * 6. Transfers coral to dump roller
 */
public class IntakeAssist extends Command {
    
    // Subsystems
    private final CoralDetectionSubsystem coralDetection;
    private final CommandSwerveDrivetrain drivetrain;
    private final PivotIntakeSubsystem pivotIntake;
    private final SuperstructureSubsystem superstructure;
    
    // State machine reference
    private final RobotStateMachine stateMachine = RobotStateMachine.getInstance();
    
    // PID controllers for driving to coral
    private final PIDController translationController;
    private final PIDController rotationController;
    
    // Swerve drive request
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();
    
    // Configuration constants
    private static final double MAX_DRIVE_SPEED_MPS = 2.0;          // Max translation speed (m/s)
    private static final double MAX_ROT_SPEED_RADPS = 2.0;          // Max rotation speed (rad/s)
    private static final double MIN_DRIVE_SPEED_MPS = 0.3;          // Minimum movement speed
    private static final double INTAKE_ACTIVATION_DISTANCE_M = 0.5; // Start intake wheels (meters)
    private static final double COLLECTION_DISTANCE_M = 0.15;       // Close enough to stop driving
    private static final double ALIGNMENT_TOLERANCE_DEG = 5.0;      // Acceptable angle error
    private static final double SEARCH_TIMEOUT_SECONDS = 3.0;       // Time to wait for initial detection
    private static final double COMMAND_TIMEOUT_SECONDS = 8.0;      // Max total command duration
    private static final int MAX_LOST_FRAMES = 30;                  // ~0.6 seconds at 50Hz before giving up
    
    // Internal state tracking
    private final Timer commandTimer = new Timer();
    private final Timer searchTimer = new Timer();
    private boolean coralCollected = false;
    private int lostDetectionFrames = 0;
    private Translation2d lastKnownPosition = null;
    
    /**
     * Create IntakeAssist from RobotContainer
     */
    public IntakeAssist(RobotContainer robotContainer) {
        this(robotContainer.coralDetection, 
             robotContainer.drivetrain, 
             robotContainer.pivotSub, 
             robotContainer.superstructure);
    }
    
    /**
     * Create IntakeAssist with subsystems
     */
    public IntakeAssist(
            CoralDetectionSubsystem coralDetection,
            CommandSwerveDrivetrain drivetrain,
            PivotIntakeSubsystem pivotIntake,
            SuperstructureSubsystem superstructure) {
        
        this.coralDetection = coralDetection;
        this.drivetrain = drivetrain;
        this.pivotIntake = pivotIntake;
        this.superstructure = superstructure;
        
        // Translation PID - controls driving toward coral
        translationController = new PIDController(8.0, 0.0, 0.01); // TODO: need to test and see (from AlignReef)
        translationController.setTolerance(0.05); // 5cm tolerance
        
        // Rotation PID - controls facing the coral
        rotationController = new PIDController(3.0, 0.0, 0.02); // TODO: need to test and see (from AlignReef)
        rotationController.setTolerance(2.0); // 2 degree tolerance
        rotationController.enableContinuousInput(-180, 180);
        
        // Add requirements
        addRequirements(drivetrain, pivotIntake);
    }
    
    @Override
    public void initialize() {
        System.out.println("=== INTAKE ASSIST: Starting vision-guided collection ===");
        
        // Reset timers
        commandTimer.reset();
        commandTimer.start();
        searchTimer.reset();
        searchTimer.start();
        
        // Reset internal tracking
        coralCollected = false;
        lostDetectionFrames = 0;
        lastKnownPosition = null;
        
        // Reset PID controllers
        translationController.reset();
        rotationController.reset();
        
        // Deploy intake to ground position
        pivotIntake.setPivotSetpoint(PivotIntakeConstants.INTAKE_POSITION);
        
        // Enable detection
        coralDetection.setDetectionEnabled(true);
        
        // Update state machine - searching for coral
        stateMachine.setGameState(GameState.INTAKE_ASSIST_SEARCHING);
        
        SmartDashboard.putString("IntakeAssist/Status", "Searching");
    }
    
    @Override
    public void execute() {
        // Get current state from the central state machine
        GameState currentState = stateMachine.getGameState();
        SmartDashboard.putString("IntakeAssist/GameState", currentState.name());
        
        // Execute based on current state from RobotStateMachine
        switch (currentState) {
            case INTAKE_ASSIST_SEARCHING:
                executeSearching();
                break;
            case INTAKE_ASSIST_APPROACHING:
            case INTAKE_ASSIST_ALIGNING:
                executeApproaching();
                break;
            case INTAKE_ASSIST_COLLECTING:
                executeCollecting();
                break;
            case CORAL_SECURED:
                executeStowing();
                break;
            case TRANSFERRING:
                executeTransferring();
                break;
            case CORAL_LOADED:
            case IDLE:
                // Command complete - handled by isFinished()
                break;
            default:
                // If we're in an unexpected state during this command, 
                // either continue searching or end gracefully
                if (!coralCollected) {
                    executeSearching();
                }
                break;
        }
    }
    
    /**
     * SEARCHING state - Look for coral using vision
     */
    private void executeSearching() {
        // Check if coral detected
        if (coralDetection.hasCoral()) {
            System.out.println("Coral detected! Transitioning to APPROACHING");
            stateMachine.setGameState(GameState.INTAKE_ASSIST_APPROACHING);
            SmartDashboard.putString("IntakeAssist/Status", "Coral found - approaching");
            return;
        }
        
        // Check search timeout - stay in searching but mark as failed
        if (searchTimer.hasElapsed(SEARCH_TIMEOUT_SECONDS)) {
            System.out.println("No coral found within timeout");
            SmartDashboard.putString("IntakeAssist/Status", "No coral found");
            stateMachine.setGameState(GameState.IDLE);  // Return to idle on failure
        }
    }
    
    /**
     * APPROACHING state - Drive toward detected coral
     */
    private void executeApproaching() {
        // Check if coral collected by sensor
        if (pivotIntake.isCoralDetected()) {
            transitionToCollected();
            return;
        }
        
        // Get coral position
        var coralRobotRelative = coralDetection.getCoralRobotRelative();
        
        if (coralRobotRelative.isEmpty() || !coralDetection.hasCoral()) {
            // Lost detection
            lostDetectionFrames++;
            SmartDashboard.putString("IntakeAssist/Status", "Lost detection (" + lostDetectionFrames + ")");
            
            // Coast toward last known position if we haven't lost for too long
            if (lastKnownPosition != null && lostDetectionFrames < MAX_LOST_FRAMES / 2) {
                driveTowardPosition(lastKnownPosition);
            } else {
                stopDriving();
            }
            
            // Give up if lost for too long - return to idle
            if (lostDetectionFrames > MAX_LOST_FRAMES) {
                System.out.println("Lost coral detection - giving up");
                stateMachine.setGameState(GameState.IDLE);
            }
            return;
        }
        
        // Valid detection
        lostDetectionFrames = 0;
        Translation2d coralPosition = coralRobotRelative.get();
        lastKnownPosition = coralPosition;
        
        double distanceMeters = coralPosition.getNorm();
        
        SmartDashboard.putNumber("IntakeAssist/Distance_m", distanceMeters);
        SmartDashboard.putNumber("IntakeAssist/Angle_deg", coralPosition.getAngle().getDegrees());
        
        // Start intake wheels when close - transition to collecting
        if (distanceMeters < INTAKE_ACTIVATION_DISTANCE_M) {
            pivotIntake.setIntakeSpeed(PivotIntakeConstants.INTAKE_SPEED);
            stateMachine.setGameState(GameState.INTAKE_ASSIST_COLLECTING);
            SmartDashboard.putString("IntakeAssist/Status", "Collecting");
        }
        
        // Drive toward coral
        if (distanceMeters > COLLECTION_DISTANCE_M) {
            driveTowardPosition(coralPosition);
        } else {
            stopDriving();
        }
    }
    
    /**
     * COLLECTING state - Intake running, waiting for coral
     */
    private void executeCollecting() {
        // Check if coral collected
        if (pivotIntake.isCoralDetected()) {
            transitionToCollected();
            return;
        }
        
        // Keep driving toward coral if we can see it
        var coralRobotRelative = coralDetection.getCoralRobotRelative();
        if (coralRobotRelative.isPresent()) {
            Translation2d coralPosition = coralRobotRelative.get();
            lastKnownPosition = coralPosition;
            
            if (coralPosition.getNorm() > COLLECTION_DISTANCE_M) {
                driveTowardPosition(coralPosition);
            } else {
                stopDriving();
            }
        } else {
            // Lost detection but keep intake running
            lostDetectionFrames++;
            if (lastKnownPosition != null && lostDetectionFrames < MAX_LOST_FRAMES) {
                driveTowardPosition(lastKnownPosition);
            } else {
                stopDriving();
            }
        }
    }
    
    /**
     * Transition to collected state - coral is secured in intake
     */
    private void transitionToCollected() {
        coralCollected = true;
        System.out.println("Coral collected!");
        
        // Stop driving
        stopDriving();
        
        // Mark coral in intake - update state machine
        pivotIntake.setHasCoralInIntake(true);
        stateMachine.setCoralState(CoralState.IN_INTAKE);
        stateMachine.setGameState(GameState.CORAL_SECURED);
        
        // Extra intake time to secure
        pivotIntake.setIntakeSpeed(PivotIntakeConstants.INTAKE_SPEED);
        
        SmartDashboard.putString("IntakeAssist/Status", "Coral Secured - Stowing");
        
        // Start stowing
        pivotIntake.setPivotSetpoint(PivotIntakeConstants.STOWED_POSITION_WITH_CORAL);
    }
    
    /**
     * CORAL_SECURED state - Stow intake with coral
     */
    private void executeStowing() {
        // Check if pivot is stowed
        if (pivotIntake.isPivotAtSetpoint()) {
            pivotIntake.setIntakeSpeed(0);
            
            System.out.println("Stowed - transferring to dump");
            stateMachine.setCoralState(CoralState.TRANSFERRING);
            stateMachine.setGameState(GameState.TRANSFERRING);
            SmartDashboard.putString("IntakeAssist/Status", "Transferring");
        }
    }
    
    /**
     * TRANSFERRING state - Transfer coral to dump roller
     */
    private void executeTransferring() {
        // The superstructure handles the actual transfer mechanics
        // Mark as complete
        stateMachine.setCoralState(CoralState.IN_DUMP);
        stateMachine.setGameState(GameState.CORAL_LOADED);
        SmartDashboard.putString("IntakeAssist/Status", "Complete");
        System.out.println("=== INTAKE ASSIST: Complete! ===");
    }
    
    /**
     * Drive the robot toward a robot-relative position.
     */
    private void driveTowardPosition(Translation2d robotRelativePosition) {
        double distanceMeters = robotRelativePosition.getNorm();
        double angleDegrees = robotRelativePosition.getAngle().getDegrees();
        
        // Calculate rotation to face the coral
        double rotationSpeed = rotationController.calculate(angleDegrees, 0);
        rotationSpeed = MathUtil.clamp(rotationSpeed, -MAX_ROT_SPEED_RADPS, MAX_ROT_SPEED_RADPS);
        
        // Calculate drive speed based on distance
        double driveSpeed = translationController.calculate(0, distanceMeters);
        driveSpeed = MathUtil.clamp(driveSpeed, -MAX_DRIVE_SPEED_MPS, MAX_DRIVE_SPEED_MPS);
        
        // Ensure minimum movement if not at target
        if (distanceMeters > COLLECTION_DISTANCE_M && Math.abs(driveSpeed) < MIN_DRIVE_SPEED_MPS) {
            driveSpeed = MIN_DRIVE_SPEED_MPS * Math.signum(driveSpeed);
        }
        
        // Convert to velocity components
        double xSpeed = (robotRelativePosition.getX() / distanceMeters) * driveSpeed;
        double ySpeed = (robotRelativePosition.getY() / distanceMeters) * driveSpeed;
        
        // Reduce speed if not aligned
        if (Math.abs(angleDegrees) > ALIGNMENT_TOLERANCE_DEG * 2) {
            xSpeed *= 0.3;
            ySpeed *= 0.3;
        }
        
        SmartDashboard.putNumber("IntakeAssist/VelX", xSpeed);
        SmartDashboard.putNumber("IntakeAssist/VelY", ySpeed);
        SmartDashboard.putNumber("IntakeAssist/VelRot", rotationSpeed);
        
        // Apply robot-relative driving
        drivetrain.setControl(
            new SwerveRequest.RobotCentric()
                .withVelocityX(xSpeed)
                .withVelocityY(ySpeed)
                .withRotationalRate(rotationSpeed)
        );
    }
    
    /**
     * Stop the drivetrain
     */
    private void stopDriving() {
        drivetrain.setControl(driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop drivetrain
        stopDriving();
        
        // Stop intake wheels if no coral
        if (!coralCollected) {
            pivotIntake.setIntakeSpeed(0);
            pivotIntake.setPivotSetpoint(PivotIntakeConstants.STOWED_POSITION);
            stateMachine.setGameState(GameState.IDLE);
        }
        
        String status = interrupted ? "Interrupted" : (coralCollected ? "SUCCESS" : "No coral");
        SmartDashboard.putString("IntakeAssist/Status", status);
        System.out.println("IntakeAssist ended - " + status);
    }
    
    @Override
    public boolean isFinished() {
        // Check if state machine indicates completion
        GameState currentState = stateMachine.getGameState();
        if (currentState == GameState.CORAL_LOADED || currentState == GameState.IDLE) {
            // Only finish if we either succeeded or gave up
            return true;
        }
        
        // Timeout
        if (commandTimer.hasElapsed(COMMAND_TIMEOUT_SECONDS)) {
            System.out.println("IntakeAssist timeout");
            return true;
        }
        
        return false;
    }
    
    /**
     * Check if coral was successfully collected.
     */
    public boolean wasSuccessful() {
        return coralCollected;
    }
    
    /**
     * Get the current game state.
     */
    public GameState getCurrentState() {
        return stateMachine.getGameState();
    }
}