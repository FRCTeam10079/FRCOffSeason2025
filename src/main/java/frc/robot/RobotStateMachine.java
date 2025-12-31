package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.ArrayList;
import java.util.List;

/**
 * MASTER ROBOT STATE MACHINE
 * This is the HIGHEST level state machine that controls the ENTIRE robot.
 * Every aspect of robot operation flows through this state machine.
 */
public class RobotStateMachine extends SubsystemBase {
    
    // Singleton instance
    private static RobotStateMachine instance;
    
    // Current states
    private MatchState matchState = MatchState.DISABLED;
    private DrivetrainMode driveMode = DrivetrainMode.FIELD_CENTRIC;
    private GameState gameState = GameState.IDLE;
    private AllianceColor alliance = AllianceColor.UNKNOWN;
    private CoralState coralState = CoralState.NONE;  // Coral tracking
    
    // Alignment tracking (for conditional auto-scoring)
    private boolean isAlignedToTarget = false;  // True if robot has successfully auto-aligned to scoring target
    
    // State tracking
    private double stateStartTime = 0;
    private double matchStartTime = 0;
    private MatchState previousMatchState = MatchState.DISABLED;
    private GameState previousGameState = GameState.IDLE;
    
    // State history (circular buffer for last 20 states)
    private static final int STATE_HISTORY_SIZE = 20;
    private final List<StateHistoryEntry> stateHistory = new ArrayList<>();
    
    // Cycle counting for auto performance
    private int coralsScoredAuto = 0;
    private int coralsScoredTeleop = 0;
    private int intakeCyclesCompleted = 0;
    private int scoringCyclesCompleted = 0;
    private double lastCycleTime = 0;
    private double fastestCycleTime = Double.MAX_VALUE;
    
    // Driver feedback
    private CommandXboxController driverController;
    private CommandXboxController operatorController;
    private double rumbleEndTime = 0;
    
    /**
     * CORAL STATE - Where is the coral at?
     * Separate from GameState to ensure reliable tracking
     */
    public enum CoralState {
        NONE("No Coral", "Robot has no coral"),
        IN_INTAKE("In Intake", "Coral secured in pivot intake"),
        TRANSFERRING("Transferring", "Coral moving from intake to dump"),
        IN_DUMP("In Dump Roller", "Coral loaded in dump, ready to score"),
        SCORING("Scoring", "Actively ejecting coral");
        
        public final String name;
        public final String description;
        
        CoralState(String name, String description) {
            this.name = name;
            this.description = description;
        }
    }
    
    /**
     * State History Entry for debugging
     */
    private static class StateHistoryEntry {
        public final double timestamp;
        public final String stateType;
        public final String fromState;
        public final String toState;
        
        public StateHistoryEntry(double timestamp, String stateType, String fromState, String toState) {
            this.timestamp = timestamp;
            this.stateType = stateType;
            this.fromState = fromState;
            this.toState = toState;
        }
        
        @Override
        public String toString() {
            return String.format("[%.2f] %s: %s -> %s", timestamp, stateType, fromState, toState);
        }
    }
    
    /**
     * MATCH STATE - Robot's lifecycle phase
     */
    public enum MatchState {
        DISABLED("Robot Disabled", false, false),
        AUTO_INIT("Autonomous Initializing", true, true),
        AUTO_RUNNING("Autonomous Running", true, true),
        TELEOP_INIT("Teleop Initializing", true, false),
        TELEOP_RUNNING("Teleop Running", true, false),
        TEST_INIT("Test Mode Initializing", true, false),
        TEST_RUNNING("Test Mode Running", true, false),
        ENDGAME("Endgame Period", true, false),
        ESTOP("Emergency Stop", false, false);
        
        public final String description;
        public final boolean enabled;
        public final boolean autonomous;
        
        MatchState(String description, boolean enabled, boolean autonomous) {
            this.description = description;
            this.enabled = enabled;
            this.autonomous = autonomous;
        }
    }
    
    /**
     * DRIVETRAIN MODE - How the drivetrain is being controlled
     */
    public enum DrivetrainMode {
        FIELD_CENTRIC("Field-Centric Drive", "Driver controls relative to field"),
        ROBOT_CENTRIC("Robot-Centric Drive", "Driver controls relative to robot"),
        PATH_FOLLOWING("Path Following", "Following PathPlanner trajectory"),
        VISION_TRACKING("Vision Tracking", "Limelight auto-aiming"),
        LOCKED("Wheels Locked", "X-formation brake"),
        SLOW_MODE("Slow Precision Mode", "Reduced speed for alignment"),
        AUTO_ALIGN("Auto-Aligning", "Automatic positioning for scoring"),
        DISABLED("Drivetrain Disabled", "No movement");
        
        public final String name;
        public final String description;
        
        DrivetrainMode(String name, String description) {
            this.name = name;
            this.description = description;
        }
    }
    
    /**
     * GAME STATE - High-level robot strategy/behavior
     * This is the GAME STRATEGY state machine
     */
    public enum GameState {
        // Pre-game
        IDLE("Idle/Stowed", "Robot waiting, all mechanisms stowed"),
        PRE_MATCH_CHECK("Pre-Match Check", "Running systems check"),
        
        // Autonomous
        AUTO_SCORING_PRELOAD("Auto: Scoring Preload", "Autonomous scoring preloaded coral"),
        AUTO_COLLECTING("Auto: Collecting Coral", "Autonomous ground collection"),
        AUTO_NAVIGATING("Auto: Navigating", "Following path to position"),
        AUTO_SCORING("Auto: Scoring", "Autonomous scoring sequence"),
        
        // Teleop - Collecting
        SEEKING_CORAL("Seeking Coral", "Driving to coral location"),
        COLLECTING_GROUND("Collecting from Ground", "Ground intake active"),
        CORAL_SECURED("Coral Secured", "Coral in intake, ready to transfer"),
        TRANSFERRING("Transferring Coral", "Moving coral to dump roller"),
        
        // Teleop - Scoring
        CORAL_LOADED("Coral Loaded", "Coral in dump, ready to score"),
        APPROACHING_SCORING("Approaching Scoring Position", "Driving to score location"),
        ALIGNING_TO_SCORE("Aligning to Score", "Fine-tuning position for score"),
        SCORING_PROCESSOR("Scoring Processor (L1)", "Scoring at Level 1"),
        SCORING_REEF_L2("Scoring Reef Level 2", "Scoring at Level 2"),
        SCORING_REEF_L3("Scoring Reef Level 3", "Scoring at Level 3"),
        SCORING_REEF_L4("Scoring Reef Level 4", "Scoring at Level 4"),
        SCORING_BARGE("Scoring Barge", "Placing coral on barge"),
        
        // Teleop - Special Actions
        DEFENDING("Defending", "Blocking opponent access"),
        DEEP_COLLECT("Deep Collecting", "Collecting from deep zone"),
        ALGAE_MANAGEMENT("Algae Management", "Handling algae"),
        REPOSITIONING("Repositioning", "Moving to strategic position"),
        
        // Endgame - added some new stuff even tho we dont have climb YET... :(
        APPROACHING_CLIMB("Approaching Climb", "Driving to cage/barge for climb"),
        PREPARING_CLIMB("Preparing to Climb", "Setting up mechanisms for climb"),
        CLIMBING_SHALLOW("Climbing Shallow", "Climbing to shallow position (2pts)"),
        CLIMBING_DEEP("Climbing Deep", "Climbing to deep position (6pts)"),
        CLIMBING("Climbing", "Actively climbing"),
        CLIMBED_SHALLOW("Climbed Shallow", "Successfully on shallow bar"),
        CLIMBED_DEEP("Climbed Deep", "Successfully on deep bar"),
        CLIMBED("Climbed", "Successfully on bar"),
        
        // Error/Recovery
        RECOVERING("Recovering from Error", "Attempting error recovery"),
        MANUAL_OVERRIDE("Manual Override", "Driver direct control"),
        EMERGENCY_STOP("Emergency Stop", "All systems halted");
        
        public final String name;
        public final String description;
        
        GameState(String name, String description) {
            this.name = name;
            this.description = description;
        }
        
        // Helper methods for state categories
        public boolean isCollecting() {
            return this == COLLECTING_GROUND || this == AUTO_COLLECTING || this == DEEP_COLLECT;
        }
        
        public boolean isScoring() {
            return this == SCORING_PROCESSOR || this == SCORING_REEF_L2 || 
                   this == SCORING_REEF_L3 || this == SCORING_REEF_L4 || 
                   this == SCORING_BARGE || this == AUTO_SCORING;
        }
        
        public boolean isNavigating() {
            return this == APPROACHING_SCORING || this == APPROACHING_CLIMB || 
                   this == AUTO_NAVIGATING || this == SEEKING_CORAL;
        }
        
        public boolean isEndgame() {
            // Even though we don't have climbing yet, this is good practice/learning for next year/season :]
            return this == APPROACHING_CLIMB || this == PREPARING_CLIMB || 
                   this == CLIMBING || this == CLIMBING_SHALLOW || this == CLIMBING_DEEP ||
                   this == CLIMBED || this == CLIMBED_SHALLOW || this == CLIMBED_DEEP;
        }
        
        public boolean isClimbed() {
            return this == CLIMBED || this == CLIMBED_SHALLOW || this == CLIMBED_DEEP;
        }
        
        public boolean isClimbing() {
            return this == CLIMBING || this == CLIMBING_SHALLOW || this == CLIMBING_DEEP;
        }
    }
    
    /**
     * ALLIANCE COLOR
     */
    public enum AllianceColor {
        RED("Red Alliance"),
        BLUE("Blue Alliance"),
        UNKNOWN("Unknown Alliance");
        
        public final String description;
        
        AllianceColor(String description) {
            this.description = description;
        }
    }
    
    /**
     * Singleton pattern
     */
    public static RobotStateMachine getInstance() {
        if (instance == null) {
            instance = new RobotStateMachine();
        }
        return instance;
    }
    
    private RobotStateMachine() {
        updateAlliance();
    }
    
    /**
     * Update alliance color from DriverStation
     */
    public void updateAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance().get() == Alliance.Red ? 
                       AllianceColor.RED : AllianceColor.BLUE;
        } else {
            alliance = AllianceColor.UNKNOWN;
        }
    }
    
    /**
     * Transition to new match state
     */
    public void setMatchState(MatchState newState) {
        if (matchState != newState) {
            previousMatchState = matchState;
            matchState = newState;
            stateStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            onMatchStateChange();
            logStateChange("Match", previousMatchState.name(), newState.name());
        }
    }
    
    /**
     * Transition to new game state
     */
    public void setGameState(GameState newState) {
        if (gameState != newState) {
            previousGameState = gameState;
            gameState = newState;
            stateStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
            onGameStateChange();
            logStateChange("Game", previousGameState.name(), newState.name());
        }
    }
    
    /**
     * Transition to new drivetrain mode
     */
    public void setDrivetrainMode(DrivetrainMode newMode) {
        if (driveMode != newMode) {
            DrivetrainMode previousMode = driveMode;
            driveMode = newMode;
            logStateChange("Drivetrain", previousMode.name(), newMode.name());
        }
    }
    
    /**
     * Handle match state transitions
     */
    private void onMatchStateChange() {
        switch (matchState) {
            case DISABLED:
                setGameState(GameState.IDLE);
                setDrivetrainMode(DrivetrainMode.DISABLED);
                isAlignedToTarget = false;  // Reset alignment on disable
                break;
                
            case AUTO_INIT:
                setGameState(GameState.AUTO_SCORING_PRELOAD);
                setDrivetrainMode(DrivetrainMode.PATH_FOLLOWING);
                isAlignedToTarget = false;  // Reset alignment on match start
                updateAlliance();
                break;
                
            case TELEOP_INIT:
                setGameState(GameState.IDLE);
                setDrivetrainMode(DrivetrainMode.FIELD_CENTRIC);
                isAlignedToTarget = false;  // Reset alignment on teleop start
                break;
                
            case ENDGAME:
                // Could auto-transition to climbing strategy
                System.out.println("ENDGAME PERIOD STARTED!");
                break;
                
            default:
                break;
        }
    }
    
    /**
     * Handle game state transitions
     * Drivetrain mode is now decoupled from game state! :)
     * This allows drivetrain alignment to run independently of manipulator operations.
     * Only critical safety states (EMERGENCY_STOP) still affect drivetrain.
     */
    private void onGameStateChange() {
        switch (gameState) {
            // Only EMERGENCY_STOP should affect drivetrain
            // This decouples alignment from intake/scoring operations
            case EMERGENCY_STOP:
                setDrivetrainMode(DrivetrainMode.DISABLED);
                break;
                
            case CLIMBING:
            case CLIMBING_SHALLOW:
            case CLIMBING_DEEP:
            case CLIMBED:
            case CLIMBED_SHALLOW:
            case CLIMBED_DEEP:
                // Climbing is a special case - lock wheels for safety
                setDrivetrainMode(DrivetrainMode.LOCKED);
                break;
                
            // All other game states do NOT change drivetrain mode
            // This allows AlignReef to run while intake is collecting!
            default:
                // Do nothing - let drivetrain commands control their own mode
                break;
        }
    }
    
    /**
     * Get time in current state (seconds)
     */
    public double getTimeInState() {
        return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - stateStartTime;
    }
    
    /**
     * Check if match time indicates endgame
     */
    public boolean isEndgamePeriod() {
        return DriverStation.isTeleopEnabled() && 
               DriverStation.getMatchTime() > 0 && 
               DriverStation.getMatchTime() <= 30;
    }
    
    /**
     * Automatic endgame detection
     */
    private void checkEndgameTransition() {
        if (matchState == MatchState.TELEOP_RUNNING && isEndgamePeriod()) {
            setMatchState(MatchState.ENDGAME);
        }
    }
    
    /**
     * State validation - prevent invalid transitions
     */
    public boolean canTransitionTo(GameState newState) {
        // Disabled can only be IDLE
        if (matchState == MatchState.DISABLED && newState != GameState.IDLE) {
            return false;
        }
        
        // Auto states only in auto
        if (newState.name().startsWith("AUTO_") && !matchState.autonomous) {
            return false;
        }
        
        // Can always emergency stop
        if (newState == GameState.EMERGENCY_STOP) {
            return true;
        }
        
        return true;
    }
    
    /**
     * Safe state transition with validation
     */
    public void requestGameState(GameState newState) {
        if (canTransitionTo(newState)) {
            setGameState(newState);
        } else {
            System.out.println("INVALID STATE TRANSITION: " + gameState + " -> " + newState);
        }
    }
    
    /**
     * Log state changes
     */
    private void logStateChange(String type, String from, String to) {
        String message = String.format("%s State: %s -> %s", type, from, to);
        System.out.println(message);
        
        // AdvantageKit logging
        Logger.recordOutput("StateMachine/" + type + "/Transition", message);
        Logger.recordOutput("StateMachine/" + type + "/From", from);
        Logger.recordOutput("StateMachine/" + type + "/To", to);
        Logger.recordOutput("StateMachine/" + type + "/Timestamp", edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
        
        // SmartDashboard (legacy)
        SmartDashboard.putString("Last State Change", message);
    }
    
    @Override
    public void periodic() {
        // Automatic endgame detection
        checkEndgameTransition();
        
        // Update driver feedback
        updateRumble();

        // Update telemetry
        updateTelemetry();
    }
    
    /**
     * Telemetry logging
     */
    private void updateTelemetry() {
        // ADVANTAGEKIT LOGGING
        
        // Match State
        Logger.recordOutput("StateMachine/Match/State", matchState.name());
        Logger.recordOutput("StateMachine/Match/Description", matchState.description);
        Logger.recordOutput("StateMachine/Match/Enabled", matchState.enabled);
        Logger.recordOutput("StateMachine/Match/Autonomous", matchState.autonomous);
        
        // Game State
        Logger.recordOutput("StateMachine/Game/State", gameState.name());
        Logger.recordOutput("StateMachine/Game/Description", gameState.description);
        Logger.recordOutput("StateMachine/Game/IsCollecting", gameState.isCollecting());
        Logger.recordOutput("StateMachine/Game/IsScoring", gameState.isScoring());
        Logger.recordOutput("StateMachine/Game/IsNavigating", gameState.isNavigating());
        Logger.recordOutput("StateMachine/Game/IsEndgame", gameState.isEndgame());
        
        // Drivetrain Mode
        Logger.recordOutput("StateMachine/Drivetrain/Mode", driveMode.name);
        Logger.recordOutput("StateMachine/Drivetrain/Description", driveMode.description);
        
        // Coral State
        Logger.recordOutput("StateMachine/Coral/State", coralState.name());
        Logger.recordOutput("StateMachine/Coral/HasCoral", hasCoral());
        Logger.recordOutput("StateMachine/Coral/ReadyToScore", isReadyToScore());
        
        // Cycle Statistics (auto performance tracking)
        Logger.recordOutput("StateMachine/Cycles/CoralsScoredAuto", coralsScoredAuto);
        Logger.recordOutput("StateMachine/Cycles/CoralsScoredTeleop", coralsScoredTeleop);
        Logger.recordOutput("StateMachine/Cycles/TotalScored", getTotalCoralsScored());
        Logger.recordOutput("StateMachine/Cycles/IntakeCycles", intakeCyclesCompleted);
        Logger.recordOutput("StateMachine/Cycles/ScoringCycles", scoringCyclesCompleted);
        Logger.recordOutput("StateMachine/Cycles/FastestCycleTime", getFastestCycleTime());
        Logger.recordOutput("StateMachine/Cycles/MatchElapsedTime", getMatchElapsedTime());
        
        // Alliance & Timing
        Logger.recordOutput("StateMachine/Alliance", alliance.description);
        Logger.recordOutput("StateMachine/TimeInState", getTimeInState());
        Logger.recordOutput("StateMachine/IsEndgamePeriod", isEndgamePeriod());
        
        // Previous States (for transition tracking)
        Logger.recordOutput("StateMachine/Match/PreviousState", previousMatchState.name());
        Logger.recordOutput("StateMachine/Game/PreviousState", previousGameState.name());
        
        // SMARTDASHBOARD (LEGACY)
        SmartDashboard.putString("Match State", matchState.name());
        SmartDashboard.putString("Match Description", matchState.description);
        SmartDashboard.putString("Game State", gameState.name());
        SmartDashboard.putString("Game Description", gameState.description);
        SmartDashboard.putString("Drivetrain Mode", driveMode.name);
        SmartDashboard.putString("Drive Description", driveMode.description);
        SmartDashboard.putString("Alliance", alliance.description);
        SmartDashboard.putNumber("Time in State", getTimeInState());
        SmartDashboard.putBoolean("Is Endgame", isEndgamePeriod());
        SmartDashboard.putBoolean("Robot Enabled", matchState.enabled);
        SmartDashboard.putBoolean("Is Autonomous", matchState.autonomous);
        SmartDashboard.putBoolean("Is Collecting", gameState.isCollecting());
        SmartDashboard.putBoolean("Is Scoring", gameState.isScoring());
        SmartDashboard.putBoolean("Is Navigating", gameState.isNavigating());
        
        // Coral SmartDashboard
        SmartDashboard.putString("Coral State", coralState.name());
        SmartDashboard.putBoolean("Has Coral", hasCoral());
        SmartDashboard.putBoolean("Ready to Score", isReadyToScore());
        SmartDashboard.putNumber("Corals Scored (Auto)", coralsScoredAuto);
        SmartDashboard.putNumber("Corals Scored (Teleop)", coralsScoredTeleop);
        SmartDashboard.putNumber("Total Corals Scored", getTotalCoralsScored());
    }
    
    // Getters
    public MatchState getMatchState() { return matchState; }
    public GameState getGameState() { return gameState; }
    public DrivetrainMode getDrivetrainMode() { return driveMode; }
    public AllianceColor getAlliance() { return alliance; }
    public CoralState getCoralState() { return coralState; }
    public boolean isEnabled() { return matchState.enabled; }
    public boolean isAutonomous() { return matchState.autonomous; }
    public boolean isTeleop() { return matchState == MatchState.TELEOP_RUNNING || matchState == MatchState.ENDGAME; }
    
    // State queries
    public boolean isInState(MatchState state) { return matchState == state; }
    public boolean isInState(GameState state) { return gameState == state; }
    public boolean isInState(DrivetrainMode mode) { return driveMode == mode; }
    public boolean isInState(CoralState state) { return coralState == state; }
    
    // Coral state management
    public boolean hasCoral() { 
        return coralState == CoralState.IN_INTAKE || 
               coralState == CoralState.IN_DUMP || 
               coralState == CoralState.TRANSFERRING; 
    }
    public boolean hasCoralInIntake() { return coralState == CoralState.IN_INTAKE; }
    public boolean hasCoralInDump() { return coralState == CoralState.IN_DUMP; }
    public boolean isReadyToScore() { return coralState == CoralState.IN_DUMP; }
    
    /**
     * Alignment tracking methods (for conditional auto-scoring)
     */
    public void setAlignedToTarget(boolean aligned) {
        if (isAlignedToTarget != aligned) {
            isAlignedToTarget = aligned;
            Logger.recordOutput("StateMachine/AlignedToTarget", aligned);
            if (aligned) {
                rumbleDriver(0.4, 0.15); // Quick rumble - alignment successful
            }
        }
    }
    
    public boolean isAlignedToTarget() {
        return isAlignedToTarget;
    }
    
    /**
     * Set coral state with automatic driver feedback
     */
    public void setCoralState(CoralState newState) {
        if (coralState != newState) {
            CoralState previousState = coralState;
            coralState = newState;
            
            // Log state change
            addToStateHistory("Coral", previousState.name(), newState.name());
            Logger.recordOutput("StateMachine/Coral/State", newState.name());
            Logger.recordOutput("StateMachine/Coral/HasCoral", hasCoral());
            
            // Driver feedback on transitions
            switch (newState) {
                case IN_INTAKE:
                    rumbleDriver(0.3, 0.2); // Short rumble - coral collected
                    intakeCyclesCompleted++;
                    break;
                case IN_DUMP:
                    rumbleDriver(0.5, 0.3); // Medium rumble - ready to score
                    break;
                case NONE:
                    if (previousState == CoralState.SCORING) {
                        rumbleDriver(1.0, 0.5); // Strong rumble - scored!
                        if (isAutonomous()) {
                            coralsScoredAuto++;
                        } else {
                            coralsScoredTeleop++;
                        }
                        scoringCyclesCompleted++;
                        updateCycleTime();
                    }
                    break;
                default:
                    break;
            }
        }
    }
    
    /**
     * Register controllers for driver feedback
     */
    public void registerControllers(CommandXboxController driver, CommandXboxController operator) {
        this.driverController = driver;
        this.operatorController = operator;
    }
    
    /**
     * Rumble driver controller for feedback
     */
    public void rumbleDriver(double intensity, double durationSeconds) {
        if (driverController != null) {
            driverController.getHID().setRumble(RumbleType.kBothRumble, intensity);
            rumbleEndTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() + durationSeconds;
        }
    }
    
    /**
     * Rumble operator controller for feedback
     */
    public void rumbleOperator(double intensity, double durationSeconds) {
        if (operatorController != null) {
            operatorController.getHID().setRumble(RumbleType.kBothRumble, intensity);
            rumbleEndTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() + durationSeconds;
        }
    }
    
    /**
     * Check and stop rumble when duration elapsed
     */
    private void updateRumble() {
        if (rumbleEndTime > 0 && edu.wpi.first.wpilibj.Timer.getFPGATimestamp() >= rumbleEndTime) {
            if (driverController != null) {
                driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
            }
            if (operatorController != null) {
                operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
            }
            rumbleEndTime = 0;
        }
    }
    
    /**
     * Add entry to state history
     */
    private void addToStateHistory(String stateType, String from, String to) {
        StateHistoryEntry entry = new StateHistoryEntry(
            edu.wpi.first.wpilibj.Timer.getFPGATimestamp(),
            stateType, from, to
        );
        stateHistory.add(entry);
        if (stateHistory.size() > STATE_HISTORY_SIZE) {
            stateHistory.remove(0);
        }
    }
    
    /**
     * Get state history for debugging
     */
    public List<StateHistoryEntry> getStateHistory() {
        return new ArrayList<>(stateHistory);
    }
    
    /**
     * Print state history to console
     */
    public void printStateHistory() {
        System.out.println("=== STATE HISTORY ===");
        for (StateHistoryEntry entry : stateHistory) {
            System.out.println(entry);
        }
        System.out.println("====================");
    }
    
    // Cycle time tracking
    private void updateCycleTime() {
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (lastCycleTime > 0) {
            double cycleTime = now - lastCycleTime;
            if (cycleTime < fastestCycleTime && cycleTime > 1.0) { // Ignore <1s cycles as they are most likely errors
                fastestCycleTime = cycleTime;
            }
        }
        lastCycleTime = now;
    }
    
    // Cycle statistics getters
    public int getCoralsScoredAuto() { return coralsScoredAuto; }
    public int getCoralsScoredTeleop() { return coralsScoredTeleop; }
    public int getTotalCoralsScored() { return coralsScoredAuto + coralsScoredTeleop; }
    public int getIntakeCyclesCompleted() { return intakeCyclesCompleted; }
    public int getScoringCyclesCompleted() { return scoringCyclesCompleted; }
    public double getFastestCycleTime() { return fastestCycleTime == Double.MAX_VALUE ? 0 : fastestCycleTime; }
    
    /**
     * Reset cycle counters (call at match start)
     */
    public void resetCycleCounters() {
        coralsScoredAuto = 0;
        coralsScoredTeleop = 0;
        intakeCyclesCompleted = 0;
        scoringCyclesCompleted = 0;
        lastCycleTime = 0;
        fastestCycleTime = Double.MAX_VALUE;
        stateHistory.clear();
        matchStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }
    
    /**
     * Get match elapsed time
     */
    public double getMatchElapsedTime() {
        if (matchStartTime == 0) return 0;
        return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - matchStartTime;
    }
}
