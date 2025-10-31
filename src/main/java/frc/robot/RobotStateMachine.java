package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    
    // State tracking
    private double stateStartTime = 0;
    private MatchState previousMatchState = MatchState.DISABLED;
    private GameState previousGameState = GameState.IDLE;
    
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
        
        // Endgame
        APPROACHING_CLIMB("Approaching Climb", "Driving to climb position"),
        PREPARING_CLIMB("Preparing to Climb", "Setting up for climb"),
        CLIMBING("Climbing", "Actively climbing"),
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
            return this == APPROACHING_CLIMB || this == PREPARING_CLIMB || 
                   this == CLIMBING || this == CLIMBED;
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
                break;
                
            case AUTO_INIT:
                setGameState(GameState.AUTO_SCORING_PRELOAD);
                setDrivetrainMode(DrivetrainMode.PATH_FOLLOWING);
                updateAlliance();
                break;
                
            case TELEOP_INIT:
                setGameState(GameState.IDLE);
                setDrivetrainMode(DrivetrainMode.FIELD_CENTRIC);
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
     */
    private void onGameStateChange() {
        switch (gameState) {
            case ALIGNING_TO_SCORE:
                setDrivetrainMode(DrivetrainMode.VISION_TRACKING);
                break;
                
            case AUTO_NAVIGATING:
                setDrivetrainMode(DrivetrainMode.PATH_FOLLOWING);
                break;
                
            case CLIMBING:
            case CLIMBED:
                setDrivetrainMode(DrivetrainMode.LOCKED);
                break;
                
            case MANUAL_OVERRIDE:
                setDrivetrainMode(DrivetrainMode.ROBOT_CENTRIC);
                break;
                
            case EMERGENCY_STOP:
                setDrivetrainMode(DrivetrainMode.DISABLED);
                break;
                
            default:
                if (matchState == MatchState.TELEOP_RUNNING && 
                    driveMode != DrivetrainMode.FIELD_CENTRIC &&
                    driveMode != DrivetrainMode.SLOW_MODE) {
                    setDrivetrainMode(DrivetrainMode.FIELD_CENTRIC);
                }
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
    }
    
    // Getters
    public MatchState getMatchState() { return matchState; }
    public GameState getGameState() { return gameState; }
    public DrivetrainMode getDrivetrainMode() { return driveMode; }
    public AllianceColor getAlliance() { return alliance; }
    public boolean isEnabled() { return matchState.enabled; }
    public boolean isAutonomous() { return matchState.autonomous; }
    public boolean isTeleop() { return matchState == MatchState.TELEOP_RUNNING || matchState == MatchState.ENDGAME; }
    
    // State queries
    public boolean isInState(MatchState state) { return matchState == state; }
    public boolean isInState(GameState state) { return gameState == state; }
    public boolean isInState(DrivetrainMode mode) { return driveMode == mode; }
}
