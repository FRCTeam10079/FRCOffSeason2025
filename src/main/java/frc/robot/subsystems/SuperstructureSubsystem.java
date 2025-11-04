package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateMachine;
import frc.robot.RobotStateMachine.GameState;

/**
 * Superstructure state machine for the robot.
 * This subsystem coordinates all mechanisms (elevator, pivot intake, dump roller)
 * and manages game-specific scoring sequences through state transitions.
 */
public class SuperstructureSubsystem extends SubsystemBase {
    
    // Master state machine reference
    private final RobotStateMachine masterStateMachine = RobotStateMachine.getInstance();
    
    // Subsystem dependencies
    private final ElevatorSubsystem elevator;
    private final PivotIntakeSubsystem pivotIntake;
    private final DumpRollerSubsystem dumpRoller;
    
    // Current state
    private RobotState currentState = RobotState.IDLE;
    private RobotState requestedState = RobotState.IDLE;
    
    /**
     * Robot State Machine States
     * Game-specific states for coral manipulation and reef scoring
     */
    public enum RobotState {
        // IDLE/HOME STATES
        IDLE(0, 0.42, false, "Robot stowed and ready"),
        
        // INTAKE STATES - Ground coral collection
        GROUND_INTAKE_DEPLOY(0, 0.0, false, "Deploying intake to ground"),
        GROUND_INTAKE_COLLECTING(0, 0.0, true, "Collecting coral from ground"),
        GROUND_INTAKE_STOWING(0, 0.42, false, "Stowing intake with coral"),
        
        // TRANSFER STATES - Moving coral from intake to dump roller
        TRANSFERRING_TO_DUMP(0, 0.42, true, "Transferring coral to dump roller"),
        CORAL_LOADED(0, 0.42, false, "Coral loaded in dump roller, ready to score"),
        
        // SCORING STATES - Reef scoring at different levels
        SCORING_L1_PREP(1, 0.42, false, "Preparing to score Level 1"),
        SCORING_L1_EXECUTE(1, 0.42, false, "Scoring at Level 1"),
        
        SCORING_L2_PREP(2, 0.42, false, "Preparing to score Level 2"),
        SCORING_L2_EXECUTE(2, 0.42, false, "Scoring at Level 2"),
        
        SCORING_L3_PREP(3, 0.42, false, "Preparing to score Level 3"),
        SCORING_L3_EXECUTE(3, 0.42, false, "Scoring at Level 3"),
        
        SCORING_L4_PREP(4, 0.42, false, "Preparing to score Level 4"),
        SCORING_L4_EXECUTE(4, 0.42, false, "Scoring at Level 4"),
        
        // REEF SCORING STATE - For L1/L2 reef placement
        REEF_SCORING_PREP(1, 0.35, false, "Preparing for reef scoring"),
        REEF_SCORING_EXECUTE(1, 0.35, false, "Executing reef score"),
        
        // MANUAL OVERRIDE
        MANUAL(0, 0.42, false, "Manual control active");
        
        // State properties - goals for each subsystem
        public final int elevatorLevel;
        public final double pivotPosition;
        public final boolean intakeWheelsActive;
        public final String description;
        
        RobotState(int elevatorLevel, double pivotPosition, boolean intakeWheelsActive, String description) {
            this.elevatorLevel = elevatorLevel;
            this.pivotPosition = pivotPosition;
            this.intakeWheelsActive = intakeWheelsActive;
            this.description = description;
        }
    }
    
    /**
     * Constructor - inject subsystem dependencies
     */
    public SuperstructureSubsystem(
            ElevatorSubsystem elevator,
            PivotIntakeSubsystem pivotIntake,
            DumpRollerSubsystem dumpRoller) {
        this.elevator = elevator;
        this.pivotIntake = pivotIntake;
        this.dumpRoller = dumpRoller;
    }
    
    /**
     * Request a state transition
     */
    public void requestState(RobotState newState) {
        requestedState = newState;
        // basically just put some code here that works
    }
    
    /**
     * Get current state
     */
    public RobotState getCurrentState() {
        return currentState;
    }
    
    /**
     * Check if robot is in a specific state
     */
    public boolean isInState(RobotState state) {
        return currentState == state;
    }
    
    /**
     * Check if subsystems have reached state goals
     */
    private boolean hasReachedStateGoals() {
        RobotState state = currentState;
        
        // Check elevator position
        boolean elevatorAtGoal = Math.abs(
            elevator.getPosition() - elevator.positions[state.elevatorLevel]
        ) < 0.5;
        
        // Check pivot position
        boolean pivotAtGoal = Math.abs(
            pivotIntake.getPivotPosition() - state.pivotPosition
        ) < 0.02;
        
        return elevatorAtGoal && pivotAtGoal;
    }
    
    @Override
    public void periodic() {
        // State machine update - apply state goals to subsystems
        if (currentState != requestedState) {
            // Transition to new state
            currentState = requestedState;
            System.out.println("State transition: " + currentState.name() + " - " + currentState.description);
            
            // AdvantageKit: Log state transition
            Logger.recordOutput("Superstructure/StateTransition", currentState.name() + " - " + currentState.description);
            Logger.recordOutput("Superstructure/TransitionTimestamp", edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
        }
        
        // Apply state goals to subsystems
        applyStateGoals();
        
        // Update telemetry (AdvantageKit + SmartDashboard)
        updateTelemetry();
    }
    
    /**
     * Update telemetry
     */
    private void updateTelemetry() {
        // ADVANTAGEKIT LOGGING
        Logger.recordOutput("Superstructure/State/Current", currentState.name());
        Logger.recordOutput("Superstructure/State/Description", currentState.description);
        Logger.recordOutput("Superstructure/State/RequestedState", requestedState.name());
        Logger.recordOutput("Superstructure/Goals/AtGoals", hasReachedStateGoals());
        Logger.recordOutput("Superstructure/Goals/ElevatorLevel", currentState.elevatorLevel);
        Logger.recordOutput("Superstructure/Goals/PivotPosition", currentState.pivotPosition);
        Logger.recordOutput("Superstructure/Goals/IntakeWheelsActive", currentState.intakeWheelsActive);
        Logger.recordOutput("Superstructure/Coral/InIntake", pivotIntake.hasCoralInIntake());
        Logger.recordOutput("Superstructure/Coral/InDump", dumpRoller.hasCoralLoaded());
        
        // SMARTDASHBOARD (LEGACY)
        SmartDashboard.putString("Superstructure State", currentState.name());
        SmartDashboard.putString("State Description", currentState.description);
        SmartDashboard.putBoolean("At State Goals", hasReachedStateGoals());
        SmartDashboard.putBoolean("Has Coral in Intake", pivotIntake.hasCoralInIntake());
        SmartDashboard.putBoolean("Has Coral in Dump", dumpRoller.hasCoralLoaded());
    }
    
    /**
     * Apply current state's goals to all subsystems
     */
    private void applyStateGoals() {
        // Set elevator position
        elevator.pos = currentState.elevatorLevel;
        
        // Set pivot position
        pivotIntake.setPivotSetpoint(currentState.pivotPosition);
        
        // Intake wheels are controlled by commands, not continuous state
        // This allows for precise timing in sequences
    }
    
    /**
     * GROUND INTAKE: Collect coral from ground
     * Updates master state machine with game states
     */
    public Command collectCoralFromGround() {
        return Commands.sequence(
            // Update master game state
            Commands.runOnce(() -> masterStateMachine.setGameState(GameState.COLLECTING_GROUND)),
            
            // State 1: Deploy intake
            Commands.runOnce(() -> requestState(RobotState.GROUND_INTAKE_DEPLOY)),
            Commands.waitUntil(this::hasReachedStateGoals),
            
            // State 2: Run wheels and collect
            Commands.runOnce(() -> requestState(RobotState.GROUND_INTAKE_COLLECTING)),
            pivotIntake.intakeWheels().until(pivotIntake::isCoralDetected),
            pivotIntake.stopWheels(),
            Commands.runOnce(() -> pivotIntake.setHasCoralInIntake(true)),
            Commands.waitSeconds(0.125),
            
            // State 3: Stow with coral
            Commands.runOnce(() -> requestState(RobotState.GROUND_INTAKE_STOWING)),
            Commands.waitUntil(this::hasReachedStateGoals),
            
            // Ready state
            Commands.runOnce(() -> requestState(RobotState.IDLE)),
            Commands.runOnce(() -> masterStateMachine.setGameState(GameState.CORAL_SECURED))
        );
    }
    
    /**
     * FULL AUTO INTAKE: Collect and transfer to dump in one sequence
     */
    public Command collectAndTransfer() {
        return Commands.sequence(
            // Collect from ground
            collectCoralFromGround(),
            
            // Transfer to dump
            transferCoralToDump()
        );
    }
    
    /**
     * TRANSFER: Move coral from intake to dump roller
     */
    public Command transferCoralToDump() {
        return Commands.sequence(
            // Update master game state
            Commands.runOnce(() -> masterStateMachine.setGameState(GameState.TRANSFERRING)),
            
            // State: Transferring
            Commands.runOnce(() -> requestState(RobotState.TRANSFERRING_TO_DUMP)),
            
            // Run both motors until current spike
            Commands.parallel(
                pivotIntake.reverseIntakeWheels(),
                dumpRoller.dropCoral(0.2)
                    .until(() -> dumpRoller.coralMotor.getStatorCurrent().getValueAsDouble() > 35)
            ),
            
            // Stop motors and update states
            pivotIntake.stopWheels(),
            dumpRoller.keepCoral(),
            Commands.runOnce(() -> {
                pivotIntake.setHasCoralInIntake(false);
                dumpRoller.setCoralLoaded(true);
            }),
            
            // State: Coral loaded and ready
            Commands.runOnce(() -> requestState(RobotState.CORAL_LOADED)),
            Commands.runOnce(() -> masterStateMachine.setGameState(GameState.CORAL_LOADED))
        );
    }
    
    /**
     * SCORE L1: Score coral at Level 1 (Processor)
     */
    public Command scoreLevel1() {
        return scoreAtLevel(RobotState.SCORING_L1_PREP, RobotState.SCORING_L1_EXECUTE, 0.1);
    }
    
    /**
     * SCORE L2: Score coral at Level 2
     */
    public Command scoreLevel2() {
        return scoreAtLevel(RobotState.SCORING_L2_PREP, RobotState.SCORING_L2_EXECUTE, 0.2);
    }
    
    /**
     * SCORE L3: Score coral at Level 3
     */
    public Command scoreLevel3() {
        return scoreAtLevel(RobotState.SCORING_L3_PREP, RobotState.SCORING_L3_EXECUTE, 0.2);
    }
    
    /**
     * SCORE L4: Score coral at Level 4
     */
    public Command scoreLevel4() {
        return scoreAtLevel(RobotState.SCORING_L4_PREP, RobotState.SCORING_L4_EXECUTE, 0.2);
    }
    
    /**
     * SCORE REEF: Score coral on reef (L1/L2 height with pivot out)
     */
    public Command scoreReef() {
        return Commands.sequence(
            // Prep: Move to reef scoring position
            Commands.runOnce(() -> requestState(RobotState.REEF_SCORING_PREP)),
            Commands.waitUntil(this::hasReachedStateGoals),
            
            // Execute: Launch coral
            Commands.runOnce(() -> requestState(RobotState.REEF_SCORING_EXECUTE)),
            dumpRoller.dropCoral(0.2).withTimeout(0.5),
            Commands.runOnce(() -> dumpRoller.setCoralLoaded(false)),
            dumpRoller.keepCoral(),
            
            // Return to idle
            Commands.runOnce(() -> requestState(RobotState.IDLE))
        );
    }
    
    /**
     * Generic scoring sequence helper
     * Updates master game state during scoring
     */
    private Command scoreAtLevel(RobotState prepState, RobotState executeState, double launchSpeed) {
        // Determine game state based on level
        GameState scoringState = GameState.SCORING_PROCESSOR;
        if (prepState == RobotState.SCORING_L2_PREP) scoringState = GameState.SCORING_REEF_L2;
        else if (prepState == RobotState.SCORING_L3_PREP) scoringState = GameState.SCORING_REEF_L3;
        else if (prepState == RobotState.SCORING_L4_PREP) scoringState = GameState.SCORING_REEF_L4;
        
        final GameState finalScoringState = scoringState;
        
        return Commands.sequence(
            // Update master game state
            Commands.runOnce(() -> masterStateMachine.setGameState(finalScoringState)),
            
            // Prep: Move elevator and mechanisms to scoring position
            Commands.runOnce(() -> requestState(prepState)),
            Commands.waitUntil(this::hasReachedStateGoals),
            
            // Execute: Launch coral
            Commands.runOnce(() -> requestState(executeState)),
            dumpRoller.dropCoral(launchSpeed).withTimeout(0.5),
            Commands.runOnce(() -> dumpRoller.setCoralLoaded(false)),
            dumpRoller.keepCoral(),
            
            // Return to idle
            Commands.runOnce(() -> requestState(RobotState.IDLE)),
            Commands.runOnce(() -> masterStateMachine.setGameState(GameState.IDLE)),
            Commands.waitSeconds(0.5),
            elevator.setPosition(0)
        );
    }
    
    /**
     * IDLE: Return robot to home/stowed position
     */
    public Command returnToIdle() {
        return Commands.sequence(
            Commands.runOnce(() -> requestState(RobotState.IDLE)),
            Commands.waitUntil(this::hasReachedStateGoals)
        );
    }
    
    /**
     * MANUAL MODE: Allow direct subsystem control (bypasses state machine)
     */
    public Command enterManualMode() {
        return Commands.runOnce(() -> requestState(RobotState.MANUAL));
    }
    
    /**
     * EXIT MANUAL MODE: Return to state machine control
     */
    public Command exitManualMode() {
        return Commands.runOnce(() -> requestState(RobotState.IDLE));
    }
    
    /**
     * PREPARE CORAL OUT: Small adjustment for scoring alignment
     */
    public Command prepareCoralOut() {
        return dumpRoller.PrepareCoral(true);
    }
    
    /**
     * PREPARE CORAL IN: Small adjustment to secure coral
     */
    public Command prepareCoralIn() {
        return dumpRoller.PrepareCoral(false);
    }
    
    /**
     * VISION ALIGN TO REEF: Limelight-based vision alignment
     * @param alignCommand The AlignReef command to execute
     * @return Wrapped command with state machine integration
     */
    public Command visionAlignReef(Command alignCommand) {
        return Commands.sequence(
            // Update master game state
            Commands.runOnce(() -> masterStateMachine.setGameState(GameState.ALIGNING_TO_SCORE)),
            
            // Execute vision alignment (AlignReef already sets VISION_TRACKING mode)
            alignCommand,
            
            // Return to idle game state after alignment
            Commands.runOnce(() -> masterStateMachine.setGameState(GameState.IDLE))
        );
    }
}
