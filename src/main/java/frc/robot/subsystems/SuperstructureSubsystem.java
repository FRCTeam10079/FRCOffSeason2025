package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.RobotStateMachine;
import frc.robot.RobotStateMachine.GameState;
import frc.robot.RobotStateMachine.CoralState;

/**
 * Superstructure state machine for the robot.
 * This subsystem coordinates all mechanisms (elevator, pivot intake, dump
 * roller)
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

    // Track previous enabled state for safety reset on disable/enable transitions
    private boolean wasDisabled = true;

    /**
     * Robot State Machine States
     * Game-specific states for coral manipulation and reef scoring
     */
    public enum RobotState {
        // IDLE/HOME STATES
        IDLE(0, Constants.PivotIntakeConstants.STOWED_POSITION, false, "Robot stowed and ready"),

        // INTAKE STATES - Ground coral collection
        GROUND_INTAKE_DEPLOY(0, Constants.PivotIntakeConstants.INTAKE_POSITION, false, "Deploying intake to ground"),
        GROUND_INTAKE_COLLECTING(0, Constants.PivotIntakeConstants.INTAKE_POSITION, true,
                "Collecting coral from ground"),
        GROUND_INTAKE_STOWING(0, Constants.PivotIntakeConstants.STOWED_POSITION_WITH_CORAL, false,
                "Stowing intake with coral"),

        // TRANSFER STATES - Moving coral from intake to dump roller
        TRANSFERRING_TO_DUMP(0, Constants.PivotIntakeConstants.STOWED_POSITION_WITH_CORAL, true,
                "Transferring coral to dump roller"),
        CORAL_LOADED(0, Constants.PivotIntakeConstants.STOWED_POSITION, false,
                "Coral loaded in dump roller, ready to score"),

        // SCORING STATES - Reef scoring at different levels
        SCORING_L1_PREP(1, Constants.PivotIntakeConstants.STOWED_POSITION, false, "Preparing to score Level 1"),
        SCORING_L1_EXECUTE(1, Constants.PivotIntakeConstants.STOWED_POSITION, false, "Scoring at Level 1"),

        SCORING_L2_PREP(2, Constants.PivotIntakeConstants.STOWED_POSITION, false, "Preparing to score Level 2"),
        SCORING_L2_EXECUTE(2, Constants.PivotIntakeConstants.STOWED_POSITION, false, "Scoring at Level 2"),

        SCORING_L3_PREP(3, Constants.PivotIntakeConstants.STOWED_POSITION, false, "Preparing to score Level 3"),
        SCORING_L3_EXECUTE(3, Constants.PivotIntakeConstants.STOWED_POSITION, false, "Scoring at Level 3"),

        SCORING_L4_PREP(4, Constants.PivotIntakeConstants.STOWED_POSITION, false, "Preparing to score Level 4"),
        SCORING_L4_EXECUTE(4, Constants.PivotIntakeConstants.STOWED_POSITION, false, "Scoring at Level 4"),

        // REEF SCORING STATE - For L1/L2 reef placement
        REEF_SCORING_PREP(1, Constants.PivotIntakeConstants.REEF_SCORING_POSITION, false, "Preparing for reef scoring"),
        REEF_SCORING_EXECUTE(1, Constants.PivotIntakeConstants.REEF_SCORING_POSITION, false, "Executing reef score"),

        // MANUAL OVERRIDE
        MANUAL(0, Constants.PivotIntakeConstants.STOWED_POSITION, false, "Manual control active");

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
     * Immediately applies the state transition for responsive control
     */
    public void requestState(RobotState newState) {
        requestedState = newState;
        currentState = newState;

        // Log state transition immediately
        System.out.println("State transition: " + currentState.name() + " - " + currentState.description);
        Logger.recordOutput("Superstructure/StateTransition", currentState.name() + " - " + currentState.description);
        Logger.recordOutput("Superstructure/TransitionTimestamp", edu.wpi.first.wpilibj.Timer.getFPGATimestamp());

        // Apply state goals immediately
        applyStateGoals();
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
     * Used by commands to wait for mechanisms to reach position before continuing
     */
    private boolean hasReachedStateGoals() {
        RobotState state = currentState;

        // Check elevator position - 0.5 rotation tolerance
        boolean elevatorAtGoal = Math.abs(
                elevator.getPosition() - elevator.positions[state.elevatorLevel]) < 0.5;

        // Check pivot position - use the constant defined tolerance (0.09)
        boolean pivotAtGoal = Math.abs(
                pivotIntake.getPivotPosition() - state.pivotPosition) < Constants.PivotIntakeConstants.PIVOT_TOLERANCE;

        boolean atGoal = elevatorAtGoal && pivotAtGoal;

        // Debug output when not at goal
        if (!atGoal) {
            SmartDashboard.putBoolean("Elevator At Goal", elevatorAtGoal);
            SmartDashboard.putBoolean("Pivot At Goal", pivotAtGoal);
            SmartDashboard.putNumber("Elevator Error",
                    Math.abs(elevator.getPosition() - elevator.positions[state.elevatorLevel]));
            SmartDashboard.putNumber("Pivot Error", Math.abs(pivotIntake.getPivotPosition() - state.pivotPosition));
            //SmartDashboard.putNumber("Elevator Tolerance", elevatorTolerance);
        }

        return atGoal;
    }

    @Override
    public void periodic() {
        // SAFETY: Reset state to IDLE when robot is disabled or on enable transition
        // This prevents the robot from resuming dangerous states (like intake running)
        // when re-enabled after being disabled
        boolean isDisabled = DriverStation.isDisabled();
        
        if (isDisabled) {
            // Robot is disabled - force to IDLE state for safety
            if (currentState != RobotState.IDLE) {
                System.out.println("[SAFETY] Robot disabled - resetting state from " + currentState.name() + " to IDLE");
                currentState = RobotState.IDLE;
                requestedState = RobotState.IDLE;
                
                // Also update master state machine
                masterStateMachine.setGameState(GameState.IDLE);
            }
            
            // Stop all motors immediately when disabled
            pivotIntake.setIntakeSpeed(0);
        } else if (wasDisabled) {
            // Just became enabled - ensure we start from a safe state
            System.out.println("[SAFETY] Robot enabled - starting from IDLE state");
            currentState = RobotState.IDLE;
            requestedState = RobotState.IDLE;
            masterStateMachine.setGameState(GameState.IDLE);
        }
        
        // Track previous state for next cycle
        wasDisabled = isDisabled;
        
        // Only apply state goals when enabled
        if (!isDisabled) {
            // State machine continuously applies state goals to subsystems
            // This ensures the elevator stays at the correct level even if disturbed
            applyStateGoals();
        }

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
     * This continuously updates subsystems based on the current state
     */
    private void applyStateGoals() {
        // Set elevator position based on current state
        // This runs every periodic cycle to ensure elevator maintains position
        int targetLevel = currentState.elevatorLevel;
        if (elevator.pos != targetLevel) {
            System.out.println(
                    "[DEBUG] applyStateGoals: Updating elevator.pos from " + elevator.pos + " to " + targetLevel);
        }
        elevator.pos = targetLevel;

        // Set pivot position based on current state
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
                Commands.runOnce(() -> {
                    masterStateMachine.setGameState(GameState.COLLECTING_GROUND);
                    System.out.println("=== INTAKE SEQUENCE STARTED ===");
                }),

                // State 1: Deploy intake
                Commands.runOnce(() -> {
                    System.out.println("=== INTAKE: Deploying pivot to ground ===");
                    requestState(RobotState.GROUND_INTAKE_DEPLOY);
                }),
                Commands.waitUntil(this::hasReachedStateGoals).withTimeout(3.0),
                Commands.runOnce(() -> System.out.println("Pivot deployed - starting collection")),

                // State 2: Run wheels and collect
                Commands.runOnce(() -> {
                    System.out.println("=== INTAKE: Starting collection wheels ===");
                    requestState(RobotState.GROUND_INTAKE_COLLECTING);
                }),

                // Run intake until coral detected OR timeout
                pivotIntake.intakeWheels()
                        .until(pivotIntake::isCoralDetected)
                        .withTimeout(10.0),

                Commands.runOnce(
                        () -> System.out.println("Coral detected or timeout! Running intake for 0.5s more...")),

                // Keep running intake for 0.5s after detection to fully pull coral in
                pivotIntake.intakeWheels().withTimeout(0.5),

                // Stop wheels IMMEDIATELY with runOnce
                Commands.runOnce(() -> {
                    pivotIntake.setIntakeSpeed(0);
                    pivotIntake.setHasCoralInIntake(true);
                    masterStateMachine.setCoralState(CoralState.IN_INTAKE);
                    System.out.println("Intake wheels stopped - coral secured");
                }),

                Commands.waitSeconds(0.2), // Brief pause before moving

                // State 3: Stow with coral
                Commands.runOnce(() -> {
                    System.out.println("=== INTAKE: Stowing pivot with coral ===");
                    System.out.println("Target position: " + RobotState.GROUND_INTAKE_STOWING.pivotPosition);
                    System.out.println("Current position: " + pivotIntake.getPivotPosition());
                    requestState(RobotState.GROUND_INTAKE_STOWING);
                }),

                // Wait for pivot to stow with timeout
                Commands.waitUntil(this::hasReachedStateGoals).withTimeout(5.0),

                Commands.runOnce(() -> System.out.println("Pivot stowed successfully!")),

                // Update game state - coral secured in intake
                Commands.runOnce(() -> {
                    masterStateMachine.setGameState(GameState.CORAL_SECURED);
                    System.out.println("=== INTAKE SEQUENCE COMPLETE ===");
                }));
    }

    /**
     * FULL AUTO INTAKE: Collect and transfer to dump in one sequence
     */
    public Command collectAndTransfer() {
        return Commands.sequence(
                // Collect from ground
                // Commands.runOnce(() -> dumpRoller.coralMotor.set(1.0)),
                collectCoralFromGround(), // This timeout is too long and this is way too jankky. Need to come
                                    // up with a better method.
                Commands.runOnce(() -> dumpRoller.coralMotor.set(1.0)),
                // Transfer to dump
                transferCoralToDump());
    }

    /**
     * TRANSFER: Move coral from intake to dump roller
     * Starts dump roller spinning BEFORE moving pivot to transfer position
     */
    public Command transferCoralToDump() {
        return Commands.sequence(
                // Update master game state
                Commands.runOnce(() -> {
                    masterStateMachine.setGameState(GameState.TRANSFERRING);
                    System.out.println("=== TRANSFER: Starting dump roller motor ===");
                }),

                // Start dump roller FIRST so it's already spinning when pivot arrives
                // Commands.runOnce(() -> {
                //     System.out.println("[DEBUG] Setting dumpRoller.coralMotor to 1.0");
                //     dumpRoller.coralMotor.set(1.0);
                //     System.out.println("Dump roller motor set to 1.0");
                // }),

                // Small delay to let dump roller spin up
                Commands.waitSeconds(0.1),

                // State: Move to transfer position
                Commands.runOnce(() -> {
                    System.out.println("[DEBUG] Moving pivot to transfer position");
                    System.out.println("=== TRANSFER: Moving pivot to transfer position ===");
                    requestState(RobotState.TRANSFERRING_TO_DUMP);
                }),
                Commands.waitUntil(this::hasReachedStateGoals).withTimeout(3.0),
                Commands.runOnce(() -> System.out.println("Pivot at transfer position")),

                // Wait 0.25s at transfer position before starting transfer
                Commands.waitSeconds(0.1),

                // Transfer coral - RACE ends when FIRST command finishes
                Commands.runOnce(() -> System.out.println("=== TRANSFER: Starting coral transfer ===")),
                Commands.parallel(
                        // Dump roller intake until current spike detected
                        new IntakeCoral(dumpRoller),

                        // Pivot wheels reverse (push coral out) with 3s safety timeout
                        Commands.sequence(
                                Commands.waitSeconds(0.1),
                                pivotIntake.reverseIntakeWheels()
                        )
                ),
                
                pivotIntake.moveSlightlyUpPivot(),
                Commands.waitUntil(this::hasReachedStateGoals).withTimeout(3.0),

                Commands.runOnce(() -> System.out.println("Transfer complete - stopping motors")),

                // Stop motors and update states
                pivotIntake.stopWheels(),
                dumpRoller.keepCoral(),

                pivotIntake.stowPivot(),
                Commands.waitUntil(this::hasReachedStateGoals).withTimeout(3.0),

                // Update coral tracking states
                Commands.runOnce(() -> {
                    pivotIntake.setHasCoralInIntake(false);
                    dumpRoller.setCoralLoaded(true);
                    masterStateMachine.setCoralState(CoralState.IN_DUMP);
                    System.out.println("Coral transferred to dump roller");
                }),

                
                // State: Coral loaded and ready
                Commands.runOnce(() -> requestState(RobotState.CORAL_LOADED)),
                Commands.runOnce(() -> masterStateMachine.setGameState(GameState.CORAL_LOADED)));
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
        System.out.println("=== SCORE L4 COMMAND TRIGGERED ===");
        return Commands.sequence(
                Commands.runOnce(() -> System.out
                        .println("[DEBUG] scoreLevel4() called, elevator pos: " + elevator.getPosition())),
                scoreAtLevel(RobotState.SCORING_L4_PREP, RobotState.SCORING_L4_EXECUTE, 0.2));
    }

    /**
     * SCORE REEF: Score coral on reef (L1/L2 height with pivot out)
     */
    public Command scoreReef() {
        return Commands.sequence(
                // Prep: Move to reef scoring position
                Commands.runOnce(() -> {
                    masterStateMachine.setCoralState(CoralState.SCORING);
                    requestState(RobotState.REEF_SCORING_PREP);
                }),
                Commands.waitUntil(this::hasReachedStateGoals),

                // Execute: Launch coral
                Commands.runOnce(() -> requestState(RobotState.REEF_SCORING_EXECUTE)),
                dumpRoller.dropCoral(0.2).withTimeout(0.5),
                Commands.runOnce(() -> {
                    dumpRoller.setCoralLoaded(false);
                    masterStateMachine.setCoralState(CoralState.NONE); // Coral scored!
                }),
                dumpRoller.keepCoral(),

                // Return to idle
                Commands.runOnce(() -> requestState(RobotState.IDLE)));
    }

    /**
     * Generic scoring sequence helper
     * Updates master game state during scoring
     * Ensures dump roller shoots coral at all levels
     */
    private Command scoreAtLevel(RobotState prepState, RobotState executeState, double launchSpeed) {
        // Determine game state based on level
        GameState scoringState = GameState.SCORING_PROCESSOR;
        if (prepState == RobotState.SCORING_L2_PREP)
            scoringState = GameState.SCORING_REEF_L2;
        else if (prepState == RobotState.SCORING_L3_PREP)
            scoringState = GameState.SCORING_REEF_L3;
        else if (prepState == RobotState.SCORING_L4_PREP)
            scoringState = GameState.SCORING_REEF_L4;

        final GameState finalScoringState = scoringState;

        return Commands.sequence(
                // Debug output
                Commands.runOnce(() -> System.out.println(
                        "Starting scoring sequence: " + prepState.name() + " -> Level " + prepState.elevatorLevel)),

                // Update master game state
                Commands.runOnce(() -> masterStateMachine.setGameState(finalScoringState)),

                // Prep: Move elevator and mechanisms to scoring position
                Commands.runOnce(() -> {
                    System.out.println("Setting prep state: " + prepState.name());
                    masterStateMachine.setCoralState(CoralState.SCORING);
                    requestState(prepState);
                }),
                Commands.waitUntil(this::hasReachedStateGoals),

                // Execute: Launch coral
                Commands.runOnce(() -> {
                    System.out.println("[DEBUG] Executing score at level " + executeState.elevatorLevel + " with speed "
                            + launchSpeed);
                    requestState(executeState);
                }),
                Commands.runOnce(() -> System.out.println("[DEBUG] Calling dumpRoller.dropCoral(" + launchSpeed + ")")),
                dumpRoller.dropCoral(launchSpeed).withTimeout(0.5),
                Commands.runOnce(() -> {
                    System.out.println("[DEBUG] Setting dumpRoller coralLoaded to false");
                    dumpRoller.setCoralLoaded(false);
                    masterStateMachine.setCoralState(CoralState.NONE); // Coral is scored!
                }),
                Commands.runOnce(() -> System.out.println("[DEBUG] Calling dumpRoller.keepCoral()")),
                dumpRoller.keepCoral(),

                // Wait 0.5s at scoring level before returning elevator to home
                Commands.runOnce(() -> System.out.println("[DEBUG] Waiting 0.5s before returning elevator to home")),
                Commands.waitSeconds(0.5),

                // Return to idle/home
                Commands.runOnce(() -> {
                    System.out.println("[DEBUG] Returning elevator to home after scoring");
                    System.out.println(
                            "[DEBUG] Current elevator pos: " + elevator.pos + ", position: " + elevator.getPosition());
                    requestState(RobotState.IDLE);
                    System.out.println("[DEBUG] After requestState(IDLE), elevator.pos set to: " + elevator.pos);
                }),
                Commands.waitSeconds(0.1), // Brief delay to ensure state is applied
                Commands.runOnce(() -> System.out.println("[DEBUG] Waiting for elevator to reach home. Current: "
                        + elevator.getPosition() + ", Target: " + elevator.positions[0])),
                Commands.waitUntil(this::hasReachedStateGoals).withTimeout(5.0), // Wait for elevator to return to home
                                                                                 // position with timeout
                Commands.runOnce(
                        () -> System.out.println("[DEBUG] Elevator reached home position: " + elevator.getPosition())));
    }

    /**
     * IDLE: Return robot to home/stowed position
     */
    public Command returnToIdle() {
        return Commands.sequence(
                Commands.runOnce(() -> requestState(RobotState.IDLE)),
                Commands.waitUntil(this::hasReachedStateGoals));
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
     * This does NOT modify GameState because alignment is a drivetrain operation, not a manipulator operation. This allows alignment to run in parallel with intake/transfer operations without state conflicts.
     * @param alignCommand The AlignReef command to execute
     * @return The alignment command (AlignReef already manages DrivetrainMode)
     */
    public Command visionAlignReef(Command alignCommand) {
        // Just return the align command - it manages DrivetrainMode internally
        // GameState is not modified to allow parallel intake/transfer operations
        return alignCommand;
    }
}