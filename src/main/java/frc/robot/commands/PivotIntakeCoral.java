// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;
// import frc.robot.Constants.PivotIntakeConstants;
// import frc.robot.subsystems.PivotIntakeSubsystem;

// public class PivotIntakeCoral extends Command {
    
//     private final PivotIntakeSubsystem pivotIntake;
//     private final Timer timer = new Timer();
    
//     // States for the intake sequence
//     private enum IntakeState {
//         SPINNING_WHEELS,
//         CORAL_DETECTED,
//         MOVING_TO_SETPOINT,
//         COMPLETE
//     }
    
//     private IntakeState currentState;
//     private boolean coralDetected = false;
//     private double coralDetectionTime = 0.0;
    
//     // Timing constants
//     private static final double CORAL_DETECTION_DELAY = 0.1; // Wait 0.1s after coral detected before moving to setpoint
//     private static final double SETPOINT_TIMEOUT = 2.0; // Max time to wait for pivot to reach setpoint
    
//     public PivotIntakeCoral(RobotContainer robotContainer) {
//         this.pivotIntake = robotContainer.pivotIntake;
//         addRequirements(pivotIntake);
//     }
    
//     @Override
//     public void initialize() {
//         currentState = IntakeState.SPINNING_WHEELS;
//         coralDetected = false;
//         timer.restart();
        
//         // Start by spinning intake wheels (pivot should already be in position)
//         System.out.println("Starting coral intake sequence - spinning wheels");
//     }
    
//     @Override
//     public void execute() {
//         switch (currentState) {
//             case SPINNING_WHEELS:
//                 // Step 1: Spin intake wheels and wait for coral detection
//                 pivotIntake.setIntakeSpeed(PivotIntakeConstants.INTAKE_SPEED);
                
//                 if (pivotIntake.isCoralDetected() && !coralDetected) {
//                     coralDetected = true;
//                     coralDetectionTime = timer.get();
//                     currentState = IntakeState.CORAL_DETECTED;
//                     System.out.println("Coral detected! Securing coral...");
//                 }
//                 break;
                
//             case CORAL_DETECTED:
//                 // Step 2: Continue intake briefly after detection to secure coral
//                 pivotIntake.setIntakeSpeed(PivotIntakeConstants.INTAKE_SPEED * 0.3); // Reduced speed
                
//                 if (timer.get() - coralDetectionTime >= CORAL_DETECTION_DELAY) {
//                     currentState = IntakeState.MOVING_TO_SETPOINT;
//                     timer.restart();
                    
//                     // Step 3: Move pivot to reef scoring position to prepare for scoring
//                     pivotIntake.setPivotSetpoint(PivotIntakeConstants.REEF_SCORING_POSITION);
//                     System.out.println("Moving pivot to reef scoring position with coral secured");
//                 }
//                 break;
                
//             case MOVING_TO_SETPOINT:
//                 // Wait for pivot to reach the setpoint position
//                 if (pivotIntake.isPivotAtSetpoint()) {
//                     currentState = IntakeState.COMPLETE;
//                     pivotIntake.setIntakeSpeed(0); // Stop intake wheels
//                     System.out.println("Coral intake sequence complete - pivot at reef scoring position, ready to score!");
//                 } else if (timer.hasElapsed(SETPOINT_TIMEOUT)) {
//                     // Timeout - complete anyway
//                     currentState = IntakeState.COMPLETE;
//                     pivotIntake.setIntakeSpeed(0);
//                     System.out.println("Setpoint timeout - completing intake");
//                 }
//                 break;
                
//             case COMPLETE:
//                 // Do nothing - command will end
//                 break;
//         }
//     }
    
//     @Override
//     public boolean isFinished() {
//         return currentState == IntakeState.COMPLETE;
//     }
    
//     @Override
//     public void end(boolean interrupted) {
//         // Ensure motors are stopped
//         pivotIntake.setIntakeSpeed(0);
        
//         if (interrupted) {
//             System.out.println("Coral intake sequence interrupted");
//             // If interrupted, stop intake wheels
//             pivotIntake.setIntakeSpeed(0);
//         } else {
//             System.out.println("Coral intake sequence completed successfully - ready for reef scoring");
//         }
//     }
// }
