package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PivotIntakeConstants;
import frc.robot.commands.IntakeCoral;
import edu.wpi.first.math.MathUtil;

/**
 * ALTERNATE VERSION: Dynamic Clamps Approach
 * This version dynamically adjusts soft limits based on target position
 */
public class PivotIntakeSubsystem_DynamicClamps extends SubsystemBase {

    // Motors and sensors
    private final TalonFX pivotMotor = new TalonFX(PivotIntakeConstants.PIVOT_MOTOR_ID);
    private final TalonFX intakeWheelMotor = new TalonFX(PivotIntakeConstants.INTAKE_WHEEL_MOTOR_ID);
    private final CANcoder pivotEncoder = new CANcoder(PivotIntakeConstants.PIVOT_ENCODER_ID);
    private final CANrange coralSensor = new CANrange(PivotIntakeConstants.CORAL_SENSOR_ID);
    
    // Position control request for pivot
    private final PositionVoltage pivotPositionControl = new PositionVoltage(0).withSlot(0);
    private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);

    // ensure coralSensor outputs correctly
    private boolean previousSensorOutput = false;
    
    // Current pivot setpoint
    private double currentSetpoint = PivotIntakeConstants.STOWED_POSITION;
    
    // Track if coral has been collected (set manually after collection)
    private boolean hasCoralInIntake = false;
    
    public PivotIntakeSubsystem_DynamicClamps() {
        configurePivotMotor(PivotIntakeConstants.STOWED_POSITION, PivotIntakeConstants.INTAKE_POSITION);
        configureIntakeMotor();
        
        // Robot ALWAYS starts physically at stowed position
        // We tell the encoder that the current position is STOWED_POSITION (0.42)
        // This means horizontal extended will be at 0.0
        // Wait a moment for encoder to initialize
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            // ignore
        }
        
        pivotEncoder.setPosition(PivotIntakeConstants.STOWED_POSITION);
        pivotMotor.setPosition(PivotIntakeConstants.STOWED_POSITION);
        
        // Set initial setpoint to match current position to prevent sudden movement
        currentSetpoint = PivotIntakeConstants.STOWED_POSITION;
        
        SmartDashboard.putString("Pivot Init Status", "Initialized at STOWED position (0.42) - DYNAMIC CLAMPS MODE");
    }

    /**
     * Zero the pivot encoders to horizontal extended position (0.0).
     * ONLY call this when the pivot is manually positioned at horizontal extended.
     * This is for calibration/debugging purposes.
     */
    public void zeroPositionEncoders() {
        // Set both encoder and motor to 0 (horizontal extended position)
        pivotEncoder.setPosition(0);
        pivotMotor.setPosition(0);
        currentSetpoint = 0;
        
        SmartDashboard.putString("Pivot Init Status", "Manually zeroed at horizontal (0.0)");
    }
    
    private void configurePivotMotor(double ForwardSoft, double ReverseSoft) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        config.Slot0.GravityType = com.ctre.phoenix6.signals.GravityTypeValue.Arm_Cosine; // For pivoting arms

        MotionMagicConfigs mm = config.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(0.5)) // 0.5 rotations per second cruise
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(1)) // Take approximately 0.5 seconds to reach max vel
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0));

        Slot0Configs slot0 = config.Slot0;
        slot0.kS = 0.35; // Add 0.35 V output to overcome static friction
        slot0.kV = 12 ; // A velocity target of 1 rps results in 12 V output
        slot0.kG = 0;    // No gravity
        slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0.kP = 0.02; // A position error of 0.2 rotations results in 12 V output
        slot0.kI = 0;    // No output for integrated error
        slot0.kD = 0.5;  // A velocity error of 1 rps results in 0.5 V output
        
        // Use the remote CANcoder for absolute position feedback
        config.Feedback.FeedbackRemoteSensorID = PivotIntakeConstants.PIVOT_ENCODER_ID;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.SensorToMechanismRatio = 1.0; // Adjust if there's gearing
        config.Feedback.RotorToSensorRatio = 81.2; // Not really sure if needed but included anyways
        
        // Current limits for pivot motor
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        // DYNAMIC CLAMPS: These will be adjusted based on target position
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ForwardSoft;
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ReverseSoft;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        
        pivotMotor.getConfigurator().apply(config);
        
        // Set position update frequency for better feedback
        pivotMotor.getPosition().setUpdateFrequency(100);
        pivotEncoder.getPosition().setUpdateFrequency(100);
        pivotMotor.optimizeBusUtilization();
    }

    private void configureIntakeMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        
        // Current limits for intake motor
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        intakeWheelMotor.getConfigurator().apply(config);
    }
    
    /**
     * Set the pivot position setpoint with dynamic clamp adjustment
     * IMPORTANT: Motor can ONLY move to the exact clamp positions!
     * We must set BOTH clamps to the target position to make it move there.
     */
    public void setPivotSetpoint(double setpoint) {
        currentSetpoint = setpoint;
        
        // IMPORTANT WORKAROUND:
        // The motor can ONLY go to the clamp positions, not anywhere in between
        // So we set BOTH forward and reverse clamps to the SAME value (the target)
        // This forces the motor to go exactly to that position
        configurePivotMotor(setpoint, setpoint);
        
        // Command the motor to the target position
        pivotMotor.setControl(motionMagic.withPosition(setpoint));
    }
    
    // Get current pivot position
    public double getPivotPosition() {
        return pivotEncoder.getPosition().getValueAsDouble();
    }
    
    // Check if pivot is at setpoint
    public boolean isPivotAtSetpoint() {
        return Math.abs(getPivotPosition() - currentSetpoint) < PivotIntakeConstants.PIVOT_TOLERANCE;
    }
    
    // Run intake wheels
    public void setIntakeSpeed(double speed) {
        intakeWheelMotor.set(speed);
    }
    
    // Check if coral is detected by CanRange sensor
    public boolean isCoralDetected() {
        double distance = coralSensor.getDistance().getValueAsDouble();
        if (distance < PivotIntakeConstants.CORAL_TOO_LOW_DISTANCE) {
            // basically makes it so if the sensor tweaks out, it just returns the previous value
            return previousSensorOutput;
        }
        boolean result = distance < PivotIntakeConstants.CORAL_DETECTED_DISTANCE_M;

        previousSensorOutput = result;
        return result;
    }
    
    // Set whether coral is in the intake (called by commands)
    public void setHasCoralInIntake(boolean hasCoral) {
        hasCoralInIntake = hasCoral;
    }
    
    // Get whether coral is currently in the intake
    public boolean hasCoralInIntake() {
        return hasCoralInIntake;
    }
    
    // COMMAND METHODS
    
    // Command to move pivot to stowed position
    public Command stowPivot() {
        return Commands.runOnce(() -> setPivotSetpoint(PivotIntakeConstants.STOWED_POSITION));
    }
    
    // Command to move pivot to intake position
    public Command deployPivot() {
        return Commands.runOnce(() -> setPivotSetpoint(PivotIntakeConstants.INTAKE_POSITION));
    }
    
    // Command to move pivot to reef scoring position
    public Command intermediatePivot() {
        return Commands.runOnce(() -> setPivotSetpoint(PivotIntakeConstants.REEF_SCORING_POSITION));
    }
    
    // Command to run intake wheels forward
    public Command intakeWheels() {
        return Commands.run(() -> setIntakeSpeed(PivotIntakeConstants.INTAKE_SPEED), this);
    }
    
    // Command to run intake wheels in reverse (for scoring)
    public Command reverseIntakeWheels() {
        return Commands.run(() -> setIntakeSpeed(PivotIntakeConstants.INTAKE_REVERSE_SPEED), this);
    }
    
    // Command to stop intake wheels
    public Command stopWheels() {
        return Commands.run(() -> setIntakeSpeed(0), this);
    }
    
    /**
     * Complete sequence to collect coral from ground and transfer to dump roller.
     * Sequence:
     * 1. Deploy pivot to intake position
     * 2. Run intake wheels until coral detected by sensor
     * 3. Wait 0.125s, then raise pivot to transfer position (0.44)
     * 4. Transfer coral (RACE - stops when FIRST command finishes):
     *    - Pivot wheels REVERSE (push coral out)
     *    - Dump roller wheels RUN (pull coral in - ends on current spike)
     *    - Whichever finishes first stops the other
     *    - 3 second safety timeout if current spike never detected
     * 5. Stop pivot wheels, return to normal stowed (0.42), clear coral state
     * 
     * @param dumpRoller The DumpRollerSubsystem to coordinate with
     * @return Command sequence for the complete intake operation
     */
    public Command collectAndTransferCoral(DumpRollerSubsystem dumpRoller) {
        return Commands.sequence(
            // STEP 1: Deploy Pivot to Intake Position
            Commands.runOnce(() -> setPivotSetpoint(PivotIntakeConstants.INTAKE_POSITION)),
            Commands.waitUntil(this::isPivotAtSetpoint),
            
            // STEP 2: Run Intake Wheels until coral detected
            Commands.run(() -> setIntakeSpeed(PivotIntakeConstants.INTAKE_SPEED), this)
                .until(this::isCoralDetected),
            Commands.runOnce(() -> setIntakeSpeed(0), this),
            Commands.runOnce(() -> setHasCoralInIntake(true)), // Mark coral as collected
            
            // STEP 2.5: Wait 0.125 seconds
            Commands.waitSeconds(0.125),
            
            // STEP 3 & 4: Move to transfer position AND start dump roller immediately (DEADLINE)
            // The pivot movement to 0.44 sets the deadline, dump roller and pivot reverse run in parallel
            Commands.deadline(
                // DEADLINE: Wait for pivot to reach transfer position
                Commands.sequence(
                    Commands.runOnce(() -> setPivotSetpoint(PivotIntakeConstants.STOWED_POSITION_WITH_CORAL)),
                    Commands.waitUntil(this::isPivotAtSetpoint)
                ),
                
                // PARALLEL: Dump roller starts immediately as pivot moves
                Commands.race(
                    // Dump roller wheels RUN (pull coral in) - runs until current spike detected
                    new IntakeCoral(dumpRoller),

                    // Pivot wheels REVERSE (push coral out) - stops when IntakeCoral finishes
                    Commands.run(() -> setIntakeSpeed(PivotIntakeConstants.INTAKE_REVERSE_SPEED), this)
                        .withTimeout(3.0) // Safety timeout in case current spike never detected
                )
            ),
            
            // STEP 5: Stop Pivot Wheels (redundant safety), return to normal stowed position, and clear coral state
            Commands.runOnce(() -> setIntakeSpeed(0), this),
            Commands.runOnce(() -> setPivotSetpoint(PivotIntakeConstants.STOWED_POSITION)),
            Commands.runOnce(() -> setHasCoralInIntake(false)) // Coral transferred to dump roller
        );
    }
    
    /**
     * Simpler version: Just collect coral from ground
     * Runs intake for 3 seconds with timeout, then stows
     */
    public Command collectCoral() {
        return Commands.sequence(
            Commands.run(() -> setIntakeSpeed(PivotIntakeConstants.INTAKE_SPEED), this)
                .withTimeout(3.0),
            Commands.runOnce(() -> setIntakeSpeed(0), this),
            Commands.runOnce(() -> setHasCoralInIntake(true)), // Mark coral as collected
            Commands.runOnce(() -> setPivotSetpoint(PivotIntakeConstants.STOWED_POSITION))
        );
    }
    
    /**
     * Transfer coral from pivot intake to dump roller.
     * Uses current spike detection to determine when coral has been transferred.
     * Assumes pivot is already at stowed position with coral.
     * Temporarily raises pivot to 0.44 (0.02 higher) for transfer, then returns to normal stow (0.42)
     * Race ends when dump roller detects current spike OR 5 second safety timeout
     */
    public Command transferCoralToDumpRoller(DumpRollerSubsystem dumpRoller) {
        return Commands.sequence(
            // Raise pivot slightly for transfer (0.44 instead of 0.42)
            Commands.runOnce(() -> setPivotSetpoint(PivotIntakeConstants.STOWED_POSITION_WITH_CORAL)),
            Commands.waitSeconds(0.1), // Brief wait for pivot to move
            
            // Transfer Coral (RACE - ends when coral detected OR 5 second timeout)
            Commands.race(
                // Dump roller wheels RUN (pull coral in) - runs until current spike detected
                new IntakeCoral(dumpRoller),

                // Pivot wheels REVERSE (push coral out) - stops when IntakeCoral finishes
                Commands.run(() -> setIntakeSpeed(PivotIntakeConstants.INTAKE_REVERSE_SPEED), this)
                    .withTimeout(5.0) // Safety timeout in case current spike never detected
            ),
            
            // Stop pivot wheels and return to normal stowed position
            Commands.runOnce(() -> setIntakeSpeed(0), this),
            Commands.runOnce(() -> setPivotSetpoint(PivotIntakeConstants.STOWED_POSITION)),
            Commands.runOnce(() -> setHasCoralInIntake(false)) // Coral transferred to dump roller
        );
    }
    
    @Override
    public void periodic() {
        // Motor is controlled via position control set in setPivotSetpoint()
        
        // Update SmartDashboard
        SmartDashboard.putNumber("Pivot Position", getPivotPosition());
        SmartDashboard.putNumber("Pivot Setpoint", currentSetpoint);
        SmartDashboard.putNumber("Pivot Error", currentSetpoint - getPivotPosition());
        SmartDashboard.putBoolean("Pivot At Setpoint", isPivotAtSetpoint());
        SmartDashboard.putBoolean("Coral Detected", hasCoralInIntake); // State-based, not real-time sensor
        SmartDashboard.putBoolean("Coral Sensor Active", isCoralDetected()); // Real-time sensor reading
        SmartDashboard.putNumber("Intake Wheel Speed", intakeWheelMotor.get());
        SmartDashboard.putNumber("Pivot Motor Current", pivotMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Motor Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Motor Position (internal)", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Motor Duty Cycle", pivotMotor.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Intake Motor Current", intakeWheelMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Sensor raw output", coralSensor.getDistance().getValueAsDouble());
    }
}
