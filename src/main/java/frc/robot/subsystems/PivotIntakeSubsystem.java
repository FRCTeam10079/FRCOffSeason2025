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

public class PivotIntakeSubsystem extends SubsystemBase {

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
    
    public PivotIntakeSubsystem() {
        configurePivotMotor(PivotIntakeConstants.STOWED_POSITION, PivotIntakeConstants.INTAKE_POSITION);
        configureIntakeMotor();
        
        // // Reset pivot encoder to 0 at startup
        // pivotEncoder.setPosition(0);
        
        // // Sync the TalonFX position with the CANcoder
        // // This ensures the motor controller knows the current position
        // pivotMotor.setPosition(0
    }

    public void zeroPositionEncoders() {
        // just tells both the motor and the encoder that they are at pos 0;
        pivotEncoder.setPosition(0);
        pivotMotor.setPosition(0);
    }
    
    private void configurePivotMotor(double ForwardSoft, double ReserveSoft) {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        
        // PID Configuration for position control
        // config.Slot0.kP = PivotIntakeConstants.PIVOT_KP;
        // //config.Slot0.kI = PivotIntakeConstants.PIVOT_KI;
        // config.Slot0.kD = PivotIntakeConstants.PIVOT_KD;
        // //config.Slot0.kA = 0; // ensure they are reset to 0
        // //config.Slot0.kS = 0; // ensure they are reset to 0
        // //config.Slot0.kV = 0; // ensure they are reset to 0
        // config.Slot0.kG = PivotIntakeConstants.PIVOT_KG;  // Gravity compensation
        // //config.Slot0.kA = PivotIntakeConstants.PIVOT_KA;
        config.Slot0.GravityType = com.ctre.phoenix6.signals.GravityTypeValue.Arm_Cosine; // For pivoting arms
        
        // // shrug i hop eit works
        // config.MotionMagic.MotionMagicCruiseVelocity = 125;
        // config.MotionMagic.MotionMagicAcceleration = 250;
        // config.MotionMagic.MotionMagicJerk = 0; // resetting the value

        MotionMagicConfigs mm = config.MotionMagic;
        mm.withMotionMagicCruiseVelocity(RotationsPerSecond.of(0.5)) // 5 (mechanism) rotations per second cruise
        .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(1)) // Take approximately 0.5 seconds to reach max vel
        .withMotionMagicJerk(RotationsPerSecondPerSecond.per(Second).of(0));

        Slot0Configs slot0 = config.Slot0;
        slot0.kS = 0.35; // Add 0.25 V output to overcome static friction
        slot0.kV = 12 ; // A velocity target of 1 rps results in 0.12 V output
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
        
        // Limits to prevent over-rotation
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ForwardSoft; // Slightly past intake position
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ReserveSoft; // Slightly past stowed
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
    
    // Set the pivot position setpoint
    public void setPivotSetpoint(double setpoint) {
        currentSetpoint = setpoint;
        double direction = Math.signum(setpoint - getPivotPosition());

        if (direction > 0) {
            configurePivotMotor(PivotIntakeConstants.INTAKE_POSITION, setpoint);
        } else {
            configurePivotMotor(setpoint, PivotIntakeConstants.STOWED_POSITION);
        }

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
    
    // Get the distance reading from CanRange sensor
    //public double getCoralDistance() {
        //return coralSensor.getDistance().getValueAsDouble();
        //return 0d;
    //}
    
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
     * Uses TIME-BASED detection (3 seconds) instead of sensors.
     * Sequence:
     * 1. Deploy pivot to intake position
     * 2. Run intake wheels for 3 seconds
     * 3. Wait 0.25s, then stow pivot to home
     * 4. Transfer coral (ALL PARALLEL):
     *    - Pivot wheels REVERSE
     *    - Dump roller wheels RUN
     *    - Wait 0.2s then pivot to 0.08
     *    - Runs until current spike detected
     * 5. Stop pivot wheels
     * 
     * @param dumpRoller The DumpRollerSubsystem to coordinate with
     * @return Command sequence for the complete intake operation
     */
    public Command collectAndTransferCoral(DumpRollerSubsystem dumpRoller) {
        return Commands.sequence(
            // STEP 1: Deploy Pivot to Intake Position
            Commands.runOnce(() -> setPivotSetpoint(PivotIntakeConstants.INTAKE_POSITION)),
            Commands.waitUntil(this::isPivotAtSetpoint),
            
            // STEP 2: Run Intake Wheels for 3 seconds (TIME-BASED, NO SENSOR)
            Commands.run(() -> setIntakeSpeed(PivotIntakeConstants.INTAKE_SPEED), this)
                .until(this::isCoralDetected),
            Commands.runOnce(() -> setIntakeSpeed(0), this),
            Commands.runOnce(() -> setHasCoralInIntake(true)), // Mark coral as collected
            
            // STEP 2.5: Wait 0.25 seconds
            Commands.waitSeconds(0.125),
            
            // STEP 3: Stow Pivot to Home (0.0 rotations)
            Commands.runOnce(() -> setPivotSetpoint(PivotIntakeConstants.STOWED_POSITION)),
            Commands.waitUntil(this::isPivotAtSetpoint),
            
            // STEP 4: Transfer Coral (ALL PARALLEL)
            Commands.parallel(
                // Dump roller wheels RUN (pull coral in) - runs until current spike detected
                new IntakeCoral(dumpRoller),

                // Pivot wheels REVERSE (push coral out)
                Commands.sequence(
                    Commands.runOnce(() -> setIntakeSpeed(PivotIntakeConstants.INTAKE_REVERSE_SPEED), this),
                    Commands.waitSeconds(2.5),
                    Commands.runOnce(() -> setIntakeSpeed(0), this)
                )
            ),
            
            // STEP 5: Stop Pivot Wheels and clear coral state
            Commands.runOnce(() -> setIntakeSpeed(0), this),
            Commands.runOnce(() -> setHasCoralInIntake(false)) // Coral transferred to dump roller
        );
    }
    
    /**
     * Simpler version: Just collect coral from ground
     * Deploys, runs intake for 3 seconds (TIME-BASED, NO SENSOR), then stows
     */
    public Command collectCoral() {
        return Commands.sequence(
            //Commands.runOnce(() -> setPivotSetpoint(PivotIntakeConstants.INTAKE_POSITION)),
            //Commands.waitUntil(this::isPivotAtSetpoint),
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
     */
    public Command transferCoralToDumpRoller(DumpRollerSubsystem dumpRoller) {
        return Commands.sequence(
            // Run both motors simultaneously with current spike detection
            Commands.parallel(
                // Dump roller wheels RUN (pull coral in) - runs until current spike detected
                new IntakeCoral(dumpRoller),

                // Pivot wheels REVERSE (push coral out)
                Commands.sequence(
                    Commands.runOnce(() -> setPivotSetpoint(0.02)),
                    Commands.runOnce(() -> setIntakeSpeed(PivotIntakeConstants.INTAKE_REVERSE_SPEED), this),
                    Commands.waitSeconds(5),
                    Commands.runOnce(() -> setIntakeSpeed(0), this)
                )
            ),
            
            // Stop pivot wheels (dump roller already stopped by IntakeCoral)
            Commands.runOnce(() -> setIntakeSpeed(0), this)
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
        //SmartDashboard.putNumber("Coral Distance (mm)", getCoralDistance());
        SmartDashboard.putNumber("Intake Wheel Speed", intakeWheelMotor.get());
        SmartDashboard.putNumber("Pivot Motor Current", pivotMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Motor Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Motor Position (internal)", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Pivot Motor Duty Cycle", pivotMotor.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Intake Motor Current", intakeWheelMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Sensor raw output", coralSensor.getDistance().getValueAsDouble());
    }
}
