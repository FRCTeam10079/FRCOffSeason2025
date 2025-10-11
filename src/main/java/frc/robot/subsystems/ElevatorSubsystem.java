    package frc.robot.subsystems;
    import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.Commands;
    import edu.wpi.first.wpilibj2.command.SubsystemBase;
    import java.util.function.DoubleSupplier;

    import com.ctre.phoenix6.configs.TalonFXConfiguration;
    import com.ctre.phoenix6.controls.*;
    import com.ctre.phoenix6.hardware.*;
    import com.ctre.phoenix6.signals.GravityTypeValue;
    import com.ctre.phoenix6.signals.InvertedValue;
    import com.ctre.phoenix6.signals.NeutralModeValue;

    public class ElevatorSubsystem extends SubsystemBase{
        // Declares the two motors controling the elevator
        private final TalonFX backElevator = new TalonFX(15, "rio");
        private final TalonFX frontElevator = new TalonFX(14, "rio");
        // Defines motionMagic stuff(allows the elevator to move smoothly)
        private final MotionMagicVoltage motionMagic = new MotionMagicVoltage(0).withSlot(0);
        // Defines a static voltage for the elevator
        private final VoltageOut voltageOut = new VoltageOut(0);

        // Represents a list of the number of rotations to get to each level
        public Double[] positions = {0.125, 2.125, 10.75, 22.0, 39.0}; // gotta test vals here tmr...
        public int pos = 0;

        // Initializes the motors and controller
        public ElevatorSubsystem() {
            // Back Elevator follows front elevator
            backElevator.setControl(new Follower(frontElevator.getDeviceID(), false));
            // Make sure elevator is at base postion each time rio restarts.
            frontElevator.setPosition(0, 100);
            
            //frontElevator.setControl(new DutyCycleOut(0.2d));
            
            //frontElevator.getPosition().setUpdateFrequency(100);
            //frontElevator.optimizeBusUtilization();

            //backElevator.getPosition().setUpdateFrequency(100);
            //backElevator.optimizeBusUtilization();
            
            // Applies necessary configuration for each motor
            configureMotor(frontElevator);
            configureMotor(backElevator);
        }

        // Configures settings for motor at start
        private void configureMotor(TalonFX motor) {
            TalonFXConfiguration config = new TalonFXConfiguration();
            // PID
            // Reaction to error, too high will cause oscillation, too low will cause slow response
            config.Slot0.kP = 1;
            // Reaction to steady-state error, not needed to change
            config.Slot0.kI = 0;       
            // Reaction to change in error, lessens overshooting
            config.Slot0.kD = 0;   
            // Feedforward
            config.Slot0.kG = 0.45;        
            config.Slot0.kS = 0.45;
            config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
            // Motion Magic - Increase this stuff for faster speeds
            config.MotionMagic.MotionMagicCruiseVelocity = 100;
            config.MotionMagic.MotionMagicAcceleration = 250;
            // Current limits
            config.CurrentLimits.SupplyCurrentLimit = 40;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            // Limits for the height of the elevator
            config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.125;
            config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
            config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 40.5;
            config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
            // Brakes when there is no output
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            // Applies configuration in 1 second
            motor.getConfigurator().apply(config, 1);
        }

        // Sets the voltage of the elevator
        public Command setOpenLoop(DoubleSupplier voltage) {
            System.out.println("We're here " + voltage.getAsDouble());
            return Commands.run(() -> frontElevator.setControl(voltageOut.withOutput(voltage.getAsDouble())), this);
        }
        // Holds the position of the elevator at a certain rotation
        // The rio will assume position 0 is when you click start, so always reset the elevator to the bottom
        public Command setCloseLoop(DoubleSupplier position) {
            System.out.println("The close loop is here");
            return Commands.run(() -> frontElevator.setControl(motionMagic.withPosition(position.getAsDouble())), this);
        }

    @Override
    public void periodic() {
        // Displays telemetry about the elevators position and power
        SmartDashboard.putNumber("elevator position", frontElevator.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("elevator applied", frontElevator.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putString("Elevator Level", "L" + pos);
    }        // Moves the elevator to a set position with a threshold of 0.5
        public Command setPositionwithThreshold(int targetPos){
            return setPosition(targetPos).until(() -> Math.abs(positions[targetPos] - getPosition()) < 0.5);
        }

        // Moves the elevator to the position based on the list
        public Command setPosition(int targetPos){
            pos = targetPos;
            SmartDashboard.putNumber("Elevator Level Index", pos);
            //return setOpenLoop(() -> 12d);
            return setCloseLoop(() -> positions[targetPos]);
        }

        /** Sets the position of both motors.
         * @param rotations is the number of rotations it will hold at.
         */
        public void setRotation(double rotations){
            frontElevator.setPosition(rotations);
        }

        public void resetRotation() {
            frontElevator.setPosition(0);
        }

        // Returns the position of the elevator
        public double getPosition(){
            return frontElevator.getPosition().getValueAsDouble();
        }
    }
