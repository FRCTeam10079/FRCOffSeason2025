package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class DumpRollerSubsystem extends SubsystemBase{
    
    // Initializes the motor
    public TalonFX coralMotor = new TalonFX(19, "rio");
    private DigitalInput coralSensor;
    public Timer a_timer = new Timer();
    // Indicates if the launcher is in action
    public Boolean isRunning = false;
    // Rotation power
    public double power = 1.0;

    // The current threshold to stop the motor
    public final double MAX_CURRENT = 30;
    // Indicates if the coral is being held
    public boolean isHolding = false;
    // Tracks if coral is currently in the dump roller
    private boolean hasCoralLoaded = false;

    // Initializes the motors and controller
    public DumpRollerSubsystem() {
        coralSensor = new DigitalInput(9); // On port 9 of the roboRIO
    }

    // Outtakes the coral
    public Command dropCoral(double voltage){
        return Commands.run(() -> coralMotor.set(voltage * power), this);
    }
    
    // Holds the motor
    public Command keepCoral(){
        return Commands.runOnce(() -> coralMotor.set(0), this);
    }

    // Returns the sensor input, If a coral was found
    public boolean getSensorInput() {
        return coralSensor.get();
    }
    
    // Sets the state when coral is detected (called by IntakeCoral command)
    public void setCoralLoaded(boolean loaded) {
        hasCoralLoaded = loaded;
    }
    
    // Returns if coral is currently loaded
    public boolean hasCoralLoaded() {
        return hasCoralLoaded;
    }

    // Controls the position of the coral
    public Command PrepareCoral(boolean out){
        // Sticks the coral out
        if (out){
            return Commands.sequence(
                dropCoral(0.15).withTimeout(0.25),
                keepCoral().withTimeout(0.1)
            );
        }
        // Pushes the coral back inside
        else{
            return Commands.sequence(
                dropCoral(-0.2).withTimeout(0.25),
                keepCoral().withTimeout(0.1)
            );
        }
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Coral Motor Current", coralMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("Intake Sensor", coralSensor.get());
        SmartDashboard.putBoolean("Dump Roller Has Coral", hasCoralLoaded);
    }
}
