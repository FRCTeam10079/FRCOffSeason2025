// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.PivotIntakeConstants;
import frc.robot.Constants.ReefPos;
import frc.robot.commands.AlignReef;
import frc.robot.commands.IntakeCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.DumpRollerSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotIntakeSubsystem;
import frc.robot.subsystems.SuperstructureSubsystem;

public class RobotContainer {
    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7; // kSpeedAt12Volts desired top speed
    public double MaxAngularRate = RotationsPerSecond.of(0.8).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // MASTER STATE MACHINE - Controls the entire robot
    private final RobotStateMachine robotStateMachine = RobotStateMachine.getInstance();
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Joysticks
    public final CommandXboxController joystick = new CommandXboxController(0);
    public final CommandXboxController joystick2 = new CommandXboxController(1);
    public final CommandXboxController joystick3 = new CommandXboxController(2); // Test controller

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final LimelightSubsystem limelight = new LimelightSubsystem(this);
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final PivotIntakeSubsystem pivotSub = new PivotIntakeSubsystem();
    public final DumpRollerSubsystem dumpRoller = new DumpRollerSubsystem();
    
    // STATE MACHINE - Coordinates all mechanisms
    public final frc.robot.subsystems.SuperstructureSubsystem superstructure = new frc.robot.subsystems.SuperstructureSubsystem(elevator, pivotSub, dumpRoller);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // STATE MACHINE NAMED COMMANDS - Use these for autonomous!
        
        // GROUND INTAKE
        NamedCommands.registerCommand("Collect Coral", superstructure.collectCoralFromGround());
        NamedCommands.registerCommand("Collect and Transfer", superstructure.collectAndTransfer());
        NamedCommands.registerCommand("Transfer to Dump", superstructure.transferCoralToDump());
        
        // SCORING
        NamedCommands.registerCommand("Score L1", superstructure.scoreLevel1());
        NamedCommands.registerCommand("Score L2", superstructure.scoreLevel2());
        NamedCommands.registerCommand("Score L3", superstructure.scoreLevel3());
        NamedCommands.registerCommand("Score L4", superstructure.scoreLevel4());
        NamedCommands.registerCommand("Score Reef", superstructure.scoreReef());
        
        // UTILITY
        NamedCommands.registerCommand("Return to Idle", superstructure.returnToIdle());
        NamedCommands.registerCommand("Prepare Coral Out", superstructure.prepareCoralOut());
        NamedCommands.registerCommand("Prepare Coral In", superstructure.prepareCoralIn());
        
        // VISION ALIGNMENT (State machine aware)
        NamedCommands.registerCommand("Align Left Reef", new AlignReef(this, ReefPos.LEFT));
        NamedCommands.registerCommand("Align Right Reef", new AlignReef(this, ReefPos.RIGHT));
        
        // LEGACY COMMANDS (for backward compatibility with old autos)
        NamedCommands.registerCommand("Deploy Pivot", pivotSub.deployPivot());
        NamedCommands.registerCommand("Stow Pivot", pivotSub.stowPivot());
        NamedCommands.registerCommand("Intermediate Pivot", pivotSub.intermediatePivot());
        NamedCommands.registerCommand("Start Intake Wheels", pivotSub.intakeWheels());
        NamedCommands.registerCommand("Reverse Intake Wheels", pivotSub.reverseIntakeWheels());
        NamedCommands.registerCommand("Stop Intake Wheels", pivotSub.stopWheels());
        NamedCommands.registerCommand("Raise L0", elevator.setPositionwithThreshold(0));
        NamedCommands.registerCommand("Raise L1", elevator.setPositionwithThreshold(1));
        NamedCommands.registerCommand("Raise L2", elevator.setPositionwithThreshold(2));
        NamedCommands.registerCommand("Raise L3", elevator.setPositionwithThreshold(3));
        NamedCommands.registerCommand("Raise L4", elevator.setPositionwithThreshold(4));
        NamedCommands.registerCommand("Intake Coral", new IntakeCoral(this));
        NamedCommands.registerCommand("Drop Coral", dumpRoller.dropCoral(0.2).withTimeout(0.5));
        NamedCommands.registerCommand("Keep Coral", dumpRoller.keepCoral());
        
        // Build auto chooser with PathPlanner
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // Register driver and operator controllers for rumble feedback
        robotStateMachine.registerControllers(joystick, joystick2);

        // DO NOT CHANGE.
        // Open Loop doesn't use feedback, Close Loop uses feedback
        //Supposedly Elevator is controlled via periodic() based on pos variable so there is no default command needed cuz the state machine will handle the position logic automatically 
        //elevator.setDefaultCommand(elevator.setOpenLoop(() -> 0.2));
        dumpRoller.setDefaultCommand(dumpRoller.keepCoral());

        // Configures the Bindings
        configureBindings();
    }

    // Configures the bindings
    private void configureBindings() {
        /////////////////////////////
        // DRIVER CONTROL
        /////////////////////////////
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // A Button: Brake (locks wheels) - Updates state machine to track locked state
        //joystick.a().onTrue(Commands.runOnce(() -> 
            //robotStateMachine.setDrivetrainMode(RobotStateMachine.DrivetrainMode.LOCKED)))
            //.onFalse(Commands.runOnce(() -> 
            //robotStateMachine.setDrivetrainMode(RobotStateMachine.DrivetrainMode.FIELD_CENTRIC)))
            //.whileTrue(drivetrain.applyRequest(() -> brake));

        // State machine tracks vision alignment automatically via AlignReef
        joystick.rightBumper().whileTrue(new AlignReef(this, Constants.ReefPos.RIGHT));
        joystick.leftBumper().whileTrue(new AlignReef(this, Constants.ReefPos.LEFT));
        
        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on Y button press
        joystick.leftTrigger().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Overdrive button for speed
        //joystick.b().whileTrue(increaseSpeed()).onFalse(decreaseSpeed()); 

        // Smart scoring based on elevator level - triggers when right trigger pressed past threshold
        //joystick.rightTrigger().onTrue(SmartScore());

        // Level 1 - triggers when left trigger pressed past threshold
        //joystick.leftTrigger().onTrue(superstructure.scoreLevel1());

        /////////////////////////////
        // OPERATOR CONTROL - STATE MACHINE
        /////////////////////////////
        
        // SCORING BUTTONS - Automatic state machine sequences
        joystick.b().onTrue(superstructure.scoreLevel1());
        joystick.a().onTrue(superstructure.scoreLevel2());
        joystick.x().onTrue(superstructure.scoreLevel3());
        joystick.y().onTrue(superstructure.scoreLevel4());
        

        // INTAKE CONTROLS - Full auto sequence
        joystick.rightTrigger().onTrue(superstructure.collectAndTransfer());

        // MANUAL OVERRIDES - Direct subsystem control (bypasses state machine)
        // Enter manual mode, then control subsystems directly
        joystick2.povRight().onTrue(superstructure.enterManualMode());
        
        // Manual dump roller control
        joystick2.leftBumper().whileTrue(dumpRoller.dropCoral(0.2))
            .onFalse(dumpRoller.keepCoral());
        
        // Manual pivot control
        joystick2.rightTrigger().onTrue(new InstantCommand(() -> pivotSub.setPivotSetpoint(0.4d)));
        
        // Zero pivot encoders
        joystick2.rightBumper().onTrue(new InstantCommand(() -> pivotSub.zeroPositionEncoders()));

        // Manual intake into dump (legacy)
        joystick2.leftTrigger().onTrue(new IntakeCoral(this));
        
        // Return to state machine control
        joystick2.start().onTrue(superstructure.exitManualMode());

        /////////////////////////////
        // TEST CONTROLLER (Joystick 3) - Individual testing commands
        /////////////////////////////
        joystick3.povUp().onTrue(superstructure.collectCoralFromGround());   // Test: Just collect
        joystick3.povDown().onTrue(superstructure.transferCoralToDump());    // Test: Just transfer
        joystick3.start().onTrue(superstructure.returnToIdle());             // Test: Return to idle
    }

    /**
     * Smart Score - Uses state machine to score at current elevator level
     * Scores at the current elevator position when right trigger is pressed
     */
    private Command SmartScore() {
        return Commands.either(
            // If pos <= 1 (Level 0 or 1), score at Level 1
            Commands.sequence(
                Commands.runOnce(() -> System.out.println("=== SMART SCORE: Level 1 (pos=" + elevator.pos + ") ===")),
                superstructure.scoreLevel1()
            ),
            Commands.either(
                // If pos == 2, score at Level 2
                Commands.sequence(
                    Commands.runOnce(() -> System.out.println("=== SMART SCORE: Level 2 (pos=" + elevator.pos + ") ===")),
                    superstructure.scoreLevel2()
                ),
                Commands.either(
                    // If pos == 3, score at Level 3
                    Commands.sequence(
                        Commands.runOnce(() -> System.out.println("=== SMART SCORE: Level 3 (pos=" + elevator.pos + ") ===")),
                        superstructure.scoreLevel3()
                    ),
                    // Otherwise (pos == 4), score at Level 4
                    Commands.sequence(
                        Commands.runOnce(() -> System.out.println("=== SMART SCORE: Level 4 (pos=" + elevator.pos + ") ===")),
                        superstructure.scoreLevel4()
                    ),
                    () -> elevator.pos == 3
                ),
                () -> elevator.pos == 2
            ),
            () -> elevator.pos <= 1
        );
    }
    
    /**
     * Legacy Coral Outtake - Direct subsystem control (bypasses state machine)
     * Kept for backward compatibility
     */
    private Command CoralOuttake(){
        // List of speeds for each elevator level (0-4)
        // Position 0 uses a slower speed (0.1), all other positions use 0.2
        double[] launchSpeeds = {0.1, 0.1, 0.2, 0.2, 0.2};
        
        // Safety check to prevent array index out of bounds
        int posIndex = Math.min(elevator.pos, launchSpeeds.length - 1);

        // Sequence of Commands
        return Commands.sequence(   
            dumpRoller.dropCoral(launchSpeeds[posIndex]).withTimeout(0.5),
            Commands.runOnce(() -> dumpRoller.setCoralLoaded(false)), // Clear coral state after launch
            dumpRoller.keepCoral().withTimeout(0.01),
            // Drops to Level 0 after done
            elevator.setPosition(0)
        );
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }

    public Command increaseSpeed(){
        return Commands.runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
    }

    public Command decreaseSpeed(){
        return Commands.runOnce(() -> MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7);
    }
}
