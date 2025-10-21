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

public class RobotContainer {
    public double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7; // kSpeedAt12Volts desired top speed
    public double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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
    public final CommandXboxController joystick3 = new CommandXboxController(2);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final LimelightSubsystem limelight = new LimelightSubsystem(this);
    public final ElevatorSubsystem elevator = new ElevatorSubsystem();
    public final PivotIntakeSubsystem pivotSub = new PivotIntakeSubsystem();
    public final DumpRollerSubsystem dumpRoller = new DumpRollerSubsystem();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Register PathPlanner Named Commands
        // PIVOT INTAKE COMMANDS
        NamedCommands.registerCommand("Deploy Pivot", pivotSub.deployPivot()); // Move pivot to ground intake position
        NamedCommands.registerCommand("Stow Pivot", pivotSub.stowPivot()); // Move pivot to home position
        NamedCommands.registerCommand("Intermediate Pivot", pivotSub.intermediatePivot()); // Move pivot to reef scoring position
        NamedCommands.registerCommand("Collect Coral", pivotSub.collectCoral()); // Deploy, intake until detected, stow
        NamedCommands.registerCommand("Collect and Transfer Coral", pivotSub.collectAndTransferCoral(dumpRoller)); // Full auto: collect + transfer to dump
        NamedCommands.registerCommand("Transfer to Dump", pivotSub.transferCoralToDumpRoller(dumpRoller)); // Transfer coral to dump roller only
        NamedCommands.registerCommand("Score L1/ Eject", pivotSub.collectForL1(dumpRoller));
        
        // INTAKE WHEEL COMMANDS
        NamedCommands.registerCommand("Start Intake Wheels", pivotSub.intakeWheels()); // Run intake wheels forward
        NamedCommands.registerCommand("Reverse Intake Wheels", pivotSub.reverseIntakeWheels()); // Run intake wheels backward
        NamedCommands.registerCommand("Stop Intake Wheels", pivotSub.stopWheels()); // Stop intake wheels
        
        // ELEVATOR COMMANDS
        NamedCommands.registerCommand("Raise L0", elevator.setPositionwithThreshold(0)); // Move elevator to level 0
        NamedCommands.registerCommand("Raise L1", elevator.setPositionwithThreshold(1)); // Move elevator to level 1
        NamedCommands.registerCommand("Raise L2", elevator.setPositionwithThreshold(2)); // Move elevator to level 2
        NamedCommands.registerCommand("Raise L3", elevator.setPositionwithThreshold(3)); // Move elevator to level 3
        NamedCommands.registerCommand("Raise L4", elevator.setPositionwithThreshold(4)); // Move elevator to level 4
        
        // DUMP ROLLER COMMANDS
        NamedCommands.registerCommand("Intake Coral", new IntakeCoral(this)); // Run dump roller until current spike detected
        NamedCommands.registerCommand("Drop Coral", dumpRoller.dropCoral(0.2).withTimeout(0.5)); // Outtake coral for 0.5s
        NamedCommands.registerCommand("Keep Coral", dumpRoller.keepCoral()); // Hold/stop dump roller
        NamedCommands.registerCommand("Prepare Coral Out", dumpRoller.PrepareCoral(true)); // Push coral out slightly
        NamedCommands.registerCommand("Prepare Coral In", dumpRoller.PrepareCoral(false)); // Pull coral in slightly
        NamedCommands.registerCommand("SCORE", CoralOuttake());


        // Aligns to the Left Reef side
        NamedCommands.registerCommand("Align Left", new AlignReef(this, ReefPos.LEFT).withTimeout(1.0));
        // Aligns to the Right Reef side
        NamedCommands.registerCommand("Align Right", new AlignReef(this, ReefPos.RIGHT).withTimeout(1.0));
        
        // Build auto chooser with PathPlanner
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        System.out.println("Have added the paths");
        SmartDashboard.putData("Auto Mode", autoChooser);

        // DO NOT CHANGE.
        // Open Loop doesn't use feedback, Close Loop uses feedback
        elevator.setDefaultCommand(elevator.setOpenLoop(() -> 0.2));
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
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        joystick.rightBumper().whileTrue(new AlignReef(this, Constants.ReefPos.LEFT));

        joystick.leftBumper().whileTrue(new AlignReef(this, Constants.ReefPos.RIGHT));
        
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
        joystick.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Overdrive button for speed
        joystick.b().whileTrue(increaseSpeed()).onFalse(decreaseSpeed()); 

        // Outtakes coral 
        joystick.rightTrigger().onTrue(CoralOuttake());

        // Level 1
        joystick.leftTrigger().onTrue(elevator.setPosition(1));

        /////////////////////////////
        // OPERATOR CONTROL
        /////////////////////////////
        // TOGGLE ELEVATOR POSITIONS
        // Level 0
        joystick2.b().onTrue(elevator.setPosition(0));
        // Level 2
        joystick2.a().onTrue(elevator.setPosition(2));
        // Level 3
        joystick2.x().onTrue(elevator.setPosition(3));
        // Level 4
        joystick2.y().onTrue(elevator.setPosition(4));

        // PIVOT INTAKE CONTROLS
        // Full auto sequence: collect coral and transfer to dump roller
        joystick2.povUp().onTrue(pivotSub.collectAndTransferCoral(dumpRoller));
        joystick2.povRight().onTrue(pivotSub.collectForL1(dumpRoller));
        //joystick2.povRight().onTrue(pivotSub.setIntakeSpeed(PivotIntakeConstants.INTAKE_REVERSE_SPEED));
        
        // Continually runs Dump Roller until coral detected
        joystick3.leftTrigger().onTrue(new IntakeCoral(this));

        // JOYSTICK 3 = DEBUG JOYSTICK
        // Just collect coral from ground
        joystick3.povDown().onTrue(pivotSub.collectCoral());

        // Sticks coral out when holding Left Dpad
        //joystick2.povLeft().whileTrue(dumpRoller.dropCoral(0.15)).onFalse(dumpRoller.keepCoral().withTimeout(0.1));

        joystick3.povLeft().onTrue(pivotSub.transferCoralToDumpRoller(dumpRoller));

        // Sticks coral in when holding Right Dpad
        joystick3.povRight().whileTrue(dumpRoller.dropCoral(-0.15)).onFalse(dumpRoller.keepCoral().withTimeout(0.1));

        
        // Manual pivot position control
        joystick3.rightTrigger().onTrue(new InstantCommand(() -> pivotSub.setPivotSetpoint(0.4d)));

        // Zero pivot encoders
        joystick3.rightBumper().onTrue(new InstantCommand(() -> pivotSub.zeroPositionEncoders()));

        // Manual dump roller control
        joystick3.leftBumper().onTrue(dumpRoller.dropCoral(.5));
    }

    // Outtakes Dump Roller Coral onto Reef
    private Command CoralOuttake(){
        // List of speeds for each elevator level (0-4)
        // Position 0 uses a slower speed (0.1), all other positions use 0.2
        double[] launchSpeeds = {0.2, 0.2, 0.2, 0.3, 0.3};
        
        // Safety check to prevent array index out of bounds
        int posIndex = Math.min(elevator.pos, launchSpeeds.length - 1);

        // Sequence of Commands
        return Commands.sequence(   
            dumpRoller.dropCoral(0.25).withTimeout(0.5),
            Commands.runOnce(() -> dumpRoller.setCoralLoaded(false)), // Clear coral state after launch
            dumpRoller.keepCoral().withTimeout(0.1),
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
