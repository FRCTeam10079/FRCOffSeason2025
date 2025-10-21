package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;


import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

public class AlignReef extends Command {

    // Subsystems from RobotContainer
    private LimelightSubsystem limelight;
    private CommandSwerveDrivetrain drivetrain;
    private RobotContainer robotContainer;

    Timer timer = new Timer();
    
    /* ----- VELOCITY ----- */
    // TUNE: Increase for faster alignment, decrease for better control
    private final double maxVelocity = 6.0;  // Max translation speed (m/s)
    // TUNE: Increase for faster rotation, decrease if spinning too fast
    private final double maxAngularVelocity = 4.5;
    ;  // Max rotation speed (rad/s)

    /* ----- PIDs ----- */
    // TUNE: Increase kP for faster approach, decrease if overshooting
    private PIDController pidDistance = new PIDController(6.0, maxVelocity * 0.5, maxVelocity * 0.25);  // Translation: Increase P for more aggressive, decrease for smoother
    // TUNE: Increase kP for faster rotation, decrease if rotation is jerky
    private PIDController pidRotate = new PIDController(4.0, 3.0, 1.5);    // Rotation: Increase P for faster snap, decrease for smooth turn

    // Creates a swerve request that specifies the robot to move FieldCentric
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.Velocity); // Uses ClosedLoopVoltage for PID
    // Creates a swerve request to stop all motion by setting velocities and rotational rate to 0
    SwerveRequest stop = driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0);

    // The Desired position to go to
    private Pose2d targetPose;

    /* ----- FRICTION COMPENSATION ----- */
    // TUNE: Increase if robot stops short of target, decrease if overshooting
    private final double frictionConstant = 0.00;  // Friction compensation: Increase if stopping too early, decrease if overshooting

    /* ----- TOLERANCES ----- */
    private final double positionTolerance = 0.25;  // Position tolerance (m)
    private final double yawTolerance = Math.PI / 32;  // Rotation tolerance (~5.6°)

    // Distance threshold for friction compensation
    private final double frictionDistanceThreshold = Units.inchesToMeters(0.5);

    /* ----- VISION CORRECTION ----- */
    // TUNE: Set to false if vision updates cause jerking
    private final boolean useVisionCorrection = true;  // Enable/disable vision updates: false if jerky
    // TUNE: Increase if vision updates too frequent, decrease for more corrections
    private final double visionUpdateInterval = 0.1;  // Vision update rate (seconds)
    private double lastVisionUpdateTime = 0;
    private final int minTagCountForVisionUpdate = 1;  // Minimum tags needed: We only have one limelight

    // Alignment parameters
    ReefPos reefPos;
    // X and Y Offset from the April Tag
    private double offsetX = 0;
    private double offsetY = 0;
    // Targetted Tag ID
    private int tID;
    // Indicates if tag was detected
    private boolean tagDetected;

    // CONSTRUCTOR
    public AlignReef(RobotContainer robotContainer, ReefPos reefPos) {
        this.robotContainer = robotContainer;
        this.drivetrain = robotContainer.drivetrain;
        this.limelight = robotContainer.limelight;
        this.reefPos = reefPos;

        // -180 and 180 degrees are the same point, so its continuous
        pidRotate.enableContinuousInput(-Math.PI, Math.PI);

        // Added Logging
        System.out.println("AlignReef command created for " + reefPos + " position");
        SmartDashboard.putString("AlignReef/ReefPosition", reefPos.toString());
    }

    @Override
    public void initialize(){
        // Starts timer
        timer.restart();
        lastVisionUpdateTime = 0;
        
        SmartDashboard.putBoolean("AlignReef/CommandStarted", true);
        SmartDashboard.putNumber("AlignReef/StartTime", timer.get());

        // Get the tag ID from the Limelight
        if (!limelight.isTagDetected()) {
            tagDetected = false;
            SmartDashboard.putBoolean("AlignReef/TagDetected", false);
            System.out.println("Error: No AprilTag detected by Limelight.");
            SmartDashboard.putString("AlignReef/Error", "No AprilTag detected");
            return;
        }

        tID = limelight.getTid();
        SmartDashboard.putNumber("AlignReef/DetectedTagID", tID);

        // Calculate target pose based on the detected tag
        if (!calculateTargetPose()) {
            tagDetected = false;
            SmartDashboard.putBoolean("AlignReef/TagDetected", false);
            return;
        }

        tagDetected = true;
        SmartDashboard.putBoolean("AlignReef/TagDetected", true);

        // Sets Robot Max Speed for Alignment - Might wanna change it
        robotContainer.MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 1.0;
    }

    /**
     * Calculate the target pose based on AprilTag position and offsets
     * @return true if successful, false otherwise
     */
    private boolean calculateTargetPose() {
        double[] aprilTagList = Constants.AprilTagMaps.aprilTagMap.get(tID);
        // Checks if the tag exists within the list of all tags
        if (aprilTagList == null) {
            System.out.println("Error: Target pose array is null for Tag ID: " + tID);
            SmartDashboard.putString("AlignReef/Error", "Target pose array null for Tag " + tID);
            return false;
        }

        Pose2d aprilTagPose = new Pose2d(aprilTagList[0] * Constants.inToM, aprilTagList[1] * Constants.inToM, new Rotation2d(aprilTagList[3] * Math.PI / 180));
        SmartDashboard.putNumber("AlignReef/TargetTagID", tID);
        SmartDashboard.putNumberArray("AlignReef/AprilTagPose", new double[]{
            aprilTagPose.getX(), aprilTagPose.getY(), aprilTagPose.getRotation().getDegrees()
        });

        // Reef Offset Positions - log the chosen offsets
        if (Constants.contains(new double[] { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 }, tID)) {
            if (reefPos == ReefPos.LEFT) {
                offsetX = -0.41;
                offsetY = 0.13;
            } else if (reefPos == ReefPos.RIGHT) {
                offsetX = -0.41;
                offsetY = -0.2335;
            }
            SmartDashboard.putNumber("AlignReef/OffsetX", offsetX);
            SmartDashboard.putNumber("AlignReef/OffsetY", offsetY);
        }

        // The target rotation of the robot is opposite of the april tag's rotation
        double targetRotation = aprilTagPose.getRotation().getRadians() - Math.PI;
        // AngleModulus normalizes the difference to always take the shortest path
        targetRotation = MathUtil.angleModulus(targetRotation);
        SmartDashboard.putNumber("AlignReef/TargetRotation", Math.toDegrees(targetRotation));

        // Rotate offsets to match field orientation
        double newOffsetX = (offsetX * Math.cos(targetRotation)) - (offsetY * Math.sin(targetRotation));
        double newOffsetY = (offsetX * Math.sin(targetRotation)) + (offsetY * Math.cos(targetRotation));
        SmartDashboard.putNumber("AlignReef/RotatedOffsetX", newOffsetX);
        SmartDashboard.putNumber("AlignReef/RotatedOffsetY", newOffsetY);

        // Create target pose
        targetPose = new Pose2d(aprilTagPose.getX() + newOffsetX, aprilTagPose.getY() + newOffsetY, new Rotation2d(targetRotation));
        SmartDashboard.putNumberArray("AlignReef/TargetPose", new double[]{
            targetPose.getX(), targetPose.getY(), targetPose.getRotation().getDegrees()
        });

        // Set PID setpoint for rotation
        pidRotate.setSetpoint(targetPose.getRotation().getRadians());
        // Note: Distance PID setpoint is always 0 (we want distance to target = 0)

        return true;
    }

    /**
     * Update robot pose using vision if a valid tag is seen.
     * This continuously corrects odometry drift during alignment.
is     */
    private void updatePoseFromVision() {
        if (!useVisionCorrection) {
            return;
        }

        double currentTime = timer.get();
        if (currentTime - lastVisionUpdateTime < visionUpdateInterval) {
            return;
        }

        // Check if we see the target tag
        if (!limelight.isTagDetected()) {
            SmartDashboard.putString("AlignReef/VisionUpdate", "No tag detected");
            return;
        }

        int currentTagID = limelight.getTid();
        
        // Only use vision update if we see our target tag
        if (currentTagID != tID) {
            SmartDashboard.putString("AlignReef/VisionUpdate", "Wrong tag: " + currentTagID);
            return;
        }

        // Get tag count to verify quality
        double tagCount = limelight.getTagCount();
        if (tagCount < minTagCountForVisionUpdate) {
            SmartDashboard.putString("AlignReef/VisionUpdate", "Insufficient tag count");
            return;
        }

        // Get the vision-based pose estimate
        var llMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        
        if (llMeasurement != null && llMeasurement.tagCount > 0) {
            // Reset translation only (not rotation) to avoid jerky movements
            // This corrects position drift while keeping rotation smooth
            Pose2d visionPose = llMeasurement.pose;
            Pose2d currentPose = drivetrain.getState().Pose;
            
            // Create a new pose with vision translation but current rotation
            Pose2d correctedPose = new Pose2d(
                visionPose.getTranslation(),
                currentPose.getRotation()
            );
            
            // Reset the drivetrain's pose estimate
            drivetrain.resetPose(correctedPose);
            
            lastVisionUpdateTime = currentTime;
            SmartDashboard.putString("AlignReef/VisionUpdate", "Success");
            SmartDashboard.putNumberArray("AlignReef/VisionPose", new double[]{
                correctedPose.getX(), correctedPose.getY(), correctedPose.getRotation().getDegrees()
            });
        } else {
            SmartDashboard.putString("AlignReef/VisionUpdate", "Invalid measurement");
        }
    }

    @Override
    public void execute() {
        if (!tagDetected) {
            SmartDashboard.putBoolean("AlignReef/ExecuteSkipped", true);
            //drivetrain.setControl(stop); // May or may not be needed
            return;
        }

        // Update pose from vision (corrects odometry drift)
        updatePoseFromVision();

        // Get current pose
        Pose2d currentPose = drivetrain.getState().Pose;
        SmartDashboard.putNumberArray("AlignReef/CurrentPose", new double[]{
            currentPose.getX(), currentPose.getY(), currentPose.getRotation().getDegrees()
        });
        SmartDashboard.putNumber("AlignReef/ExecuteTime", timer.get());

        // Calculate the direct path (distance and direction) to target
        Translation2d translationToTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
        double linearDistance = translationToTarget.getNorm();  // How far away
        Rotation2d directionOfTravel = translationToTarget.getAngle();  // Which direction

        // Apply friction compensation when close to prevent overshoot
        double frictionComp = 0.0;
        if (linearDistance >= frictionDistanceThreshold) {
            frictionComp = frictionConstant * maxVelocity;
        }

        // Calculate velocity magnitude using PID - uses distance error where setpoint is 0 (at target)
        // PID outputs higher velocity when far away, lower velocity when close
        double velocityMagnitude = pidDistance.calculate(0, linearDistance);
        velocityMagnitude = Math.min(Math.abs(velocityMagnitude) + frictionComp, maxVelocity);

        // Decompose velocity into x and y components based on direction to target
        double velocityX = velocityMagnitude * directionOfTravel.getCos();
        double velocityY = velocityMagnitude * directionOfTravel.getSin();

        // Calculate rotational velocity (happens simultaneously with translation)
        double velocityYaw = pidRotate.calculate(currentPose.getRotation().getRadians());
        velocityYaw = MathUtil.clamp(velocityYaw, -maxAngularVelocity, maxAngularVelocity);

        // SmartDashboard for logging
        SmartDashboard.putNumber("AlignReef/LinearDistance", linearDistance);
        SmartDashboard.putNumber("AlignReef/DirectionOfTravel", directionOfTravel.getDegrees());
        SmartDashboard.putNumber("AlignReef/VelocityMagnitude", velocityMagnitude);
        double yawErrorDegrees = Math.toDegrees(MathUtil.angleModulus(pidRotate.getSetpoint() - currentPose.getRotation().getRadians()));
        SmartDashboard.putNumber("AlignReef/YawError", yawErrorDegrees);
        SmartDashboard.putNumber("AlignReef/VelocityX", velocityX);
        SmartDashboard.putNumber("AlignReef/VelocityY", velocityY);
        SmartDashboard.putNumber("AlignReef/VelocityYaw", velocityYaw);
        SmartDashboard.putNumber("AlignReef/FrictionComp", frictionComp);
        SmartDashboard.putNumber("AlignReef/CurrentTagID", limelight.getTid());

        // Apply velocities
        // Robot moves toward target and rotates simultaneously
        drivetrain.setControl(driveRequest
                .withVelocityX(velocityX)
                .withVelocityY(velocityY)
                .withRotationalRate(velocityYaw));
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("AlignReef/CommandEnded", true);
        SmartDashboard.putBoolean("AlignReef/Interrupted", interrupted);
        SmartDashboard.putNumber("AlignReef/EndTime", timer.get());
        
        drivetrain.setControl(stop); //May or may not be needed
        robotContainer.MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    @Override
    public boolean isFinished() {
        if (!tagDetected) {
            return true;
        }

        Pose2d currentPose = drivetrain.getState().Pose;
        Translation2d translationToTarget = targetPose.getTranslation().minus(currentPose.getTranslation());
        double linearDistance = translationToTarget.getNorm();
        double yawError = Math.abs(MathUtil.angleModulus(
                targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()));

        boolean atPosition = linearDistance < positionTolerance;
        boolean atRotation = yawError < yawTolerance;

        SmartDashboard.putBoolean("AlignReef/AtPosition", atPosition);
        SmartDashboard.putBoolean("AlignReef/AtRotation", atRotation);

        return atPosition && atRotation;
    }
}