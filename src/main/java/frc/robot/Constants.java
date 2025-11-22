package frc.robot;

import java.util.HashMap;

public final class Constants {

    // Inches to Meters
    public static final double inToM = 0.0254;

    public enum StartingPos {
        LEFT,
        RIGHT,
        CENTER
    }

    public enum ReefPos {
        LEFT,
        RIGHT
    }

    // Pivot Intake Constants - These things should be adjusted to actual vals
    public static class PivotIntakeConstants {
        // Motor CAN IDs
        public static final int PIVOT_MOTOR_ID = 20;
        public static final int INTAKE_WHEEL_MOTOR_ID = 18;
        public static final int PIVOT_ENCODER_ID = 21;
        public static final int CORAL_SENSOR_ID = 22;
        
        // Pivot positions (in rotations) - 200° total range
        // Pivot encoder zeros at horizontal extended position
        // The robot physically starts at stowed position
        // Encoder offset is applied in PivotIntakeSubsystem constructor
        public static final double STOWED_POSITION = 0.46;          // Home position
        public static final double STOWED_POSITION_WITH_CORAL = 0.367; // Slightly above stowed for coral transfer - THIS NEEDS TO BE HIGHER!!!!!
        public static final double INTAKE_POSITION = 0.0;         // Ground collection
        public static final double REEF_SCORING_POSITION = 0.25;   // Reef scoring position
        
        // Motor speeds
        public static final double INTAKE_SPEED = -0.45;             // Collecting coral
        public static final double INTAKE_REVERSE_SPEED = 0.2;    // Intake wheels reverse during scoring
        
        // Sensor thresholds
        public static final double CORAL_DETECTED_DISTANCE_M = 0.03;
        public static final double CORAL_TOO_LOW_DISTANCE = 0.01;
        
        // PID Constants - Reduced kP to slow down response
        // READ THE DOCS ON THE CTRE WEBSITE - PUT IN HERE LATER
        public static final double PIVOT_KP = 0.2;  // Reduced from 24.0 //Increase from 1
        //public static final double PIVOT_KI = 0.02;  // Increased from 0
        public static final double PIVOT_KD = 0.02;   // setting to 14 caused really fast
        public static final double PIVOT_KG = -3; // Increased from .45 
        //public static final double PIVOT_KA = 3; // Feedforward thingy
        public static final double PIVOT_TOLERANCE = 0.09; // Increased from 0.02 - less tight for faster settling
    }

    // Returns true if the value is inside the list
    public static boolean contains(double[] array, double value) {
        for (double element : array) {
            if (element == value) {
                return true;
            }
        }
        return false;
    }

    // Holds all the positions of the April Tags
    public class AprilTagMaps {
        // Field Map Source: https://firstfrc.blob.core.windows.net/frc2025/FieldAssets/2025FieldDrawings-FieldLayoutAndMarking.pdf
        // A HashMap of April Tag positions: Key = ID, Value = [X, Y, Z, Yaw, Pitch]
        public static final HashMap<Integer, double[]> aprilTagMap = new HashMap<>();
        static {
            // Points are in inches, Angles are in degrees
            // RED SIDE
            // aprilTagMap.put(1, new double[]{657.37, 25.80, 58.50, 126.0, 0.0});
            // aprilTagMap.put(2, new double[]{657.37, 291.20, 58.50, 234.0, 0.0});
            aprilTagMap.put(3, new double[]{455.15, 317.15, 51.25, 270.0, 0.0});
            aprilTagMap.put(4, new double[]{365.20, 241.64, 73.54, 0.0, 30.0});
            aprilTagMap.put(5, new double[]{365.20, 75.39, 73.54, 0.0, 30.0});
            aprilTagMap.put(6, new double[]{530.49, 130.17, 12.13, 300.0, 0.0});
            aprilTagMap.put(7, new double[]{546.87, 158.50, 12.13, 0.0, 0.0});
            aprilTagMap.put(8, new double[]{530.49, 186.83, 12.13, 60.0, 0.0});
            aprilTagMap.put(9, new double[]{497.77, 186.83, 12.13, 120.0, 0.0});
            aprilTagMap.put(10, new double[]{481.39, 158.50, 12.13, 180.0, 0.0});
            aprilTagMap.put(11, new double[]{497.77, 130.17, 12.13, 240.0, 0.0});
            // BLUE SIDE
            //aprilTagMap.put(12, new double[]{33.51, 25.80, 58.50, 54.0, 0.0});
            // aprilTagMap.put(13, new double[]{33.51, 291.20, 58.50, 306.0, 0.0});
            aprilTagMap.put(14, new double[]{325.68, 241.64, 73.54, 180.0, 30.0});
            aprilTagMap.put(15, new double[]{325.68, 75.39, 73.54, 180.0, 30.0});
            aprilTagMap.put(16, new double[]{235.73, -0.15, 51.25, 90.0, 0.0});
            // Tag 17: (4.073906m, 3.306318m, 0.308102m, 4PI/3 rad, 0 rad)
            aprilTagMap.put(17, new double[]{160.39, 130.17, 12.13, 240.0, 0.0});
            // Tag 18: (3.6576m, 4.0259m, 0.308102m, PI rad, 0 rad)
            aprilTagMap.put(18, new double[]{144.00, 158.50, 12.13, 180.0, 0.0});
            aprilTagMap.put(19, new double[]{160.39, 186.83, 12.13, 120.0, 0.0});
            aprilTagMap.put(20, new double[]{193.10, 186.83, 12.13, 60.0, 0.0});
            aprilTagMap.put(21, new double[]{209.49, 158.50, 12.13, 0.0, 0.0});
            aprilTagMap.put(22, new double[]{193.10, 130.17, 12.13, 300.0, 0.0});
        }
    }



}
