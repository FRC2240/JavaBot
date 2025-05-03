package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class Vision_Constants {
    // stores tag layout for the current year's feild
    public static AprilTagFieldLayout april_tag_layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // assuming stores distance from center of bot
    public static Transform3d camera_0_pos = new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)); // TBD
    public static Transform3d camera_1_pos = new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)); // TBD

    public static double max_uncertainty = 0.0; // TBD
    public static double max_z_error = 0.25; // TBD

    // all vals should be redone
    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    // static remains the same across every instance of the class but can be changed
    public static double linear_stdev_baseline = 0.02; // Meters
    public static double angular_stdev_baseline = 0.06; // Radians

    // completly stolen will come back to
    public static double linear_stdev_megatag2_factor = 0.5;
    public static double angular_stdev_megatag2_factor = Double.POSITIVE_INFINITY;

    // some camera trusted more than others
    public static double[] camera_stdev_factors = new double[] {
            1.0, // camera 0
            1.0 // camera 1
    };
}