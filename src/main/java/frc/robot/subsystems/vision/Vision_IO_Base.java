package frc.robot.subsystems.vision;

//Java includes
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

// anything that uses the interface must provide methods
public interface Vision_IO_Base {
    // AdvantageKit logging
    @AutoLog
    public static class Vision_IO_Base_Input {
        // array of april tag IDs
        public int[] april_tag_IDs = new int[0];
        public pose_estimation_data[] pose_estimation_data = new pose_estimation_data[0];
        public rotation angle_to_tag = new rotation(new Rotation2d(), new Rotation2d());
        public boolean controller_found = false;
    }

    // a record is like an array
    // rot x and y are pitch and yaw i think
    public static record rotation(Rotation2d rot_x, Rotation2d rot_y) {}

    // data taken every 1/20th of a second to estimate position
    public static record pose_estimation_data(
        double timestamp,
        // assuming how confident it is in the estimate
        double confidence,
        int april_tag_count,
        double average_tag_distance,
        Pose3d position,
        vision_configuration_type type) {}

    public static enum vision_configuration_type {
        METATAG_1,
        METATAG_2,
        PHOTOVISION
    }

    //creates default method for interface calling object using interface runs method
    public default void update_inputs(Vision_IO_Base_Input inputs){}
}
