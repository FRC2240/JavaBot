package frc.robot.subsystems.vision;

//Java includes
import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

// anything that uses the interface must provide methods
public interface Base_Vision_IO {
    // AdvantageKit logging
    @AutoLog
    public static class Base_Vision_IO_Input {
        // array of april tag IDs
        public int[] april_tag_IDs = new int[0];
        public pose_estimation_data[] pose_estimation_data = new pose_estimation_data[0];
        public rotation angle_to_tag = new rotation(new Rotation2d(), new Rotation2d());
        public boolean controller_found = false;
    }

    // a record is like an array
    //rot_x & y refer to the angle to the tag
    public static record rotation(Rotation2d rot_x, Rotation2d rot_y) {}

    // data taken every 1/20th of a second to estimate position
    public static record pose_estimation_data(
        double timestamp,
        // assuming how confident it is in the estimate
        double uncertainty,
        int april_tag_count,
        double average_tag_distance,
        Pose3d position,
        vision_configuration_type type) {}

    public static enum vision_configuration_type {
        MEGATAG_1,
        MEGATAG_2,
        PHOTOVISION
    }

    //creates default method for interface calling object using interface runs method
    public default void update_inputs(Base_Vision_IO_Input inputs){}
}
