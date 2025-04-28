package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.vision.Vision_IO_Base.Vision_IO_Base_Input;
import frc.robot.subsystems.vision.Vision_Constants;

import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
//import edu.wpi.first.wpilibj.Alert; disconnected logging

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Vision_Subsystem extends SubsystemBase {

    // empty but can hold an object that implements vision consumer
    private final vision_consumer consumer;
    // empty array that can accept any object implementing the interface
    private final Vision_IO_Base[] IO_base;
    private final Vision_IO_Base_Input[] input;

    // elipces means multiple objects of vision_IO_Base class can be passed in so
    // multiple camras
    public Vision_Subsystem(vision_consumer consumer, Vision_IO_Base... IO_base) {
        // this passes the private final consumer in
        // so running the method uses parameters to define private final variables which
        // then can't be changed

        this.consumer = consumer;
        this.IO_base = IO_base;

        this.input = new Vision_IO_Base_Input[IO_base.length];

    }

    public Rotation2d getTargetX(int cameraIndex) {
        return input[cameraIndex].angle_to_tag.rot_x();
    }

    // updates input and logs for each camera
    @Override
    public void periodic() {
        for (int i = 0; i < IO_base.length; i++) {
            IO_base[i].update_inputs(input[i]);
            // Logger.processInputs("Vision", input[i]);
        }

        List<Pose3d> all_tag_poses = new LinkedList<>();
        List<Pose3d> all_robot_poses = new LinkedList<>();
        List<Pose3d> all_accepted_poses = new LinkedList<>();
        List<Pose3d> all_rejected_poses = new LinkedList<>();

        for (int i = 0; i < IO_base.length; i++) {

            List<Pose3d> tag_poses = new LinkedList<>();
            List<Pose3d> robot_poses = new LinkedList<>();
            List<Pose3d> accepted_poses = new LinkedList<>();
            List<Pose3d> rejected_poses = new LinkedList<>();

            // each tag ID appends its position to tag poses
            for (int tag_ID : input[i].april_tag_IDs) {
                var tag_pose = Vision_Constants.april_tag_layout.getTagPose(tag_ID);
                if (tag_pose.isPresent()) {
                    tag_poses.add(tag_pose.get());
                }
            }

            //confirms estimation data is with in thresholds
            //example has many unecissary seeming conditions
            //come back to
            //potential point of failure
            for (var estimation : input[i].pose_estimation_data) {
                boolean reject_pose = 
                        estimation.april_tag_count() == 0 //rejects estimates made without tags
                        || (estimation.april_tag_count() == 1 && estimation.uncertainty() > Vision_Constants.max_uncertainty);
            }
            
        }
    }

    // marks a function interface
    // functional interface allows for 1 method here accepts()
    @FunctionalInterface
    // consumer accepts but returns nothing
    public static interface vision_consumer {
        public void accepts(
                double timestamp,
                Pose2d robot_pose,
                // how much uncertainty there is
                // <N3, N1> assumedly gives 3 standerd deviations or takes standerd devaition of
                // three numbers
                Matrix<N3, N1> Stdevs);
    }
}