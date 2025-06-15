package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.Vision_Constants.april_tag_layout;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.Set;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public class Parent_Photonvision_Vision_IO implements Base_Vision_IO {
    protected final PhotonCamera camera;
    protected final Transform3d camera_pos;

    // creates camera object
    public Parent_Photonvision_Vision_IO(String name, Transform3d camera_pos) {
        camera = new PhotonCamera(name);
        // position relative to bot
        this.camera_pos = camera_pos;
    }

    // nearly identical to Limelight reference limelight
    @Override
    public void update_inputs(Base_Vision_IO_Input inputs) {
        inputs.controller_found = camera.isConnected();

        Set<Short> april_tag_IDs = new HashSet<>();
        List<pose_estimation_data> pose_estimation_data = new LinkedList<>();

        for (var results : camera.getAllUnreadResults()) {
            if (results.hasTargets()) {
                inputs.angle_to_tag = new rotation(
                        Rotation2d.fromDegrees(results.getBestTarget().getYaw()),
                        Rotation2d.fromDegrees(results.getBestTarget().getPitch()));
            } else {
                inputs.angle_to_tag = new rotation(new Rotation2d(), new Rotation2d());
            }
            if (results.multitagResult.isPresent()) { // multitag
                var multi_tag_data = results.multitagResult.get();

                // position calc
                Transform3d cam_field_pos = multi_tag_data.estimatedPose.best; // position relative to field
                Transform3d temp_2 = cam_field_pos.plus(camera_pos);
                Pose3d bot_pos = new Pose3d(temp_2.getTranslation(), temp_2.getRotation()); // TODO rename

                // average tag distance calc
                double total_tag_dist = 0.0;
                for (var target : results.targets) {
                    total_tag_dist += target.bestCameraToTarget.getTranslation().getNorm();
                }

                // add tag IDs
                april_tag_IDs.addAll(multi_tag_data.fiducialIDsUsed);

                // add estimations
                pose_estimation_data.add(
                        new pose_estimation_data(
                                results.getTimestampSeconds(),
                                multi_tag_data.estimatedPose.ambiguity,
                                multi_tag_data.fiducialIDsUsed.size(),
                                total_tag_dist / results.targets.size(),
                                bot_pos,
                                vision_configuration_type.PHOTONVISION));

            } else if (!results.targets.isEmpty()) { // one tag
                //
                List<PhotonTrackedTarget> single_target = results.getTargets();
                var target = single_target.get(0);

                //
                var tag_pos = april_tag_layout.getTagPose(target.getFiducialId());

                if (tag_pos.isPresent()) {
                    // bot pos
                    Transform3d fieldToTarget = new Transform3d(tag_pos.get().getTranslation(),
                            tag_pos.get().getRotation());
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(camera_pos.inverse());
                    Pose3d bot_pos = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

                    april_tag_IDs.add((short) target.getFiducialId());

                    // add estimations
                    pose_estimation_data.add(
                            new pose_estimation_data(
                                    results.getTimestampSeconds(),
                                    target.getPoseAmbiguity(),
                                    1,
                                    target.bestCameraToTarget.getTranslation().getNorm(),
                                    bot_pos,
                                    vision_configuration_type.PHOTONVISION));
                }
            }
        }

        // save estimation data
        inputs.pose_estimation_data = new pose_estimation_data[pose_estimation_data.size()];
        for (int i = 0; i < pose_estimation_data.size(); i++) {
            inputs.pose_estimation_data[i] = pose_estimation_data.get(i);
        }

        // save tag IDs
        inputs.april_tag_IDs = new int[april_tag_IDs.size()];
        int i = 0;
        for (int ID : april_tag_IDs) {
            inputs.april_tag_IDs[i++] = ID;
        }
    }
}