package frc.robot.subsystems.vision;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;

import static frc.robot.subsystems.vision.Vision_Constants.april_tag_layout;

import java.util.function.Supplier;

public class Sim_Photonvision_Vision_IO extends Parent_Photonvision_Vision_IO {

    private static VisionSystemSim vision_sim;
    private final PhotonCameraSim camera_sim;

    private final Supplier<Pose2d> pose_supplier;


    public Sim_Photonvision_Vision_IO(String name, Supplier<Pose2d> pose_supplier, Transform3d camera_pos){
        super(name, camera_pos); //calls parent
        this.pose_supplier = pose_supplier;

        //sim init
        if (vision_sim == null) {
            vision_sim = new VisionSystemSim("Vision Sim"); 
            vision_sim.addAprilTags(april_tag_layout);
        }

        //add sim camera
        var cam_properties = new SimCameraProperties();
        camera_sim = new PhotonCameraSim(camera, cam_properties, april_tag_layout);
        vision_sim.addCamera(camera_sim, camera_pos);
    }

    @Override
    public void update_inputs(Base_Vision_IO_Input inputs){
        vision_sim.update(pose_supplier.get());
        super.update_inputs(inputs);
    }
}

