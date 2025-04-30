package frc.robot.subsystems.vision;

import java.util.HashSet;
import java.util.LinkedList;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;

public class Parent_Photonvision_Vision_IO implements Base_Vision_IO{
    protected final PhotonCamera camera;
    protected final Transform3d camera_pos;

    //creates camera object
    public Parent_Photonvision_Vision_IO(String name, Transform3d camera_pos){
        camera = new PhotonCamera(name);
        this.camera_pos = camera_pos;
    }

    //nearly identical to Limelight reference limelight
    @Override 
    public void update_inputs(Base_Vision_IO_Input inputs){
        

        //Set<Integer> april_tag_IDs = new HashSet<>();
        //List<pose_estimation_data> pose_estimation_data = new LinkedList<>();

        for (var raw_data : camera.getAllUnreadResults()) {
            if (raw_data.hasTargets()){
                
            }
            else{

            }

            
        }
    }
    
}