package frc.robot.subsystems.vision;

import java.lang.ModuleLayer.Controller;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.function.Supplier;
import java.util.Set;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//import java.util.function.Supplier;
//import edu.wpi.first.math.geometry.Rotation2d;

public class Vision_IO_Real_Limelight implements Vision_IO_Base {
    // supliers store functions so they are more like variables
    private final Supplier<Rotation2d> rotation_supplier;

    // publisher sends data in/on a topic which works like a channel subscriber on
    // same topic receives it
    private final DoubleArrayPublisher orientation_publisher;

    private final DoubleSubscriber latency_subscriber;
    private final DoubleSubscriber rot_x_subscriber;
    private final DoubleSubscriber rot_y_subscriber;
    private final DoubleArraySubscriber metatag1Subscriber;
    private final DoubleArraySubscriber metatag2Subscriber;

    public Vision_IO_Real_Limelight(String name, Supplier<Rotation2d> rotation_supplier) {
        var table = NetworkTableInstance.getDefault().getTable(name);
        this.rotation_supplier = rotation_supplier;
        orientation_publisher = table.getDoubleArrayTopic("robot orientation set").publish();

        latency_subscriber = table.getDoubleTopic("latency").subscribe(0.0);
        rot_x_subscriber = table.getDoubleTopic("rot_x").subscribe(0.0);
        rot_y_subscriber = table.getDoubleTopic("rot_y").subscribe(0.0);
        metatag1Subscriber = table.getDoubleArrayTopic("bot_pos").subscribe(new double[] {});
        metatag2Subscriber = table.getDoubleArrayTopic("bot_pos_orb").subscribe(new double[] {});
    }

    // overrides default method
    @Override
    public void update_inputs(Vision_IO_Base_Input inputs) {
        // checks controller connection based of off if there was an update in the last 250 ms
        inputs.controller_found = ((RobotController.getFPGATime() - latency_subscriber.getLastChange()) / 1000) < 250;
        // update all inputs
        inputs.angle_to_tag = new rotation(Rotation2d.fromDegrees(rot_x_subscriber.get()), Rotation2d.fromDegrees(rot_y_subscriber.getLastChange()));

        // Set is like an unordered array without duplicates
        Set<Integer> april_tag_IDs = new HashSet<>();
        // in a linked list each index is call in order and leads to the next
        List<pose_estimation_data> pose_estimation_data = new LinkedList<>();

        //publisher sends to network table
        //.get gets stored function
        orientation_publisher.accept (new double[] {rotation_supplier.get().getDegrees()});
        
        // updates network table
        NetworkTableInstance.getDefault().flush();

        // for each bit of raw data that has changed since the last call
        for (var raw_data : metatag2Subscriber.readQueue()) {
            if (raw_data.value.length == 0)
                continue;
            for (int i = 1; i < raw_data.value.length; i++){
                
            }

            // .add() appends to list
            pose_estimation_data.add(
                    new pose_estimation_data(

                            0,

                            0,

                            0,

                            0,

                            null,

                            vision_configuration_type.METATAG_2));

        }

        // saves estimation data to objects
        for (int i = 0; i < pose_estimation_data.size(); i++) {
            inputs.pose_estimation_data[i] = pose_estimation_data.get(i);
        }

    }

}
