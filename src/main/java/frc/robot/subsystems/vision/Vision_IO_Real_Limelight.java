package frc.robot.subsystems.vision;

import java.lang.ModuleLayer.Controller;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;

//import java.util.function.Supplier;
//import edu.wpi.first.math.geometry.Rotation2d;

public class Vision_IO_Real_Limelight implements Vision_IO_Base {
    //supliers store functions so they are more like variables
    private final Supplier<Rotation2d> rotation_supplier; 

    //publisher sends data in/on a topic which works like a channel subscriber on same topic receives it
    private final DoubleArrayPublisher orientationPublisher;

    private final DoubleSubscriber latency_subscriber;
    private final DoubleSubscriber rot_x_subscriber;
    private final DoubleSubscriber rot_y_subscriber;
    private final DoubleArraySubscriber metatag1Subscriber;
    private final DoubleArraySubscriber metatag2Subscriber;



    //creates a limelight object
    public Vision_IO_Real_Limelight(String name, Supplier<Rotation2d> rotation_supplier){
        var table = NetworkTableInstance.getDefault().getTable(name);
        this.rotation_supplier = rotation_supplier;
        orientationPublisher = table.getDoubleArrayTopic("robot orientation set").publish();
        
        latency_subscriber = table.getDoubleTopic("latency").subscribe(0.0);
        rot_x_subscriber = table.getDoubleTopic("rot_x").subscribe(0.0);
        rot_y_subscriber = table.getDoubleTopic("rot_y").subscribe(0.0);
        metatag1Subscriber = table.getDoubleArrayTopic("bot_pos").subscribe(new double[] {});
        metatag2Subscriber = table.getDoubleArrayTopic("bot_pos_orb").subscribe(new double[] {});
    }

    //overrides default method
    @Override
    public void update_inputs(Vision_IO_Base_Input inputs) {
        //checks controller connection based of off if there was an update in the last 250 ms
        inputs.controller_found = ((RobotController.getFPGATime() - latency_subscriber.getLastChange())/1000) <  250;
        //update all inputs
        inputs.angle_to_tag = new rotation(Rotation2d.fromDegrees(rot_x_subscriber.get()), Rotation2d.fromDegrees(rot_y_subscriber.getLastChange()));

        Vision_IO_Base.super.update_inputs(inputs);
    }
}
