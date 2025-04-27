package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.Vision_IO_Base.Vision_IO_Base_Input;

import java.util.function.Consumer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
//import edu.wpi.first.wpilibj.Alert; disconnected logging

import org.littletonrobotics.junction.Logger;



public class Vision_Subsystem extends SubsystemBase {

    //empty but can hold an object that implements vision consumer
    private final vision_consumer consumer;
    //empty array that can accept any object implementing the interface
    private final Vision_IO_Base[] IO_Base;
    private final Vision_IO_Base_Input[] input;


    //elipces means multiple objects of vision_IO_Base class can be passed in so multiple camras
    public Vision_Subsystem(vision_consumer consumer, Vision_IO_Base... IO_Base){
        //this passes the private final consumer in 
        //so running the method uses parameters to define private final variables which then can't be changed
        
        this.consumer = consumer;
        this.IO_Base = IO_Base;

        this.input = new Vision_IO_Base_Input[IO_Base.length];

    }


    //marks a function interface 
    //functional interface allows for 1 method here accepts()
    @FunctionalInterface
    //consumer accepts but returns nothing
    public static interface vision_consumer {
        public void accepts(    
            double timestamp,
            Pose2d robot_pose,
            //how much uncertainty there is
            //<N3, N1> assumedly gives 3 standerd deviations or takes standerd devaition of three numbers
            Matrix<N3, N1> Stdevs
    );
    }
}