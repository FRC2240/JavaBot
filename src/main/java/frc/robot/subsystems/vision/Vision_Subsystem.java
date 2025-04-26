package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import java.util.function.Consumer;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class Vision_Subsystem extends SubsystemBase {


    //private final vision_consumer consumer;
    //array from Vision IO base elements are objects that implement io base
    //private final Vision_IO_Base[] IO_Base;




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