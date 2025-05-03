package frc.robot.subsystems.wrist;

// FRC
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.utils.Motor_Template;

public class Wrist_Subsystem {
    // Motors
    final static int WRIST_ID = 1;
    final static com.ctre.phoenix6.hardware.TalonFX m_wrist = new TalonFX(WRIST_ID);

    final static int GRABBER_ID = 1;
    final static com.ctre.phoenix6.hardware.TalonFX m_grabber = new TalonFX(GRABBER_ID);

    // Motor Template
    public static Motor_Template Wrist = new Motor_Template(m_wrist);
    public static Motor_Template Grabber = new Motor_Template(m_grabber);

    // Positions
    static final Angle Pos_A = Angle.ofBaseUnits(180, Degrees);
    static final AngularVelocity Vel_A = AngularVelocity.ofBaseUnits(0.05, RotationsPerSecond);

    static final Angle Pos_B = Angle.ofBaseUnits(360, Degrees);
    static final AngularVelocity Vel_B = AngularVelocity.ofBaseUnits(10, RotationsPerSecond);

    // Go to Pos
    public void go_to_A() {
        Wrist.set_position(Pos_A);
        Grabber.set_velocity(Vel_A);
    }

    public void go_to_B() {
        Wrist.set_position(Pos_B);
        Grabber.set_velocity(Vel_B);
    }

    public void grabber_stop() {
        Grabber.set_velocity(AngularVelocity.ofBaseUnits(0, RotationsPerSecond));
    }
}
