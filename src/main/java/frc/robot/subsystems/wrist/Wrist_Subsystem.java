package frc.robot.subsystems.wrist;

// Subsystems
import frc.robot.subsystems.wrist.Wrist_Constants;

// FRC
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class Wrist_Subsystem {
    // Motor
    final int MOTOR_ID = 1;
    public com.ctre.phoenix6.hardware.TalonFX m_motor;
    public com.ctre.phoenix6.hardware.TalonFX motor_meth() {m_motor = new com.ctre.phoenix6.hardware.TalonFX(MOTOR_ID); return m_motor;}

    private static Angle Pos_A = Angle.ofBaseUnits(10, null);

    private void go_to_A() {}
}
