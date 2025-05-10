package frc.robot.utils;

// FRC
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class Motor_Template {

    // Motor Instance Constructor
    public com.ctre.phoenix6.hardware.TalonFX m_motor;

    public Motor_Template(TalonFX motor) {
        m_motor = motor;
    }

    // Position
    public Angle mod_angle = Angle.ofBaseUnits(0, Degrees);

    public StatusSignal<Angle> get_position() {
        return m_motor.getPosition();
    }

    public void set_position(Angle angle) {
        m_motor.setPosition(angle.plus(mod_angle));
    }

    public void position_modifier(Angle value) {
        mod_angle = value;
    }

    // Velocity
    public AngularVelocity mod_velocity = AngularVelocity.ofBaseUnits(0, RotationsPerSecond);

    public StatusSignal<AngularVelocity> get_velocity() {
        return m_motor.getVelocity();
    }

    public void set_velocity(AngularVelocity velocity) {
        m_motor.setControl(control_request.withVelocity(velocity.plus(mod_velocity)));
    }

    public void velocity_modifier(AngularVelocity value) {
        mod_velocity = value;
    }

    // Private Variables
    private static com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC control_request;
}
