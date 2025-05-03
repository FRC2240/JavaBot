package frc.robot.utils;

// FRC
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

// CTRE
import com.ctre.phoenix6.StatusSignal;

public class Motor_Template {

    // Position
    public static Angle mod_angle;

    public static StatusSignal<Angle> get_position(com.ctre.phoenix6.hardware.TalonFX motor) {
        return motor.getPosition();
    }

    public static void set_position(com.ctre.phoenix6.hardware.TalonFX motor, Angle angle) {
        motor.setPosition(angle.plus(mod_angle));
    }

    public static void position_modifier(Angle value) {
        mod_angle = value;
    }

    // Velocity
    public static AngularVelocity mod_velocity;

    public static StatusSignal<AngularVelocity> get_velocity(com.ctre.phoenix6.hardware.TalonFX motor) {
        return motor.getVelocity();
    }

    public static void set_velocity(com.ctre.phoenix6.hardware.TalonFX motor, AngularVelocity velocity) {
        motor.setControl(control_request.withVelocity(velocity.plus(mod_velocity)));
    }

    public static void velocity_modifier(AngularVelocity value) {
        mod_velocity = value;
    }

    // Private Variables
    private static com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC control_request;
}
