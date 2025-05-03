package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.wrist.Wrist_Subsyetm_IO.WRIST_IO;

public class Wrist_Commands implements WRIST_IO {
    private final Wrist_Subsystem wrist = new Wrist_Subsystem();

    @Override
    public void go_to_A_override() {
        wrist.grabber_stop();

    }

    @Override
    public void go_to_b_override() {
        wrist.grabber_stop();

    }

    @Override
    public void grabber_stop_override() {
        wrist.grabber_stop();

    }

    public Command command_go_to_A() {
        return Commands.runOnce(this::go_to_A_override);
    }

    public Command command_go_to_B() {
        return Commands.runOnce(this::go_to_b_override);
    }

    public Command command_grabber_stop() {
        return Commands.runOnce(this::grabber_stop_override);
    }
}
