package frc.robot.subsystems.swerve;

import java.io.File;

import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.Constants.RobotMode;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve_IO_Real implements Swerve_IO_Base {

    private final SwerveDrive swerveDrive;

    public Swerve_IO_Real() {

        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

        File directory = new File(Filesystem.getDeployDirectory(), Swerve_Constants.CONFIG_DIRECTORY);
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Swerve_Constants.MAX_SPEED);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        swerveDrive.setHeadingCorrection(false);

        swerveDrive.setCosineCompensator(Constants.CURRENT_MODE != RobotMode.SIM);

        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    }

    // Updates the inputs with the current values.
    @Override
    public void updateInputs(SwerveInputs inputs) {
        inputs.pose = swerveDrive.getPose();
        inputs.yaw = swerveDrive.getYaw();
        inputs.odometryHeading = swerveDrive.getOdometryHeading();
    }

    // Drives
    @Override
    public void drive(Translation2d translation, double rotation, boolean isFieldRelative, boolean isOpenLoop) {
        swerveDrive.drive(translation, rotation, isFieldRelative, isOpenLoop);
    }

    @Override
    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    @Override
    public void resetOdometry(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    @Override
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
    }

    @Override
    public SwerveDriveKinematics getKinematics() {
        return swerveDrive.kinematics;
    }



}
