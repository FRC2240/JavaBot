package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.AutoLog;

import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public interface Swerve_IO_Base {
    @AutoLog
    public static class SwerveInputs {
        public Pose2d pose = new Pose2d();
        public Rotation2d yaw = new Rotation2d();
        public Rotation2d odometryHeading = new Rotation2d();
    }

    // Updates the inputs with the current values.
    void updateInputs(SwerveInputs inputs);

    // Drives
    void drive(Translation2d translation, double rotation, boolean isFieldRelative, boolean isOpenLoop);

    Pose2d getPose();

    void resetOdometry(Pose2d pose);

    ChassisSpeeds getRobotVelocity();

    SwerveDriveKinematics getKinematics();
}
