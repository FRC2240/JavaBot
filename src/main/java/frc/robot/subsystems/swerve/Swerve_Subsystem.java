package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.json.simple.parser.ParseException;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveInputStream;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve_Subsystem extends SubsystemBase {

    private final SwerveDrive swerveDrive;

    public Swerve_Subsystem() {
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
        // objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            File directory = new File(Filesystem.getDeployDirectory(), Swerve_Constants.CONFIG_DIRECTORY);
            swerveDrive = new SwerveParser(directory).createSwerveDrive(Swerve_Constants.MAX_SPEED);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        // Heading correction should only be used while controlling the robot via angle
        swerveDrive.setHeadingCorrection(false);

        // Disables cosine compensation for simulations since it causes discrepancies
        // not seen in real life.
        swerveDrive.setCosineCompensator(!RobotBase.isSimulation());

        // Correct for skew that gets worse as angular velocity increases.
        swerveDrive.setAngularVelocityCompensation(true, true, 0.1);

        // Enable if you want to resynchronize your absolute encoders and motor encoders
        // periodically when they are not moving.
        swerveDrive.setModuleEncoderAutoSynchronize(false, 1);

        setup_pathplanner();

        // RobotModeTriggers.autonomous().onTrue(Commands.runOnce(this::zeroGyroWithAlliance));
    }

    @Override
    public void periodic() {
        swerveDrive.updateOdometry();
    }

    @Override
    public void simulationPeriodic() {
    }

    public void setup_pathplanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            final boolean enableFeedforward = true;
            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::get_pose,
                    // Robot pose supplier
                    this::reset_odometry,
                    // Method to reset odometry (will be called if your auto has a starting pose)
                    this::get_robot_velocity,
                    // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speedsRobotRelative, moduleFeedForwards) -> {
                        if (enableFeedforward) {
                            swerveDrive.drive(
                                    speedsRobotRelative,
                                    swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                                    moduleFeedForwards.linearForces());
                        } else {
                            swerveDrive.setChassisSpeeds(speedsRobotRelative);
                        }
                    },
                    // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
                    // optionally outputs individual module feedforwards
                    new PPHolonomicDriveController(
                            // PPHolonomicController is the built in path following controller for holonomic
                            // drive trains
                            new PIDConstants(5.0, 0.0, 0.0),
                            // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0)
                    // Rotation PID constants
                    ),
                    config,
                    // The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red
                        // alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this
            // Reference to this subsystem to set requirements
            );

        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Preload PathPlanner Path finding
        // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
        PathfindingCommand.warmupCommand().schedule();
    }

    // Drives to a pose on the field
    public Command drive_to_pose_command(Pose2d pose) {
        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                swerveDrive.getMaximumChassisVelocity(), Swerve_Constants.Auto_Constraints.MAX_LINEAR_ACCEL,
                swerveDrive.getMaximumChassisAngularVelocity(), Swerve_Constants.Auto_Constraints.MAX_ANGULAR_ACCEL);

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                0 // Goal end velocity in meters/sec
        );
    }

    // Drive with setpoint generator by team 254
    private Command driveWithSetpointGenerator(Supplier<ChassisSpeeds> robotRelativeChassisSpeed)
            throws IOException, ParseException {
        SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(RobotConfig.fromGUISettings(),
                swerveDrive.getMaximumChassisAngularVelocity());
        AtomicReference<SwerveSetpoint> prevSetpoint = new AtomicReference<>(
                new SwerveSetpoint(swerveDrive.getRobotVelocity(),
                        swerveDrive.getStates(),
                        DriveFeedforwards.zeros(swerveDrive.getModules().length)));
        AtomicReference<Double> previousTime = new AtomicReference<>();

        return startRun(() -> previousTime.set(Timer.getFPGATimestamp()),
                () -> {
                    double newTime = Timer.getFPGATimestamp();
                    SwerveSetpoint newSetpoint = setpointGenerator.generateSetpoint(prevSetpoint.get(),
                            robotRelativeChassisSpeed.get(),
                            newTime - previousTime.get());
                    swerveDrive.drive(newSetpoint.robotRelativeSpeeds(),
                            newSetpoint.moduleStates(),
                            newSetpoint.feedforwards().linearForces());
                    prevSetpoint.set(newSetpoint);
                    previousTime.set(newTime);

                });
    }

    // Drive with setpoint generator by team 254 field relative
    public Command driveWithSetpointGeneratorFieldRelative(Supplier<ChassisSpeeds> fieldRelativeSpeeds) {
        try {
            return driveWithSetpointGenerator(() -> {
                return ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds.get(), get_heading());

            });
        } catch (Exception e) {
            DriverStation.reportError(e.toString(), true);
        }
        return Commands.none();

    }

    // Command to characterize the robot drive motors using SysId
    public Command sys_id_drive_motor_command() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(
                        new Config(),
                        this, swerveDrive, 12, true),
                3.0, 5.0, 3.0);
    }

    // Command to characterize the robot angle motors using SysId
    public Command sys_id_angle_motor_command() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(
                        new Config(),
                        this, swerveDrive),
                3.0, 5.0, 3.0);
    }

    // Centers the all modules
    public Command center_modules_command() {
        return run(() -> Arrays.asList(swerveDrive.getModules())
                .forEach(it -> it.setAngle(0.0)));
    }

    // Drives with a controller
    public Command controller_drive_command(CommandXboxController driverController) {
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(swerveDrive,
                () -> driverController.getLeftY() * -1,
                () -> driverController.getLeftX() * -1)
                .withControllerRotationAxis(driverController::getRightX)
                .deadband(0.1)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true);

        SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(swerveDrive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX())
                .withControllerRotationAxis(() -> driverController.getRawAxis(
                        2))
                .deadband(0.1)
                .scaleTranslation(0.8)
                .allianceRelativeControl(true);

        if (RobotBase.isSimulation()) {
            return drive_field_oriented_command(driveAngularVelocityKeyboard);
        } else {
            return drive_field_oriented_command(driveAngularVelocity);
        }
    }

    // Replaces the feed forward coefficients on all modules
    public void replace_swerve_module_feed_forward(double kS, double kV, double kA) {
        swerveDrive.replaceSwerveModuleFeedforward(new SimpleMotorFeedforward(kS, kV, kA));
    }

    // Drives the robot
    public Command drive_command(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier angularRotationX) {
        return run(() -> {
            swerveDrive.drive(SwerveMath.scaleTranslation(new Translation2d(
                    translationX.getAsDouble() * swerveDrive.getMaximumChassisVelocity(),
                    translationY.getAsDouble() * swerveDrive.getMaximumChassisVelocity()), 0.8),
                    Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumChassisAngularVelocity(),
                    true,
                    false);
        });
    }

    // Drives the robot
    public Command drive_command(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
            DoubleSupplier headingY) {
        // swerveDrive.setHeadingCorrection(true); // Normally you would want heading
        // correction for this kind of control.
        return run(() -> {

            Translation2d scaledInputs = SwerveMath.scaleTranslation(new Translation2d(translationX.getAsDouble(),
                    translationY.getAsDouble()), 0.8);

            // Make the robot move
            drive_field_oriented(swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(), scaledInputs.getY(),
                    headingX.getAsDouble(),
                    headingY.getAsDouble(),
                    swerveDrive.getOdometryHeading().getRadians(),
                    swerveDrive.getMaximumChassisVelocity()));
        });
    }

    // The primary method for controlling the drivebase
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    // Drive the robot given a chassis field oriented velocity.
    public void drive_field_oriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    // Drive the robot given a chassis field oriented velocity.
    public Command drive_field_oriented_command(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    // Drives given a chassis robot oriented velocity
    public void drive(ChassisSpeeds velocity) {
        swerveDrive.drive(velocity);
    }

    // Get the swerve drive kinematics object.
    public SwerveDriveKinematics get_kinematics() {
        return swerveDrive.kinematics;
    }

    /*
     * Resets odometry to the given pose. Gyro angle and module positions do not
     * need to be reset when calling this method. However, if either gyro angle
     * or module position is reset, this must be called in order for odometry to
     * keep working.
     */
    public void reset_odometry(Pose2d initialHolonomicPose) {
        swerveDrive.resetOdometry(initialHolonomicPose);
    }

    // Gets the current odometry pose of the robot
    public Pose2d get_pose() {
        return swerveDrive.getPose();
    }

    // Set chassis speeds with closed-loop velocity control.
    public void set_chassis_speeds(ChassisSpeeds chassisSpeeds) {
        swerveDrive.setChassisSpeeds(chassisSpeeds);
    }

    // Post the trajectory to the field.
    public void post_trajectory(Trajectory trajectory) {
        swerveDrive.postTrajectory(trajectory);
    }

    // Resets the gyro and odometry rotation to 0
    public void zero_gyro() {
        swerveDrive.zeroGyro();
    }

    // Checks if the alliance is red, defaults to false if alliance isn't available.
    private boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
    }

    // This will zero the robot to assume the current position is facing forward
    public void zero_gyro_with_alliance() {
        if (isRedAlliance()) {
            zero_gyro();
            // Set the pose 180 degrees
            reset_odometry(new Pose2d(get_pose().getTranslation(), Rotation2d.fromDegrees(180)));
        } else {
            zero_gyro();
        }
    }

    // Sets the drive motors to brake/coast mode.
    public void set_motor_brake(boolean brake) {
        swerveDrive.setMotorIdleMode(brake);
    }

    /*
     * Gets the current yaw angle of the robot, as reported by the swerve pose
     * estimator in the underlying drivebase.
     */
    public Rotation2d get_heading() {
        return get_pose().getRotation();
    }

    /*
     * Get the chassis speeds based on controller input of 2 joysticks. One for
     * speeds in which direction. The other for the angle of the robot.
     */
    public ChassisSpeeds get_target_speeds(double xInput, double yInput, double headingX, double headingY) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));
        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                headingX,
                headingY,
                get_heading().getRadians(),
                Swerve_Constants.MAX_SPEED);
    }

    /*
     * Get the chassis speeds based on controller input of 1 joystick and one angle.
     * Control the robot at an offset of 90deg.
     */
    public ChassisSpeeds get_target_speeds(double xInput, double yInput, Rotation2d angle) {
        Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

        return swerveDrive.swerveController.getTargetSpeeds(scaledInputs.getX(),
                scaledInputs.getY(),
                angle.getRadians(),
                get_heading().getRadians(),
                Swerve_Constants.MAX_SPEED);
    }

    // Gets the current field-relative velocity (x, y and omega) of the robot
    public ChassisSpeeds get_field_velocity() {
        return swerveDrive.getFieldVelocity();
    }

    // Gets the current velocity (x, y and omega) of the robot
    public ChassisSpeeds get_robot_velocity() {
        return swerveDrive.getRobotVelocity();
    }

    // Get the SwerveController in the swerve drive
    public SwerveController get_swerve_controller() {
        return swerveDrive.swerveController;
    }

    // Get the SwerveDriveConfiguration object
    public SwerveDriveConfiguration get_swerve_drive_configuration() {
        return swerveDrive.swerveDriveConfiguration;
    }

    // Lock the swerve drive to prevent it from moving.
    public void lock() {
        swerveDrive.lockPose();
    }

    // Gets the current pitch angle of the robot, as reported by the imu.
    public Rotation2d get_pitch() {
        return swerveDrive.getPitch();
    }

    // Gets the vision measurement
    public void add_vision_measurement(double timestamp, Pose2d robot_pose, Matrix<N3, N1> stdevs) {
        swerveDrive.addVisionMeasurement(robot_pose, timestamp, stdevs);
    }

    // Gets the swerve drive object
    public SwerveDrive get_swerve_drive() {
        return swerveDrive;
    }
}
