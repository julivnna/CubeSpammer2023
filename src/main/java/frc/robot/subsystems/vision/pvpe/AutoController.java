
package frc.robot.subsystems.vision.pvpe;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;

/**
 * Implements logic to convert a set of desired waypoints (ie, a trajectory) and the current
 * estimate of where the robot is at (ie, the estimated Pose) into motion commands for a drivetrain.
 * The Ramaste controller is used to smoothly move the robot from where it thinks it is to where it
 * thinks it ought to be.
 */
public class AutoController {
    private Trajectory trajectory;

    private RamseteController ramsete = new RamseteController();

    private Timer timer = new Timer();

    boolean isRunning = false;

    Trajectory.State desiredDtState;

    public AutoController() {
    //     "ID": 5,
    //   "pose": {
    //     "translation": {
    //       "x": 0.36195,
    //       "y": 6.749796,
    //       "z": 0.695452
    //     },
    //     "rotation": {
    //       "quaternion": {
    //         "W": 1.0,
    //         "X": 0.0,
    //         "Y": 0.0,
    //         "Z": 0.0
    //       }
        // Change this trajectory if you need the robot to follow different paths.
        trajectory =
                TrajectoryGenerator.generateTrajectory(
                        new Pose2d(0.36195, 6.749796, new Rotation2d()),
                        List.of(),
                        new Pose2d(1.36195, 6.749796, new Rotation2d()),
                        new TrajectoryConfig(0.002, 0.002));
                        //new TrajectoryConfig(2, 2));
    }

    public AutoController( Trajectory trajectory)
    {
        this.trajectory = trajectory;
        //trajectory.
    }

    public void setNewTrajectory( Trajectory trajectory) // to be done: alot reset need to do
    {
        stopPath();
        this.trajectory = trajectory;
        //trajectory.
    }

    /**
     * @return The starting (initial) pose of the currently-active trajectory
     */
    public Pose2d getInitialPose() {
        return trajectory.getInitialPose();
    }

    /** Starts the controller running. Call this at the start of autonomous */
    public void startPath() {
        timer.reset();
        timer.start();
        isRunning = true;
    }

    /** Stops the controller from generating commands */
    public void stopPath() {
        isRunning = false;
        timer.reset();
    }

    /**
     * Given the current estimate of the robot's position, calculate drivetrain speed commands which
     * will best-execute the active trajectory. Be sure to call `startPath()` prior to calling this
     * method.
     *
     * @param curEstPose Current estimate of drivetrain pose on the field
     * @return The commanded drivetrain motion
     */
    public ChassisSpeeds getCurMotorCmds(Pose2d curEstPose) {
        if (isRunning) {
            double elapsed = timer.get();
            desiredDtState = trajectory.sample(elapsed);

            // to be checked?? if the time is expired for this trajectory???
        } else {
            desiredDtState = new Trajectory.State();
        }

        return ramsete.calculate(curEstPose, desiredDtState);
    }

    /**
     * @return The position which the auto controller is attempting to move the drivetrain to right
     *     now.
     */
    public Pose2d getCurPose2d() {
        return desiredDtState.poseMeters;
    }

    public Trajectory.State getCurrentState()
    {
        return desiredDtState;
    }
}
