/**
 * Copyright 2024 Ryan Fitz-Gerald
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to 
 * deal in the Software without restriction, including without limitation the 
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is 
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 */

package ftc.lib2960.subsystems;

import frc.lib2960.util.*;
import frc.lib2960.controllers.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.*;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

/**
 * Base class for a differential drivetrain
 */
public class DiffDriveBase extends SubSystemBase implements Drivetrain {
    /**
     * Differential Drivetrain settings
     */
    public class Settings {
        // TODO Add appropriate settings
    }


    public final Settings settings;

    public DiffDriveBase(Settings settings) {
        this.settings = settings;
    }

    public void setRobotRelativeSpeeds(ChassisSpeeds speeds) {
        // TODO Implement
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        // TODO Implement
        return new ChassisSpeeds();
    }

    public Pose2d getEstimatedPos()  {
        // TODO Implement
        return new Pose2d();
    }

    public void resetPoseEst(Pose2d new_pose) {
        // TODO Implement
    }

    public void addVisionPose(Pose2d pose, double time_stamp) {
        // TODO Implement
    }

    public void addVisionPose(Pose2d pose, double time_stamp, Matrix<N3,​N1> std_dev) {
        // TODO Implement
    }
}