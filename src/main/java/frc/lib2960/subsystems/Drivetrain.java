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

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;

/**
 * Interface for all drivetrain objects
 */
public interface Drivetrain {
    public void setRobotRelativeSpeeds(ChassisSpeeds speeds);
    public ChassisSpeeds getRobotRelativeSpeeds();
    public Pose2d getEstimatedPos() 
    public void resetPoseEst(Pose2d new_pose);
    public void addVisionPose(Pose2d pose, double time_stamp);
    public void addVisionPose(Pose2d pose, double time_stamp, Matrix<N3,​N1> std_dev);
}