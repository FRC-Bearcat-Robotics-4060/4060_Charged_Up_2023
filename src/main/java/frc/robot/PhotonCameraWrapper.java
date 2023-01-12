/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*
Click on the WPI icon on the top right of your VS Code window or hit Ctrl+Shift+P (Cmd+Shift+P on macOS) to bring up the command palette.
Type, “Manage Vendor Libraries” and select the “WPILib: Manage Vendor Libraries” option. 
Then, select the “Install new library (online)” option.
Paste the following URL into the box that pops up:
// https://maven.photonvision.org/repository/internal/org/photonvision/PhotonLib-json/1.0/PhotonLib-json-1.0.json
*/

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
//import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.VisionConstants;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PhotonCameraWrapper {
    public PhotonCamera photonCamera;
    public RobotPoseEstimator robotPoseEstimator;

    public PhotonCameraWrapper() {

        // Set up arena using apriltags positions provided at 
        //      https://firstfrc.blob.core.windows.net/frc2023/FieldAssets/2023LayoutMarkingDiagram.pdf
        ArrayList<AprilTag> atList = new ArrayList<AprilTag>();
        
        atList.add(new AprilTag(1,new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22),new Rotation3d(0,0,Units.degreesToRadians(180)))));
        atList.add(new AprilTag(2,new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22),new Rotation3d(0,0,Units.degreesToRadians(180)))));
        atList.add(new AprilTag(3,new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22),new Rotation3d(0,0,Units.degreesToRadians(180)))));
        atList.add(new AprilTag(4,new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38),new Rotation3d(0,0,Units.degreesToRadians(180)))));
        atList.add(new AprilTag(5,new Pose3d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38),new Rotation3d(0,0,Units.degreesToRadians(0)))));
        atList.add(new AprilTag(6,new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22),new Rotation3d(0,0,Units.degreesToRadians(0)))));
        atList.add(new AprilTag(7,new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22),new Rotation3d(0,0,Units.degreesToRadians(0)))));
        atList.add(new AprilTag(8,new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22),new Rotation3d(0,0,Units.degreesToRadians(0)))));
        
        AprilTagFieldLayout atfl =
                new AprilTagFieldLayout(atList, FieldConstants.length, FieldConstants.width);

        // Forward Camera
        photonCamera =
                new PhotonCamera(
                        VisionConstants
                                .cameraName); // Change the name of your camera here to whatever it is in the
        // PhotonVision UI.

        // ... Add other cameras here

        // Assemble the list of cameras & mount locations
        var camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(photonCamera, VisionConstants.robotToCam));

        
        robotPoseEstimator = new RobotPoseEstimator(atfl, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camList);
    }

    /**
     * @param estimatedRobotPose The current best guess at robot pose
     * @return A pair of the fused camera observations to a single Pose2d on the field, and the time
     *     of the observation. Assumes a planar field and the robot is always firmly on the ground
     */
    public Pair<Pose2d, Double> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        robotPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
        if (result.isPresent()) {
                
                Pose3d p3d1 = result.get().getFirst();
                Double p3d2 = result.get().getSecond();
                if (p3d1 == null || p3d2 == null){
                         return new Pair<Pose2d, Double>(null, 0.0);
                } else {
                        return new Pair<Pose2d, Double>(
                                p3d1.toPose2d(), currentTime - p3d2);
                }
           
        } else {
            return new Pair<Pose2d, Double>(null, 0.0);
        }
    }
}
