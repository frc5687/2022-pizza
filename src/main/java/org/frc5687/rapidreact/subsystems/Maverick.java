package org.frc5687.rapidreact.subsystems;

import org.frc5687.rapidreact.config.Constants;
import org.frc5687.rapidreact.util.OutliersContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Maverick extends OutliersSubsystem{

    private DriveTrain _driveTrain;
    private Pose2d destnation;
    
    public Maverick(OutliersContainer container, DriveTrain driveTrain){
        super(container);
        _driveTrain = driveTrain;
    }

    /**
     * Get the velocity at an angle
     * @param vx x velocity
     * @param vy y velocity
     * @return velocity
     */
    public double getVelocityTheta(double vx, double vy){
        //The fraction would look like vx/vy
        return Math.atan(vy/vx);
    }

    /**
     * Check to see if the set point is inside the field
     * @param x1 top conner x
     * @param y1 top conner y
     * @param x2 bottom conner x
     * @param y2 bottom conner y
     * @param x set point x
     * @param y set point y
     * @return is in the field perimeter
     */
    public boolean getCheckPoints(int x1, int y1, int x2, int y2, int x, int y){
        if(x > x2 && x < x2 && y > y1 && y > y2){
            //Inside of the rectangle
            System.out.println("Maverick: " + true);
            //блин!!
            return true;
        }
        else{
            //Not inside of the rectangle
            System.out.println("Maverick: " + false);
            return false;
        }
    }
    
    public void wayPointMove(){
        //Iterate through all of the waypoints
        metric("MAVERICK", "Running");
        for(int i = 0; i < Constants.Maverick.numberOfWaypoints; i++){
            metric("MAVERICK", "At waypoint: " + i);
            //Create translations and rotations based off of the Maverick presets
            Translation2d move = new Translation2d(Constants.Maverick.waypointsX[i], Constants.Maverick.waypointsY[i]);
            Rotation2d rotation = new Rotation2d(Constants.Maverick.rotations[i]);
            destnation = new Pose2d(move, rotation);
            //Update the speeds with the realivent Maverick speed
            //Move the robot
            _driveTrain.poseFollower(destnation, Constants.Maverick.speeds[i]);
        }
        metric("MAVERICK", "Move(s) Complete");
    }

    /**
     * Check to see if the robot is at the set pose
     * @return true if at pose / false if at pose
     */
    public boolean isAtPose(){
        return _driveTrain.MaverickDone(destnation);
    }


    @Override
    public void updateDashboard() {

    }
}
