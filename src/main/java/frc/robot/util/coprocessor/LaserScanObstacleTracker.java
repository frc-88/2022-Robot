// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.coprocessor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.util.Units;

public class LaserScanObstacleTracker {
    private ArrayList<PointObstacle> obstacles = new ArrayList<>();
    private BoundingBox bbox;
    private final double kReverseFanRadians = Units.degreesToRadians(180.0);  // Range of angles to accept as a valid reverse command

    public LaserScanObstacleTracker() {
        
    }

    public void setBoundingBox(BoundingBox bbox) {
        this.bbox = bbox;
    }

    public int size() {
        return obstacles.size();
    }

    public ArrayList<PointObstacle> getPoints() {
        return obstacles;
    }

    public void setPoints(double[] laserXs, double[] laserYs) {
        if (laserXs.length != laserYs.length) {
            System.out.println(String.format(
                "Warning! Laser obstacle coordinate lengths do not match. %d != %d", 
                laserXs.length, laserYs.length
            ));
        }

        int length = Math.min(laserXs.length, laserYs.length);
        while (obstacles.size() < length) {
            obstacles.add(new PointObstacle());
        }

        for (int index = 0; index < length; index++) {
            obstacles.get(index).set(laserXs[index], laserYs[index]);
        }
    }

    public boolean isObstacleWithinBounds(double speed)
    {
        for (PointObstacle obstacle : obstacles) {
            if (bbox.isObstacleWithinBounds(obstacle, speed)) {
                return true;
            }
        }
        return false;
    }

    public boolean isDirectionAllowed(double heading, double speed)
    {
        List<PointObstacle> in_bound_obs = new ArrayList<>(obstacles);
        in_bound_obs.removeIf(obs -> !bbox.isObstacleWithinBounds(obs, speed));
        double angles[] = new double[in_bound_obs.size()];
        for (int index = 0; index < angles.length; index++) {
            PointObstacle obstacle = in_bound_obs.get(index);
            angles[index] = Helpers.boundHalfAngle(Math.atan2(obstacle.getY(), obstacle.getX()) + Math.PI);
        }

        return Helpers.isDirectionAllowed(heading, angles, kReverseFanRadians);
    }

    public static boolean testGetAllowableReverseDirections(ArrayList<PointObstacle> obstacles, VelocityCommand command)
    {
        double minAngle = Double.NaN;
        double maxAngle = Double.NaN;
        double kReverseFanRadians = Units.degreesToRadians(180.0);
        
        for (PointObstacle obstacle : obstacles) {
            double angle = Helpers.boundHalfAngle(Math.atan2(obstacle.getY(), obstacle.getX()) + Math.PI);
            double minObsAngle = Helpers.boundHalfAngle(angle - kReverseFanRadians / 2.0);
            double maxObsAngle = Helpers.boundHalfAngle(angle + kReverseFanRadians / 2.0);
            // System.out.println(String.format("min: %4.1f, max: %4.1f, obs: %4.1f", 
            //     Units.radiansToDegrees(minObsAngle),
            //     Units.radiansToDegrees(maxObsAngle),
            //     Units.radiansToDegrees(angle)
            // ));
            if (Double.isNaN(minAngle) || minObsAngle > minAngle) {
                minAngle = minObsAngle;
            }
            if (Double.isNaN(maxAngle) || maxObsAngle < maxAngle) {
                maxAngle = maxObsAngle;
            }
        }
        
        double heading = Helpers.boundHalfAngle(Math.atan2(command.vy, command.vx));

        System.out.println(String.format("min: %4.1f, max: %4.1f, command: %4.1f\t", 
            Units.radiansToDegrees(minAngle),
            Units.radiansToDegrees(maxAngle),
            Units.radiansToDegrees(heading)));
        if (maxAngle < minAngle) {
            return !(maxAngle < heading && heading < minAngle);
        }
        else {
            return minAngle <= heading && heading <= maxAngle;
        }
    }

    public static void main(String[] args) {
        ArrayList<PointObstacle> obstacles = new ArrayList<>();
        VelocityCommand command;

        obstacles.add(new PointObstacle(1.0, 1.0));
        obstacles.add(new PointObstacle(1.0, 0.0));
        command = new VelocityCommand(-1.0, 0.0, 0.0);

        if (!testGetAllowableReverseDirections(obstacles, command)) { System.out.println("Fail!"); }

        obstacles = new ArrayList<>();
        obstacles.add(new PointObstacle(1.0, 1.0));
        obstacles.add(new PointObstacle(1.0, 0.0));
        command = new VelocityCommand(1.0, 1.0, 0.0);

        if (testGetAllowableReverseDirections(obstacles, command)) { System.out.println("Fail!"); }

        obstacles = new ArrayList<>();
        obstacles.add(new PointObstacle(1.0, 1.0));
        obstacles.add(new PointObstacle(1.0, 0.0));
        command = new VelocityCommand(-1.0, -1.0, 0.0);

        if (!testGetAllowableReverseDirections(obstacles, command)) { System.out.println("Fail!"); }
        
        obstacles = new ArrayList<>();
        obstacles.add(new PointObstacle(1.0, 1.0));
        obstacles.add(new PointObstacle(1.0, 0.0));
        obstacles.add(new PointObstacle(1.0, -1.0));

        System.out.println("\n");
        command = new VelocityCommand(1.0, 1.0, 0.0);
        if (testGetAllowableReverseDirections(obstacles, command)) { System.out.println("Fail!"); }
        command = new VelocityCommand(1.0, 0.0, 0.0);
        if (testGetAllowableReverseDirections(obstacles, command)) { System.out.println("Fail!"); }
        command = new VelocityCommand(1.0, -1.0, 0.0);
        if (testGetAllowableReverseDirections(obstacles, command)) { System.out.println("Fail!"); }
        command = new VelocityCommand(0.0, -1.0, 0.0);
        if (testGetAllowableReverseDirections(obstacles, command)) { System.out.println("Fail!"); }
        command = new VelocityCommand(-1.0, -1.0, 0.0);
        if (!testGetAllowableReverseDirections(obstacles, command)) { System.out.println("Fail!"); }
        command = new VelocityCommand(-1.0, 0.0, 0.0);
        if (!testGetAllowableReverseDirections(obstacles, command)) { System.out.println("Fail!"); }
        command = new VelocityCommand(-1.0, 1.0, 0.0);
        if (!testGetAllowableReverseDirections(obstacles, command)) { System.out.println("Fail!"); }
        command = new VelocityCommand(0.0, 1.0, 0.0);
        if (testGetAllowableReverseDirections(obstacles, command)) { System.out.println("Fail!"); }
    }
}
