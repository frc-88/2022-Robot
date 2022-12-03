package frc.robot.util.coprocessor;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Helpers {
    
    /**
     * sets angle between -PI and PI.
     *
     * @param angle current to be changed.
     * @param radians determines if angle is radians or not.
     * @return changed angle.
     */
    public static double boundHalfAngle(double angle) {
        angle %= 2.0 * Math.PI;
        if (angle >= Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        if (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    public static String getTeamColorName(DriverStation.Alliance team_color, boolean invert) {
        String team_name = "";
        if (team_color == Alliance.Red) {
            if (invert) {
                team_name = "blue";
            }
            else {
                team_name = "red";
            }
        }
        else if (team_color == Alliance.Blue) {
            if (invert) {
                team_name = "red";
            }
            else {
                team_name = "blue";
            }
        }
        return team_name;
    }

    public static String getTeamColorName(DriverStation.Alliance team_color) {
        return getTeamColorName(team_color, false);
    }

    public static String parseName(String waypointName) {
        String parsed = waypointName.replaceAll("<team>", getTeamColorName(DriverStation.getAlliance(), false));
        parsed = parsed.replaceAll("<!team>", getTeamColorName(DriverStation.getAlliance(), true));
        return parsed;
    }

    public static boolean isDirectionAllowed(double direction_angle, double[] angles, double reverseFanRadians)
    {
        double minAngle = Double.NaN;
        double maxAngle = Double.NaN;
        for (double angle : angles) {
            angle = Helpers.boundHalfAngle(angle + Math.PI);
            double minObsAngle = Helpers.boundHalfAngle(angle - reverseFanRadians / 2.0);
            double maxObsAngle = Helpers.boundHalfAngle(angle + reverseFanRadians / 2.0);
            if (Double.isNaN(minAngle) || minObsAngle > minAngle) {
                minAngle = minObsAngle;
            }
            if (Double.isNaN(maxAngle) || maxObsAngle < maxAngle) {
                maxAngle = maxObsAngle;
            }
        }

        direction_angle = Helpers.boundHalfAngle(direction_angle);
        if (maxAngle < minAngle) {
            return !(maxAngle < direction_angle && direction_angle < minAngle);
        }
        else {
            return minAngle <= direction_angle && direction_angle <= maxAngle;
        }
    }
}
