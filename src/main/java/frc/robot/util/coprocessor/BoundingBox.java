package frc.robot.util.coprocessor;

public class BoundingBox {
    double upperRightX = 0.0;
    double upperRightY = 0.0;
    double lowerLeftX = 0.0;
    double lowerLeftY = 0.0;
    double minBoundaryInflate = 0.0;
    double maxBoundaryInflate = 0.0;
    double maxSpeed = 0.0;

    public BoundingBox(double upperRightX, double upperRightY, double lowerLeftX, double lowerLeftY,
            double minBoundaryInflate, double maxBoundaryInflate, double maxSpeed) {
        this.upperRightX = upperRightX;
        this.upperRightY = upperRightY;
        this.lowerLeftX = lowerLeftX;
        this.lowerLeftY = lowerLeftY;
        this.minBoundaryInflate = minBoundaryInflate;
        this.maxBoundaryInflate = maxBoundaryInflate;
        this.maxSpeed = maxSpeed;
    }

    public boolean isObstacleWithinBounds(PointObstacle obstacle, double speed) {
        double inflate = (maxBoundaryInflate - minBoundaryInflate) / maxSpeed * speed + minBoundaryInflate;
        double infUpperRightX = upperRightX + inflate;
        double infUpperRightY = upperRightY + inflate;
        double infLowerLeftX = lowerLeftX - inflate;
        double infLowerLeftY = lowerLeftY - inflate;
        return infLowerLeftX <= obstacle.getX() && obstacle.getX() <= infUpperRightX &&
                infLowerLeftY <= obstacle.getY() && obstacle.getY() <= infUpperRightY;
    }
}
