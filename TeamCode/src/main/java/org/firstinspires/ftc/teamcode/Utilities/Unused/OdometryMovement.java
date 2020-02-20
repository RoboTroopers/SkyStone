package org.firstinspires.ftc.teamcode.Utilities.Unused;

class OdometryMovement {



    /*
    public void spedToPosition(double xInches, double yInches, double movementSpeed, double preferredAngle_rad, double turnSpeed) {
        double accuracyRange = inchesToTicks(0.5);
        double rotAccuracyRange = toRadians(2);
        double x = inchesToTicks(xInches);
        double y = inchesToTicks(yInches);
        double distanceToTarget;

        double movement_x;
        double movement_y;
        double movement_turn;

        boolean translationComplete = false;
        boolean rotationComplete = false;

        while (!(translationComplete || rotationComplete)) {
            distanceToTarget = Math.hypot(x -robot.odometry.worldXPosition, y - robot.odometry.worldYPosition);

            double absoluteAngleToTarget = Math.atan2(y - robot.odometry.worldYPosition, x - robot.odometry.worldXPosition);
            double relativeAngleToPoint = MathFunctions.angleWrap(absoluteAngleToTarget - (robot.sensors.getWorldAngleRad() - toRadians(90)));

            double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

            double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

            movement_x = movementXPower * movementSpeed;
            movement_y = movementYPower * movementSpeed;

            double relativeTurnAngle = relativeAngleToPoint - toRadians(180) + preferredAngle_rad;
            movement_turn = Range.clip(relativeTurnAngle / toRadians(30), -1, 1) * turnSpeed;

            translationComplete = (Math.abs(distanceToTarget) < accuracyRange);
            rotationComplete = (Math.abs(distanceToTarget) < rotAccuracyRange);

            if (translationComplete) {
                movement_x = 0;
                movement_y = 0;

            }

            if (rotationComplete) {
                movement_turn = 0;

            }

            robot.driveTrain.applyMovement(movement_x, movement_y, movement_turn);
        }
        robot.driveTrain.brake();
    }


    // Stations the robot in current position
    public void brakePID() {
        double desiredXInches = robot.odometry.getXPosInches();
        double desiredYInches = robot.odometry.getYPosInches();
        double desiredAngle_rad = robot.sensors.getWorldAngleRad();
        robot.driveTrain.brake();
        myGoToPosition(desiredXInches, desiredYInches, 0.1, desiredAngle_rad, 0.1);

    }


    public void turnPID(double desiredRadians, double turnSpeed) {
        double desiredXInches = robot.odometry.getXPosInches();
        double desiredYInches = robot.odometry.getYPosInches();
        myGoToPosition(desiredXInches, desiredYInches, 0.1, desiredRadians, turnSpeed);

    }

    */


}
