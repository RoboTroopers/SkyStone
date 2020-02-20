package org.firstinspires.ftc.teamcode.Utilities.Unused;

import static org.firstinspires.ftc.teamcode.Globals.DriveConstants.inchesToTicks;
import static org.firstinspires.ftc.teamcode.Globals.FieldConstants.TILE_LENGTH;

public class FieldPosition {


    public final double xInches;
    public final double yInches;


    public double getXInches() {
        return xInches;
    }

    public double getYInches() {
        return yInches;
    }


    public final double fieldXTicks;
    public final double fieldYTicks;


    public double getXTicks() {
        return fieldXTicks;
    }

    public double getYTicks() {
        return fieldYTicks;
    }


    public FieldPosition(double xTile, double yTile, double addXInches, double addYInches) {

        this.xInches = (xTile*TILE_LENGTH) + addXInches;
        this.yInches = (yTile*TILE_LENGTH) + addYInches;

        fieldXTicks = inchesToTicks(this.xInches);
        fieldYTicks = inchesToTicks(this.yInches);

    }


    public FieldPosition(double xInches, double yInches) {

        this.xInches = xInches;
        this.yInches = yInches;

        fieldXTicks = inchesToTicks(this.xInches);
        fieldYTicks = inchesToTicks(this.yInches);

    }


    public FieldPosition (FieldPosition fieldPos) {

        this.xInches = fieldPos.xInches;
        this.yInches = fieldPos.yInches;

        fieldXTicks = inchesToTicks(xInches);
        fieldYTicks = inchesToTicks(yInches);

    }



}
