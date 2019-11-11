package org.firstinspires.ftc.teamcode.Utilities;

import static org.firstinspires.ftc.teamcode.Globals.DriveConstants.inchesToTicks;
import static org.firstinspires.ftc.teamcode.Globals.FieldConstants.TILE_LENGTH;

public class FieldPosition {
    

    public final double fieldXInches;
    public final double fieldYInches;
    
    
    public double getXInches() {
        return fieldXInches;
    }

    public double getYInches() {
        return fieldYInches;
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
        
        fieldXInches = (xTile*TILE_LENGTH) + addXInches;
        fieldYInches = (yTile*TILE_LENGTH) + addYInches;
        
        fieldXTicks = inchesToTicks(fieldXInches);
        fieldYTicks = inchesToTicks(fieldYInches);  
        
    }
    
    
    public FieldPosition(double xInches, double yInches) {
        
        fieldXInches = xInches;
        fieldYInches = yInches;

        fieldXTicks = inchesToTicks(fieldXInches);
        fieldYTicks = inchesToTicks(fieldYInches);
        
    }
    
    
    
}
