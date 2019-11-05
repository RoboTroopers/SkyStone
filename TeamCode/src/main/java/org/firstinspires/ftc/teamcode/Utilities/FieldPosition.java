package org.firstinspires.ftc.teamcode.Utilities;

import static org.firstinspires.ftc.teamcode.Utilities.FieldConstants.TILE_LENGTH;

public class FieldPosition {
    
    public double xTile;
    public double yTile;
    
    public double addXInches;
    public double addYInches;
    
    public double fieldXInches;
    public double fieldYInches;
    
    public double getFieldXInches(){
        return fieldXInches;
    }
    
    public double getFieldYInches(){
        return fieldYInches;
    }
    
    
    public FieldPosition(double xTile, double yTile, double addXInches, double addYInches) {
        
        this.xTile = xTile;
        this.yTile = yTile;
        
        this.addXInches = addXInches;
        this.addXInches = addXInches;

        this.fieldXInches = (xTile*TILE_LENGTH)+(addXInches);
        this.fieldYInches = (yTile*TILE_LENGTH)+(addYInches);
        
    }
    
    
}
