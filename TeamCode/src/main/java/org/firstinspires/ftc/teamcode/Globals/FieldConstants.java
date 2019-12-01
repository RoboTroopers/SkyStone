package org.firstinspires.ftc.teamcode.Globals;


public class FieldConstants {

    //All linear measurements use inches

    public static final double TILE_LENGTH = 24;
    public static final double TILES_PER_SIDE = 6;

    public static final double FIELD_LENGTH = TILE_LENGTH*TILES_PER_SIDE;


    public double tileToInches(double tile){
        return tile*TILE_LENGTH;
    }


    public static final double STONE_WIDTH = 8.0;
    public static final double STONE_DEPTH = 4;


    public enum AllianceColors { RED, BLUE }


    // Specific key field positions
    public static final double BRIDGE_X = FIELD_LENGTH/2;

    public static final double BLUE_SKYSTONE_Y = FIELD_LENGTH-(TILE_LENGTH*2);
    public static final double RED_SKYSTONE_Y = TILE_LENGTH*2;


}
