package org.firstinspires.ftc.teamcode.Utilities;

public class StonePosChecker {


    /** Returns the ordinal number of the stone the camera is in front of based on encoder positions */
    public static int getFirstStoneNumRed(int encoderPos) {
        int stoneNum = 0;
        if (encoderPos < 1400) { // Add some to this to give room for error.
            stoneNum = 1;
        } else if (encoderPos < 1600) {
            stoneNum = 2;
        } else {
            stoneNum = 3;
        }
        return stoneNum;
    }


    /** Returns the ordinal number of the stone the camera is in front of based on encoder positions */
    public static int getFirstStoneNumBlue(int encoderPos) {
        int stoneNum = 0;
        if (encoderPos < 100) {
            stoneNum = 1;
        } else if (encoderPos < 500) {
            stoneNum = 2;
        } else if (encoderPos < 1000) {
            stoneNum = 3;
        }
        return stoneNum;
    }



}
