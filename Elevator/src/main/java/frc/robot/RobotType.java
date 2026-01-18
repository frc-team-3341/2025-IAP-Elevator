// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Special class that defines the type of robot used (sim, Competition, IAP,
 * etc.)
 */
public enum RobotType {

    //These offsets should be in degrees

    // 2023 IAP Robot
    ROBOT_2023_IAP_SLOTH(new double[] {
        - 62.51, //FL++
        -179.82, //FR
         108.11, //BL
         82.62}, //BR         
        
        new int[][] {{1, 2, 9}, {3, 4, 10}, {7, 8, 12}, {5, 6, 11}},

        new boolean[] {false, true, false, true}
        ),

    // 2024 Competition Robot
    ROBOT_2024_COMPETITION(new double[] {
        - 12.21,   //FL
        -121.29,   //FR
        -133.154,   //BL
        - 40.97}, //BR

        new int[][] {{1, 2, 10}, {3, 4, 11}, {5, 6, 12}, {7, 8, 13}},

        new boolean[] {false, true, false, true}
        ),

    //2025 Competetion Robot        
    ROBOT_2025_COMPETITION(new double[] {
        - 57.30,  //FL
           3.60,  //FR
        - 83.51,  //BL
         153.13},  //BR
         
         new int[][] {{1, 2, 9}, {3, 4, 10}, {7, 8, 12}, {5, 6, 11}},

         new boolean[] {false, false, true, true}
         ),

    // Simulation robot
    ROBOT_SIMULATION(new double[] {0, 0, 0, 0},
        new int[][] {{1, 2, 9}, {3, 4, 10}, {7, 8, 12}, {5, 6, 11}}, //ts doesnt matter cuz its not used for sim lOl

        new boolean[] {false, false, true, true} //same w this 
        
    );

    /**
     * Angular offsets of CANCoders
     */
    public double[] moduleAngleOffsets;
    public int[][] ids;
    public boolean[] moduleInverts;


    private RobotType(double[] offsets, int[][] ids, boolean[] moduleInverts) {
        this.moduleAngleOffsets = offsets;
        this.ids = ids;
        this.moduleInverts = moduleInverts;
    }

}