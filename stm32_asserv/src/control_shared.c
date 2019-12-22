/**
 * Author : Paul Constant
 * Date : 22/12/29
 * 
 * This file is used in both stm32_asserv and asserv_simu.py.
 * It regroups all functions linked to the command law of the robot.
 * 
 * A shared library is generated from this file and imported in the python 
 * code of the simulation. This way, the stm32 command law can be tested without 
 * the robots.
 * The file is imported like any other file in stm32_asserv.
 *
 * Be careful when editing : 
 * 
 * - If you change a function definition (name, parameters, return value),
 *   update the simulation asserv so that it does not break.
 * 
 * - If you change the name of this file, make the change in the compile.sh script as well.
**/

// Empty main body to generate lib (compiles as a standalone)
int main() {}