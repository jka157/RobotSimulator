//-----------------------------------------------------------------------------
// ensc-488.h 
// 
// This file provides basic interface to both
//    (a) Powercube Scara robot hardware set up in ENSC-488 Lab, and
//    (b) A graphics simulator of above hardware.
// 
// For overview, the interface include the following functions:
//     - bool DisplayConfiguration(JOINT &conf);
//     - bool MoveToConfiguration(JOINT &conf, bool wait = false);
//     - bool MoveWithConfVelAcc(JOINT &conf, JOINT &vel, JOINT &acc);
//     - bool GetConfiguration(JOINT &conf);
//     - bool GetState(JOINT &state);
//     - bool ResetRobot();
//     - void StopRobot();
// For more detailed descriptions, please read comments for each function.
//
// Revision History:
//    - Oct 10, 2007, 
//      Version 0.1 released by Zhenwang Yao.
//
//-----------------------------------------------------------------------------

#define PI                (3.1415926)
#define DEG2RAD(x)        (x*PI/180.)
#define RAD2DEG(x)        (x*180./PI)
#define MS2SEC(x)         (x/1000.)
#define MM2M(x)           (x/1000.)
#define M2MM(x)           (x*1000.)

#pragma comment( lib, "PowerCubeSim.lib" )

typedef double JOINT[4];

//-----------------------------------------------------------------------------
//bool MoveToConfiguration(JOINT &conf, bool wait = false);
//
//Description:
//      This function moves the robot to the desired configuration.
//
//Input Parameter:
//      conf : Desired configuration
//      wait : Whether to wait until the robot reach the desired configuration.
//             wait = false, function return right away;
//             wait = true,  function will not return until the desired 
//                            configuration is reached.
//
//Ouput Parameter:
//      N/A
//
//Return Value:
//      true,  if success
//      false, otherwise
//
//Validity:
//      Hardware and simulation.
//      
//-----------------------------------------------------------------------------
__declspec( dllimport) bool MoveToConfiguration(JOINT &conf, bool wait = false);

//-----------------------------------------------------------------------------
//bool MoveWithConfVelAcc(JOINT &conf, JOINT &vel, JOINT &acc);
//
//Description:
//      This function moves the robot with the desired profile, namely to 
//         acheive the desired velocity/acceleration at the desired configuration.
//
//Input Parameter:
//      conf: Desired configuration
//      vel : Desired velocity
//      acc : Desired acceleration 
//
//Ouput Parameter:
//      N/A
//
//Return Value:
//      true,  if success
//      false, otherwise
//
//Validity:
//      Hardware and simulation.
//      
//-----------------------------------------------------------------------------
__declspec( dllimport) bool MoveWithConfVelAcc(JOINT &conf, JOINT &vel, JOINT &acc);


//-----------------------------------------------------------------------------
//bool GetConfiguration(JOINT &conf);
//
//Description:
//      This function retrieves current robot configuration.
//
//Input Parameter:
//      N/A
//
//Ouput Parameter:
//      conf : Current robot configuration.
//
//Return Value:
//      true,  if success
//      false, otherwise
//
//Validity:
//      Hardware and simulation.
//      
//-----------------------------------------------------------------------------
__declspec( dllimport) bool GetConfiguration(JOINT &conf);

//-----------------------------------------------------------------------------
//bool GetState(JOINT &state);
//
//Description:
//      This function retrieves current state of the robot.
//
//Input Parameter:
//      N/A
//
//Ouput Parameter:
//      state : Current state for every joint.
//              for detailed description about stateid, please refer to "stateid.h".
//         Note that, the state should be a unsigned long integer for each joint,
//              however for simplicity, double float is used in the argument,
//              so you need to cast double to long before you can find out the ID.
//
//Return Value:
//      true,  if success
//      false, otherwise
//
//Validity:
//      Hardware and simulation.
//      
//-----------------------------------------------------------------------------
__declspec( dllimport) bool GetState(JOINT &state);


//-----------------------------------------------------------------------------
//bool Grasp(bool close);
//
//Description:
//      This function open or close the gripper.
//
//Input Parameter:
//      close:  close = true, close the gripper;
//              close = false, open the gripper
//
//Ouput Parameter:
//      N/A
//
//Return Value:
//      true,  if success
//      false, otherwise
//
//Validity:
//      Hardware and simulation.
//      
//-----------------------------------------------------------------------------
__declspec( dllimport) bool Grasp(bool close);

//-----------------------------------------------------------------------------
//void StopRobot();
//
//Description:
//      This function stops the robot movement.
//
//Input Parameter:
//      N/A
//
//Ouput Parameter:
//      N/A
//
//Return Value:
//      N/A
//
//Validity:
//      Hardware and simulation.
//      
//-----------------------------------------------------------------------------
__declspec( dllimport) void StopRobot();

//-----------------------------------------------------------------------------
//bool ResetRobot();
//
//Description:
//      This function reset the robot, after calling StopRobot(). 
//      After StopRobot() is called, one can not more the robot before 
//      resettting the robot by calling this function.
//
//Input Parameter:
//      N/A
//
//Ouput Parameter:
//      N/A
//
//Return Value:
//      N/A
//
//Validity:
//      Hardware and simulation.
//      
//-----------------------------------------------------------------------------
__declspec( dllimport) bool ResetRobot();

//-----------------------------------------------------------------------------
//bool DisplayConfiguration(JOINT &conf);
//
//Description:
//      This function display the robot with given configuration.
//
//Input Parameter:
//      conf : Given configuration
//
//Ouput Parameter:
//      N/A
//
//Return Value:
//      true,  if success
//      false, otherwise
//
//Validity:
//      Simulation only.
//      
//-----------------------------------------------------------------------------
__declspec( dllimport) bool DisplayConfiguration(JOINT &conf);

//-----------------------------------------------------------------------------
//void OpenMonitor();
//
//Description:
//      This function open the monitor window.
//
//Input Parameter:
//      N/A
//
//Ouput Parameter:
//      N/A
//
//Return Value:
//      N/A
//
//Validity:
//      Hardware and Simulation.
//      
//-----------------------------------------------------------------------------
__declspec( dllimport) void OpenMonitor();

//-----------------------------------------------------------------------------
//void CloseMonitor();
//
//Description:
//      This function close the monitor window.
//
//Input Parameter:
//      N/A
//
//Ouput Parameter:
//      N/A
//
//Return Value:
//      N/A
//
//Validity:
//      Hardware and Simulation.
//      
//-----------------------------------------------------------------------------
__declspec( dllimport) void CloseMonitor();
