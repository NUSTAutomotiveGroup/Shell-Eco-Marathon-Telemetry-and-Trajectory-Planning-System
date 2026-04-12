/* This file has code for a simple steering control that is also used as an example
   for VS commands. Comments in this file apply to this specific example.
   
   Revision log:   
   Jul 10, 2015. M. Sayers. Added VS_EXT_AFTER_READ in external_calc.
   Sep 27, 14. M. Sayers. Use newer API functions introduced in CarSim 9.0.
   Apr 22, 10. M. Sayers. Added VS_EXT_EQ_SAVE in external_calc.
*/

// header files for standard C libraries 
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <windows.h>

// VehicleSim header file and prototypes for these function 
#include "vs_deftypes.h" // VS types and definitions
#include "vs_api.h"  // VS API functions
#include "external.h" // prototypes for the functions in this file

// define some static variables used for the controller
static vs_real sXprev, sYprev, // X and Y coordinates of a preview point
               sLfwd,          // length forward for preview (m)
               sLPrevErr,      // lateral error of preview point (m)
               sLatTrack,      // Lateral offset for tracking target
               sGainStr,       // parameter for a control gain
               *sXcg, *sYcg, *sYaw, // pointers to vehicle  X, Y, and yaw
               *sImpStr, // pointer to imported steering wheel angle 
               *sTstart; // pointer to start time
static int sUseExternal; // option to use external model.               

/* ---------------------------------------------------------------------------------
   Set up variables for the model extension. For the steering controller, define new
   units and parameters and set default values of the parameters that will be used
   if nothing is specified at run time.
---------------------------------------------------------------------------------- */
void external_setdef (void) {
 
  // set default values for parameters defined in this file
  sUseExternal = 0;

  // Make sure deg/m units are defined
  vs_define_units ("deg/m", 180.0/PI);
  
  // setup header for Echo file
  vs_add_echo_header ("EXTERNAL STEERING MODEL (FROM C)");

  // define new parameters
  vs_define_par ("OPT_USE_EXT_STEER", &sUseExternal, 1, NULL, 1, 0, // integer
                 "Use steering model from external C code?" );
  vs_define_par ("L_FORWARD", &sLfwd, 20.0, "M", 1, 0,
                 "Distance preview point is forward of vehicle CG" );
  vs_define_par ("LAT_TRACK", &sLatTrack, -1.6, "M", 1, 0,
                 "Lateral offset (to driver's left) for target");
  vs_define_par ("GAIN_STEER_CTRL", &sGainStr, 10.0, "DEG/M", 1, 0,
                 "Control parameter: steering angle per meter of lateral offset");
  
  // define new outputs using variables that live in this file
  vs_define_out ("Xpreview", "X coordinate of preview point", &sXprev, "M",
    "X coordinate", "Steer control look point", "External steer control");
  vs_define_out ("Ypreview", "Y coordinate of preview point", &sYprev, "M",
    "X coordinate", "Steer control look point", "External steer control");
  vs_define_out ("LPrevErr", "Lateral error of preview point",  &sLPrevErr, "M",
    "Lateral error", "Steer control look point", "External steer control");

  // get pointers to vehicle X, Y, yaw, and imported steering wheel angle that will
  // be used in calculation made during the run
  sXcg    = vs_get_var_ptr("XCG_TM");
  sYcg    = vs_get_var_ptr("YCG_TM");
  sYaw    = vs_get_var_ptr("YAW");
  sImpStr = vs_get_var_ptr("IMP_STEER_SW");
  sTstart = vs_get_var_ptr("TSTART");
}

/* ---------------------------------------------------------------------------------
   Perform calculations involving the model extensions. This function is called from
   nine places as defined in vs_deftypes.h.
---------------------------------------------------------------------------------- */
void  external_calc (vs_real t, vs_ext_loc where) {
  switch (where) {
    case VS_EXT_AFTER_READ: // after having read the Parsfile inputs
      if (sUseExternal) // activate IMPORT variable
        vs_statement ("IMPORT", "IMP_STEER_SW vs_replace", 1); 
      break;

    case VS_EXT_EQ_IN: // calculations at the start of a time step
      if (!sUseExternal) ; // no effect if sUseExternal is FALSE
      else if (t <= *sTstart) *sImpStr = 0.0; // no steering at the start
      else { // steer proportional to the lateral error
        sLPrevErr = vs_road_l(sXprev, sYprev);
        *sImpStr = sGainStr*(sLatTrack - sLPrevErr);
      }
      break;

    case VS_EXT_EQ_OUT: // calculate output variables at the end of a time step
      // calculate X and Y coordinates of preview point
      sXprev = *sXcg + sLfwd*cos(*sYaw);
      sYprev = *sYcg + sLfwd*sin(*sYaw);
      break;

    /* unused locations for this example: 
       VS_EXT_EQ_PRE_INIT, VS_EXT_EQ_INIT, VS_EXT_EQ_INIT2,
       VS_EXT_EQ_SAVE, VS_EXT_EQ_FULL_STEP, VS_EXT_EQ_END */
  }
}

/* ---------------------------------------------------------------------------------
   Write information into the current output echo file using the VS API function
   vs_write_to_echo_file. This function is called four times when generating the 
   echo file as indicated with the argument where, which can have the values:
   VS_EXT_ECHO_TOP, VS_EXT_ECHO_SYPARS, VS_EXT_ECHO_PARS, and VS_EXT_ECHO_END
   (defined in vs_deftypes.h).
---------------------------------------------------------------------------------- */
void external_echo (vs_ext_loc where) {
  switch (where) {
    case VS_EXT_ECHO_TOP: // top of echo file
      vs_write_to_echo_file ("\n"
        "! This model was extended with custom C code to provide a point-follower\n"
        "! steer controller. To disable it, set OPT_USE_EXT_STEER = 0\n");
      break;
      
    case VS_EXT_ECHO_SYPARS: // end of system parameter section
      break;
      
    case VS_EXT_ECHO_PARS: // end of model parameter section
      break;
    
    case VS_EXT_ECHO_END: // end of echo file
      break;
  }
}

/* ---------------------------------------------------------------------------------
   Scan a line read from the current input parsfile. Return TRUE if the keyword
   is recognized, FALSE if not.

   keyword -> string with current ALL CAPS keyword to be tested
   buffer  -> string with rest of line from parsfile.
--------------------------------------------------------------------------------- */
vs_bool external_scan (char *keyword, char *buffer) {
  // place for code to look at keyword and buffer
  return FALSE;
}
