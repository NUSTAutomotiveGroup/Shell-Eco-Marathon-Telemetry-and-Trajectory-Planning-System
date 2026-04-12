/* This file has functions that can be used to extend a VS solver. The functions 
   are installed in the VS solver using the API functions as shown in the example
   wrapper program.
   
   Revision log:
   Jul 10, 2015. M. Sayers. Added VS_EXT_AFTER_READ in external_calc.
   Apr 22, 2010. M. Sayers. Added VS_EXT_EQ_SAVE in external_calc.

*/

// header files for standard C libraries 
#include <math.h>
#include <string.h>
#include <windows.h>


// VehicleSim header file and prototypes for these function 
#include "vs_deftypes.h" // VS types and definitions
#include "vs_api.h"      // VS API functions
#include "external.h"    // prototypes for the functions in this file


/* ---------------------------------------------------------------------------------
   Set up variables for the model extension. For the steering controller, define new
   units and parameters and set default values of the parameters that will be used
   if nothing is specified at run time.
---------------------------------------------------------------------------------- */
void external_setdef (void) {
  // Add code here...
}


/* ---------------------------------------------------------------------------------
   Perform calculations involving the model extensions. This function is called from
   nine places as defined in vs_deftypes.h.
---------------------------------------------------------------------------------- */
void  external_calc (vs_real t, vs_ext_loc where) {
  switch (where) {

    // just after real Parsfile has been read
    case VS_EXT_AFTER_READ:
      // Add code here...
      break;
      
    // initialization after reading parsfile but before init
    case VS_EXT_EQ_PRE_INIT:
      // Add code here...
      break;

    case VS_EXT_EQ_INIT: // initialization after built-in init
      // Add code here...
      break;

    case VS_EXT_EQ_INIT2: // initialization after outputs are calculated
      // Add code here...
      break;

    case VS_EXT_EQ_IN: // calculate import variables at the start of a time step
      // Add code here...
      break;

    case VS_EXT_EQ_OUT: // calculate output variables at the end of a time step
       // Add code here...
      break;

    case VS_EXT_EQ_SAVE: // save values for use in next time step
       // Add code here...
      break;

    case VS_EXT_EQ_FULL_STEP: // calculate things only at the end of a full step
       // Add code here...
      break;

    case VS_EXT_EQ_END: // calculations done at end of run
      // Add code here...
      break;
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
      // Add code here...
      break;
      
    case VS_EXT_ECHO_SYPARS: // end of system parameter section
      // Add code here...
      break;
      
    case VS_EXT_ECHO_PARS: // end of model parameter section
      // Add code here...
      break;
    
    case VS_EXT_ECHO_END: // end of echo file
      // Add code here...
      break;
  }
}

/* ---------------------------------------------------------------------------------
   Scan a line read from the current input parsfile. Return TRUE if the keyword is
   recognized, FALSE if not. This is not needed for variables and parameters added
   using VS API funcitons, but might be helpful for some custom code.

   keyword -> string with current ALL CAPS keyword to be tested
   buffer  -> string with rest of line from parsfile.
--------------------------------------------------------------------------------- */
vs_bool external_scan (char *keyword, char *buffer) {
  // Add code here to check the keyword and do something if recognized...
  return FALSE;
}
