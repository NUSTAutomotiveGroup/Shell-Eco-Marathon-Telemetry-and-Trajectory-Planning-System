/* Wrapper program that can be launched anywhere in Windows, and will then load
   a VehicleSim solver DLL. This example does not use vs_run, but instead gives
   access to the integration loop. It applies some VS Commands after the regular
   input Parsfile has been read.

   Log:
   July 27, 2015. M. Sayers. Use vs_statement function to apply VS Commands.
*/

#include <windows.h>

#include "vs_deftypes.h" // VS types and definitions
#include "vs_api.h"  // VS API functions

static char *sMsg = NULL; // pointer to support local sPringMsg function

// Print text and reset (set first character to 0)
static void sPrintMsg () {
  if (!*sMsg) return;
  printf (sMsg);
  *sMsg = 0;
}

/* ---------------------------------------------------------------------------------
   Main program to run DLL with VS API. 
--------------------------------------------------------------------------------- */
int main(int argc, char **argv) {
  HMODULE vsDLL = NULL; // DLL with VS API
  double t; // simulation time
  char   pathDLL[FILENAME_MAX], simfile[FILENAME_MAX]={"simfile.sim"};
  int    ibarg = 0; // counter for text bar graph

  // get simfile from argument list and load DLL
  if (argc > 1) strcpy (simfile, &argv[1][0]);
  if (vs_get_dll_path(simfile, pathDLL)) return 1;
  vsDLL = LoadLibrary(pathDLL);

  // get API functions
  if (vs_get_api(vsDLL, pathDLL)) return 1;

  sMsg = vs_get_output_message(); // pointer to text from DLL for sPrintMsg()

  // Read normal inputs
  t = vs_setdef_and_read(simfile, NULL, NULL);
  if (vs_error_occurred ()) return 1;

  // Use VS Commands to add simple steer controller
  vs_statement ("DEFINE_UNITS", "deg/m DR", 1);
  
  vs_statement ("DEFINE_PARAMETER", "L_FORWARD 20; m ; Distance to view point", 1);
  vs_statement ("DEFINE_PARAMETER", "LAT_TRACK -1.6; m "
                "; Distance vehicle is offset from road centerline", 1);
  vs_statement ("DEFINE_PARAMETER", "GAIN_STEER_CTRL 10; deg/m ; Control gain", 1);
  
  vs_statement ("DEFINE_OUTPUT", "Xpreview = XCG_TM + L_FORWARD*cos(YAW); m"
              "; X coordinate of preview point", 1);
  vs_statement ("DEFINE_OUTPUT", "Ypreview = YCG_TM + L_FORWARD*sin(YAW); m"
              "; Y coordinate of preview point", 1);
  vs_statement ("SET_OUTPUT_COMPONENT", "Xpreview Steer control preview point", 1);
  vs_statement ("SET_OUTPUT_COMPONENT", "Ypreview Steer control preview point", 1);
              
  vs_statement ("IMPORT", "IMP_STEER_SW vs_replace", 1); // activate IMPORT variable
  vs_statement ("EQ_IN", "IMP_STEER_SW = if_gt_0_then(t, "
                "(LAT_TRACK - road_l(Xpreview, Ypreview))*GAIN_STEER_CTRL, 0)", 1);

  // initialize
  vs_initialize (t, NULL, NULL);
  sPrintMsg ();
  if (vs_error_occurred ()) return 1;

  // Run. Each loop advances time one step
  while (!vs_stop_run()) {
    vs_integrate (&t, NULL);
    vs_bar_graph_update (&ibarg); // update bar graph?
  }

  // Terminate
  vs_terminate (t, NULL);
  sPrintMsg ();
  
  // pause?
  if (vs_opt_pause()) {
    printf (
      "\n\nThe run ended normally. OPT_PAUSE was set to keep this display visible."
      "\nPress the Return key to exit. ");
    fgetc (stdin);
  }
  
  vs_free_library (vsDLL);
  
  return 0;
}
