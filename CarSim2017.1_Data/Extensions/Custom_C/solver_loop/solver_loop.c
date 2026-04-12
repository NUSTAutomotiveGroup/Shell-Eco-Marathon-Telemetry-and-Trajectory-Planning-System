/* Wrapper program that can be launched anywhere in Windows, and will then load
   a VehicleSim solver DLL. This example does not use vs_run, but instead gives
   access to the integration loop.

   Log:
   July 21, 2015. M. Sayers. Updated to avoid some redundant operations.
   May 05, 2010. M. Sayers. Extended solver_simple to show details of run loop.
*/

#include <string.h>

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
  if (argc > 1) strcpy(simfile, &argv[1][0]);
  if (vs_get_dll_path(simfile, pathDLL)) return 1;
  vsDLL = vs_load_library(pathDLL);

  // get API functions
  if (vs_get_api(vsDLL, pathDLL)) return 1;

  sMsg = vs_get_output_message(); // pointer to text from DLL for sPrintMsg()

  // Read inputs and initialize
  t = vs_setdef_and_read(simfile, NULL, NULL);
  if (vs_error_occurred()) {
    printf("\n\nError occurred reading simfile \"%s\"", simfile);
    return 1;
  }
  
  vs_initialize (t, NULL, NULL);
  sPrintMsg ();

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
    printf ("\nOPT_PAUSE was set to keep this display visible."
            "\nPress the Return key to exit. ");
    fgetc (stdin);
  }
  
  vs_free_library(vsDLL);
  
  return 0;
}