/* Wrapper program that can be launched anywhere in Windows, and will then load
   a VehicleSim solver DLL. This is a simple example that just runs the model
   without adding any extensions.

   Log:
   Dec 30, 15. C. Clark. Added multi-platform support
   Apr 24, 10. M. Sayers. New API function: vs_get_api
   May 18, 09. M. Sayers. New for CarSim 8.0.
*/

#include <string.h>

#include "vs_deftypes.h" // VS types and definitions
#include "vs_api.h"      // VS API functions

/* ---------------------------------------------------------------------------------
   Main program to run DLL with VS API. 
--------------------------------------------------------------------------------- */
int main(int argc, char **argv) {
  HMODULE vsDLL = NULL;           // DLL with VS API
  char    pathDLL[FILENAME_MAX], 
          simfile[FILENAME_MAX] = {"simfile.sim"};

  // Get simfile from argument list and load DLL
  if (argc > 1) strcpy(simfile, &argv[1][0]);
  if (vs_get_dll_path(simfile, pathDLL)) return 1;
  vsDLL = vs_load_library(pathDLL);

  // Get API functions
  if (vs_get_api(vsDLL, pathDLL)) return 1;

  // Make the run; vs_run returns 0 if the run is OK.
  if (vs_run(simfile)) {
#if (defined(_WIN32) || defined(_WIN64))
    MessageBox(NULL, vs_get_error_message(), NULL, MB_ICONERROR);
#else
    printf("%s\n", vs_get_error_message());
#endif
  }

  vs_free_library(vsDLL);
  
  return 0;
}
