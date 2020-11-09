/** \file     encmain.cpp
    \brief    Encoder application main
*/

#include <time.h>
#include <iostream>
#include "TAppEncTop.h"
#include "TAppCommon/program_options_lite.h"

using namespace std;
namespace po = df::program_options_lite;

//! \ingroup TAppEncoder
//! \{

// ====================================================================================================================
// Main function
// ====================================================================================================================

int main(int argc, char *argv[])
{
    TAppEncTop cTAppEncTop;

    // print information
    fprintf(stdout, "\n");
    fprintf(stdout, "HM software: Encoder Version [%s]", NV_VERSION);
    fprintf(stdout, NVM_ONOS);
    fprintf(stdout, NVM_COMPILEDBY);
    fprintf(stdout, NVM_BITS);
    fprintf(stdout, "\n");

    // create application encoder class
    cTAppEncTop.create();

    // parse configuration
    try
    {
        if (!cTAppEncTop.parseCfg(argc, argv))
        {
            cTAppEncTop.destroy();
            return 1;
        }
    }
    catch (po::ParseFailure &e)
    {
        cerr << "Error parsing option \"" << e.arg << "\" with argument \"" << e.val << "\"." << endl;
        return 1;
    }

    // starting time
    double dResult;
    long lBefore = clock();

    // call encoding function
    cTAppEncTop.encode();

    // ending time
    dResult = (double)(clock() - lBefore) / CLOCKS_PER_SEC;
    printf("\n Total Time: %12.3f sec.\n", dResult);

    // destroy application encoder class
    cTAppEncTop.destroy();

    return 0;
}

//! \}
