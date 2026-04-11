/**
 * VGMTrans (c) - 2002-2021
 * Licensed under the zlib license
 * See the included LICENSE for more information
 */

#include <cstdlib>
#include <iostream>

#include "CLIVGMRoot.h"
#include "Options.h"
#include "VGMExport.h"

using namespace std;
namespace fs = std::filesystem;

int main(int argc, char *argv[]) {
  for(int i = 1; i < argc; ++i) {
    string s(argv[i]);
    if ((s == "-h") || (s == "--help")) {
      cliroot.displayHelp();
      return EXIT_SUCCESS;
    }
    else if (s == "--rmf") {
      if (cliroot.exportMode == CLIExportMode::RMIOnly ||
          cliroot.exportMode == CLIExportMode::ZMFOnly) {
        cerr << "Error: --rmf, --zmf, and --rmi are mutually exclusive" << endl;
        cliroot.displayUsage();
        return EXIT_FAILURE;
      }
      cliroot.exportMode = CLIExportMode::RMFOnly;
      ConversionOptions::the().setForceZmfExport(false);
    }
    else if (s == "--zmf") {
      if (cliroot.exportMode == CLIExportMode::RMIOnly ||
          cliroot.exportMode == CLIExportMode::RMFOnly) {
        cerr << "Error: --rmf, --zmf, and --rmi are mutually exclusive" << endl;
        cliroot.displayUsage();
        return EXIT_FAILURE;
      }
      cliroot.exportMode = CLIExportMode::ZMFOnly;
      ConversionOptions::the().setForceZmfExport(true);
    }
    else if (s == "--rmi") {
      if (cliroot.exportMode == CLIExportMode::RMFOnly ||
          cliroot.exportMode == CLIExportMode::ZMFOnly) {
        cerr << "Error: --rmf, --zmf, and --rmi are mutually exclusive" << endl;
        cliroot.displayUsage();
        return EXIT_FAILURE;
      }
      cliroot.exportMode = CLIExportMode::RMIOnly;
      ConversionOptions::the().setForceZmfExport(false);
    }
    else if (s == "-o") {
      if (i == argc - 1) {
        cerr << "Error: expected output path" << endl;
        cliroot.displayUsage();
        return EXIT_FAILURE;
      }
      else {
        cliroot.outputDir = fs::path(argv[++i]);
        cliroot.outputPathExplicitlySet = true;
      }
    }
    else {
      cliroot.inputFiles.insert(fs::path(s));
    }
  }

  bool success = false;

  if (cliroot.init()) {
    if (cliroot.exportAllCollections()) {
      success = true;
    }
  }

  return success ? EXIT_SUCCESS : EXIT_FAILURE;
}
