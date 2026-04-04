/*
 * VGMTrans (c) 2002-2026
 * Licensed under the zlib license,
 * refer to the included LICENSE.txt file
 */
#pragma once

#include <filesystem>

class VGMColl;

namespace conversion {

bool requiresZmfForRMF(const VGMColl &coll);

bool saveAsRMF(const VGMColl &coll,
			   const std::filesystem::path &filepath,
			   std::filesystem::path *actual_filepath = nullptr);

}