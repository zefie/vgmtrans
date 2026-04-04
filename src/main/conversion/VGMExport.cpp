/*
* VGMTrans (c) 2002-2024
 * Licensed under the zlib license,
 * refer to the included LICENSE.txt file
 */
#include <spdlog/fmt/fmt.h>
#include <fstream>
#include <algorithm>

#include "VGMExport.h"
#include "helper.h"
#include "VGMInstrSet.h"
#include "VGMSampColl.h"
#include "VGMSamp.h"
#include "VGMColl.h"
#include "SynthFile.h"

namespace fs = std::filesystem;

namespace conversion {

namespace {

void appendFourCC(std::vector<uint8_t> &buf, const char (&fourcc)[5]) {
  buf.insert(buf.end(), fourcc, fourcc + 4);
}

void appendUint32LE(std::vector<uint8_t> &buf, uint32_t value) {
  buf.push_back(static_cast<uint8_t>(value & 0xffu));
  buf.push_back(static_cast<uint8_t>((value >> 8u) & 0xffu));
  buf.push_back(static_cast<uint8_t>((value >> 16u) & 0xffu));
  buf.push_back(static_cast<uint8_t>((value >> 24u) & 0xffu));
}

void appendBytes(std::vector<uint8_t> &buf, const std::vector<uint8_t> &bytes) {
  buf.insert(buf.end(), bytes.begin(), bytes.end());
}

void padEven(std::vector<uint8_t> &buf) {
  if ((buf.size() & 1u) != 0u) {
    buf.push_back(0);
  }
}

std::vector<uint8_t> buildRmiBuffer(const std::vector<uint8_t> &midi_buffer,
                                    const std::vector<uint8_t> &dls_buffer) {
  std::vector<uint8_t> rmi_buffer;
  rmi_buffer.reserve(20 + midi_buffer.size() + dls_buffer.size());

  appendFourCC(rmi_buffer, "RIFF");
  const size_t riff_size_offset = rmi_buffer.size();
  appendUint32LE(rmi_buffer, 0);
  appendFourCC(rmi_buffer, "RMID");

  appendFourCC(rmi_buffer, "data");
  appendUint32LE(rmi_buffer, static_cast<uint32_t>(midi_buffer.size()));
  appendBytes(rmi_buffer, midi_buffer);
  padEven(rmi_buffer);

  appendBytes(rmi_buffer, dls_buffer);
  padEven(rmi_buffer);

  const uint32_t riff_size = static_cast<uint32_t>(rmi_buffer.size() - 8u);
  rmi_buffer[riff_size_offset + 0] = static_cast<uint8_t>(riff_size & 0xffu);
  rmi_buffer[riff_size_offset + 1] = static_cast<uint8_t>((riff_size >> 8u) & 0xffu);
  rmi_buffer[riff_size_offset + 2] = static_cast<uint8_t>((riff_size >> 16u) & 0xffu);
  rmi_buffer[riff_size_offset + 3] = static_cast<uint8_t>((riff_size >> 24u) & 0xffu);

  return rmi_buffer;
}

} // namespace

bool saveAsDLS(VGMInstrSet &set, const fs::path &filepath) {
  VGMColl* coll = !set.assocColls.empty() ? set.assocColls.front() : nullptr;
  if (!coll && !set.sampColl)
    return false;

  std::vector<VGMInstrSet*> instrsets;
  std::vector<VGMSampColl*> sampcolls;
  if (coll) {
    instrsets = coll->instrSets();
    sampcolls = coll->sampColls();
  } else {
    instrsets.emplace_back(&set);
  }

  DLSFile dlsfile;
  if (createDLSFile(dlsfile, instrsets, sampcolls, coll)) {
    return dlsfile.saveDLSFile(filepath);
  }
  return false;
}

bool saveAsSF2(VGMInstrSet &set, const fs::path &filepath) {
  VGMColl* coll = !set.assocColls.empty() ? set.assocColls.front() : nullptr;
  if (!coll && !set.sampColl)
    return false;

  std::vector<VGMInstrSet*> instrsets;
  std::vector<VGMSampColl*> sampcolls;
  if (coll) {
    instrsets = coll->instrSets();
    sampcolls = coll->sampColls();
  } else {
    instrsets.emplace_back(&set);
  }

  if (auto sf2file = createSF2File(instrsets, sampcolls, coll); sf2file) {
    bool bResult = sf2file->saveSF2File(filepath);
    delete sf2file;
    return bResult;
  }

  return false;
}

bool saveAsSF2(const VGMColl &coll, const fs::path &filepath) {
  if (auto sf2file = createSF2File(coll.instrSets(), coll.sampColls(), &coll); sf2file) {
    bool bResult = sf2file->saveSF2File(filepath);
    delete sf2file;
    return bResult;
  }

  return false;
}

void saveAllAsWav(const VGMSampColl &coll, const fs::path &save_dir) {
  const auto sampCollName = makeSafeFileName(coll.name()).u8string();
  for (auto &sample : coll.samples) {
    std::u8string filename = sampCollName + u8" - " + makeSafeFileName(sample->name()).u8string() + u8".wav";
    sample->saveAsWav(save_dir / filename);
  }
}

bool saveDataToFile(const char* begin, uint32_t length, const fs::path& filepath) {
  std::ofstream out(filepath, std::ios::out | std::ios::binary);

  if (!out) {
    return false;
  }

  try {
    std::copy_n(begin, length, std::ostreambuf_iterator<char>(out));
  } catch (...) {
    return false;
  }

  if (!out.good()) {
    return false;
  }

  out.close();
  return true;
}

bool saveAsRMI(const VGMColl &coll, const fs::path &filepath) {
  if (coll.seq() == nullptr) {
    return false;
  }

  std::unique_ptr<MidiFile> midi(coll.seq()->convertToMidi(&coll));
  if (!midi) {
    return false;
  }

  std::vector<uint8_t> midi_buffer;
  midi->writeMidiToBuffer(midi_buffer);
  if (midi_buffer.empty()) {
    return false;
  }

  DLSFile dlsfile;
  if (!createDLSFile(dlsfile, coll)) {
    return false;
  }

  std::vector<uint8_t> dls_buffer;
  dlsfile.writeDLSToBuffer(dls_buffer);
  if (dls_buffer.empty()) {
    return false;
  }

  const std::vector<uint8_t> rmi_buffer = buildRmiBuffer(midi_buffer, dls_buffer);
  if (rmi_buffer.empty()) {
    return false;
  }

  return saveDataToFile(reinterpret_cast<const char *>(rmi_buffer.data()),
                        static_cast<uint32_t>(rmi_buffer.size()),
                        filepath);
}

bool saveAsOriginal(const RawFile& rawfile, const fs::path& filepath) {
  return saveDataToFile(rawfile.begin(), rawfile.size(), filepath);
}

bool saveAsOriginal(const VGMFile& file, const fs::path& filepath) {
  return saveDataToFile(file.rawFile()->begin() + file.offset(), file.length(), filepath);
}
}  // namespace conversion
