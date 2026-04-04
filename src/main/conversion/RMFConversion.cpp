/*
 * VGMTrans (c) 2002-2026
 * Licensed under the zlib license,
 * refer to the included LICENSE.txt file
 */

#include <chrono>
#include <memory>
#include <string>
#include <set>
#include <map>
#include <system_error>
#include <tuple>
#include <vector>
#include <filesystem>
#include <cmath>

#include "common.h"
#include "GenSnd.h"
#include "LogManager.h"
#include "MidiFile.h"
#include "NeoBAE.h"
#include "Options.h"
#include "RMFConversion.h"
#include "SF2Conversion.h"
#include "SF2File.h"
#include "ScaleConversion.h"
#include "VGMColl.h"
#include "VGMInstrSet.h"
#include "VGMRgn.h"
#include "VGMSamp.h"
#include "VGMSampColl.h"
#include "VGMSeq.h"

extern "C" {
#include "sf2_hsb_converter.h"
}

namespace conversion {

namespace {

struct RegionSampleBinding {
  VGMSamp *sample;
  const VGMRgn *region;
  uint16_t source_collection_bank;
  uint16_t source_document_bank;
  uint8_t source_program;
};

struct ProgramADSRSource {
  std::pair<uint16_t, uint8_t> target;
  const VGMRgn *region;
  bool sample_and_hold;
  uint8_t root_key;
  int16_t split_volume;
};

struct ProgramVariant {
  const VGMRgn *region;
  std::pair<uint16_t, uint8_t> target;
};

struct BindingMaterializationSignature {
  uint64_t sample_identity;
  uint32_t loop_start;
  uint32_t loop_end;
  uint8_t key_low;
  uint8_t key_high;
  uint8_t vel_low;
  uint8_t vel_high;
  uint8_t root_key;
  int16_t split_volume;
  int8_t pan;
  int16_t coarse_tune;
  int16_t fine_tune;
  bool sample_and_hold;
  int32_t sustain_level;
  int32_t attack_us;
  int32_t hold_us;
  int32_t decay_us;
  int32_t sustain_us;
  int32_t release_us;
  bool has_lfo;
  int32_t lfo_period;
  int32_t lfo_level;
  int32_t lfo_delay_us;

  bool operator<(const BindingMaterializationSignature &other) const {
    return std::tie(sample_identity,
                    loop_start,
                    loop_end,
                    key_low,
                    key_high,
                    vel_low,
                    vel_high,
                    root_key,
                    split_volume,
                    pan,
                    coarse_tune,
                    fine_tune,
                    sample_and_hold,
                    sustain_level,
                    attack_us,
                    hold_us,
                    decay_us,
                    sustain_us,
                    release_us,
                    has_lfo,
                    lfo_period,
                    lfo_level,
                    lfo_delay_us) <
           std::tie(other.sample_identity,
                    other.loop_start,
                    other.loop_end,
                    other.key_low,
                    other.key_high,
                    other.vel_low,
                    other.vel_high,
                    other.root_key,
                    other.split_volume,
                    other.pan,
                    other.coarse_tune,
                    other.fine_tune,
                    other.sample_and_hold,
                    other.sustain_level,
                    other.attack_us,
                    other.hold_us,
                    other.decay_us,
                    other.sustain_us,
                    other.release_us,
                    other.has_lfo,
                    other.lfo_period,
                    other.lfo_level,
                    other.lfo_delay_us);
  }

  bool operator==(const BindingMaterializationSignature &other) const {
    return std::tie(sample_identity,
                    loop_start,
                    loop_end,
                    key_low,
                    key_high,
                    vel_low,
                    vel_high,
                    root_key,
                    split_volume,
                    pan,
                    coarse_tune,
                    fine_tune,
                    sample_and_hold,
                    sustain_level,
                    attack_us,
                    hold_us,
                    decay_us,
                    sustain_us,
                    release_us,
                    has_lfo,
                    lfo_period,
                    lfo_level,
                    lfo_delay_us) ==
           std::tie(other.sample_identity,
                    other.loop_start,
                    other.loop_end,
                    other.key_low,
                    other.key_high,
                    other.vel_low,
                    other.vel_high,
                    other.root_key,
                    other.split_volume,
                    other.pan,
                    other.coarse_tune,
                    other.fine_tune,
                    other.sample_and_hold,
                    other.sustain_level,
                    other.attack_us,
                    other.hold_us,
                    other.decay_us,
                    other.sustain_us,
                    other.release_us,
                    other.has_lfo,
                    other.lfo_period,
                    other.lfo_level,
                    other.lfo_delay_us);
  }
};

struct VariantMaterializationSignature {
  BindingMaterializationSignature binding;
  BindingMaterializationSignature representative;
  bool is_default_program;

  bool operator<(const VariantMaterializationSignature &other) const {
    return std::tie(binding, representative, is_default_program) <
           std::tie(other.binding, other.representative, other.is_default_program);
  }

  bool operator==(const VariantMaterializationSignature &other) const {
    return std::tie(binding, representative, is_default_program) ==
           std::tie(other.binding, other.representative, other.is_default_program);
  }
};

using SourceInstrumentRef = std::pair<uint16_t, uint8_t>;
using AuthoredInstrumentKey = std::pair<uint8_t, uint8_t>;

static constexpr uint16_t kEmbeddedTargetBanks[3] = {
    static_cast<uint16_t>(0u << 7),
    static_cast<uint16_t>(1u << 7),
    static_cast<uint16_t>(2u << 7),
};

static constexpr SourceInstrumentRef kSilentPercussionFallbackTarget = {
  static_cast<uint16_t>(2u << 7),
  static_cast<uint8_t>(127),
};

static SourceInstrumentRef makeTargetInstrumentRefFromSlot(uint16_t slot) {
  const uint16_t bank_index = static_cast<uint16_t>(slot / 128u);
  const uint8_t program = static_cast<uint8_t>(slot % 128u);
  return {kEmbeddedTargetBanks[bank_index], program};
}

static uint16_t targetInstrumentSlot(const SourceInstrumentRef &target) {
  uint16_t bank_index = 0;
  if (target.first == kEmbeddedTargetBanks[1]) {
    bank_index = 1;
  }
  else if (target.first == kEmbeddedTargetBanks[2]) {
    bank_index = 2;
  }
  return static_cast<uint16_t>(bank_index * 128u + target.second);
}

static uint32_t targetInstrumentInstID(const SourceInstrumentRef &target) {
  return static_cast<uint32_t>(target.first) * 2u + static_cast<uint32_t>(target.second);
}

static uint16_t convertCollectionBankToDocumentBank(uint16_t source_bank);

struct TempDirGuard {
  std::filesystem::path path;

  ~TempDirGuard() {
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
  }
};

struct BaeRuntimeGuard {
  bool is_setup = false;
  BAEMixer mixer = nullptr;
  BAEBankToken bank_token = nullptr;
  unsigned char *bank_data = nullptr;

  ~BaeRuntimeGuard() {
    if (bank_token != nullptr && mixer != nullptr) {
      BAEMixer_UnloadBank(mixer, bank_token);
    }
    if (bank_data != nullptr) {
      XDisposePtr(reinterpret_cast<XPTR>(bank_data));
    }
    if (mixer != nullptr) {
      BAEMixer_Delete(mixer);
    }
    if (is_setup) {
      BAE_Cleanup();
    }
  }
};

constexpr unsigned char kZbfUseSampleRate = 0x08;
constexpr unsigned char kZbfSampleAndHold = 0x04;
constexpr unsigned char kZbfEnableInterpolate = 0x80;
constexpr unsigned char kZbfUseSoundModifierAsRootKey = 0x08;
constexpr unsigned char kZbfAdvancedInterpolation = 0x80;

static int8_t convertRegionPanToRmfPan(const VGMRgn &region) {
  const double clamped_pan = std::clamp(region.pan, 0.0, 1.0);
  // Beatnik INST panPlacement is documented as -63..63. Use VGMTrans's existing
  // perceptual pan curve so a slight bias remains audible in the opposite side,
  // then map to Beatnik's range. Beatnik playback ends up inverted relative to
  // VGMTrans's 0=left, 1=right convention, so flip the sign here.
  // Keep the effective range intentionally narrower than the full Beatnik span:
  // in practice, full-scale panPlacement sounds closer to a hard pan than the
  // source instrument bias we usually have in VGMTrans region data.
  constexpr double kBeatnikPanScale = 0.45;
  const uint8_t midi_pan = convertLinearPercentPanValToStdMidiVal(clamped_pan);
  const int centered_pan = static_cast<int>(midi_pan) - 64;
  const int beatnik_pan = -static_cast<int>(std::lround(centered_pan * kBeatnikPanScale));
  return static_cast<int8_t>(std::clamp(beatnik_pan, -63, 63));
}

static uint8_t resolveRootKey(const VGMSamp &sample, const VGMRgn &region) {
  int root_key = 60;
  if (region.unityKey != -1) {
    root_key = region.unityKey;
  }
  else if (sample.unityKey != -1) {
    root_key = sample.unityKey;
  }

  // Region coarse tune is a semitone offset from the keymap's nominal pitch.
  // RMF doesn't have a dedicated coarse tune field per split, so fold it into root key.
  root_key -= region.coarseTune;
  root_key = std::clamp(root_key, 0, 127);
  return static_cast<uint8_t>(root_key);
}

static void applyFineTuneToSampleRate(BAESampleInfo &sample_info,
                                      const VGMSamp &sample,
                                      const VGMRgn &region) {
  const int total_fine_cents = static_cast<int>(sample.fineTune) + static_cast<int>(region.fineTune);
  if (total_fine_cents == 0 || sample_info.sampledRate == 0) {
    return;
  }

  const double rate = static_cast<double>(sample_info.sampledRate) / 65536.0;
  if (!(rate > 0.0)) {
    return;
  }

  const double ratio = std::pow(2.0, static_cast<double>(total_fine_cents) / 1200.0);
  const double tuned_rate = std::clamp(rate * ratio, 1.0, 65535.0);
  sample_info.sampledRate = static_cast<BAE_UNSIGNED_FIXED>(std::llround(tuned_rate * 65536.0));
}

static bool computeLoopFrames(const Loop &loop, const VGMSamp &sample, uint32_t &out_start, uint32_t &out_end) {
  if (loop.loopStatus == -1) {
    return false;
  }

  uint64_t loop_length = loop.loopLength;
  if (loop.loopStatus && loop_length == 0) {
    if (sample.dataLength <= loop.loopStart) {
      return false;
    }
    loop_length = static_cast<uint64_t>(sample.dataLength - loop.loopStart);
  }

  auto convert_to_frames = [&sample](uint64_t value, uint8_t measure) -> uint32_t {
    if (measure == LM_BYTES) {
      const double frames = (static_cast<double>(value) * sample.compressionRatio()) /
                            static_cast<double>(sample.bytesPerSample());
      if (frames <= 0.0) {
        return 0;
      }
      // Match VGMSamp::saveAsWav conversion semantics (truncate, not round).
      return static_cast<uint32_t>(frames);
    }
    return static_cast<uint32_t>(value);
  };

  const uint32_t loop_start = convert_to_frames(loop.loopStart, loop.loopStartMeasure);
  const uint32_t loop_len = convert_to_frames(loop_length, loop.loopLengthMeasure);
  if (loop_len == 0) {
    return false;
  }

  const uint32_t loop_end = loop_start + loop_len;
  if (loop_end <= loop_start) {
    return false;
  }

  out_start = loop_start;
  out_end = loop_end;
  return true;
}

static void materializeSampleLoopMetadata(const VGMSamp &sample) {
  if (sample.loop.loopStatus != -1) {
    return;
  }

  auto &mutable_sample = const_cast<VGMSamp &>(sample);
  mutable_sample.toPcm(mutable_sample.signedness(), mutable_sample.endianness(), mutable_sample.bps());

  if (mutable_sample.loop.loopStatus == -1) {
    mutable_sample.setLoopStatus(0);
  }
}

static bool resolveLoopPoints(const VGMSamp &sample,
                              const VGMRgn &region,
                              uint32_t &out_start,
                              uint32_t &out_end) {
  materializeSampleLoopMetadata(sample);

  if (sample.bPSXLoopInfoPrioritizing) {
    if (computeLoopFrames(sample.loop, sample, out_start, out_end)) {
      return true;
    }
    return computeLoopFrames(region.loop, sample, out_start, out_end);
  }

  // Prefer sample loop metadata first: it is usually the canonical decoded loop.
  if (computeLoopFrames(sample.loop, sample, out_start, out_end)) {
    return true;
  }
  return computeLoopFrames(region.loop, sample, out_start, out_end);
}

static bool loadSamplePcm16(const VGMSamp &sample,
                            std::vector<int16_t> &out_pcm,
                            uint32_t &out_frames) {
  std::vector<uint8_t> pcm_bytes = const_cast<VGMSamp &>(sample).toPcm(
      Signedness::Signed,
      Endianness::Little,
      BPS::PCM16);

  const size_t bytes_per_frame = static_cast<size_t>(sample.channels) * sizeof(int16_t);
  if (bytes_per_frame == 0 || pcm_bytes.size() < bytes_per_frame) {
    return false;
  }

  out_frames = static_cast<uint32_t>(pcm_bytes.size() / bytes_per_frame);
  out_pcm.resize(pcm_bytes.size() / sizeof(int16_t));
  std::memcpy(out_pcm.data(), pcm_bytes.data(), pcm_bytes.size());
  return out_frames > 1;
}

static uint64_t scoreLoopBoundary(const std::vector<int16_t> &pcm,
                                  uint8_t channels,
                                  uint32_t loop_start,
                                  uint32_t loop_end) {
  if (channels == 0 || loop_end <= loop_start + 1) {
    return UINT64_MAX;
  }

  uint64_t score = 0;
  const uint32_t last_frame = loop_end - 1;
  const uint32_t prev_last_frame = loop_end - 2;
  const uint32_t next_frame = (loop_start + 1 < loop_end) ? loop_start + 1 : loop_start;

  for (uint8_t ch = 0; ch < channels; ++ch) {
    const auto first = pcm[static_cast<size_t>(loop_start) * channels + ch];
    const auto last = pcm[static_cast<size_t>(last_frame) * channels + ch];
    const auto prev_last = pcm[static_cast<size_t>(prev_last_frame) * channels + ch];
    const auto next = pcm[static_cast<size_t>(next_frame) * channels + ch];

    const uint64_t amp_jump = static_cast<uint64_t>(std::llabs(static_cast<long long>(last) - first));
    const long long slope_left = static_cast<long long>(last) - prev_last;
    const long long slope_right = static_cast<long long>(next) - first;
    const uint64_t slope_jump = static_cast<uint64_t>(std::llabs(slope_left - slope_right));

    score += (amp_jump * 4u) + slope_jump;
  }

  return score;
}

static void refineLoopPointsWithPcm(const VGMSamp &sample,
                                    uint32_t wave_frames,
                                    uint32_t &loop_start,
                                    uint32_t &loop_end,
                                    std::map<VGMSamp *, std::vector<int16_t>> &pcm_cache,
                                    std::map<VGMSamp *, uint32_t> &frame_cache) {
  if (loop_end <= loop_start + 2 || wave_frames <= 2) {
    return;
  }

  auto pcm_it = pcm_cache.find(const_cast<VGMSamp *>(&sample));
  auto frames_it = frame_cache.find(const_cast<VGMSamp *>(&sample));
  if (pcm_it == pcm_cache.end() || frames_it == frame_cache.end()) {
    std::vector<int16_t> pcm;
    uint32_t frames = 0;
    if (!loadSamplePcm16(sample, pcm, frames)) {
      return;
    }
    pcm_it = pcm_cache.emplace(const_cast<VGMSamp *>(&sample), std::move(pcm)).first;
    frames_it = frame_cache.emplace(const_cast<VGMSamp *>(&sample), frames).first;
  }

  const auto &pcm = pcm_it->second;
  const uint32_t pcm_frames = frames_it->second;
  if (pcm_frames < 3) {
    return;
  }

  const uint32_t max_frames = std::min(wave_frames, pcm_frames);
  uint32_t base_start = std::min(loop_start, max_frames - 2);
  uint32_t base_end = std::min(loop_end, max_frames);
  if (base_end <= base_start + 1) {
    return;
  }

  const uint32_t loop_len = base_end - base_start;
  uint32_t best_start = base_start;
  uint32_t best_end = base_end;
  uint64_t best_score = scoreLoopBoundary(pcm, sample.channels, base_start, base_end);

  // Test inclusive->exclusive interpretation mismatch quickly.
  if (base_end > base_start + 2) {
    const uint64_t end_minus_one_score = scoreLoopBoundary(pcm, sample.channels, base_start, base_end - 1);
    if (end_minus_one_score < best_score) {
      best_score = end_minus_one_score;
      best_end = base_end - 1;
    }
  }

  constexpr int kWindow = 32;
  for (int delta = -kWindow; delta <= kWindow; ++delta) {
    const long long shifted_start_ll = static_cast<long long>(base_start) + delta;
    if (shifted_start_ll < 0) {
      continue;
    }

    const uint32_t shifted_start = static_cast<uint32_t>(shifted_start_ll);
    const uint32_t shifted_end = shifted_start + loop_len;
    if (shifted_end > max_frames || shifted_end <= shifted_start + 1) {
      continue;
    }

    const uint64_t score = scoreLoopBoundary(pcm, sample.channels, shifted_start, shifted_end);
    if (score < best_score) {
      best_score = score;
      best_start = shifted_start;
      best_end = shifted_end;
    }
  }

  loop_start = best_start;
  loop_end = best_end;
}

static bool collectRegionSampleBindings(const VGMColl &coll, std::vector<RegionSampleBinding> &bindings) {
  std::vector<VGMSampColl *> final_samp_colls;

  if (!coll.sampColls().empty()) {
    for (auto *samp_coll : coll.sampColls()) {
      final_samp_colls.push_back(samp_coll);
    }
  }
  else {
    for (auto *instr_set : coll.instrSets()) {
      if (instr_set->sampColl != nullptr) {
        final_samp_colls.push_back(instr_set->sampColl);
      }
    }
  }

  if (final_samp_colls.empty()) {
    return true;
  }

  auto resolve_sample_from_collections = [&final_samp_colls](VGMSampColl *preferred_samp_coll,
                                                             size_t requested_sample_num,
                                                             VGMSampColl *&resolved_samp_coll,
                                                             size_t &resolved_sample_num) {
    if (preferred_samp_coll != nullptr && requested_sample_num < preferred_samp_coll->samples.size()) {
      resolved_samp_coll = preferred_samp_coll;
      resolved_sample_num = requested_sample_num;
      return true;
    }

    size_t cumulative_offset = 0;
    for (auto *candidate_samp_coll : final_samp_colls) {
      if (candidate_samp_coll == nullptr) {
        continue;
      }

      const size_t sample_count = candidate_samp_coll->samples.size();
      if (requested_sample_num < cumulative_offset + sample_count) {
        resolved_samp_coll = candidate_samp_coll;
        resolved_sample_num = requested_sample_num - cumulative_offset;
        return true;
      }

      cumulative_offset += sample_count;
    }

    return false;
  };

  for (auto *set : coll.instrSets()) {
    for (auto *instr : set->aInstrs) {
      for (auto *rgn : instr->regions()) {
        VGMSampColl *samp_coll = rgn->sampCollPtr;
        if (samp_coll == nullptr) {
          samp_coll = set->sampColl != nullptr ? set->sampColl : final_samp_colls[0];
        }

        if (samp_coll == nullptr || samp_coll->samples.empty()) {
          continue;
        }

        size_t real_sample_num = 0;
        if (rgn->sampOffset != -1) {
          bool found = false;
          for (uint32_t s = 0; s < samp_coll->samples.size(); ++s) {
            const auto *sample = samp_coll->samples[s];
            if (rgn->sampOffset == sample->offset() ||
                rgn->sampOffset == sample->offset() - samp_coll->offset() - samp_coll->sampDataOffset) {
              if (rgn->sampDataLength != -1 && rgn->sampDataLength != sample->dataLength) {
                continue;
              }
              real_sample_num = s;
              found = true;
              break;
            }
          }

          if (!found) {
            continue;
          }
        }
        else {
          VGMSampColl *resolved_samp_coll = samp_coll;
          size_t resolved_sample_num = 0;
          if (!resolve_sample_from_collections(samp_coll,
                                               static_cast<size_t>(rgn->sampNum),
                                               resolved_samp_coll,
                                               resolved_sample_num)) {
            continue;
          }

          samp_coll = resolved_samp_coll;
          real_sample_num = resolved_sample_num;
        }

        if (real_sample_num >= samp_coll->samples.size()) {
          continue;
        }

        bindings.push_back({
            samp_coll->samples[real_sample_num],
            rgn,
            static_cast<uint16_t>(instr->bank),
            convertCollectionBankToDocumentBank(static_cast<uint16_t>(instr->bank)),
            static_cast<uint8_t>(instr->instrNum & 0x7f)
        });
      }
    }
  }

  return true;
}

static int32_t secondsToMicrosecondsClamped(double seconds) {
  if (!(seconds > 0.0)) {
    return 0;
  }

  constexpr double kMaxMicrosAsDouble = 2147483647.0;
  const double micros = std::round(seconds * 1000000.0);
  if (micros >= kMaxMicrosAsDouble) {
    return 2147483647;
  }
  return static_cast<int32_t>(micros);
}

static void populateLfoDelayAdsr(double delay_seconds, BAERmfEditorADSRInfo &adsr) {
  constexpr int32_t kVolumeRange = 4096;
  const int32_t delay_us = secondsToMicrosecondsClamped(delay_seconds);

  adsr = {};
  if (delay_us > 0) {
    adsr.stageCount = 2;
    adsr.stages[0].level = 0;
    adsr.stages[0].time = delay_us;
    adsr.stages[0].flags = ADSR_LINEAR_RAMP_LONG;
    adsr.stages[1].level = kVolumeRange;
    adsr.stages[1].time = 0;
    adsr.stages[1].flags = ADSR_SUSTAIN_LONG;
  }
  else {
    adsr.stageCount = 1;
    adsr.stages[0].level = kVolumeRange;
    adsr.stages[0].time = 0;
    adsr.stages[0].flags = ADSR_SUSTAIN_LONG;
  }
}

static void populateRegionLfos(const VGMRgn &region, BAERmfEditorInstrumentExtInfo &ext_info) {
  ext_info.lfoCount = 0;
  std::memset(ext_info.lfos, 0, sizeof(ext_info.lfos));

  if (!(region.lfoVibDepthCents() > 0.0) || !(region.lfoVibFreqHz() > 0.0) ||
      ext_info.lfoCount >= BAE_EDITOR_MAX_LFOS) {
    return;
  }

  auto &lfo = ext_info.lfos[ext_info.lfoCount++];
  lfo.destination = FOUR_CHAR('P', 'I', 'T', 'C');
  lfo.period = static_cast<int32_t>(
      std::clamp(std::llround(1000000.0 / region.lfoVibFreqHz()), 10000ll, 10000000ll));
  lfo.waveShape = FOUR_CHAR('S', 'I', 'N', 'E');
  lfo.DC_feed = 0;
  lfo.level = static_cast<int32_t>(
      std::clamp(std::llround(std::abs(region.lfoVibDepthCents()) * 41.0), 1ll, 524288ll));
  populateLfoDelayAdsr(region.lfoVibDelaySeconds(), lfo.adsr);
}

static int32_t resolveSustainLevel(const VGMRgn &region) {
  constexpr int32_t kVolumeRange = 4096;

  if (region.sustain_level < 0.0) {
    return kVolumeRange;
  }

  const double clamped = std::clamp(region.sustain_level, 0.0, 1.0);
  return static_cast<int32_t>(std::round(clamped * static_cast<double>(kVolumeRange)));
}

static double approximateLinearAmpDecaySeconds(double converted_seconds) {
  if (!(converted_seconds > 0.0)) {
    return 0.0;
  }

  double low = 0.0;
  double high = converted_seconds;
  while (linearAmpDecayTimeToLinDBDecayTime(high) < converted_seconds) {
    low = high;
    high *= 2.0;
    if (high >= 60.0) {
      break;
    }
  }

  for (int iteration = 0; iteration < 32; ++iteration) {
    const double mid = (low + high) * 0.5;
    if (linearAmpDecayTimeToLinDBDecayTime(mid) < converted_seconds) {
      low = mid;
    }
    else {
      high = mid;
    }
  }

  return high;
}

static bool usesSnesStyleFadeToZero(const VGMRgn &region) {
  return region.sustain_time > 0.0 && region.sustain_level >= 0.0 && region.sustain_level <= 0.001;
}

static double resolveRmfAttackSeconds(const VGMRgn &region) {
  if (!(region.attack_time > 0.0)) {
    return 0.0;
  }

  return region.attack_time;
}

static double resolveRmfDecaySeconds(const VGMRgn &region) {
  if (!(region.decay_time > 0.0)) {
    return 0.0;
  }

  // SNES ADSR import already stretches time constants to match SF2/DLS dB-based envelopes.
  // RMF's LINE stage is a direct level ramp, so zero-sustain held fades need that stretch undone.
  if (usesSnesStyleFadeToZero(region)) {
    return approximateLinearAmpDecaySeconds(region.decay_time);
  }

  return region.decay_time;
}

static double resolveRmfReleaseSeconds(const VGMRgn &region) {
  if (!(region.release_time > 0.0)) {
    return 0.0;
  }

  if (usesSnesStyleFadeToZero(region)) {
    return approximateLinearAmpDecaySeconds(region.release_time);
  }

  return region.release_time;
}

static int16_t convertDbAttenuationToRmfVolume(double attenuation_db) {
  if (!(attenuation_db > 0.0)) {
    return 100;
  }

  // BAE's integer mixing pipeline (Volume = Volume * miscParameter2 / 100) has a narrower
  // effective dynamic range than a floating-point engine. Use a half-dB exponent (/40 instead
  // of the spec-exact /20) so that heavily attenuated splits remain audible and the scale is
  // consistent with the SF2-to-HSB converter's per-split volumes (which also use /40). Using
  // the exact /20 formula causes single-sample instruments to be attenuated more aggressively
  // than multi-sample keysplit instruments whose per-split volumes come from the SF2 converter.
  const double volume = 100.0 * std::pow(10.0, -attenuation_db / 40.0);
  const int rounded = static_cast<int>(std::lround(volume));
  return static_cast<int16_t>(std::clamp(rounded, 1, 127));
}

static int16_t resolveSplitVolume(const VGMSamp &sample, const VGMRgn &region) {
  const double region_attenuation_db = const_cast<VGMRgn &>(region).attenDb();
  const double total_attenuation_db = std::max(0.0, sample.attenDb() + region_attenuation_db);
  return convertDbAttenuationToRmfVolume(total_attenuation_db);
}

static bool shouldEnableSampleAndHold(const VGMRgn &region, bool has_loop) {
  if (!has_loop) {
    return false;
  }

  if (region.sustain_level < 0.0) {
    return true;
  }

  // Keep loop playback alive during release only for regions that actually sustain.
  return region.sustain_level > 0.01;
}

static bool rangesOverlap(uint8_t low_a, uint8_t high_a, uint8_t low_b, uint8_t high_b) {
  return !(high_a < low_b || high_b < low_a);
}

static BindingMaterializationSignature buildBindingMaterializationSignature(const RegionSampleBinding &binding) {
  uint32_t loop_start = 0;
  uint32_t loop_end = 0;
  const bool has_loop = binding.sample != nullptr && binding.region != nullptr &&
                        resolveLoopPoints(*binding.sample, *binding.region, loop_start, loop_end) &&
                        loop_end > loop_start;

  bool has_lfo = false;
  int32_t lfo_period = 0;
  int32_t lfo_level = 0;
  int32_t lfo_delay_us = 0;
  if (binding.region != nullptr && binding.region->lfoVibDepthCents() > 0.0 && binding.region->lfoVibFreqHz() > 0.0) {
    has_lfo = true;
    lfo_period = static_cast<int32_t>(
        std::clamp(std::llround(1000000.0 / binding.region->lfoVibFreqHz()), 10000ll, 10000000ll));
    lfo_level = static_cast<int32_t>(
        std::clamp(std::llround(std::abs(binding.region->lfoVibDepthCents()) * 41.0), 1ll, 524288ll));
    lfo_delay_us = secondsToMicrosecondsClamped(binding.region->lfoVibDelaySeconds());
  }

  return BindingMaterializationSignature{
      static_cast<uint64_t>(reinterpret_cast<uintptr_t>(binding.sample)),
      has_loop ? loop_start : 0u,
      has_loop ? loop_end : 0u,
      binding.region != nullptr ? binding.region->keyLow : static_cast<uint8_t>(0),
      binding.region != nullptr ? binding.region->keyHigh : static_cast<uint8_t>(0),
      binding.region != nullptr ? binding.region->velLow : static_cast<uint8_t>(0),
      binding.region != nullptr ? binding.region->velHigh : static_cast<uint8_t>(0),
      (binding.sample != nullptr && binding.region != nullptr) ? resolveRootKey(*binding.sample, *binding.region) : static_cast<uint8_t>(0),
      (binding.sample != nullptr && binding.region != nullptr) ? resolveSplitVolume(*binding.sample, *binding.region) : static_cast<int16_t>(0),
      binding.region != nullptr ? convertRegionPanToRmfPan(*binding.region) : static_cast<int8_t>(0),
      binding.region != nullptr ? static_cast<int16_t>(binding.region->coarseTune) : static_cast<int16_t>(0),
      binding.region != nullptr ? static_cast<int16_t>(binding.region->fineTune) : static_cast<int16_t>(0),
      binding.region != nullptr ? shouldEnableSampleAndHold(*binding.region, has_loop) : false,
      binding.region != nullptr ? resolveSustainLevel(*binding.region) : 0,
      binding.region != nullptr ? secondsToMicrosecondsClamped(resolveRmfAttackSeconds(*binding.region)) : 0,
      binding.region != nullptr ? secondsToMicrosecondsClamped(binding.region->hold_time) : 0,
      binding.region != nullptr ? secondsToMicrosecondsClamped(resolveRmfDecaySeconds(*binding.region)) : 0,
      binding.region != nullptr ? secondsToMicrosecondsClamped(binding.region->sustain_time) : 0,
      binding.region != nullptr ? secondsToMicrosecondsClamped(resolveRmfReleaseSeconds(*binding.region)) : 0,
      has_lfo,
      lfo_period,
      lfo_level,
      lfo_delay_us,
  };
}

static bool authoredBindingNeedsSeparateVariant(const RegionSampleBinding &candidate,
                                                const RegionSampleBinding &representative) {
  // Only create separate variants when ADSR or LFO parameters differ
  // All other parameters (sample, pan, key ranges, velocity, tuning) handled per-embedding
  
  if (candidate.region == nullptr || representative.region == nullptr) {
    return false;
  }

  // ADSR comparison
  if (resolveSustainLevel(*candidate.region) != resolveSustainLevel(*representative.region)) {
    return true;
  }
  if (secondsToMicrosecondsClamped(resolveRmfAttackSeconds(*candidate.region)) !=
      secondsToMicrosecondsClamped(resolveRmfAttackSeconds(*representative.region))) {
    return true;
  }
  if (secondsToMicrosecondsClamped(candidate.region->hold_time) !=
      secondsToMicrosecondsClamped(representative.region->hold_time)) {
    return true;
  }
  if (secondsToMicrosecondsClamped(resolveRmfDecaySeconds(*candidate.region)) !=
      secondsToMicrosecondsClamped(resolveRmfDecaySeconds(*representative.region))) {
    return true;
  }
  if (secondsToMicrosecondsClamped(candidate.region->sustain_time) !=
      secondsToMicrosecondsClamped(representative.region->sustain_time)) {
    return true;
  }
  if (secondsToMicrosecondsClamped(resolveRmfReleaseSeconds(*candidate.region)) !=
      secondsToMicrosecondsClamped(resolveRmfReleaseSeconds(*representative.region))) {
    return true;
  }

  // LFO comparison
  const bool candidate_has_lfo = candidate.region->lfoVibDepthCents() > 0.0 && candidate.region->lfoVibFreqHz() > 0.0;
  const bool representative_has_lfo = representative.region->lfoVibDepthCents() > 0.0 && representative.region->lfoVibFreqHz() > 0.0;
  if (candidate_has_lfo != representative_has_lfo) {
    return true;
  }
  if (candidate_has_lfo) {
    const int32_t candidate_period = static_cast<int32_t>(
        std::clamp(std::llround(1000000.0 / candidate.region->lfoVibFreqHz()), 10000ll, 10000000ll));
    const int32_t representative_period = static_cast<int32_t>(
        std::clamp(std::llround(1000000.0 / representative.region->lfoVibFreqHz()), 10000ll, 10000000ll));
    const int32_t candidate_level = static_cast<int32_t>(
        std::clamp(std::llround(std::abs(candidate.region->lfoVibDepthCents()) * 41.0), 1ll, 524288ll));
    const int32_t representative_level = static_cast<int32_t>(
        std::clamp(std::llround(std::abs(representative.region->lfoVibDepthCents()) * 41.0), 1ll, 524288ll));
    if (candidate_period != representative_period ||
        candidate_level != representative_level ||
        secondsToMicrosecondsClamped(candidate.region->lfoVibDelaySeconds()) !=
            secondsToMicrosecondsClamped(representative.region->lfoVibDelaySeconds())) {
      return true;
    }
  }

  return false;
}

static bool preferRegionForProgramADSR(const VGMRgn &candidate, const VGMRgn &current) {
  const int32_t candidate_sustain = resolveSustainLevel(candidate);
  const int32_t current_sustain = resolveSustainLevel(current);

  if (candidate_sustain != current_sustain) {
    return candidate_sustain > current_sustain;
  }

  const double candidate_hold = candidate.hold_time;
  const double current_hold = current.hold_time;
  if (candidate_hold != current_hold) {
    return candidate_hold > current_hold;
  }

  const double candidate_decay = candidate.decay_time;
  const double current_decay = current.decay_time;
  if (candidate_decay != current_decay) {
    return candidate_decay > current_decay;
  }

  return candidate.release_time > current.release_time;
}

static void populateVolumeADSRFromRegion(const VGMRgn &region, BAERmfEditorADSRInfo &adsr) {
  auto add_stage = [&adsr](int32_t level, int32_t time_us, int32_t flags) {
    if (adsr.stageCount >= BAE_EDITOR_MAX_ADSR_STAGES) {
      return;
    }

    auto &stage = adsr.stages[adsr.stageCount++];
    stage.level = level;
    stage.time = time_us;
    stage.flags = flags;
  };

  constexpr int32_t kVolumeRange = 4096;
  int32_t sustain_level = resolveSustainLevel(region);

  adsr.stageCount = 0;

  const int32_t attack_us = secondsToMicrosecondsClamped(resolveRmfAttackSeconds(region));
  const int32_t hold_us = secondsToMicrosecondsClamped(region.hold_time);
  int32_t decay_us = secondsToMicrosecondsClamped(resolveRmfDecaySeconds(region));
  const int32_t sustain_us = secondsToMicrosecondsClamped(region.sustain_time);
  const int32_t release_us = secondsToMicrosecondsClamped(resolveRmfReleaseSeconds(region));

  add_stage(kVolumeRange, attack_us, ADSR_LINEAR_RAMP_LONG);

  if (hold_us > 0) {
    add_stage(kVolumeRange, hold_us, ADSR_LINEAR_RAMP_LONG);
  }

  if (decay_us > 0 || sustain_level != kVolumeRange) {
    add_stage(sustain_level, decay_us, ADSR_LINEAR_RAMP_LONG);
  }

  if (sustain_us > 0 && sustain_level > 0) {
    add_stage(GM_SetSustainDecayLevelInTime(static_cast<uint32_t>(sustain_us)), 0, ADSR_SUSTAIN_LONG);
  }
  else {
    add_stage(sustain_level, 0, ADSR_SUSTAIN_LONG);
  }
  const int32_t clamped_release_us = std::max<int32_t>(1000, release_us);
  add_stage(0, clamped_release_us, ADSR_TERMINATE_LONG);
}

static void applyProgramADSR(BAERmfEditorDocument *document,
                             const std::map<SourceInstrumentRef, ProgramADSRSource> &program_adsr_sources) {
  for (const auto &[target, source] : program_adsr_sources) {
    if (source.region == nullptr) {
      continue;
    }

    const uint32_t inst_id = targetInstrumentInstID(target);

    BAERmfEditorInstrumentExtInfo ext_info{};
    BAEResult get_result = BAERmfEditorDocument_GetInstrumentExtInfo(document, inst_id, &ext_info);
    if (get_result != BAE_NO_ERROR) {
      ext_info = {};
      ext_info.instID = inst_id;
      ext_info.flags1 = kZbfUseSampleRate;
      ext_info.flags2 = 0;
      ext_info.midiRootKey = 60;
      ext_info.miscParameter1 = source.root_key;
      ext_info.miscParameter2 = source.split_volume;
    }

    ext_info.instID = inst_id;
    ext_info.hasExtendedData = static_cast<BAE_BOOL>(1);
    ext_info.hasDefaultMod = static_cast<BAE_BOOL>(1);
    ext_info.midiRootKey = 60;
    ext_info.miscParameter1 = source.root_key;
    ext_info.miscParameter2 = source.split_volume;
    ext_info.panPlacement = static_cast<char>(convertRegionPanToRmfPan(*source.region));
    if (source.sample_and_hold) {
      ext_info.flags1 = static_cast<unsigned char>(ext_info.flags1 | kZbfSampleAndHold);
    }
    else {
      ext_info.flags1 = static_cast<unsigned char>(ext_info.flags1 & ~kZbfSampleAndHold);
    }
    ext_info.flags1 = static_cast<unsigned char>(ext_info.flags1 | kZbfEnableInterpolate);
    ext_info.flags2 = static_cast<unsigned char>(ext_info.flags2 | kZbfUseSoundModifierAsRootKey);
    ext_info.flags2 = static_cast<unsigned char>(ext_info.flags2 & ~kZbfAdvancedInterpolation);
    populateVolumeADSRFromRegion(*source.region, ext_info.volumeADSR);
    populateRegionLfos(*source.region, ext_info);
    BAERmfEditorDocument_SetInstrumentExtInfo(document, inst_id, &ext_info);
  }
}

static void ensureTrackControllerZeroAtTick0(BAERmfEditorDocument *document, unsigned char controller) {
  uint16_t track_count = 0;
  if (BAERmfEditorDocument_GetTrackCount(document, &track_count) != BAE_NO_ERROR) {
    return;
  }

  for (uint16_t track_index = 0; track_index < track_count; ++track_index) {
    uint32_t event_count = 0;
    if (BAERmfEditorDocument_GetTrackCCEventCount(document, track_index, controller, &event_count) !=
        BAE_NO_ERROR) {
      continue;
    }

    bool has_tick0_value = false;
    for (uint32_t event_index = 0; event_index < event_count; ++event_index) {
      uint32_t tick = 0;
      unsigned char value = 0;
      if (BAERmfEditorDocument_GetTrackCCEvent(document,
                                               track_index,
                                               controller,
                                               event_index,
                                               &tick,
                                               &value) != BAE_NO_ERROR) {
        continue;
      }

      if (tick == 0) {
        has_tick0_value = true;
        break;
      }
    }

    if (!has_tick0_value) {
      BAERmfEditorDocument_AddTrackCCEvent(document, track_index, controller, 0, 0);
    }
  }
}

static std::vector<SourceInstrumentRef> collectVariantProgramsForNote(uint8_t note,
                                                                      uint8_t velocity,
                                                                      const std::vector<ProgramVariant> &variants,
                                                                      SourceInstrumentRef fallback_target) {
  std::vector<const ProgramVariant *> exact_matches;
  std::vector<const ProgramVariant *> key_only_matches;

  auto variant_sort = [](const ProgramVariant *lhs, const ProgramVariant *rhs) {
    const int lhs_span = static_cast<int>(lhs->region->keyHigh) - static_cast<int>(lhs->region->keyLow);
    const int rhs_span = static_cast<int>(rhs->region->keyHigh) - static_cast<int>(rhs->region->keyLow);
    if (lhs_span != rhs_span) {
      return lhs_span < rhs_span;
    }
    return preferRegionForProgramADSR(*lhs->region, *rhs->region);
  };

  auto append_unique_target = [](std::vector<SourceInstrumentRef> &targets, SourceInstrumentRef target) {
    for (const auto &existing : targets) {
      if (existing == target) {
        return;
      }
    }
    targets.push_back(target);
  };

  for (const auto &variant : variants) {
    if (variant.region == nullptr) {
      continue;
    }
    if (note < variant.region->keyLow || note > variant.region->keyHigh) {
      continue;
    }

    key_only_matches.push_back(&variant);

    if (velocity < variant.region->velLow || velocity > variant.region->velHigh) {
      continue;
    }

    exact_matches.push_back(&variant);
  }

  std::vector<SourceInstrumentRef> targets;
  if (!exact_matches.empty()) {
    std::sort(exact_matches.begin(), exact_matches.end(), variant_sort);
    for (const ProgramVariant *variant : exact_matches) {
      append_unique_target(targets, variant->target);
    }
    return targets;
  }

  if (!key_only_matches.empty()) {
    std::sort(key_only_matches.begin(), key_only_matches.end(), variant_sort);
    append_unique_target(targets, key_only_matches.front()->target);
    return targets;
  }

  targets.push_back(fallback_target);
  return targets;
}

static uint8_t normalizeSourceBankForAuthoredLookup(uint16_t source_bank) {
  if (source_bank > 128) {
    // Packed GS document banks are (MSB << 7) | LSB. Normalize back to
    // collection-bank semantics so authored lookup preserves bank identity.
    return static_cast<uint8_t>((source_bank >> 7) & 0x7f);
  }

  // Do NOT mask bit 7: bank 128 (GM percussion) must remain distinct from bank 0.
  return static_cast<uint8_t>(source_bank);
}

static uint16_t convertCollectionBankToDocumentBank(uint16_t source_bank) {
  if (auto style = ConversionOptions::the().bankSelectStyle(); style == BankSelectStyle::GS) {
    // Most GS source banks are MSB values (0-127) and must be shifted into
    // BAE's packed (MSB<<7)|LSB representation. Bank 128 is the SF2 percussion
    // bank identifier and is already represented as packed value 128 (MSB=1,
    // LSB=0), so shifting would incorrectly collapse it to 0.
    if (source_bank >= 128) {
      return source_bank;
    }
    return static_cast<uint16_t>((source_bank & 0x7f) << 7);
  }

  return source_bank;
}

static void collectSourceInstrumentReferences(BAERmfEditorDocument *document,
                                              std::set<SourceInstrumentRef> &instruments) {
  uint16_t track_count = 0;
  if (BAERmfEditorDocument_GetTrackCount(document, &track_count) != BAE_NO_ERROR) {
    return;
  }

  for (uint16_t t = 0; t < track_count; ++t) {
    BAERmfEditorTrackInfo track_info{};
    if (BAERmfEditorDocument_GetTrackInfo(document, t, &track_info) == BAE_NO_ERROR) {
      instruments.emplace(track_info.bank, track_info.program);
    }

    uint32_t note_count = 0;
    if (BAERmfEditorDocument_GetNoteCount(document, t, &note_count) != BAE_NO_ERROR) {
      continue;
    }

    for (uint32_t n = 0; n < note_count; ++n) {
      BAERmfEditorNoteInfo note_info{};
      if (BAERmfEditorDocument_GetNoteInfo(document, t, n, &note_info) != BAE_NO_ERROR) {
        continue;
      }
      instruments.emplace(note_info.bank, note_info.program);
    }
  }
}

static void collectBanksForProgram(BAERmfEditorDocument *document,
                                   uint8_t program,
                                   std::set<uint16_t> &banks) {
  std::set<SourceInstrumentRef> instruments;
  collectSourceInstrumentReferences(document, instruments);

  for (const auto &[bank, instrument_program] : instruments) {
    if (instrument_program == program) {
      banks.insert(bank);
    }
  }
}

static bool trackUsesPercussionBankMode(BAERmfEditorDocument *document, uint16_t track_index) {
  auto has_cc_value = [document, track_index](unsigned char controller, unsigned char value) {
    uint32_t event_count = 0;
    if (BAERmfEditorDocument_GetTrackCCEventCount(document, track_index, controller, &event_count) != BAE_NO_ERROR) {
      return false;
    }
    for (uint32_t i = 0; i < event_count; ++i) {
      uint32_t tick = 0;
      unsigned char event_value = 0;
      if (BAERmfEditorDocument_GetTrackCCEvent(document,
                                               track_index,
                                               controller,
                                               i,
                                               &tick,
                                               &event_value) != BAE_NO_ERROR) {
        continue;
      }
      (void)tick;
      if (event_value == value) {
        return true;
      }
    }
    return false;
  };

  // NRPN (MSB=5, LSB=0), Data Entry MSB values:
  // 1 = USE_NON_GM_PERC_BANK, 2 = USE_GM_PERC_BANK.
  if (!has_cc_value(99, 5) || !has_cc_value(98, 0)) {
    return false;
  }
  return has_cc_value(6, 1) || has_cc_value(6, 2);
}

static bool isPercussionAuthoredBank(uint8_t source_bank) {
  return source_bank == static_cast<uint8_t>(127) || source_bank == static_cast<uint8_t>(128);
}

static bool isPercussionDocumentBank(uint16_t bank) {
  const uint8_t normalized_bank = normalizeSourceBankForAuthoredLookup(bank);
  return normalized_bank == static_cast<uint8_t>(127) || normalized_bank == static_cast<uint8_t>(128);
}

static bool embedAuthoredBankPrograms(BAERmfEditorDocument *document, const VGMColl &coll) {
  auto count_collection_instruments = [&coll]() {
    size_t count = 0;
    for (const auto *set : coll.instrSets()) {
      if (set != nullptr) {
        count += set->aInstrs.size();
      }
    }
    return count;
  };

  const size_t instrument_count_before_presynth = count_collection_instruments();
  coll.preSynthFileCreation();
  const bool has_materialized_temporary_instruments =
      count_collection_instruments() > instrument_count_before_presynth;
  const bool strict_authored_mode = has_materialized_temporary_instruments;

  auto post_synth_guard = [&coll]() { coll.postSynthFileCreation(); };
  struct ScopeGuard {
    decltype(post_synth_guard) &fn;
    ~ScopeGuard() { fn(); }
  } guard{post_synth_guard};

  std::vector<RegionSampleBinding> bindings;
  if (!collectRegionSampleBindings(coll, bindings)) {
    return false;
  }
  if (bindings.empty()) {
    return true;
  }

  std::set<AuthoredInstrumentKey> source_instruments;
  std::map<AuthoredInstrumentKey, std::set<uint16_t>> source_document_banks_by_instrument;
  std::map<AuthoredInstrumentKey, std::vector<const RegionSampleBinding *>> bindings_by_authored;
  std::map<AuthoredInstrumentKey, std::map<const VGMRgn *, const RegionSampleBinding *>> binding_by_region_by_authored;
  std::map<uint8_t, std::set<AuthoredInstrumentKey>> authored_keys_by_program;
  for (const auto &binding : bindings) {
    if (binding.sample == nullptr || binding.region == nullptr) {
      continue;
    }

    const AuthoredInstrumentKey instrument_key{
        normalizeSourceBankForAuthoredLookup(binding.source_collection_bank),
        binding.source_program,
    };
    source_instruments.emplace(instrument_key);
    source_document_banks_by_instrument[instrument_key].insert(binding.source_document_bank);
    authored_keys_by_program[binding.source_program].insert(instrument_key);

    auto &authored_bindings = bindings_by_authored[instrument_key];
    bool exists = false;
    for (const auto *existing : authored_bindings) {
      if (existing->region == binding.region) {
        exists = true;
        break;
      }
    }
    if (!exists) {
      authored_bindings.push_back(&binding);
      binding_by_region_by_authored[instrument_key][binding.region] = &binding;
    }
  }
  if (source_instruments.empty()) {
    return true;
  }

  if (strict_authored_mode) {
    L_INFO("RMF strict authored mode enabled: {} bindings across {} authored bank/program keys",
           static_cast<unsigned>(bindings.size()),
           static_cast<unsigned>(source_instruments.size()));
    for (const auto &[authored_key, source_banks] : source_document_banks_by_instrument) {
      std::string banks;
      for (uint16_t bank : source_banks) {
        if (!banks.empty()) {
          banks += ",";
        }
        banks += std::to_string(bank);
      }
      L_INFO("RMF strict source authored key bank={} program={} documentBanks=[{}] bindings={}",
             static_cast<unsigned>(authored_key.first),
             static_cast<unsigned>(authored_key.second),
             banks,
             static_cast<unsigned>(bindings_by_authored[authored_key].size()));
    }
  }

  std::map<uint8_t, AuthoredInstrumentKey> unique_authored_by_program;
  for (const auto &[program, authored_keys] : authored_keys_by_program) {
    if (authored_keys.size() == 1) {
      unique_authored_by_program.emplace(program, *authored_keys.begin());
    }
  }

  std::map<SourceInstrumentRef, std::set<AuthoredInstrumentKey>> authored_keys_by_document_bank_program;
  for (const auto &[authored_key, source_banks] : source_document_banks_by_instrument) {
    for (uint16_t source_bank : source_banks) {
      authored_keys_by_document_bank_program[{source_bank, authored_key.second}].insert(authored_key);
    }
  }

  std::map<SourceInstrumentRef, AuthoredInstrumentKey> unique_authored_by_document_bank_program;
  for (const auto &[source_ref, authored_keys] : authored_keys_by_document_bank_program) {
    if (authored_keys.size() == 1) {
      unique_authored_by_document_bank_program.emplace(source_ref, *authored_keys.begin());
    }
  }

  if (!strict_authored_mode) {
    for (const auto &[program, authored_key] : unique_authored_by_program) {
      std::set<uint16_t> actual_document_banks;
      collectBanksForProgram(document, program, actual_document_banks);
      auto &known_banks = source_document_banks_by_instrument[authored_key];
      known_banks.insert(actual_document_banks.begin(), actual_document_banks.end());
    }
  }

  for (const auto &[authored_key, source_banks] : source_document_banks_by_instrument) {
    for (uint16_t source_bank : source_banks) {
      authored_keys_by_document_bank_program[{source_bank, authored_key.second}].insert(authored_key);
    }
  }
  unique_authored_by_document_bank_program.clear();
  for (const auto &[source_ref, authored_keys] : authored_keys_by_document_bank_program) {
    if (authored_keys.size() == 1) {
      unique_authored_by_document_bank_program.emplace(source_ref, *authored_keys.begin());
    }
  }

  std::set<uint8_t> required_programs;
  std::set<uint16_t> reserved_source_slots;
  for (const auto &[source_bank, source_program] : source_instruments) {
    required_programs.insert(source_program);
    const uint16_t packed_bank = convertCollectionBankToDocumentBank(source_bank);
    if (packed_bank == kEmbeddedTargetBanks[0] ||
        packed_bank == kEmbeddedTargetBanks[1] ||
        packed_bank == kEmbeddedTargetBanks[2]) {
      reserved_source_slots.insert(static_cast<uint16_t>((packed_bank >> 7) * 128u + source_program));
    }
  }

  auto allocate_target_program = [&reserved_source_slots](const std::set<uint16_t> &used_target_slots) {
    for (uint16_t candidate = 0; candidate < 384; ++candidate) {
      if (used_target_slots.find(candidate) != used_target_slots.end()) {
        continue;
      }
      if (reserved_source_slots.find(candidate) != reserved_source_slots.end()) {
        continue;
      }
      return candidate;
    }

    for (uint16_t candidate = 0; candidate < 384; ++candidate) {
      if (used_target_slots.find(candidate) == used_target_slots.end()) {
        return candidate;
      }
    }

    return static_cast<uint16_t>(0xffff);
  };

  auto allocate_percussion_target_program = [&reserved_source_slots](const std::set<uint16_t> &used_target_slots) {
    // Prefer embedded bank 2 (0x2:*), then bank 1, then bank 0.
    // Keeping percussion away from 0x0:* avoids accidental GM/melodic collisions.
    const std::array<std::pair<uint16_t, uint16_t>, 3> ranges = {{
        {256u, 384u},
        {128u, 256u},
        {0u, 128u},
    }};

    for (const auto &[start, end] : ranges) {
      for (uint16_t candidate = start; candidate < end; ++candidate) {
        if (used_target_slots.find(candidate) != used_target_slots.end()) {
          continue;
        }
        if (reserved_source_slots.find(candidate) != reserved_source_slots.end()) {
          continue;
        }
        return candidate;
      }
    }

    for (const auto &[start, end] : ranges) {
      for (uint16_t candidate = start; candidate < end; ++candidate) {
        if (used_target_slots.find(candidate) == used_target_slots.end()) {
          return candidate;
        }
      }
    }

    return static_cast<uint16_t>(0xffff);
  };

  auto allocate_percussion_default_target_program = [&reserved_source_slots,
                                                     &allocate_percussion_target_program](const std::set<uint16_t> &used_target_slots) {
    // First choice is slot 256: bank=2, program=0 (target instID = 512).
    constexpr uint16_t kPreferredPercussionDefaultSlot = 256u;
    if (used_target_slots.find(kPreferredPercussionDefaultSlot) == used_target_slots.end() &&
        reserved_source_slots.find(kPreferredPercussionDefaultSlot) == reserved_source_slots.end()) {
      return kPreferredPercussionDefaultSlot;
    }

    return allocate_percussion_target_program(used_target_slots);
  };

  struct PreliminaryVariantAssignment {
    const RegionSampleBinding *binding;
    const RegionSampleBinding *representative_binding;
    BindingMaterializationSignature binding_signature;
    BindingMaterializationSignature representative_signature;
    size_t group_index;
    bool is_default_program;
  };

  auto build_preliminary_variant_assignments = [&](const std::vector<const RegionSampleBinding *> &authored_bindings,
                                                   const RegionSampleBinding *default_binding) {
    constexpr size_t kNoGroup = static_cast<size_t>(-1);

    std::vector<PreliminaryVariantAssignment> assignments;
    std::vector<const RegionSampleBinding *> representative_bindings;
    std::map<BindingMaterializationSignature, size_t> group_index_by_exact_signature;
    size_t default_group_index = kNoGroup;

    for (const auto *binding : authored_bindings) {
      if (binding == nullptr || binding->region == nullptr) {
        continue;
      }

      const BindingMaterializationSignature binding_signature =
          buildBindingMaterializationSignature(*binding);
      size_t group_index = kNoGroup;

      const auto exact_group_it = group_index_by_exact_signature.find(binding_signature);
      if (exact_group_it != group_index_by_exact_signature.end()) {
        group_index = exact_group_it->second;
      }
      else {
        for (size_t i = 0; i < representative_bindings.size(); ++i) {
          if (!authoredBindingNeedsSeparateVariant(*binding, *representative_bindings[i])) {
            group_index = i;
            break;
          }
        }
      }

      if (group_index == kNoGroup) {
        group_index = representative_bindings.size();
        representative_bindings.push_back(binding);
      }

      group_index_by_exact_signature.emplace(binding_signature, group_index);
      if (binding == default_binding) {
        default_group_index = group_index;
      }

      assignments.push_back(PreliminaryVariantAssignment{
          binding,
          representative_bindings[group_index],
          binding_signature,
          buildBindingMaterializationSignature(*representative_bindings[group_index]),
          group_index,
          false,
      });
    }

    for (auto &assignment : assignments) {
      assignment.representative_binding = representative_bindings[assignment.group_index];
      assignment.representative_signature = buildBindingMaterializationSignature(*assignment.representative_binding);
      assignment.is_default_program = (default_group_index != kNoGroup && assignment.group_index == default_group_index);
    }

    return assignments;
  };

  std::set<uint16_t> used_target_slots;
  std::map<AuthoredInstrumentKey, std::vector<ProgramVariant>> variants_by_authored;
  std::map<AuthoredInstrumentKey, SourceInstrumentRef> default_variant_program_by_authored;
  std::map<AuthoredInstrumentKey, std::map<SourceInstrumentRef, const RegionSampleBinding *>> representative_binding_by_program_by_authored;
  std::map<uint8_t, std::map<std::vector<VariantMaterializationSignature>, AuthoredInstrumentKey>>
      canonical_authored_by_pre_signature_by_bank;
  std::map<AuthoredInstrumentKey, std::vector<std::pair<VariantMaterializationSignature, SourceInstrumentRef>>> assigned_entries_by_authored;
  for (const auto &[authored_key, authored_bindings] : bindings_by_authored) {
    const RegionSampleBinding *default_binding = authored_bindings.empty() ? nullptr : authored_bindings.front();
    for (const auto *binding : authored_bindings) {
      if (default_binding == nullptr ||
          (binding != nullptr && binding->region != nullptr && default_binding->region != nullptr &&
           preferRegionForProgramADSR(*binding->region, *default_binding->region))) {
        default_binding = binding;
      }
    }

    const auto preliminary_assignments = build_preliminary_variant_assignments(authored_bindings, default_binding);
    std::vector<VariantMaterializationSignature> preliminary_signature;
    preliminary_signature.reserve(preliminary_assignments.size());
    for (const auto &assignment : preliminary_assignments) {
      preliminary_signature.push_back(
          VariantMaterializationSignature{
              assignment.binding_signature,
              assignment.representative_signature,
              assignment.is_default_program,
          });
    }
    std::sort(preliminary_signature.begin(), preliminary_signature.end());

    auto &variants = variants_by_authored[authored_key];
    std::map<SourceInstrumentRef, const RegionSampleBinding *> representative_binding_by_program;
    auto &canonical_pre_for_bank = canonical_authored_by_pre_signature_by_bank[authored_key.first];
    const auto canonical_pre_it = canonical_pre_for_bank.find(preliminary_signature);
    if (!strict_authored_mode && canonical_pre_it != canonical_pre_for_bank.end()) {
      const auto canonical_entries_it = assigned_entries_by_authored.find(canonical_pre_it->second);
      if (canonical_entries_it != assigned_entries_by_authored.end() &&
          canonical_entries_it->second.size() == preliminary_assignments.size()) {
        std::vector<std::pair<VariantMaterializationSignature, size_t>> sorted_preliminary_indices;
        sorted_preliminary_indices.reserve(preliminary_assignments.size());
        for (size_t i = 0; i < preliminary_assignments.size(); ++i) {
          sorted_preliminary_indices.emplace_back(
              VariantMaterializationSignature{
                  preliminary_assignments[i].binding_signature,
                  preliminary_assignments[i].representative_signature,
                  preliminary_assignments[i].is_default_program,
              },
              i);
        }
        std::sort(sorted_preliminary_indices.begin(), sorted_preliminary_indices.end(), [](const auto &lhs, const auto &rhs) {
          return lhs.first < rhs.first;
        });

        bool all_assignments_matched = true;
        std::vector<SourceInstrumentRef> assigned_programs(
          preliminary_assignments.size(),
          SourceInstrumentRef{kEmbeddedTargetBanks[0], authored_key.second});
        for (size_t i = 0; i < sorted_preliminary_indices.size(); ++i) {
          if (!(sorted_preliminary_indices[i].first == canonical_entries_it->second[i].first)) {
            all_assignments_matched = false;
            break;
          }
          assigned_programs[sorted_preliminary_indices[i].second] = canonical_entries_it->second[i].second;
        }

        if (all_assignments_matched) {
          for (size_t i = 0; i < preliminary_assignments.size(); ++i) {
            const auto &assignment = preliminary_assignments[i];
            const SourceInstrumentRef target = assigned_programs[i];
            variants.push_back(ProgramVariant{assignment.binding->region, target});
            representative_binding_by_program.emplace(target, assignment.representative_binding);
            if (assignment.is_default_program) {
              default_variant_program_by_authored[authored_key] = target;
            }
          }

          representative_binding_by_program_by_authored[authored_key] = representative_binding_by_program;
          assigned_entries_by_authored[authored_key] = canonical_entries_it->second;
          if (default_variant_program_by_authored.find(authored_key) == default_variant_program_by_authored.end()) {
            default_variant_program_by_authored[authored_key] = default_variant_program_by_authored[canonical_pre_it->second];
          }
          continue;
        }
      }
    }

    std::map<size_t, SourceInstrumentRef> target_program_by_group_index;
    SourceInstrumentRef forced_bank128_program{kEmbeddedTargetBanks[0], 0};
    bool has_forced_bank128_program = false;
    if (isPercussionAuthoredBank(authored_key.first)) {
      const uint16_t forced_slot = allocate_percussion_default_target_program(used_target_slots);
      if (forced_slot == 0xffff) {
        L_ERROR("Ran out of embedded program slots while preparing percussion-bank RMF keysplit variants");
        return false;
      }
      forced_bank128_program = makeTargetInstrumentRefFromSlot(forced_slot);
      has_forced_bank128_program = true;
      used_target_slots.insert(forced_slot);
      L_INFO("RMF percussion authored bank {} program {} assigned dedicated target bank {} program {}",
             static_cast<unsigned>(authored_key.first),
             static_cast<unsigned>(authored_key.second),
             static_cast<unsigned>(forced_bank128_program.first),
             static_cast<unsigned>(forced_bank128_program.second));
    }
    std::vector<std::pair<VariantMaterializationSignature, SourceInstrumentRef>> assigned_entries;
    assigned_entries.reserve(preliminary_assignments.size());
    for (const auto &assignment : preliminary_assignments) {
      auto target_program_it = target_program_by_group_index.find(assignment.group_index);
      if (target_program_it == target_program_by_group_index.end()) {
        SourceInstrumentRef target_program{kEmbeddedTargetBanks[0], authored_key.second};
        if (isPercussionAuthoredBank(authored_key.first)) {
          if (assignment.is_default_program && has_forced_bank128_program) {
            target_program = forced_bank128_program;
          }
          else {
            const uint16_t allocated_slot = allocate_percussion_target_program(used_target_slots);
            if (allocated_slot == 0xffff) {
              L_ERROR("Ran out of embedded program slots while preparing percussion-bank RMF variants");
              return false;
            }
            target_program = makeTargetInstrumentRefFromSlot(allocated_slot);
            used_target_slots.insert(allocated_slot);
          }
        }
        else if (strict_authored_mode ||
                 !assignment.is_default_program ||
                 used_target_slots.find(targetInstrumentSlot(target_program)) != used_target_slots.end()) {
          const uint16_t allocated_slot = allocate_target_program(used_target_slots);
          if (allocated_slot == 0xffff) {
            L_ERROR("Ran out of embedded program slots while preparing authored RMF variants");
            return false;
          }
          target_program = makeTargetInstrumentRefFromSlot(allocated_slot);
          used_target_slots.insert(allocated_slot);
        }
        representative_binding_by_program.emplace(target_program, assignment.representative_binding);
        target_program_it = target_program_by_group_index.emplace(assignment.group_index, target_program).first;
        if (strict_authored_mode) {
          L_INFO("RMF strict allocate authored bank={} program={} group={} default={} -> target bank={} program={}",
                 static_cast<unsigned>(authored_key.first),
                 static_cast<unsigned>(authored_key.second),
                 static_cast<unsigned>(assignment.group_index),
                 assignment.is_default_program ? 1u : 0u,
                 static_cast<unsigned>(target_program.first),
                 static_cast<unsigned>(target_program.second));
        }
      }

      variants.push_back(ProgramVariant{assignment.binding->region, target_program_it->second});
      assigned_entries.emplace_back(
          VariantMaterializationSignature{
              assignment.binding_signature,
              assignment.representative_signature,
              assignment.is_default_program,
          },
          target_program_it->second);
      if (assignment.is_default_program) {
        default_variant_program_by_authored[authored_key] = target_program_it->second;
      }
    }

    representative_binding_by_program_by_authored[authored_key] = representative_binding_by_program;
    std::sort(assigned_entries.begin(), assigned_entries.end(), [](const auto &lhs, const auto &rhs) {
      return lhs.first < rhs.first;
    });
    assigned_entries_by_authored[authored_key] = assigned_entries;
    if (!strict_authored_mode) {
      canonical_pre_for_bank.emplace(preliminary_signature, authored_key);
    }

    if (default_variant_program_by_authored.find(authored_key) == default_variant_program_by_authored.end()) {
      default_variant_program_by_authored[authored_key] =
          (isPercussionAuthoredBank(authored_key.first) && has_forced_bank128_program)
              ? forced_bank128_program
              : SourceInstrumentRef{kEmbeddedTargetBanks[2], authored_key.second};
    }
  }

  std::map<AuthoredInstrumentKey, AuthoredInstrumentKey> canonical_authored_by_authored;
  std::map<AuthoredInstrumentKey, std::vector<std::pair<VariantMaterializationSignature, SourceInstrumentRef>>> materialization_entries_by_authored;

  auto build_materialization_entries = [&](const AuthoredInstrumentKey &authored_key) {
    std::vector<std::pair<VariantMaterializationSignature, SourceInstrumentRef>> entries;
    const auto bindings_it = binding_by_region_by_authored.find(authored_key);
    const auto reps_it = representative_binding_by_program_by_authored.find(authored_key);
    const auto variants_it = variants_by_authored.find(authored_key);
    if (bindings_it == binding_by_region_by_authored.end() ||
        reps_it == representative_binding_by_program_by_authored.end() ||
        variants_it == variants_by_authored.end()) {
      return entries;
    }

    const SourceInstrumentRef default_program = default_variant_program_by_authored[authored_key];
    for (const auto &variant : variants_it->second) {
      const auto binding_it = bindings_it->second.find(variant.region);
      const auto representative_it = reps_it->second.find(variant.target);
      if (binding_it == bindings_it->second.end() || representative_it == reps_it->second.end() ||
          binding_it->second == nullptr || representative_it->second == nullptr) {
        continue;
      }

      entries.emplace_back(
          VariantMaterializationSignature{
              buildBindingMaterializationSignature(*binding_it->second),
              buildBindingMaterializationSignature(*representative_it->second),
                variant.target == default_program,
          },
              variant.target);
    }

    std::sort(entries.begin(), entries.end(), [](const auto &lhs, const auto &rhs) {
      return lhs.first < rhs.first;
    });
    return entries;
  };

  std::map<uint8_t, std::map<std::vector<VariantMaterializationSignature>, AuthoredInstrumentKey>>
      canonical_authored_by_signature_by_bank;
  for (const auto &[authored_key, variants] : variants_by_authored) {
    auto entries = build_materialization_entries(authored_key);
    materialization_entries_by_authored.emplace(authored_key, entries);

    if (strict_authored_mode) {
      canonical_authored_by_authored.emplace(authored_key, authored_key);
      continue;
    }

    std::vector<VariantMaterializationSignature> signature;
    signature.reserve(entries.size());
    for (const auto &[entry_signature, target_program] : entries) {
      (void) target_program;
      signature.push_back(entry_signature);
    }

    auto &canonical_for_bank = canonical_authored_by_signature_by_bank[authored_key.first];
    const auto canonical_it = canonical_for_bank.find(signature);
    if (canonical_it == canonical_for_bank.end()) {
      canonical_for_bank.emplace(signature, authored_key);
      canonical_authored_by_authored.emplace(authored_key, authored_key);
      continue;
    }

    const AuthoredInstrumentKey canonical_authored_key = canonical_it->second;
    canonical_authored_by_authored.emplace(authored_key, canonical_authored_key);

    auto &duplicate_variants = variants_by_authored[authored_key];
    auto &duplicate_entries = materialization_entries_by_authored[authored_key];
    const auto canonical_entries_it = materialization_entries_by_authored.find(canonical_authored_key);
    if (canonical_entries_it != materialization_entries_by_authored.end() &&
        canonical_entries_it->second.size() == duplicate_entries.size()) {
      std::vector<std::pair<VariantMaterializationSignature, size_t>> duplicate_variant_indices;
      duplicate_variant_indices.reserve(duplicate_variants.size());
      for (size_t i = 0; i < duplicate_variants.size(); ++i) {
        const auto binding_it = binding_by_region_by_authored[authored_key].find(duplicate_variants[i].region);
        const auto representative_it =
          representative_binding_by_program_by_authored[authored_key].find(duplicate_variants[i].target);
        if (binding_it == binding_by_region_by_authored[authored_key].end() ||
            representative_it == representative_binding_by_program_by_authored[authored_key].end() ||
            binding_it->second == nullptr || representative_it->second == nullptr) {
          continue;
        }

        duplicate_variant_indices.emplace_back(
            VariantMaterializationSignature{
                buildBindingMaterializationSignature(*binding_it->second),
                buildBindingMaterializationSignature(*representative_it->second),
                duplicate_variants[i].target == default_variant_program_by_authored[authored_key],
            },
            i);
      }

      std::sort(duplicate_variant_indices.begin(), duplicate_variant_indices.end(), [](const auto &lhs, const auto &rhs) {
        return lhs.first < rhs.first;
      });

      const size_t reassignment_count = std::min(duplicate_variant_indices.size(), canonical_entries_it->second.size());
      for (size_t i = 0; i < reassignment_count; ++i) {
        if (!(duplicate_variant_indices[i].first == canonical_entries_it->second[i].first)) {
          continue;
        }

        duplicate_entries[i].second = canonical_entries_it->second[i].second;
        duplicate_variants[duplicate_variant_indices[i].second].target = canonical_entries_it->second[i].second;
      }
    }

    default_variant_program_by_authored[authored_key] = default_variant_program_by_authored[canonical_authored_key];
  }

  auto resolve_authored_key_for_reference = [&](uint16_t source_bank,
                                                uint8_t program,
                                                AuthoredInstrumentKey &out_authored_key) {
    const AuthoredInstrumentKey direct_key{normalizeSourceBankForAuthoredLookup(source_bank), program};
    if (variants_by_authored.find(direct_key) != variants_by_authored.end()) {
      out_authored_key = direct_key;
      if (strict_authored_mode) {
        L_INFO("RMF strict resolve direct source bank={} program={} -> authored bank={} program={}",
               static_cast<unsigned>(source_bank),
               static_cast<unsigned>(program),
               static_cast<unsigned>(out_authored_key.first),
               static_cast<unsigned>(out_authored_key.second));
      }
      return true;
    }

    const auto by_document_bank = unique_authored_by_document_bank_program.find({source_bank, program});
    if (by_document_bank != unique_authored_by_document_bank_program.end()) {
      out_authored_key = by_document_bank->second;
      if (strict_authored_mode) {
        L_INFO("RMF strict resolve by-document source bank={} program={} -> authored bank={} program={}",
               static_cast<unsigned>(source_bank),
               static_cast<unsigned>(program),
               static_cast<unsigned>(out_authored_key.first),
               static_cast<unsigned>(out_authored_key.second));
      }
      return true;
    }

    // Akao PS1 drum-bank quirk (notably FF9): the sequence can reference drum kits as
    // bank=(127-n), program=127, while authored instruments are bank=127, program=(127-n).
    // If the direct mapping fails, try this swapped lookup before broader fallbacks.
    if (program == 127) {
      const uint8_t normalized_bank = normalizeSourceBankForAuthoredLookup(source_bank);
      if (normalized_bank != static_cast<uint8_t>(127) && normalized_bank != static_cast<uint8_t>(128)) {
        const AuthoredInstrumentKey swapped_akao_drum_key{static_cast<uint8_t>(127), normalized_bank};
        if (variants_by_authored.find(swapped_akao_drum_key) != variants_by_authored.end()) {
          out_authored_key = swapped_akao_drum_key;
          if (strict_authored_mode) {
            L_INFO("RMF strict resolve Akao drum swap source bank={} program={} -> authored bank={} program={}",
                   static_cast<unsigned>(source_bank),
                   static_cast<unsigned>(program),
                   static_cast<unsigned>(out_authored_key.first),
                   static_cast<unsigned>(out_authored_key.second));
          }
          return true;
        }
      }
    }

    if (!strict_authored_mode) {
      const auto unique_authored_it = unique_authored_by_program.find(program);
      if (unique_authored_it != unique_authored_by_program.end()) {
        out_authored_key = unique_authored_it->second;
        return true;
      }
    }

    if (strict_authored_mode) {
      L_WARN("RMF strict resolve failed for source bank={} program={}",
             static_cast<unsigned>(source_bank),
             static_cast<unsigned>(program));
    }

    return false;
  };

  std::map<AuthoredInstrumentKey, std::set<SourceInstrumentRef>> used_variant_programs_by_authored;

  {
    uint16_t track_count = 0;
    if (BAERmfEditorDocument_GetTrackCount(document, &track_count) == BAE_NO_ERROR) {
      std::vector<bool> track_uses_percussion_bank_mode(track_count, false);
      for (uint16_t t = 0; t < track_count; ++t) {
        track_uses_percussion_bank_mode[t] = trackUsesPercussionBankMode(document, t);
      }

      for (uint16_t t = 0; t < track_count; ++t) {
        BAERmfEditorTrackInfo track_info{};
        const bool has_track_info =
            BAERmfEditorDocument_GetTrackInfo(document, t, &track_info) == BAE_NO_ERROR;

        if (has_track_info) {
          AuthoredInstrumentKey track_authored_key{0, 0};
          if (track_uses_percussion_bank_mode[t] && track_info.bank == 0) {
            const AuthoredInstrumentKey percussion_key_128{static_cast<uint8_t>(128), track_info.program};
            const AuthoredInstrumentKey percussion_key_127{static_cast<uint8_t>(127), track_info.program};
            if (variants_by_authored.find(percussion_key_128) != variants_by_authored.end()) {
              track_authored_key = percussion_key_128;
            }
            else if (variants_by_authored.find(percussion_key_127) != variants_by_authored.end()) {
              track_authored_key = percussion_key_127;
            }
            else {
              resolve_authored_key_for_reference(track_info.bank, track_info.program, track_authored_key);
            }
          }
          else {
            resolve_authored_key_for_reference(track_info.bank, track_info.program, track_authored_key);
          }

          if (variants_by_authored.find(track_authored_key) != variants_by_authored.end()) {
            const auto canonical_authored_it = canonical_authored_by_authored.find(track_authored_key);
            const AuthoredInstrumentKey materialized_track_authored_key =
                canonical_authored_it != canonical_authored_by_authored.end() ? canonical_authored_it->second : track_authored_key;
            used_variant_programs_by_authored[materialized_track_authored_key].insert(
                default_variant_program_by_authored[track_authored_key]);
          }
        }

        uint32_t note_count = 0;
        if (BAERmfEditorDocument_GetNoteCount(document, t, &note_count) != BAE_NO_ERROR) {
          continue;
        }

        for (uint32_t n = 0; n < note_count; ++n) {
          BAERmfEditorNoteInfo note_info{};
          AuthoredInstrumentKey authored_key{0, 0};
          std::map<AuthoredInstrumentKey, std::vector<ProgramVariant>>::const_iterator variants_it;
          SourceInstrumentRef default_program;
          bool has_program_variants = false;

          if (BAERmfEditorDocument_GetNoteInfo(document, t, n, &note_info) != BAE_NO_ERROR) {
            continue;
          }
          if (track_uses_percussion_bank_mode[t] && note_info.bank == 0) {
            const AuthoredInstrumentKey percussion_key_128{static_cast<uint8_t>(128), note_info.program};
            const AuthoredInstrumentKey percussion_key_127{static_cast<uint8_t>(127), note_info.program};
            if (variants_by_authored.find(percussion_key_128) != variants_by_authored.end()) {
              authored_key = percussion_key_128;
            }
            else if (variants_by_authored.find(percussion_key_127) != variants_by_authored.end()) {
              authored_key = percussion_key_127;
            }
            else if (has_track_info &&
                     resolve_authored_key_for_reference(track_info.bank, note_info.program, authored_key)) {
              // Prefer track bank/program context when percussion mode normalizes note bank to 0.
            }
            else if (has_track_info &&
                     resolve_authored_key_for_reference(track_info.bank, track_info.program, authored_key)) {
              // Fallback to the track's current program if note info does not preserve it.
            }
            else if (!resolve_authored_key_for_reference(note_info.bank, note_info.program, authored_key)) {
              continue;
            }
          }
          else if (!resolve_authored_key_for_reference(note_info.bank, note_info.program, authored_key)) {
            continue;
          }

          const auto canonical_authored_it = canonical_authored_by_authored.find(authored_key);
          const AuthoredInstrumentKey materialized_authored_key =
              canonical_authored_it != canonical_authored_by_authored.end() ? canonical_authored_it->second : authored_key;

          variants_it = variants_by_authored.find(authored_key);
          if (variants_it == variants_by_authored.end()) {
            continue;
          }

          default_program = default_variant_program_by_authored[authored_key];
          used_variant_programs_by_authored[materialized_authored_key].insert(default_program);
          for (const auto &variant : variants_it->second) {
            if (variant.target != default_program) {
              has_program_variants = true;
              break;
            }
          }
          if (!has_program_variants) {
            continue;
          }

          for (const auto &target : collectVariantProgramsForNote(note_info.note,
                                                                  note_info.velocity,
                                                                  variants_it->second,
                                                                  default_program)) {
            used_variant_programs_by_authored[materialized_authored_key].insert(target);
          }
        }
      }
    }
  }

  auto sf2file = std::unique_ptr<SF2File>(createSF2File(coll.instrSets(), coll.sampColls(), nullptr));
  if (!sf2file) {
    L_ERROR("Failed to create temporary SF2 for RMF bank authoring");
    return false;
  }
  const std::vector<uint8_t> sf2_buffer = sf2file->saveToMem();
  if (sf2_buffer.empty()) {
    L_ERROR("Failed to serialize temporary SF2 for RMF bank authoring");
    return false;
  }

  BaeRuntimeGuard runtime;
  if (BAE_Setup() != BAE_NO_ERROR) {
    L_ERROR("BAE_Setup failed while preparing RMF bank authoring");
    return false;
  }
  runtime.is_setup = true;

  runtime.mixer = BAEMixer_New();
  if (runtime.mixer == nullptr) {
    L_ERROR("BAEMixer_New failed while preparing RMF bank authoring");
    return false;
  }

  SF2HSBConvertOptions convert_options{};
  convert_options.forceHsb = 1;
  SF2HSBConvertReport convert_report{};
  char error_buffer[512]{};
  uint32_t bank_size = 0;
  BAEResult convert_result = SF2HSB_ConvertBankMemory(runtime.mixer,
                                                      sf2_buffer.data(),
                                                      sf2_buffer.size(),
                                                      &convert_options,
                                                      &convert_report,
                                                      error_buffer,
                                                      sizeof(error_buffer),
                                                      &runtime.bank_data,
                                                      &bank_size);
  if (convert_result != BAE_NO_ERROR) {
    L_ERROR("Failed to author in-memory Beatnik bank for RMF export (BAE error #{}): {}",
            static_cast<int>(convert_result),
            error_buffer[0] != '\0' ? error_buffer : "unknown error");
    return false;
  }

  BAEResult bank_result = BAEMixer_AddBankFromMemory(runtime.mixer,
                                                     runtime.bank_data,
                                                     bank_size,
                                                     &runtime.bank_token);
  if (bank_result != BAE_NO_ERROR) {
    L_ERROR("Failed to load in-memory Beatnik bank for RMF export (BAE error #{})",
            static_cast<int>(bank_result));
    return false;
  }

  uint32_t instrument_count = 0;
  if (BAERmfEditorBank_GetInstrumentCount(runtime.bank_token, &instrument_count) != BAE_NO_ERROR ||
      instrument_count == 0) {
    L_ERROR("Temporary Beatnik bank did not expose any instruments for RMF export");
    return false;
  }

  std::map<AuthoredInstrumentKey, uint32_t> instrument_index_by_bank_program;
  std::map<uint8_t, uint32_t> fallback_index_by_program;
  for (uint32_t instrument_index = 0; instrument_index < instrument_count; ++instrument_index) {
    BAERmfEditorBankInstrumentInfo instrument_info{};
    if (BAERmfEditorBank_GetInstrumentInfo(runtime.bank_token, instrument_index, &instrument_info) != BAE_NO_ERROR) {
      continue;
    }

    if (required_programs.find(instrument_info.program) != required_programs.end()) {
      auto fallback = fallback_index_by_program.find(instrument_info.program);
      if (fallback == fallback_index_by_program.end() || instrument_info.bank == 0) {
        fallback_index_by_program[instrument_info.program] = instrument_index;
      }
    }

    const AuthoredInstrumentKey instrument_key{
      normalizeSourceBankForAuthoredLookup(instrument_info.bank),
      instrument_info.program,
    };
    if (source_instruments.find(instrument_key) != source_instruments.end() &&
        instrument_index_by_bank_program.find(instrument_key) == instrument_index_by_bank_program.end()) {
      instrument_index_by_bank_program[instrument_key] = instrument_index;
    }
  }

  std::map<AuthoredInstrumentKey, uint32_t> instrument_index_by_authored;
  for (const auto &[source_bank, source_program] : source_instruments) {
    const AuthoredInstrumentKey authored_key{source_bank, source_program};

    auto exact = instrument_index_by_bank_program.find(authored_key);
    if (exact != instrument_index_by_bank_program.end()) {
      instrument_index_by_authored[authored_key] = exact->second;
      if (strict_authored_mode) {
        L_INFO("RMF strict temp-bank exact match authored bank={} program={} -> instrumentIndex={}",
               static_cast<unsigned>(authored_key.first),
               static_cast<unsigned>(authored_key.second),
               static_cast<unsigned>(exact->second));
      }
      continue;
    }

    if (isPercussionAuthoredBank(authored_key.first)) {
      // SF2 percussion presets may be expanded as per-note instruments in the temporary
      // bank; do not fall back to a melodic program-only match.
      continue;
    }

    const auto fallback = fallback_index_by_program.find(source_program);
    if (fallback == fallback_index_by_program.end() || strict_authored_mode) {
      L_ERROR("Temporary Beatnik bank is missing bank {} program {} required for RMF export",
              static_cast<unsigned>(authored_key.first),
              static_cast<unsigned>(source_program));
      return false;
    }

    L_WARN("Temporary Beatnik bank is missing bank {} program {}; falling back to program-only match for RMF export",
           static_cast<unsigned>(authored_key.first),
           static_cast<unsigned>(source_program));
    instrument_index_by_authored[authored_key] = fallback->second;
  }

  std::map<SourceInstrumentRef, ProgramADSRSource> program_adsr_sources;
  std::map<AuthoredInstrumentKey, std::vector<uint32_t>> base_sample_indices_by_authored;
  const auto temp_dir = std::filesystem::temp_directory_path() /
      ("vgmtrans-rmf-authored-" +
       std::to_string(static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count())));
  std::error_code temp_ec;
  std::filesystem::create_directories(temp_dir, temp_ec);
  if (temp_ec) {
    L_ERROR("Failed to create temporary directory for RMF authored sample staging: {}", temp_dir.string());
    return false;
  }
  TempDirGuard temp_guard{temp_dir};
  std::map<VGMSamp *, uint32_t> canonical_asset_id_by_sample;
  std::map<VGMSamp *, uint32_t> canonical_sample_index_by_sample;
  std::map<VGMSamp *, std::vector<int16_t>> pcm_cache;
  std::map<VGMSamp *, uint32_t> pcm_frame_cache;
  size_t staged_sample_counter = 0;

  for (const auto &[authored_key, variants] : variants_by_authored) {
    const auto canonical_authored_it = canonical_authored_by_authored.find(authored_key);
    if (canonical_authored_it != canonical_authored_by_authored.end() && canonical_authored_it->second != authored_key) {
      continue;
    }

    const AuthoredInstrumentKey materialized_authored_key =
        canonical_authored_it != canonical_authored_by_authored.end() ? canonical_authored_it->second : authored_key;
    const auto used_variant_it = used_variant_programs_by_authored.find(materialized_authored_key);
    if (used_variant_it == used_variant_programs_by_authored.end() || used_variant_it->second.empty()) {
      continue;
    }

    const SourceInstrumentRef default_target_program = default_variant_program_by_authored[authored_key];
    const uint32_t default_target_inst_id = targetInstrumentInstID(default_target_program);
    const auto representative_binding_it = representative_binding_by_program_by_authored.find(authored_key);

    if (isPercussionAuthoredBank(authored_key.first)) {
      const auto used_variant_it = used_variant_programs_by_authored.find(authored_key);
      if (used_variant_it == used_variant_programs_by_authored.end() || used_variant_it->second.empty()) {
        continue;
      }

      const auto binding_map_it = binding_by_region_by_authored.find(authored_key);
      if (binding_map_it == binding_by_region_by_authored.end()) {
        continue;
      }
      const auto representative_binding_it = representative_binding_by_program_by_authored.find(authored_key);

      std::set<std::pair<const VGMRgn *, SourceInstrumentRef>> embedded_regions;
      for (const auto &variant : variants) {
        if (variant.region == nullptr || used_variant_it->second.find(variant.target) == used_variant_it->second.end()) {
          continue;
        }
        if (!embedded_regions.emplace(variant.region, variant.target).second) {
          continue;
        }

        const SourceInstrumentRef target_program = variant.target;
        const uint32_t target_inst_id = targetInstrumentInstID(target_program);

        const auto binding_it = binding_map_it->second.find(variant.region);
        if (binding_it == binding_map_it->second.end() || binding_it->second == nullptr ||
            binding_it->second->sample == nullptr || binding_it->second->region == nullptr) {
          continue;
        }

        auto *sample = binding_it->second->sample;
        auto *region = const_cast<VGMRgn *>(binding_it->second->region);
        uint32_t resolved_loop_start = 0;
        uint32_t resolved_loop_end = 0;
        const bool has_loop = resolveLoopPoints(*sample, *region, resolved_loop_start, resolved_loop_end) &&
                              resolved_loop_end > resolved_loop_start;
        const bool sample_and_hold = shouldEnableSampleAndHold(*region, has_loop);
        const uint8_t root_key = resolveRootKey(*sample, *region);
        const int16_t split_volume = resolveSplitVolume(*sample, *region);

        BAERmfEditorSampleSetup setup{};
  setup.program = target_program.second;
        setup.rootKey = root_key;
        setup.lowKey = region->keyLow;
        setup.highKey = region->keyHigh;
        std::string display_name = sample->name();
        setup.displayName = const_cast<char *>(display_name.c_str());

        uint32_t sample_index = 0;
        BAESampleInfo out_sample_info{};
        BAEResult add_result = BAERmfEditorDocument_AddEmptySample(document, &setup, &sample_index, &out_sample_info);
        auto canonical_asset_it = canonical_asset_id_by_sample.find(sample);
        const bool has_canonical_asset = canonical_asset_it != canonical_asset_id_by_sample.end();
        if (add_result == BAE_NO_ERROR && !has_canonical_asset) {
          const auto wav_path = temp_dir / ("authored_sample_" + std::to_string(staged_sample_counter++) + ".wav");
          if (!sample->saveAsWav(wav_path)) {
            L_ERROR("Failed to stage percussion-bank sample for RMF embedding: {}", sample->name());
            continue;
          }
          const std::string wav_path_string = wav_path.string();
          add_result = BAERmfEditorDocument_ReplaceSampleFromFile(
              document,
              sample_index,
              const_cast<char *>(wav_path_string.c_str()),
              &out_sample_info);
        }
        else if (add_result == BAE_NO_ERROR) {
          add_result = BAERmfEditorDocument_SetSampleAssetForSample(
              document,
              sample_index,
              canonical_asset_it->second);
        }

        if (add_result != BAE_NO_ERROR) {
          L_ERROR("Failed to add percussion-bank keysplit sample '{}' for program {} (BAE error #{})",
                  sample->name(),
              static_cast<unsigned>(target_inst_id),
                  static_cast<int>(add_result));
          continue;
        }

        if (!has_canonical_asset) {
          uint32_t new_asset_id = 0;
          if (BAERmfEditorDocument_GetSampleAssetIDForSample(document, sample_index, &new_asset_id) == BAE_NO_ERROR) {
            canonical_asset_id_by_sample[sample] = new_asset_id;
            canonical_sample_index_by_sample[sample] = sample_index;
          }
        }

        BAERmfEditorSampleInfo sample_info{};
        if (BAERmfEditorDocument_GetSampleInfo(document, sample_index, &sample_info) == BAE_NO_ERROR) {
          sample_info.program = target_program.second;
          sample_info.rootKey = root_key;
          sample_info.lowKey = region->keyLow;
          sample_info.highKey = region->keyHigh;
          sample_info.splitVolume = split_volume;
          applyFineTuneToSampleRate(sample_info.sampleInfo, *sample, *region);

          if (has_loop) {
            uint32_t loop_start = resolved_loop_start;
            uint32_t loop_end = resolved_loop_end;
            const uint32_t wave_frames = sample_info.sampleInfo.waveFrames;
            if (wave_frames > 1) {
              const uint32_t max_start = wave_frames - 1;
              loop_start = std::min(loop_start, max_start);
              loop_end = std::min(loop_end, wave_frames);
              refineLoopPointsWithPcm(*sample,
                                      wave_frames,
                                      loop_start,
                                      loop_end,
                                      pcm_cache,
                                      pcm_frame_cache);
              if (loop_end > loop_start) {
                sample_info.sampleInfo.startLoop = loop_start;
                sample_info.sampleInfo.endLoop = loop_end;
              }
            }
          }

          BAERmfEditorDocument_SetSampleInfo(document, sample_index, &sample_info);
        }

        BAERmfEditorDocument_SetSampleInstID(document, sample_index, target_inst_id);

        const RegionSampleBinding *adsr_binding = binding_it->second;
        if (strict_authored_mode && representative_binding_it != representative_binding_by_program_by_authored.end()) {
          const auto rep_it = representative_binding_it->second.find(target_program);
          if (rep_it != representative_binding_it->second.end() && rep_it->second != nullptr) {
            adsr_binding = rep_it->second;
          }
        }

        uint32_t adsr_loop_start = 0;
        uint32_t adsr_loop_end = 0;
        const bool adsr_has_loop = resolveLoopPoints(*adsr_binding->sample, *adsr_binding->region, adsr_loop_start, adsr_loop_end) &&
                                   adsr_loop_end > adsr_loop_start;
        const bool adsr_sample_and_hold = shouldEnableSampleAndHold(*adsr_binding->region, adsr_has_loop);
        const uint8_t adsr_root_key = resolveRootKey(*adsr_binding->sample, *adsr_binding->region);
        const int16_t adsr_split_volume = resolveSplitVolume(*adsr_binding->sample, *adsr_binding->region);

        auto existing = program_adsr_sources.find(target_program);
        if (existing == program_adsr_sources.end()) {
          program_adsr_sources.emplace(
              target_program,
              ProgramADSRSource{target_program, adsr_binding->region, adsr_sample_and_hold, adsr_root_key, adsr_split_volume});
        }
        else if (existing->second.region != nullptr &&
                 preferRegionForProgramADSR(*adsr_binding->region, *existing->second.region)) {
          existing->second.region = adsr_binding->region;
          existing->second.sample_and_hold = existing->second.sample_and_hold || adsr_sample_and_hold;
          existing->second.root_key = adsr_root_key;
          existing->second.split_volume = adsr_split_volume;
        }
        else {
          existing->second.sample_and_hold = existing->second.sample_and_hold || adsr_sample_and_hold;
        }
      }

      continue;
    }

    const auto instrument_index_it = instrument_index_by_authored.find(authored_key);
    if (instrument_index_it == instrument_index_by_authored.end()) {
      continue;
    }
    uint32_t sample_count_before_clone = 0;
    uint32_t sample_count_after_clone = 0;

    if (BAERmfEditorDocument_GetSampleCount(document, &sample_count_before_clone) != BAE_NO_ERROR) {
      L_ERROR("Failed to query RMF sample count before cloning authored instrument");
      return false;
    }

    BAEResult clone_result = BAERmfEditorDocument_CloneInstrumentFromBankToInstID(document,
                                                                                  runtime.bank_token,
                                                                                  instrument_index_it->second,
                                                                                  default_target_inst_id,
                                                                                  default_target_program.second);
    if (clone_result != BAE_NO_ERROR) {
      L_ERROR("Failed to clone temporary bank instrument for RMF bank {} program {} (BAE error #{})",
              static_cast<unsigned>(authored_key.first),
              static_cast<unsigned>(authored_key.second),
              static_cast<int>(clone_result));
      return false;
    }

    if (BAERmfEditorDocument_GetSampleCount(document, &sample_count_after_clone) != BAE_NO_ERROR ||
        sample_count_after_clone < sample_count_before_clone) {
      L_ERROR("Failed to query RMF sample count after cloning authored instrument");
      return false;
    }

    {
      auto &base_sample_indices = base_sample_indices_by_authored[authored_key];
      for (uint32_t sample_index = sample_count_before_clone; sample_index < sample_count_after_clone; ++sample_index) {
        base_sample_indices.push_back(sample_index);
      }
      if (base_sample_indices.empty()) {
        L_ERROR("Authored instrument clone produced no samples for RMF bank {} program {}",
                static_cast<unsigned>(authored_key.first),
                static_cast<unsigned>(authored_key.second));
        return false;
      }
    }

    for (const auto &variant : variants) {
      const SourceInstrumentRef target_program = variant.target;
      const uint32_t target_inst_id = targetInstrumentInstID(target_program);
      const auto used_variant_it = used_variant_programs_by_authored.find(authored_key);

      if (used_variant_it == used_variant_programs_by_authored.end() ||
          used_variant_it->second.find(target_program) == used_variant_it->second.end()) {
        continue;
      }

      if (target_program != default_target_program) {
        const auto base_samples_it = base_sample_indices_by_authored.find(authored_key);
        if (base_samples_it == base_sample_indices_by_authored.end()) {
          L_ERROR("Missing base sample set for authored RMF variant bank {} program {}",
                  static_cast<unsigned>(authored_key.first),
                  static_cast<unsigned>(authored_key.second));
          return false;
        }

        for (uint32_t base_sample_index : base_samples_it->second) {
          BAERmfEditorSampleInfo base_sample_info{};
          BAERmfEditorSampleSetup setup{};
          uint32_t shared_asset_id = 0;
          uint32_t new_sample_index = 0;
          BAESampleInfo ignored_sample_info{};

          if (BAERmfEditorDocument_GetSampleInfo(document, base_sample_index, &base_sample_info) != BAE_NO_ERROR) {
              L_ERROR("Failed to read base sample metadata for authored RMF variant instrument {}",
                static_cast<unsigned>(target_inst_id));
            return false;
          }
          if (BAERmfEditorDocument_GetSampleAssetIDForSample(document, base_sample_index, &shared_asset_id) != BAE_NO_ERROR) {
                L_ERROR("Failed to read base sample asset for authored RMF variant instrument {}",
                  static_cast<unsigned>(target_inst_id));
            return false;
          }

              setup.program = target_program.second;
          setup.rootKey = base_sample_info.rootKey;
          setup.lowKey = base_sample_info.lowKey;
          setup.highKey = base_sample_info.highKey;
          setup.displayName = const_cast<char *>(base_sample_info.displayName);

          if (BAERmfEditorDocument_AddEmptySample(document, &setup, &new_sample_index, &ignored_sample_info) != BAE_NO_ERROR) {
                L_ERROR("Failed to add RMF variant sample for instrument {}",
                  static_cast<unsigned>(target_inst_id));
            return false;
          }
          if (BAERmfEditorDocument_SetSampleAssetForSample(document, new_sample_index, shared_asset_id) != BAE_NO_ERROR) {
                L_ERROR("Failed to share sample asset for RMF variant instrument {}",
                  static_cast<unsigned>(target_inst_id));
            return false;
          }

              base_sample_info.program = target_program.second;
          if (BAERmfEditorDocument_SetSampleInfo(document, new_sample_index, &base_sample_info) != BAE_NO_ERROR) {
                L_ERROR("Failed to apply RMF variant sample metadata for instrument {}",
                  static_cast<unsigned>(target_inst_id));
            return false;
          }
          if (BAERmfEditorDocument_SetSampleInstID(document, new_sample_index, target_inst_id) != BAE_NO_ERROR) {
            L_ERROR("Failed to bind RMF variant sample to instrument {}",
                  static_cast<unsigned>(target_inst_id));
            return false;
          }
        }
      }

      for (const auto &binding : bindings) {
        if (binding.sample == nullptr || binding.region == nullptr || binding.region != variant.region) {
          continue;
        }

        const AuthoredInstrumentKey binding_key{
            normalizeSourceBankForAuthoredLookup(binding.source_collection_bank),
            binding.source_program,
        };
        if (binding_key != authored_key) {
          continue;
        }

        const RegionSampleBinding *adsr_binding = &binding;
        if (strict_authored_mode && representative_binding_it != representative_binding_by_program_by_authored.end()) {
          const auto rep_it = representative_binding_it->second.find(target_program);
          if (rep_it != representative_binding_it->second.end() && rep_it->second != nullptr) {
            adsr_binding = rep_it->second;
          }
        }

        uint32_t adsr_loop_start = 0;
        uint32_t adsr_loop_end = 0;
        const bool adsr_has_loop = resolveLoopPoints(*adsr_binding->sample, *adsr_binding->region, adsr_loop_start, adsr_loop_end) &&
                                   adsr_loop_end > adsr_loop_start;
        const bool adsr_sample_and_hold = shouldEnableSampleAndHold(*adsr_binding->region, adsr_has_loop);
        const uint8_t adsr_root_key = resolveRootKey(*adsr_binding->sample, *adsr_binding->region);
        const int16_t adsr_split_volume = resolveSplitVolume(*adsr_binding->sample, *adsr_binding->region);

        auto existing = program_adsr_sources.find(target_program);
        if (existing == program_adsr_sources.end()) {
          program_adsr_sources.emplace(
              target_program,
              ProgramADSRSource{target_program, adsr_binding->region, adsr_sample_and_hold, adsr_root_key, adsr_split_volume});
        }
        else if (existing->second.region != nullptr &&
                 preferRegionForProgramADSR(*adsr_binding->region, *existing->second.region)) {
          existing->second.region = adsr_binding->region;
          existing->second.sample_and_hold = existing->second.sample_and_hold || adsr_sample_and_hold;
          existing->second.root_key = adsr_root_key;
          existing->second.split_volume = adsr_split_volume;
        }
        else {
          existing->second.sample_and_hold = existing->second.sample_and_hold || adsr_sample_and_hold;
        }
        break;
      }
    }
  }

  applyProgramADSR(document, program_adsr_sources);

  uint16_t track_count = 0;
  if (BAERmfEditorDocument_GetTrackCount(document, &track_count) == BAE_NO_ERROR) {
    std::vector<bool> track_uses_percussion_bank_mode(track_count, false);
    for (uint16_t t = 0; t < track_count; ++t) {
      track_uses_percussion_bank_mode[t] = trackUsesPercussionBankMode(document, t);
    }

    for (uint16_t t = 0; t < track_count; ++t) {
      uint32_t note_count = 0;
      if (BAERmfEditorDocument_GetNoteCount(document, t, &note_count) != BAE_NO_ERROR) {
        continue;
      }

      SourceInstrumentRef first_track_program{kEmbeddedTargetBanks[0], 0};
      bool first_track_program_set = false;
      for (uint32_t n = 0; n < note_count; ++n) {
        BAERmfEditorNoteInfo note_info{};
        AuthoredInstrumentKey authored_key{0, 0};
        if (BAERmfEditorDocument_GetNoteInfo(document, t, n, &note_info) != BAE_NO_ERROR) {
          continue;
        }

        if (track_uses_percussion_bank_mode[t] && note_info.bank == 0) {
          const AuthoredInstrumentKey percussion_key_128{static_cast<uint8_t>(128), note_info.program};
          const AuthoredInstrumentKey percussion_key_127{static_cast<uint8_t>(127), note_info.program};
          if (variants_by_authored.find(percussion_key_128) != variants_by_authored.end()) {
            authored_key = percussion_key_128;
          }
          else if (variants_by_authored.find(percussion_key_127) != variants_by_authored.end()) {
            authored_key = percussion_key_127;
          }
          else if (!resolve_authored_key_for_reference(note_info.bank, note_info.program, authored_key)) {
            continue;
          }
        }
        else if (!resolve_authored_key_for_reference(note_info.bank, note_info.program, authored_key)) {
          continue;
        }

        auto variants_it = variants_by_authored.find(authored_key);
        if (variants_it == variants_by_authored.end()) {
          continue;
        }

        const SourceInstrumentRef default_program = default_variant_program_by_authored[authored_key];
        bool has_program_variants = false;
        for (const auto &variant : variants_it->second) {
          if (variant.target != default_program) {
            has_program_variants = true;
            break;
          }
        }

        if (!has_program_variants) {
          continue;
        }

        const auto target_programs = collectVariantProgramsForNote(
            note_info.note,
            note_info.velocity,
            variants_it->second,
            default_program);
        if (target_programs.empty()) {
          continue;
        }

        if (strict_authored_mode) {
          std::string targets;
          for (const auto &target : target_programs) {
            if (!targets.empty()) {
              targets += ",";
            }
            targets += std::to_string(target.first) + ":" + std::to_string(target.second);
          }
          L_INFO("RMF strict note remap track={} noteIndex={} source {}:{} note={} vel={} authored {}:{} -> [{}]",
                 static_cast<unsigned>(t),
                 static_cast<unsigned>(n),
                 static_cast<unsigned>(note_info.bank),
                 static_cast<unsigned>(note_info.program),
                 static_cast<unsigned>(note_info.note),
                 static_cast<unsigned>(note_info.velocity),
                 static_cast<unsigned>(authored_key.first),
                 static_cast<unsigned>(authored_key.second),
                 targets);
        }

        note_info.bank = target_programs.front().first;
        note_info.program = target_programs.front().second;
        BAERmfEditorDocument_SetNoteInfo(document, t, n, &note_info);

        for (size_t extra_index = 1; extra_index < target_programs.size(); ++extra_index) {
          if (BAERmfEditorDocument_AddNote(document,
                                           t,
                                           note_info.startTick,
                                           note_info.durationTicks,
                                           note_info.note,
                                           note_info.velocity) != BAE_NO_ERROR) {
            continue;
          }

          uint32_t updated_note_count = 0;
          if (BAERmfEditorDocument_GetNoteCount(document, t, &updated_note_count) != BAE_NO_ERROR ||
              updated_note_count == 0) {
            continue;
          }

          BAERmfEditorNoteInfo duplicated_note = note_info;
          duplicated_note.program = target_programs[extra_index].second;
          duplicated_note.bank = target_programs[extra_index].first;
          BAERmfEditorDocument_SetNoteInfo(document, t, updated_note_count - 1, &duplicated_note);
        }

        if (!first_track_program_set) {
          first_track_program = target_programs.front();
          first_track_program_set = true;
        }
      }

      if (first_track_program_set) {
        BAERmfEditorDocument_SetTrackDefaultInstrument(document, t, first_track_program.first, first_track_program.second);
      }
    }
  }

  // Percussion channel fixup: standard MIDI percussion on channel 9 carries no CC0,
  // so notes parse with bank=0 rather than bank=128. Remap them directly by program
  // and inject NRPN (5,0) = USE_NORM_BANK so BAE treats channel 9 as melodic.
  {
    struct PercEntry {
      const std::vector<ProgramVariant> *variants;
      SourceInstrumentRef default_program;
    };
    std::map<uint8_t, PercEntry> perc_by_source_program;
    for (const auto &[authored_key, authored_variants] : variants_by_authored) {
      if (!isPercussionAuthoredBank(authored_key.first)) {
        continue;
      }
      const auto canonical_it = canonical_authored_by_authored.find(authored_key);
      const AuthoredInstrumentKey materialized =
          (canonical_it != canonical_authored_by_authored.end()) ? canonical_it->second : authored_key;
      const auto used_it = used_variant_programs_by_authored.find(materialized);
      if (used_it == used_variant_programs_by_authored.end() || used_it->second.empty()) {
        continue;
      }
        const SourceInstrumentRef default_prog = default_variant_program_by_authored.count(authored_key)
          ? default_variant_program_by_authored.at(authored_key)
          : SourceInstrumentRef{kEmbeddedTargetBanks[0], authored_key.second};
      perc_by_source_program[authored_key.second] = {&authored_variants, default_prog};
    }

    if (!perc_by_source_program.empty()) {
      uint16_t perc_track_count = 0;
      if (BAERmfEditorDocument_GetTrackCount(document, &perc_track_count) == BAE_NO_ERROR) {
        for (uint16_t t = 0; t < perc_track_count; ++t) {
          BAERmfEditorTrackInfo track_info{};
          if (BAERmfEditorDocument_GetTrackInfo(document, t, &track_info) != BAE_NO_ERROR) {
            continue;
          }

          const bool track_uses_percussion_mode = trackUsesPercussionBankMode(document, t);

          uint32_t note_count = 0;
          if (BAERmfEditorDocument_GetNoteCount(document, t, &note_count) != BAE_NO_ERROR) {
            continue;
          }

          bool track_has_explicit_perc_bank = isPercussionDocumentBank(track_info.bank);
          for (uint32_t n = 0; n < note_count && !track_has_explicit_perc_bank; ++n) {
            BAERmfEditorNoteInfo note_info{};
            if (BAERmfEditorDocument_GetNoteInfo(document, t, n, &note_info) != BAE_NO_ERROR) {
              continue;
            }
            if (isPercussionDocumentBank(note_info.bank)) {
              track_has_explicit_perc_bank = true;
            }
          }

          // Keep channel 10 fallback (bank may parse as 0), and also support explicit
          // non-channel-10 percussion-bank usage (e.g. ch5/ch6 with bank 128 or NRPN perc mode).
          if (!track_has_explicit_perc_bank && track_info.channel != 9 && !track_uses_percussion_mode) {
            continue;
          }

          bool track_needs_nrpn = false;
          uint32_t remapped_note_count = 0;

          // Remap track default when it is explicitly on percussion bank, or channel 10 fallback.
          if (isPercussionDocumentBank(track_info.bank) ||
              track_info.channel == 9 ||
              track_uses_percussion_mode) {
            const auto perc_it = perc_by_source_program.find(track_info.program);
            const SourceInstrumentRef default_target =
                perc_it != perc_by_source_program.end() ? perc_it->second.default_program : kSilentPercussionFallbackTarget;
            if (track_info.bank != default_target.first ||
                track_info.program != default_target.second) {
              BAERmfEditorDocument_SetTrackDefaultInstrument(document, t, default_target.first, default_target.second);
              track_needs_nrpn = true;
            }
          }

          const uint32_t original_note_count = note_count;
          for (uint32_t n = 0; n < original_note_count; ++n) {
            BAERmfEditorNoteInfo note_info{};
            if (BAERmfEditorDocument_GetNoteInfo(document, t, n, &note_info) != BAE_NO_ERROR) {
              continue;
            }
            if ((note_info.bank == kEmbeddedTargetBanks[0] ||
                 note_info.bank == kEmbeddedTargetBanks[1] ||
                 note_info.bank == kEmbeddedTargetBanks[2])) {
              continue;
            }

            const bool explicit_perc_bank_note = isPercussionDocumentBank(note_info.bank);
            const bool channel10_fallback_note = (track_info.channel == 9) && !explicit_perc_bank_note;
            const bool nrpn_perc_fallback_note = track_uses_percussion_mode && note_info.bank == 0;
            if (!explicit_perc_bank_note && !channel10_fallback_note && !nrpn_perc_fallback_note) {
              continue;
            }

            const auto perc_it = perc_by_source_program.find(note_info.program);
            std::vector<SourceInstrumentRef> targets;
            if (perc_it == perc_by_source_program.end()) {
              targets.push_back(kSilentPercussionFallbackTarget);
            }
            else {
              targets = collectVariantProgramsForNote(
                  note_info.note,
                  note_info.velocity,
                  *perc_it->second.variants,
                  perc_it->second.default_program);
              if (targets.empty()) {
                targets.push_back(kSilentPercussionFallbackTarget);
              }
            }

            note_info.bank = targets.front().first;
            note_info.program = targets.front().second;
            BAERmfEditorDocument_SetNoteInfo(document, t, n, &note_info);
            track_needs_nrpn = true;
            ++remapped_note_count;
            for (size_t extra = 1; extra < targets.size(); ++extra) {
              if (BAERmfEditorDocument_AddNote(document,
                                               t,
                                               note_info.startTick,
                                               note_info.durationTicks,
                                               note_info.note,
                                               note_info.velocity) != BAE_NO_ERROR) {
                continue;
              }
              uint32_t updated_count = 0;
              if (BAERmfEditorDocument_GetNoteCount(document, t, &updated_count) != BAE_NO_ERROR ||
                  updated_count == 0) {
                continue;
              }
              BAERmfEditorNoteInfo dup = note_info;
              dup.program = targets[extra].second;
              dup.bank = targets[extra].first;
              BAERmfEditorDocument_SetNoteInfo(document, t, updated_count - 1, &dup);
            }
          }

          if (track_needs_nrpn) {
                 L_INFO("RMF percussion remap track {} channel {} mode={} remapped {} notes to embedded banks",
                   static_cast<unsigned>(t),
                   static_cast<unsigned>(track_info.channel),
                   track_uses_percussion_mode ? "nrpn" : (track_info.channel == 9 ? "ch10" : "bank128"),
                   static_cast<unsigned>(remapped_note_count));
            // NRPN (MSB=5, LSB=0) data-entry=3 -> USE_NORM_BANK: treat this channel as melodic.
            BAERmfEditorDocument_AddTrackCCEvent(document, t, 99, 0, 5);
            BAERmfEditorDocument_AddTrackCCEvent(document, t, 98, 0, 0);
            BAERmfEditorDocument_AddTrackCCEvent(document, t, 6, 0, 3);
          }
        }
      }
    }
  }

  // Remap all remaining melodic instrument references. Percussion-bank notes were
  // already moved to embedded target banks above, so they can't be stolen by a melodic remap that
  // happens to share the same source bank=0 and program number.
  for (const auto &[authored_key, source_banks] : source_document_banks_by_instrument) {
    if (isPercussionAuthoredBank(authored_key.first)) {
      continue;
    }

    const SourceInstrumentRef default_target_program = default_variant_program_by_authored[authored_key];
    for (uint16_t source_bank : source_banks) {
      if (source_bank == default_target_program.first && default_target_program.second == authored_key.second) {
        continue;
      }

      if (strict_authored_mode) {
        L_INFO("RMF strict final remap source {}:{} -> target {}:{} (authored {}:{})",
               static_cast<unsigned>(source_bank),
               static_cast<unsigned>(authored_key.second),
               static_cast<unsigned>(default_target_program.first),
               static_cast<unsigned>(default_target_program.second),
               static_cast<unsigned>(authored_key.first),
               static_cast<unsigned>(authored_key.second));
      }

      BAERmfEditorDocument_RemapInstrumentReferences(
          document,
          source_bank,
          authored_key.second,
          default_target_program.first,
          default_target_program.second);
    }
  }

  return true;
}

static bool embedSamplesAndRemapPrograms(BAERmfEditorDocument *document, const VGMColl &coll) {
  std::vector<RegionSampleBinding> bindings;
  if (!collectRegionSampleBindings(coll, bindings)) {
    return false;
  }
  if (bindings.empty()) {
    return true;
  }

  std::set<uint8_t> source_programs;
  std::map<uint8_t, std::vector<const VGMRgn *>> regions_by_source_program;
  for (const auto &binding : bindings) {
    source_programs.insert(binding.source_program);
    auto &regions = regions_by_source_program[binding.source_program];
    bool exists = false;
    for (const auto *existing : regions) {
      if (existing == binding.region) {
        exists = true;
        break;
      }
    }
    if (!exists) {
      regions.push_back(binding.region);
    }
  }

  std::set<uint16_t> used_slots;
  for (uint8_t source_program : source_programs) {
    used_slots.insert(targetInstrumentSlot(SourceInstrumentRef{kEmbeddedTargetBanks[0], source_program}));
  }
  auto allocate_program = [&used_slots]() -> SourceInstrumentRef {
    for (uint16_t candidate = 0; candidate < 384; ++candidate) {
      if (used_slots.find(candidate) == used_slots.end()) {
        used_slots.insert(candidate);
        return makeTargetInstrumentRefFromSlot(candidate);
      }
    }
    return SourceInstrumentRef{kEmbeddedTargetBanks[0], 0xff};
  };

  std::map<uint8_t, std::vector<ProgramVariant>> variants_by_source_program;
  std::map<uint8_t, SourceInstrumentRef> default_variant_program_by_source;
  std::map<std::pair<uint8_t, const VGMRgn *>, SourceInstrumentRef> target_program_by_source_region;

  for (const auto &[source_program, regions] : regions_by_source_program) {
    const VGMRgn *default_region = regions.empty() ? nullptr : regions.front();
    for (const auto *region : regions) {
      if (default_region == nullptr ||
          (region != nullptr && preferRegionForProgramADSR(*region, *default_region))) {
        default_region = region;
      }
    }

    auto &variants = variants_by_source_program[source_program];
    for (const auto *region : regions) {
      if (region == nullptr) {
        continue;
      }

      SourceInstrumentRef target_program{kEmbeddedTargetBanks[0], source_program};
      if (region != default_region) {
        target_program = allocate_program();
        if (target_program.second == 0xff) {
          target_program = SourceInstrumentRef{kEmbeddedTargetBanks[0], source_program};
        }
      }

      variants.push_back(ProgramVariant{region, target_program});
      target_program_by_source_region[{source_program, region}] = target_program;
    }

    default_variant_program_by_source[source_program] =
        (default_region != nullptr)
            ? target_program_by_source_region[{source_program, default_region}]
            : SourceInstrumentRef{kEmbeddedTargetBanks[0], source_program};
  }

  const auto temp_dir = std::filesystem::temp_directory_path() /
      ("vgmtrans-rmf-" + std::to_string(static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count())));
  std::error_code ec;
  std::filesystem::create_directories(temp_dir, ec);
  if (ec) {
    L_ERROR("Failed to create temporary directory for RMF sample staging: {}", temp_dir.string());
    return false;
  }

  TempDirGuard temp_guard{temp_dir};
  std::set<uint8_t> remapped_programs = source_programs;
  std::map<SourceInstrumentRef, ProgramADSRSource> program_adsr_sources;
  std::map<VGMSamp *, uint32_t> canonical_asset_id_by_sample;
  std::map<VGMSamp *, uint32_t> canonical_sample_index_by_sample;
  std::map<VGMSamp *, std::vector<int16_t>> pcm_cache;
  std::map<VGMSamp *, uint32_t> pcm_frame_cache;

  std::map<uint8_t, std::set<uint16_t>> source_banks_by_program;
  for (uint8_t source_program : source_programs) {
    std::set<uint16_t> banks;
    collectBanksForProgram(document, source_program, banks);
    banks.insert(0);
    source_banks_by_program.emplace(source_program, std::move(banks));
  }

  for (size_t i = 0; i < bindings.size(); ++i) {
    const auto &binding = bindings[i];
    auto &sample = *binding.sample;
    const auto &region = *binding.region;
    uint32_t resolved_loop_start = 0;
    uint32_t resolved_loop_end = 0;
    const bool has_loop = resolveLoopPoints(sample, region, resolved_loop_start, resolved_loop_end) &&
                          resolved_loop_end > resolved_loop_start;
    const bool sample_and_hold = shouldEnableSampleAndHold(region, has_loop);
    const uint8_t root_key = resolveRootKey(sample, region);
    const int16_t split_volume = resolveSplitVolume(sample, region);

    const auto target_program_it = target_program_by_source_region.find({binding.source_program, binding.region});
    if (target_program_it == target_program_by_source_region.end()) {
      continue;
    }
    const SourceInstrumentRef target_program = target_program_it->second;

    const auto wav_path = temp_dir / ("sample_" + std::to_string(i) + ".wav");
    auto canonical_asset = canonical_asset_id_by_sample.find(binding.sample);
    bool has_canonical_asset = canonical_asset != canonical_asset_id_by_sample.end();

    if (!has_canonical_asset && !sample.saveAsWav(wav_path)) {
      L_ERROR("Failed to stage sample for RMF embedding: {}", sample.name());
      continue;
    }

    BAERmfEditorSampleSetup setup{};
    setup.program = target_program.second;
    setup.rootKey = root_key;
    setup.lowKey = region.keyLow;
    setup.highKey = region.keyHigh;

    std::string display_name = sample.name();
    setup.displayName = const_cast<char *>(display_name.c_str());

    uint32_t sample_count_before = 0;
    if (BAERmfEditorDocument_GetSampleCount(document, &sample_count_before) != BAE_NO_ERROR) {
      continue;
    }

    uint32_t sample_index = sample_count_before;
    BAESampleInfo out_sample_info{};
    BAEResult add_result = BAE_NO_ERROR;
    if (!has_canonical_asset) {
      const std::string wav_path_string = wav_path.string();
      add_result = BAERmfEditorDocument_AddSampleFromFile(
          document,
          const_cast<char *>(wav_path_string.c_str()),
          &setup,
          &out_sample_info);
    }
    else {
      add_result = BAERmfEditorDocument_AddEmptySample(
          document,
          &setup,
          &sample_index,
          &out_sample_info);
      if (add_result == BAE_NO_ERROR) {
        add_result = BAERmfEditorDocument_SetSampleAssetForSample(
            document,
            sample_index,
            canonical_asset->second);
      }
      if (add_result == BAE_NO_ERROR) {
        const auto canonical_sample_index = canonical_sample_index_by_sample.find(binding.sample);
        if (canonical_sample_index != canonical_sample_index_by_sample.end()) {
          add_result = BAERmfEditorDocument_PropagateReplacementToAsset(
              document,
              canonical_sample_index->second);
        }
      }
    }

    if (add_result != BAE_NO_ERROR) {
      L_ERROR("Failed to add RMF sample '{}' for program {} (BAE error #{})",
              sample.name(),
              static_cast<unsigned>(targetInstrumentInstID(target_program)),
              static_cast<int>(add_result));
      continue;
    }

    if (!has_canonical_asset) {
      uint32_t new_asset_id = 0;
      if (BAERmfEditorDocument_GetSampleAssetIDForSample(document, sample_index, &new_asset_id) == BAE_NO_ERROR) {
        canonical_asset_id_by_sample[binding.sample] = new_asset_id;
        canonical_sample_index_by_sample[binding.sample] = sample_index;
      }
    }

    BAERmfEditorSampleInfo sample_info{};
    if (BAERmfEditorDocument_GetSampleInfo(document, sample_index, &sample_info) == BAE_NO_ERROR) {
      sample_info.program = target_program.second;
      sample_info.rootKey = root_key;
      sample_info.lowKey = region.keyLow;
      sample_info.highKey = region.keyHigh;
      sample_info.splitVolume = split_volume;
      applyFineTuneToSampleRate(sample_info.sampleInfo, sample, region);

      uint32_t loop_start = 0;
      uint32_t loop_end = 0;
      if (has_loop) {
        loop_start = resolved_loop_start;
        loop_end = resolved_loop_end;
        const uint32_t wave_frames = sample_info.sampleInfo.waveFrames;
        if (wave_frames > 1) {
          const uint32_t max_start = wave_frames - 1;
          loop_start = std::min(loop_start, max_start);
          loop_end = std::min(loop_end, wave_frames);
          refineLoopPointsWithPcm(sample,
                                  wave_frames,
                                  loop_start,
                                  loop_end,
                                  pcm_cache,
                                  pcm_frame_cache);
          if (loop_end > loop_start) {
            sample_info.sampleInfo.startLoop = loop_start;
            sample_info.sampleInfo.endLoop = loop_end;
          }
        }
      }

      BAERmfEditorDocument_SetSampleInfo(document, sample_index, &sample_info);
    }

    const uint32_t target_inst_id = targetInstrumentInstID(target_program);
    BAERmfEditorDocument_SetSampleInstID(document, sample_index, target_inst_id);

    auto existing = program_adsr_sources.find(target_program);
    if (existing == program_adsr_sources.end()) {
      program_adsr_sources.emplace(
          target_program,
          ProgramADSRSource{target_program, &region, sample_and_hold, root_key, split_volume});
    }
    else if (existing->second.region != nullptr &&
             preferRegionForProgramADSR(region, *existing->second.region)) {
      existing->second.region = &region;
      existing->second.sample_and_hold = existing->second.sample_and_hold || sample_and_hold;
      existing->second.root_key = root_key;
      existing->second.split_volume = split_volume;
    }
    else {
      existing->second.sample_and_hold = existing->second.sample_and_hold || sample_and_hold;
    }
  }

  applyProgramADSR(document, program_adsr_sources);

  uint16_t track_count = 0;
  if (BAERmfEditorDocument_GetTrackCount(document, &track_count) == BAE_NO_ERROR) {
    for (uint16_t t = 0; t < track_count; ++t) {
      uint32_t note_count = 0;
      if (BAERmfEditorDocument_GetNoteCount(document, t, &note_count) != BAE_NO_ERROR) {
        continue;
      }

      SourceInstrumentRef first_track_program{kEmbeddedTargetBanks[0], 0};
      bool first_track_program_set = false;
      for (uint32_t n = 0; n < note_count; ++n) {
        BAERmfEditorNoteInfo note_info{};
        if (BAERmfEditorDocument_GetNoteInfo(document, t, n, &note_info) != BAE_NO_ERROR) {
          continue;
        }

        auto variants_it = variants_by_source_program.find(note_info.program);
        if (variants_it == variants_by_source_program.end()) {
          continue;
        }

        const SourceInstrumentRef default_program = default_variant_program_by_source[note_info.program];
        bool has_program_splits = false;
        for (const auto &variant : variants_it->second) {
          if (variant.target != default_program) {
            has_program_splits = true;
            break;
          }
        }

        if (!has_program_splits) {
          continue;
        }

        const auto target_programs = collectVariantProgramsForNote(
            note_info.note,
            note_info.velocity,
            variants_it->second,
            default_program);
        if (target_programs.empty()) {
          continue;
        }

        note_info.bank = target_programs.front().first;
        note_info.program = target_programs.front().second;
        BAERmfEditorDocument_SetNoteInfo(document, t, n, &note_info);

        for (size_t extra_index = 1; extra_index < target_programs.size(); ++extra_index) {
          if (BAERmfEditorDocument_AddNote(document,
                                           t,
                                           note_info.startTick,
                                           note_info.durationTicks,
                                           note_info.note,
                                           note_info.velocity) != BAE_NO_ERROR) {
            continue;
          }

          uint32_t updated_note_count = 0;
          if (BAERmfEditorDocument_GetNoteCount(document, t, &updated_note_count) != BAE_NO_ERROR ||
              updated_note_count == 0) {
            continue;
          }

          BAERmfEditorNoteInfo duplicated_note = note_info;
          duplicated_note.program = target_programs[extra_index].second;
          duplicated_note.bank = target_programs[extra_index].first;
          BAERmfEditorDocument_SetNoteInfo(document, t, updated_note_count - 1, &duplicated_note);
        }

        if (!first_track_program_set) {
          first_track_program = target_programs.front();
          first_track_program_set = true;
        }
      }

      if (first_track_program_set) {
        BAERmfEditorDocument_SetTrackDefaultInstrument(document, t, first_track_program.first, first_track_program.second);
      }
    }
  }

  for (uint8_t source_program : remapped_programs) {
    const SourceInstrumentRef default_target_program = default_variant_program_by_source[source_program];
    const auto source_banks_it = source_banks_by_program.find(source_program);
    if (source_banks_it == source_banks_by_program.end()) {
      continue;
    }

    for (uint16_t source_bank : source_banks_it->second) {
      if (source_bank == default_target_program.first && source_program == default_target_program.second) {
        continue;
      }
      BAERmfEditorDocument_RemapInstrumentReferences(
          document,
          source_bank,
          source_program,
          default_target_program.first,
          default_target_program.second);
    }
  }

  return true;
}

static bool requiresZmfOutput(const VGMColl &coll) {
  std::vector<RegionSampleBinding> bindings;
  if (!collectRegionSampleBindings(coll, bindings)) {
    return false;
  }

  for (const auto &binding : bindings) {
    if (binding.sample == nullptr || binding.region == nullptr) {
      continue;
    }

    uint32_t loop_start = 0;
    uint32_t loop_end = 0;
    if (!resolveLoopPoints(*binding.sample, *binding.region, loop_start, loop_end)) {
      continue;
    }

    if (loop_end > loop_start && (loop_end - loop_start) < MIN_LOOP_SIZE_RMF) {
      return true;
    }
  }

  return false;
}

static std::filesystem::path selectRmfOutputPath(bool requires_zmf,
                                                 const std::filesystem::path &preferred_path) {
  std::filesystem::path output_path = preferred_path;
  const bool prefer_zmf = preferred_path.extension() == ".zmf";
  if (requires_zmf || prefer_zmf) {
    output_path.replace_extension(".zmf");
  }
  else {
    output_path.replace_extension(".rmf");
  }
  return output_path;
}

} // namespace

bool requiresZmfForRMF(const VGMColl &coll) {
  return requiresZmfOutput(coll);
}

bool saveAsRMF(const VGMColl &coll,
               const std::filesystem::path &filepath,
               std::filesystem::path *actual_filepath) {
  if (coll.seq() == nullptr) {
    return false;
  }

  MidiFile *midi = coll.seq()->convertToMidi(&coll);
  if (midi == nullptr) {
    return false;
  }

  std::vector<uint8_t> midi_buffer;
  midi->writeMidiToBuffer(midi_buffer);
  delete midi;

  if (midi_buffer.empty()) {
    return false;
  }

  BAERmfEditorDocument *document = BAERmfEditorDocument_LoadFromMemory(
      midi_buffer.data(),
      static_cast<uint32_t>(midi_buffer.size()),
      BAE_MIDI_TYPE);
  if (document == nullptr) {
    return false;
  }

  BAERmfEditorDocument_SetMidiStorageType(document, BAE_EDITOR_MIDI_STORAGE_ECMI);

  if (!coll.name().empty()) {
    BAERmfEditorDocument_SetInfo(document, TITLE_INFO, coll.name().c_str());
    BAERmfEditorDocument_SetInfo(document, COPYRIGHT_INFO, "Generated by vgmtrans (zefie's fork)");
  }

  if (!embedAuthoredBankPrograms(document, coll)) {
    BAERmfEditorDocument_Delete(document);
    return false;
  }
  ensureTrackControllerZeroAtTick0(document, 91);
  ensureTrackControllerZeroAtTick0(document, 93);

  if (BAERmfEditorDocument_Validate(document) != BAE_NO_ERROR) {
    BAERmfEditorDocument_Delete(document);
    return false;
  }

  uint32_t zmf_reason = 0;
  const bool requires_zmf = BAERmfEditorDocument_RequiresZmf(document, &zmf_reason) != FALSE;
  const std::filesystem::path resolved_output_path = selectRmfOutputPath(requires_zmf, filepath);
  if (actual_filepath != nullptr) {
    *actual_filepath = resolved_output_path;
  }

    const std::string output_path = pathToUtf8String(resolved_output_path);
  BAEResult result = BAERmfEditorDocument_SaveAsRmf(
      document,
        const_cast<char *>(output_path.empty() ? "" : &output_path[0]));
  BAERmfEditorDocument_Delete(document);
  return result == BAE_NO_ERROR;
}

}