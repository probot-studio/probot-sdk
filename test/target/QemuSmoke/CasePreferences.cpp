// Copyright (c) Probot Studio. Hand-written; see LICENSE.
//
// (e) prosdk::Preferences on NVS (prosdk/platform/PreferencesStore). tools/qemu/run.sh boots the
// same flash image twice: boot 1 writes, boot 2 must read boot 1's values back from flash.
// BootNumber() keeps its own counter in raw NVS (namespace "qemusmoke"), independent of the
// code under test.

#include <nvs.h>
#include <nvs_flash.h>

#include <string>

#include <prosdk/preferences/Preferences.hpp>

#include "Harness.hpp"

namespace qemu {

int BootNumber() {
  static int boot = [] {
    nvs_handle_t h;
    if (nvs_open("qemusmoke", NVS_READWRITE, &h) != ESP_OK) {
      return -1;
    }
    int32_t n = 0;
    nvs_get_i32(h, "boot", &n);  // stays 0 when missing
    ++n;
    nvs_set_i32(h, "boot", n);
    nvs_commit(h);
    nvs_close(h);
    return static_cast<int>(n);
  }();
  return boot;
}

void CasePreferences(Ctx& ctx) {
  using prosdk::Preferences;
  const int boot = BootNumber();
  QCHECK_MSG(ctx, boot >= 1, "boot {}", boot);
  ctx.Note("boot={}", boot);

  // persisted state from the previous boot of this flash image
  if (boot == 1) {
    QCHECK(ctx, !Preferences::ContainsKey("qemu/boots"));
    QCHECK(ctx, !Preferences::ContainsKey("qemu/string"));
  } else {
    QCHECK_MSG(ctx, Preferences::GetInt("qemu/boots", -1) == boot - 1, "boots {}",
               Preferences::GetInt("qemu/boots", -1));
    QCHECK(ctx, Preferences::GetString("qemu/string") == "persisted by boot 1");
    QCHECK(ctx, Preferences::GetDouble("qemu/double") == 2.5);
    QCHECK(ctx, Preferences::GetBoolean("qemu/bool") == true);
    QCHECK(ctx, Preferences::GetLong("qemu/long") == int64_t{1} << 40);
    QCHECK(ctx, Preferences::GetFloat("qemu/float") == 0.25f);
    QCHECK(ctx, !Preferences::ContainsKey("qemu/removed"));
    ctx.Note("persisted=1");
  }

  // write (boot 1) / rewrite (later boots): every type, Init, Remove
  if (boot == 1) {
    Preferences::SetString("qemu/string", "persisted by boot 1");
    Preferences::SetDouble("qemu/double", 2.5);
    Preferences::SetBoolean("qemu/bool", true);
    Preferences::SetLong("qemu/long", int64_t{1} << 40);
    Preferences::SetFloat("qemu/float", 0.25f);
    Preferences::SetInt("qemu/removed", 7);
    QCHECK(ctx, Preferences::ContainsKey("qemu/removed"));
    Preferences::Remove("qemu/removed");
  }
  Preferences::InitInt("qemu/init", 5);
  Preferences::InitInt("qemu/init", 6);  // Init never overwrites
  QCHECK(ctx, Preferences::GetInt("qemu/init") == 5);
  Preferences::SetInt("qemu/boots", boot);
  QCHECK(ctx, Preferences::GetInt("qemu/boots") == boot);
  QCHECK(ctx, Preferences::GetString("qemu/string") == "persisted by boot 1");
  QCHECK(ctx, !Preferences::ContainsKey("qemu/removed"));
  QCHECK(ctx, Preferences::GetKeys().size() >= 7);
  ctx.Note("keys={}", Preferences::GetKeys().size());

  nvs_stats_t stats{};
  if (nvs_get_stats(nullptr, &stats) == ESP_OK) {
    ctx.Note("nvs_used_entries={} nvs_free_entries={}", stats.used_entries, stats.free_entries);
  }
}

}  // namespace qemu
