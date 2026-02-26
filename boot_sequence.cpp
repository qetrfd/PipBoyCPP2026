#include "boot_sequence.h"
#include <algorithm>
#include <cmath>

static SDL_Texture* makeText(SDL_Renderer* r, TTF_Font* f, const std::string& s, SDL_Color c, int& w, int& h) {
  SDL_Surface* surf = TTF_RenderUTF8_Blended(f, s.c_str(), c);
  if (!surf) return nullptr;
  SDL_Texture* tex = SDL_CreateTextureFromSurface(r, surf);
  w = surf->w; h = surf->h;
  SDL_FreeSurface(surf);
  return tex;
}

static void fadeToBlack(SDL_Renderer* r, int W, int H, int ms) {
  if (ms <= 0) return;

  Uint32 t0 = SDL_GetTicks();
  SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);

  while ((int)(SDL_GetTicks() - t0) < ms) {
    Uint32 now = SDL_GetTicks();
    float p = (float)(now - t0) / (float)ms;
    if (p > 1.0f) p = 1.0f;

    Uint8 a = (Uint8)std::clamp((int)std::lround(p * 255.0f), 0, 255);
    SDL_SetRenderDrawColor(r, 0, 0, 0, a);
    SDL_Rect full{0,0,W,H};
    SDL_RenderFillRect(r, &full);

    SDL_RenderPresent(r);
    SDL_Delay(16);
  }

  SDL_SetRenderDrawColor(r, 0, 0, 0, 255);
  SDL_Rect full{0,0,W,H};
  SDL_RenderFillRect(r, &full);
  SDL_RenderPresent(r);
}

static int musicDurationMs(Mix_Music* m) {
  if (!m) return 0;

  #ifdef SDL_MIXER_VERSION_ATLEAST
  #if SDL_MIXER_VERSION_ATLEAST(2,6,0)
    double sec = Mix_MusicDuration(m);
    if (sec > 0.0) return (int)std::lround(sec * 1000.0);
  #endif
  #endif

  return 0;
}

BootSequence::BootSequence(SDL_Window* win, SDL_Renderer* r,
                           int logicalW, int logicalH,
                           TTF_Font* fontBig, TTF_Font* fontSmall,
                           const BootSequenceConfig& cfg)
: win_(win), r_(r), W_(logicalW), H_(logicalH),
  fontBig_(fontBig), fontSmall_(fontSmall), cfg_(cfg) {}

bool BootSequence::pumpSkipEvents() {
  SDL_Event e;
  while (SDL_PollEvent(&e)) {
    if (e.type == SDL_QUIT) return true;
    if (!cfg_.skipOnInput) continue;
    if (e.type == SDL_KEYDOWN) return true;
    if (e.type == SDL_MOUSEBUTTONDOWN) return true;
    if (e.type == SDL_FINGERDOWN) return true;
    if (e.type == SDL_CONTROLLERBUTTONDOWN) return true;
  }
  return false;
}

void BootSequence::clear() {
  SDL_SetRenderDrawColor(r_, cfg_.bg.r, cfg_.bg.g, cfg_.bg.b, 255);
  SDL_RenderClear(r_);
}

void BootSequence::present() {
  SDL_RenderPresent(r_);
}

void BootSequence::stopMusic() {
  Mix_HaltMusic();
  if (music_) {
    Mix_FreeMusic(music_);
    music_ = nullptr;
  }
}

void BootSequence::playMusic(const std::string& path) {
  stopMusic();
  if (path.empty()) return;
  music_ = Mix_LoadMUS(path.c_str());
  if (!music_) return;
  Mix_PlayMusic(music_, 1);
}

void BootSequence::drawFullLine(const std::string& full, int x, int y) {
  int w=0,h=0;
  SDL_Texture* tex = makeText(r_, fontSmall_, full, cfg_.green, w, h);
  if (!tex) return;
  SDL_Rect dst{ x, y, w, h };
  SDL_RenderCopy(r_, tex, nullptr, &dst);
  SDL_DestroyTexture(tex);
}

void BootSequence::drawTypingLine(const std::string& full, int x, int y, Uint32 startTime, int durationMs) {
  Uint32 now = SDL_GetTicks();
  int elapsed = (int)(now - startTime);
  if (elapsed < 0) elapsed = 0;

  float t = (durationMs <= 0) ? 1.0f : (float)elapsed / (float)durationMs;
  if (t > 1.0f) t = 1.0f;

  size_t n = (size_t)std::lround((double)full.size() * (double)t);
  if (n > full.size()) n = full.size();
  std::string partial = full.substr(0, n);

  int w=0,h=0;
  SDL_Texture* tex = makeText(r_, fontSmall_, partial, cfg_.green, w, h);
  if (!tex) return;
  SDL_Rect dst{ x, y, w, h };
  SDL_RenderCopy(r_, tex, nullptr, &dst);
  SDL_DestroyTexture(tex);
}

bool BootSequence::holdFrame(const std::vector<std::string>& lines, bool scrollMode) {
  if (cfg_.endHoldMs <= 0) return true;

  Uint32 h0 = SDL_GetTicks();
  while ((int)(SDL_GetTicks() - h0) < cfg_.endHoldMs) {
    if (pumpSkipEvents()) { stopMusic(); return false; }
    clear();

    if (!scrollMode) {
      int y = cfg_.startY;
      for (auto& s : lines) { drawFullLine(s, cfg_.startX, y); y += cfg_.lineH; }
    } else {
      int maxLines = std::max(1, (H_ - cfg_.startY - 10) / std::max(1, cfg_.lineH));
      int start = 0;
      if ((int)lines.size() > maxLines) start = (int)lines.size() - maxLines;
      int y = cfg_.startY;
      for (int i=start; i<(int)lines.size(); ++i) {
        drawFullLine(lines[i], cfg_.startX, y);
        y += cfg_.lineH;
      }
    }

    present();
    SDL_Delay(16);
  }
  return true;
}

bool BootSequence::runBootBlockA_AudioSync(const std::vector<std::string>& lines, int fallbackMs, const std::string& musicPath) {
  if (fallbackMs <= 0 || lines.empty()) return true;

  playMusic(musicPath);

  int blockMs = fallbackMs;
  int audMs = musicDurationMs(music_);
  if (audMs > 0) blockMs = audMs;

  const int holdPerLine   = std::max(0, cfg_.lineHoldMs);
  const int perLineTyping = std::max(1, blockMs / (int)lines.size());
  const int perLineSpan   = perLineTyping + holdPerLine;
  const int totalMs       = perLineSpan * (int)lines.size();

  Uint32 t0 = SDL_GetTicks();
  while ((int)(SDL_GetTicks() - t0) < totalMs) {
    if (pumpSkipEvents()) { stopMusic(); return false; }
    clear();

    int y = cfg_.startY;
    Uint32 now = SDL_GetTicks();

    for (int i=0; i<(int)lines.size(); ++i) {
      Uint32 lineStart = t0 + (Uint32)(i * perLineSpan);
      if (now < lineStart) break;

      if (now < lineStart + (Uint32)perLineTyping) {
        drawTypingLine(lines[i], cfg_.startX, y, lineStart, perLineTyping);
      } else {
        drawFullLine(lines[i], cfg_.startX, y);
      }
      y += cfg_.lineH;
    }

    present();
    SDL_Delay(16);
  }

  if (!holdFrame(lines, false)) return false;
  return true;
}

bool BootSequence::runBootBlockB_ScrollFast(const std::vector<std::string>& lines, int totalMs, const std::string& musicPath) {
  if (totalMs <= 0 || lines.empty()) return true;

  playMusic(musicPath);

  const int holdPerLine   = std::max(0, cfg_.lineHoldMs);
  const int perLineTyping = std::max(1, totalMs / (int)lines.size());
  const int perLineSpan   = perLineTyping + holdPerLine;
  const int spanMs        = perLineSpan * (int)lines.size();

  int maxLines = std::max(1, (H_ - cfg_.startY - 10) / std::max(1, cfg_.lineH));

  Uint32 t0 = SDL_GetTicks();
  while ((int)(SDL_GetTicks() - t0) < spanMs) {
    if (pumpSkipEvents()) { stopMusic(); return false; }
    clear();

    Uint32 now = SDL_GetTicks();
    int currentLine = -1;
    for (int i=0; i<(int)lines.size(); ++i) {
      Uint32 lineStart = t0 + (Uint32)(i * perLineSpan);
      if (now >= lineStart) currentLine = i;
      else break;
    }
    if (currentLine < 0) { present(); SDL_Delay(16); continue; }

    int firstVisible = std::max(0, currentLine - (maxLines - 1));
    int y = cfg_.startY;

    for (int i = firstVisible; i <= currentLine; ++i) {
      Uint32 lineStart = t0 + (Uint32)(i * perLineSpan);
      if (now < lineStart + (Uint32)perLineTyping) {
        drawTypingLine(lines[i], cfg_.startX, y, lineStart, perLineTyping);
      } else {
        drawFullLine(lines[i], cfg_.startX, y);
      }
      y += cfg_.lineH;
    }

    present();
    SDL_Delay(16);
  }

  if (!holdFrame(lines, true)) return false;
  return true;
}

bool BootSequence::run() {
  static bool alreadyRan = false;
  if (alreadyRan) return true;
  alreadyRan = true;

  if (cfg_.musicVolume >= 0) Mix_VolumeMusic(cfg_.musicVolume);

  if (cfg_.pipboyMs > 0) {
    Uint32 t0 = SDL_GetTicks();
    bool finished = false;

    while (!finished) {
      if (pumpSkipEvents()) { stopMusic(); return false; }
      clear();

      std::string title = "PIP-BOY";
      float p = (float)(SDL_GetTicks() - t0) / (float)cfg_.pipboyMs;
      if (p >= 1.0f) { p = 1.0f; finished = true; }

      size_t n = (size_t)std::lround((double)title.size() * (double)p);
      if (n > title.size()) n = title.size();

      int w=0,h=0;
      SDL_Texture* t = makeText(r_, fontBig_, title.substr(0, n), cfg_.green, w, h);
      if (t) {
        SDL_Rect dst{ (W_ - w)/2, (H_/2) - 40, w, h };
        SDL_RenderCopy(r_, t, nullptr, &dst);
        SDL_DestroyTexture(t);
      }

      present();
      SDL_Delay(16);

      if ((int)(SDL_GetTicks() - t0) >= cfg_.pipboyMs) finished = true;
    }

    if (cfg_.endHoldMs > 0) {
      Uint32 h0 = SDL_GetTicks();
      while ((int)(SDL_GetTicks() - h0) < cfg_.endHoldMs) {
        if (pumpSkipEvents()) { stopMusic(); return false; }
        clear();

        int w=0,h=0;
        SDL_Texture* t = makeText(r_, fontBig_, "PIP-BOY", cfg_.green, w, h);
        if (t) {
          SDL_Rect dst{ (W_ - w)/2, (H_/2) - 40, w, h };
          SDL_RenderCopy(r_, t, nullptr, &dst);
          SDL_DestroyTexture(t);
        }
        present();
        SDL_Delay(16);
      }
    }
  }

  if (cfg_.bootAMs <= 0 && cfg_.bootBMs <= 0) {
    stopMusic();
    return true;
  }

  std::vector<std::string> bootA = {
    "0x0000A000  POST INIT .................................. OK",
    "0x0000A014  CPU PROBE: DETECTING PROCESSORS ............. OK",
    "0x0000A02C  CPU CALIBRATION: TSC / PLL LOCK ............. OK",
    "0x0000A044  MICROCODE LOAD .............................. OK",
    "0x0000A05C  MEMORY CONTROLLER: TRAINING ................. OK",
    "0x0000A074  RAM CHECK: BANK0 ............................ OK",
    "0x0000A08C  RAM CHECK: BANK1 ............................ OK",
    "0x0000A0A4  CACHE INIT: L1/L2 ............................ OK",
    "0x0000A0BC  DMA CONTROLLER ............................... OK",
    "0x0000A0D4  IRQ TABLE SETUP .............................. OK",
    "0x0000A0EC  TIMER CALIBRATION ............................ OK",
    "0x0000A104  VIDEO ADAPTER: TEXT MODE ..................... OK",
    "0x0000A11C  KEYBOARD CONTROLLER .......................... OK",
    "0x0000A134  STORAGE BUS SCAN ............................. OK",
    "0x0000A14C  DISK 0: IDENTIFY ............................. OK",
    "0x0000A164  DISK 0: READ BOOT SECTOR ..................... OK",
    "0x0000A17C  FILESYSTEM MOUNT ............................. OK",
    "0x0000A194  LOADING KERNEL ............................... OK",
    "0x0000A1AC  CHECKSUM VERIFY .............................. OK",
    "0x0000A1C4  DEVICE DRIVERS: INIT .......................... OK",
    "0x0000A1DC  SERIAL PORT: INIT ............................ OK",
    "0x0000A1F4  RTC SYNC ..................................... OK",
    "0x0000A20C  SYSTEM SERVICES: START ........................ OK",
  };

  if (!runBootBlockA_AudioSync(bootA, cfg_.bootAMs, cfg_.audioAPath)) return false;

  std::vector<std::string> bootB = {
    "ROBCO INDUSTRIES UNIFIED OPERATING SYSTEM",
    "COPYRIGHT 2075-2077 ROBCO INDUSTRIES",
    "BIOS VERSION 6.77",
    "",
    "INITIALIZING HARDWARE...",
    "CHECKING MEMORY ............ OK",
    "LOADING MODULES ............. OK",
    "SYSTEM READY",
    "",
    "PIP-BOY PERSONAL TERMINAL",
    "ACCESSING LOCAL DATABASE",
    "SYNCING MAP DATA ............ OK",
    "INITIALIZING RADIO .......... OK",
    "CALIBRATING GEIGER ......... OK",
    "",
    "WELCOME USER"
  };

  if (!runBootBlockB_ScrollFast(bootB, cfg_.bootBMs, cfg_.audioBPath)) return false;

  stopMusic();

  fadeToBlack(r_, W_, H_, cfg_.exitFadeMs);
  return true;
}
