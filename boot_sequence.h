#pragma once
#include <SDL.h>
#include <SDL_ttf.h>
#include <SDL_mixer.h>
#include <string>
#include <vector>

struct BootSequenceConfig {
  int pipboyMs  = 6000;   
  int bootAMs   = 5000; 
  int bootBMs   = 4500;  

  int exitFadeMs = 260;
  int uiFadeInMs = 450;

  int lineHoldMs = 0;     
  int endHoldMs  = 500; 

  std::string audioAPath;
  std::string audioBPath;

  bool skipOnInput = true;

  SDL_Color bg    {0,0,0,255};
  SDL_Color green {120,255,160,255};

  int lineH  = 18;
  int startX = 46;
  int startY = 60;

  int musicVolume = 110;
};

class BootSequence {
public:
  BootSequence(SDL_Window* win, SDL_Renderer* r, int logicalW, int logicalH,
               TTF_Font* fontBig, TTF_Font* fontSmall,
               const BootSequenceConfig& cfg);

  bool run();

private:
  SDL_Window* win_ = nullptr;
  SDL_Renderer* r_ = nullptr;
  int W_ = 0, H_ = 0;
  TTF_Font* fontBig_ = nullptr;
  TTF_Font* fontSmall_ = nullptr;
  BootSequenceConfig cfg_;

  Mix_Music* music_ = nullptr;

  bool pumpSkipEvents();
  void clear();
  void present();

  void playMusic(const std::string& path);
  void stopMusic();

  void drawTypingLine(const std::string& full, int x, int y, Uint32 startTime, int durationMs);
  void drawFullLine(const std::string& full, int x, int y);

  bool holdFrame(const std::vector<std::string>& lines, bool scrollMode);
  bool runBootBlockA_AudioSync(const std::vector<std::string>& lines, int fallbackMs, const std::string& musicPath);
  bool runBootBlockB_ScrollFast(const std::vector<std::string>& lines, int totalMs, const std::string& musicPath);
};
