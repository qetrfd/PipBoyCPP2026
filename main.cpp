#include <SDL.h>
#include <SDL_image.h>
#include <SDL_ttf.h>
#include <SDL_mixer.h>
#include <curl/curl.h>

#include "boot_sequence.h"

#include <string>
#include <vector>
#include <algorithm>
#include <filesystem>
#include <cmath>
#include <unordered_map>
#include <list>
#include <fstream>
#include <sstream>

#include <thread>
#include <mutex>
#include <unordered_set>
#include <atomic>
#include <deque>
#include <cstring>
#include <chrono>
#include <ctime>

#if defined(__APPLE__)
  #include <mach-o/dyld.h>
  #include <unistd.h>
#elif defined(__linux__)
  #include <unistd.h>
  #include <limits.h>
#endif

static void die(const std::string& msg) {
  SDL_Log("FATAL: %s | SDL: %s", msg.c_str(), SDL_GetError());
  std::exit(1);
}

static std::filesystem::path exeDirPath() {
#if defined(__APPLE__)
  char buf[4096];
  uint32_t size = sizeof(buf);
  if (_NSGetExecutablePath(buf, &size) != 0) return std::filesystem::current_path();
  return std::filesystem::path(std::string(buf)).parent_path();
#elif defined(__linux__)
  char buf[PATH_MAX];
  ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf)-1);
  if (len <= 0) return std::filesystem::current_path();
  buf[len] = '\0';
  return std::filesystem::path(std::string(buf)).parent_path();
#else
  return std::filesystem::current_path();
#endif
}

static std::string assetPath(const std::string& rel) {
  std::vector<std::filesystem::path> bases = {
    exeDirPath() / "assets",
    std::filesystem::current_path() / "assets",
    std::filesystem::current_path().parent_path() / "assets",
    std::filesystem::current_path().parent_path().parent_path() / "assets"
  };
  for (const auto& b : bases) {
    auto c = b / rel;
    if (std::filesystem::exists(c)) return c.string();
  }
  return (std::filesystem::current_path() / rel).string();
}

static std::filesystem::path assetsRootDir() {
  std::vector<std::filesystem::path> bases = {
    exeDirPath() / "assets",
    std::filesystem::current_path() / "assets",
    std::filesystem::current_path().parent_path() / "assets",
    std::filesystem::current_path().parent_path().parent_path() / "assets"
  };
  for (const auto& b : bases) {
    if (std::filesystem::exists(b) && std::filesystem::is_directory(b)) return b;
  }
  return std::filesystem::current_path() / "assets";
}

struct Texture {
  SDL_Texture* tex = nullptr;
  int w = 0, h = 0;
  ~Texture() { if (tex) SDL_DestroyTexture(tex); }
};

static Texture loadTexture(SDL_Renderer* r, const std::string& path) {
  SDL_Surface* s = IMG_Load(path.c_str());
  if (!s) die("IMG_Load failed: " + path);
  SDL_Texture* t = SDL_CreateTextureFromSurface(r, s);
  Texture out;
  out.tex = t;
  out.w = s->w; out.h = s->h;
  SDL_FreeSurface(s);
  if (!out.tex) die("CreateTextureFromSurface failed");
  return out;
}

static SDL_Texture* renderText(SDL_Renderer* r, TTF_Font* font, const std::string& text, SDL_Color col, int& outW, int& outH) {
  SDL_Surface* s = TTF_RenderUTF8_Blended(font, text.c_str(), col);
  if (!s) die("TTF_RenderUTF8 failed");
  SDL_Texture* t = SDL_CreateTextureFromSurface(r, s);
  outW = s->w; outH = s->h;
  SDL_FreeSurface(s);
  if (!t) die("CreateTextureFromSurface (text) failed");
  return t;
}

static int textW(TTF_Font* f, const std::string& s) {
  int w=0,h=0;
  TTF_SizeUTF8(f, s.c_str(), &w, &h);
  return w;
}

static void fillRect(SDL_Renderer* r, const SDL_Rect& rc, SDL_Color c) {
  SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
  SDL_SetRenderDrawColor(r, c.r, c.g, c.b, c.a);
  SDL_RenderFillRect(r, &rc);
}

static void drawRect(SDL_Renderer* r, const SDL_Rect& rc, SDL_Color c) {
  SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
  SDL_SetRenderDrawColor(r, c.r, c.g, c.b, c.a);
  SDL_RenderDrawRect(r, &rc);
}

static void drawLine(SDL_Renderer* r, int x1, int y1, int x2, int y2, SDL_Color c, Uint8 a=255) {
  SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
  SDL_SetRenderDrawColor(r, c.r, c.g, c.b, a);
  SDL_RenderDrawLine(r, x1, y1, x2, y2);
}

static void drawScanlines(SDL_Renderer* r, int w, int h, int spacing=3, Uint8 alpha=18) {
  SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
  SDL_SetRenderDrawColor(r, 0, 0, 0, alpha);
  for (int y = 0; y < h; y += spacing) SDL_RenderDrawLine(r, 0, y, w, y);
}

static float clamp01(float x) { return std::max(0.0f, std::min(1.0f, x)); }
static float lerp(float a, float b, float t) { return a + (b - a) * t; }

static void drawText(SDL_Renderer* r, TTF_Font* font, const std::string& text, int x, int y, SDL_Color col) {
  int w=0,h=0;
  SDL_Texture* t = renderText(r, font, text, col, w, h);
  SDL_Rect dst{x,y,w,h};
  SDL_RenderCopy(r, t, nullptr, &dst);
  SDL_DestroyTexture(t);
}

static void drawTextRight(SDL_Renderer* r, TTF_Font* font, const std::string& text, int rightX, int y, SDL_Color col) {
  int w=0,h=0;
  SDL_Texture* t = renderText(r, font, text, col, w, h);
  SDL_Rect dst{rightX - w, y, w, h};
  SDL_RenderCopy(r, t, nullptr, &dst);
  SDL_DestroyTexture(t);
}

static void drawBar(SDL_Renderer* r, const SDL_Rect& rc, float value, float maxv, SDL_Color fill, SDL_Color frame, Uint8 frameA=110) {
  drawRect(r, rc, SDL_Color{frame.r, frame.g, frame.b, frameA});
  float t = (maxv <= 0.0f) ? 0.0f : clamp01(value / maxv);
  SDL_Rect inner = { rc.x+2, rc.y+2, (int)((rc.w-4) * t), rc.h-4 };
  fillRect(r, inner, fill);
}

static bool ptIn(const SDL_Rect& rc, int mx, int my) {
  return (mx >= rc.x && mx < rc.x + rc.w && my >= rc.y && my < rc.y + rc.h);
}

static void renderImageFit(SDL_Renderer* r, SDL_Texture* tex, int tw, int th, const SDL_Rect& box, Uint8 alpha) {
  if (!tex || tw <= 0 || th <= 0) return;
  float sx = (float)box.w / (float)tw;
  float sy = (float)box.h / (float)th;
  float s = std::min(sx, sy);
  int rw = (int)std::round(tw * s);
  int rh = (int)std::round(th * s);
  SDL_Rect dst{
    box.x + (box.w - rw)/2,
    box.y + (box.h - rh)/2,
    rw,
    rh
  };
  SDL_SetTextureAlphaMod(tex, alpha);
  SDL_RenderCopy(r, tex, nullptr, &dst);
}

static size_t curlWriteToFile(void* ptr, size_t size, size_t nmemb, void* stream) {
  FILE* fp = (FILE*)stream;
  return fwrite(ptr, size, nmemb, fp);
}

struct HttpMeta {
  std::string contentType;
  long code = 0;
};

static size_t curlHeaderCb(char* buffer, size_t size, size_t nitems, void* userdata) {
  size_t total = size * nitems;
  HttpMeta* m = (HttpMeta*)userdata;
  std::string h(buffer, buffer + total);
  std::string lower = h;
  std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c){ return (char)std::tolower(c); });
  std::string key = "content-type:";
  if (lower.rfind(key, 0) == 0) {
    std::string v = h.substr(key.size());
    while (!v.empty() && (v[0] == ' ' || v[0] == '	')) v.erase(v.begin());
    while (!v.empty() && (v.back() == '\r' || v.back() == '\n' || v.back() == ' ' || v.back() == '\t')) v.pop_back();
    m->contentType = v;
  }
  return total;
}

static bool fileLooksLikePng(const std::filesystem::path& p) {
  FILE* fp = std::fopen(p.string().c_str(), "rb");
  if (!fp) return false;
  unsigned char sig[8];
  size_t n = std::fread(sig, 1, 8, fp);
  std::fclose(fp);
  if (n != 8) return false;
  const unsigned char pngSig[8] = {0x89,0x50,0x4E,0x47,0x0D,0x0A,0x1A,0x0A};
  return std::memcmp(sig, pngSig, 8) == 0;
}

static std::atomic<long long> g_nextFetchMs{0};

static void throttleFetch(int minDelayMs) {
  using namespace std::chrono;
  long long now = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
  long long want = g_nextFetchMs.load(std::memory_order_relaxed);
  if (want > now) std::this_thread::sleep_for(milliseconds((int)(want - now)));
  long long after = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count() + minDelayMs;
  g_nextFetchMs.store(after, std::memory_order_relaxed);
}

static bool httpGetToFile(const std::string& url, const std::filesystem::path& outPath) {
  std::filesystem::create_directories(outPath.parent_path());

  auto tmp = outPath;
  tmp += ".tmp";

  std::error_code ec;
  std::filesystem::remove(tmp, ec);

  int attempt = 0;
  while (attempt < 2) {
    attempt++;

    throttleFetch(220);

    FILE* fp = std::fopen(tmp.string().c_str(), "wb");
    if (!fp) return false;

    CURL* curl = curl_easy_init();
    if (!curl) { std::fclose(fp); return false; }

    HttpMeta meta;

    curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curlWriteToFile);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
    curl_easy_setopt(curl, CURLOPT_USERAGENT, "pipboy-remake/1.0");
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 6L);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 12L);
    curl_easy_setopt(curl, CURLOPT_NOSIGNAL, 1L);
    curl_easy_setopt(curl, CURLOPT_HEADERFUNCTION, curlHeaderCb);
    curl_easy_setopt(curl, CURLOPT_HEADERDATA, &meta);

    CURLcode res = curl_easy_perform(curl);
    curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &meta.code);

    curl_easy_cleanup(curl);
    std::fclose(fp);

    if (res != CURLE_OK || meta.code != 200) {
      std::filesystem::remove(tmp, ec);

      if (meta.code == 429 || meta.code == 503) {
        std::this_thread::sleep_for(std::chrono::milliseconds(900));
        g_nextFetchMs.store(0, std::memory_order_relaxed);
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(120));
      }

      continue;
    }

    std::string ct = meta.contentType;
    std::string ctl = ct;
    std::transform(ctl.begin(), ctl.end(), ctl.begin(), [](unsigned char c){ return (char)std::tolower(c); });
    if (ctl.find("image/png") == std::string::npos) {
      std::filesystem::remove(tmp, ec);
      std::this_thread::sleep_for(std::chrono::milliseconds(220));
      continue;
    }

    auto sz = std::filesystem::file_size(tmp, ec);
    if (ec || sz < 200) {
      std::filesystem::remove(tmp, ec);
      std::this_thread::sleep_for(std::chrono::milliseconds(220));
      continue;
    }

    if (!fileLooksLikePng(tmp)) {
      std::filesystem::remove(tmp, ec);
      std::this_thread::sleep_for(std::chrono::milliseconds(220));
      continue;
    }

    std::filesystem::remove(outPath, ec);
    std::filesystem::rename(tmp, outPath, ec);
    if (ec) {
      std::filesystem::remove(tmp, ec);
      return false;
    }

    return true;
  }

  return false;
}


static double deg2rad(double d){ return d * M_PI / 180.0; }
static double rad2deg(double r){ return r * 180.0 / M_PI; }

static double clampLat(double lat) {
  if (lat > 85.05112878) lat = 85.05112878;
  if (lat < -85.05112878) lat = -85.05112878;
  return lat;
}

static int wrapTileX(int x, int z) {
  int n = 1 << z;
  x %= n;
  if (x < 0) x += n;
  return x;
}

static int clampTileY(int y, int z) {
  int n = 1 << z;
  if (y < 0) y = 0;
  if (y >= n) y = n - 1;
  return y;
}

static void latLonToTile(double lat, double lon, int z, int& tx, int& ty, double& fx, double& fy) {
  lat = clampLat(lat);
  double latRad = deg2rad(lat);
  double n = (double)(1 << z);

  double x = (lon + 180.0) / 360.0 * n;
  double y = (1.0 - std::log(std::tan(latRad) + 1.0/std::cos(latRad)) / M_PI) / 2.0 * n;

  tx = (int)std::floor(x);
  ty = (int)std::floor(y);
  fx = x - tx;
  fy = y - ty;

  tx = wrapTileX(tx, z);
  ty = clampTileY(ty, z);
}

static void latLonToPixel(double lat, double lon, int z, double& px, double& py) {
  lat = clampLat(lat);
  double n = (double)(1 << z);
  double x = (lon + 180.0) / 360.0 * n;
  double latRad = deg2rad(lat);
  double y = (1.0 - std::log(std::tan(latRad) + 1.0/std::cos(latRad)) / M_PI) / 2.0 * n;
  px = x * 256.0;
  py = y * 256.0;
}

static void pixelToLatLon(double px, double py, int z, double& lat, double& lon) {
  double n = (double)(1 << z);
  double x = px / 256.0;
  double y = py / 256.0;

  lon = (x / n) * 360.0 - 180.0;
  double t = M_PI * (1.0 - 2.0 * (y / n));
  lat = rad2deg(std::atan(std::sinh(t)));

  lat = clampLat(lat);
  if (lon > 180.0) lon -= 360.0;
  if (lon < -180.0) lon += 360.0;
}

static void toLogical(SDL_Window* win, int logicalW, int logicalH, int inX, int inY, int& outX, int& outY) {
  int ww=0, wh=0;
  SDL_GetWindowSize(win, &ww, &wh);
  if (ww <= 0 || wh <= 0) { outX = inX; outY = inY; return; }
  outX = (int)std::lround((double)inX * (double)logicalW / (double)ww);
  outY = (int)std::lround((double)inY * (double)logicalH / (double)wh);
}

static void windowNormToLogical(SDL_Window* win, int logicalW, int logicalH, float nx, float ny, int& lx, int& ly) {
  int ww=0, wh=0;
  SDL_GetWindowSize(win, &ww, &wh);
  int px = (int)std::lround(nx * (float)ww);
  int py = (int)std::lround(ny * (float)wh);
  toLogical(win, logicalW, logicalH, px, py, lx, ly);
}

static double wrapPxX(double px, int z) {
  double n = (double)(1 << z) * 256.0;
  px = std::fmod(px, n);
  if (px < 0) px += n;
  return px;
}

static double clampPxY(double py, int z) {
  double n = (double)(1 << z) * 256.0;
  if (py < 0.0) py = 0.0;
  if (py > n - 1.0) py = n - 1.0;
  return py;
}

static void screenToWorldPx(const SDL_Rect& mapArea, double centerLat, double centerLon, int z, int sx, int sy, double& outPx, double& outPy) {
  double cpx, cpy;
  latLonToPixel(centerLat, centerLon, z, cpx, cpy);
  int cx = mapArea.x + mapArea.w/2;
  int cy = mapArea.y + mapArea.h/2;
  outPx = cpx + (double)(sx - cx);
  outPy = cpy + (double)(sy - cy);
  outPx = wrapPxX(outPx, z);
  outPy = clampPxY(outPy, z);
}

static void applyZoomAtCursor(double& centerLat, double& centerLon, int& zoom, const SDL_Rect& mapArea, int mouseX, int mouseY, int newZ) {
  int oldZ = zoom;
  if (newZ == oldZ) return;

  double worldPx, worldPy;
  screenToWorldPx(mapArea, centerLat, centerLon, oldZ, mouseX, mouseY, worldPx, worldPy);

  double anchorLat, anchorLon;
  pixelToLatLon(worldPx, worldPy, oldZ, anchorLat, anchorLon);

  zoom = newZ;

  double ax, ay;
  latLonToPixel(anchorLat, anchorLon, newZ, ax, ay);

  int cx = mapArea.x + mapArea.w/2;
  int cy = mapArea.y + mapArea.h/2;

  double cpx = ax - (double)(mouseX - cx);
  double cpy = ay - (double)(mouseY - cy);

  cpx = wrapPxX(cpx, newZ);
  cpy = clampPxY(cpy, newZ);

  pixelToLatLon(cpx, cpy, newZ, centerLat, centerLon);
}

struct TileKey {
  int z,x,y;
  bool operator==(const TileKey& o) const { return z==o.z && x==o.x && y==o.y; }
};

struct TileKeyHash {
  std::size_t operator()(const TileKey& k) const noexcept {
    return (std::size_t)k.z * 73856093u ^ (std::size_t)k.x * 19349663u ^ (std::size_t)k.y * 83492791u;
  }
};

struct TileTex {
  SDL_Texture* tex = nullptr;
  int w=0,h=0;
};

struct TileCache {
  SDL_Renderer* r = nullptr;
  std::filesystem::path cacheRoot;
  std::unordered_map<TileKey, std::pair<TileTex, std::list<TileKey>::iterator>, TileKeyHash> map;
  std::list<TileKey> lru;
  size_t maxItems = 320;

  std::mutex mtx;
  std::unordered_set<std::size_t> inFlight;

  std::deque<TileKey> pendingLoads;
  std::unordered_set<std::size_t> pendingSet;

  TileCache(SDL_Renderer* rr, const std::filesystem::path& root): r(rr), cacheRoot(root) {
    std::filesystem::create_directories(cacheRoot);
  }

  std::filesystem::path tilePath(int z,int x,int y) {
    return cacheRoot / std::to_string(z) / std::to_string(x) / (std::to_string(y) + ".png");
  }

  std::string tileUrl(int z,int x,int y) {
    return std::string("https://tile.openstreetmap.org/") + std::to_string(z) + "/" + std::to_string(x) + "/" + std::to_string(y) + ".png";
  }

  void touch_nolock(const TileKey& k) {
    auto it = map.find(k);
    if (it == map.end()) return;
    lru.erase(it->second.second);
    lru.push_front(k);
    it->second.second = lru.begin();
  }

  void evictIfNeeded_nolock() {
    while (map.size() > maxItems) {
      TileKey bk = lru.back();
      lru.pop_back();
      auto it = map.find(bk);
      if (it != map.end()) {
        if (it->second.first.tex) SDL_DestroyTexture(it->second.first.tex);
        map.erase(it);
      }
    }
  }

  std::size_t keyHash(int z,int x,int y) const {
    TileKey k{z,x,y};
    TileKeyHash h;
    return h(k);
  }

  bool pathLooksValidPng(const std::filesystem::path& p) {
    std::error_code ec;
    auto sz = std::filesystem::file_size(p, ec);
    if (ec || sz < 200) return false;
    return fileLooksLikePng(p);
  }

  void startDownloadIfNeeded(int z,int x,int y) {
    std::size_t hh = keyHash(z,x,y);
    {
      std::lock_guard<std::mutex> g(mtx);
      if (inFlight.find(hh) != inFlight.end()) return;
      inFlight.insert(hh);
    }

    auto p = tilePath(z,x,y);
    auto url = tileUrl(z,x,y);

    std::thread([this, hh, p, url](){
      httpGetToFile(url, p);
      std::lock_guard<std::mutex> g(mtx);
      inFlight.erase(hh);
    }).detach();
  }

  void enqueueLoadIfNeeded(int z,int x,int y) {
    TileKey k{z,x,y};
    std::size_t hh = keyHash(z,x,y);
    std::lock_guard<std::mutex> g(mtx);
    if (pendingSet.find(hh) != pendingSet.end()) return;
    pendingSet.insert(hh);
    pendingLoads.push_back(k);
  }

  void pump(int budget) {
    while (budget > 0) {
      TileKey k;
      {
        std::lock_guard<std::mutex> g(mtx);
        if (pendingLoads.empty()) break;
        k = pendingLoads.front();
        pendingLoads.pop_front();
        pendingSet.erase(keyHash(k.z,k.x,k.y));
      }

      auto p = tilePath(k.z,k.x,k.y);
      if (!std::filesystem::exists(p)) continue;
      if (!pathLooksValidPng(p)) {
        std::error_code ec;
        std::filesystem::remove(p, ec);
        startDownloadIfNeeded(k.z,k.x,k.y);
        continue;
      }

      SDL_Surface* s = IMG_Load(p.string().c_str());
      if (!s) {
        std::error_code ec;
        std::filesystem::remove(p, ec);
        startDownloadIfNeeded(k.z,k.x,k.y);
        continue;
      }

      SDL_Texture* t = SDL_CreateTextureFromSurface(r, s);
      TileTex tt{t, s->w, s->h};
      SDL_FreeSurface(s);
      if (!tt.tex) continue;

      {
        std::lock_guard<std::mutex> g(mtx);
        auto it = map.find(k);
        if (it != map.end()) {
          if (it->second.first.tex) SDL_DestroyTexture(it->second.first.tex);
          lru.erase(it->second.second);
          map.erase(it);
        }
        lru.push_front(k);
        map.emplace(k, std::make_pair(tt, lru.begin()));
        evictIfNeeded_nolock();
      }

      budget--;
    }
  }

  TileTex get(int z,int x,int y) {
    TileKey k{z,x,y};

    {
      std::lock_guard<std::mutex> g(mtx);
      auto it = map.find(k);
      if (it != map.end()) { touch_nolock(k); return it->second.first; }
    }

    auto p = tilePath(z,x,y);
    if (!std::filesystem::exists(p)) {
      startDownloadIfNeeded(z,x,y);
      return TileTex{nullptr,0,0};
    }

    if (!pathLooksValidPng(p)) {
      std::error_code ec;
      std::filesystem::remove(p, ec);
      startDownloadIfNeeded(z,x,y);
      return TileTex{nullptr,0,0};
    }

    enqueueLoadIfNeeded(z,x,y);
    return TileTex{nullptr,0,0};
  }
};


enum class TopTab { STAT, INV, DATA, MAP, RADIO };

static const char* topTabName(TopTab t) {
  switch(t){
    case TopTab::STAT:  return "STAT";
    case TopTab::INV:   return "INV";
    case TopTab::DATA:  return "DATA";
    case TopTab::MAP:   return "MAP";
    case TopTab::RADIO: return "RADIO";
  }
  return "";
}

static std::vector<std::string> subTabsFor(TopTab t) {
  if (t == TopTab::STAT)  return {"STATUS","S.P.E.C.I.A.L","SKILLS","PERKS","GENERAL"};
  if (t == TopTab::INV)   return {"WEAPONS","APPAREL","AID","MISC","JUNK"};
  if (t == TopTab::DATA)  return {"CLOCK","QUESTS","WORKSHOPS","STATS","SETTINGS"};
  if (t == TopTab::MAP)   return {"WORLD","LOCAL"};
  if (t == TopTab::RADIO) return {"STATIONS"};
  return {};
}

struct Bookmark {
  std::string name;
  double lat = 0;
  double lon = 0;
  int zoomWorld = 5;
  int zoomLocal = 15;
  std::string iconRel;
};

static std::filesystem::path bookmarksPath() {
  return assetsRootDir() / "cache" / "places.txt";
}

static void loadBookmarks(std::vector<Bookmark>& out) {
  out.clear();
  auto p = bookmarksPath();
  std::ifstream f(p);
  if (!f.good()) return;

  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::vector<std::string> cols;
    std::string token;
    while (std::getline(ss, token, '|')) cols.push_back(token);
    if (cols.size() < 5) continue;

    Bookmark b;
    b.name = cols[0];
    b.lat = std::atof(cols[1].c_str());
    b.lon = std::atof(cols[2].c_str());
    b.zoomWorld = std::atoi(cols[3].c_str());
    b.zoomLocal = std::atoi(cols[4].c_str());
    if (cols.size() >= 6) b.iconRel = cols[5];
    out.push_back(b);
  }
}

static void saveBookmarks(const std::vector<Bookmark>& v) {
  auto p = bookmarksPath();
  std::filesystem::create_directories(p.parent_path());
  std::ofstream f(p, std::ios::trunc);
  if (!f.good()) return;
  for (auto& b : v) {
    f << b.name << "|" << b.lat << "|" << b.lon << "|" << b.zoomWorld << "|" << b.zoomLocal << "|" << b.iconRel << "\n";
  }
}

static std::string fileStem(const std::string& rel) {
  std::filesystem::path p(rel);
  return p.stem().string();
}

static std::vector<std::string> listPinIconsRel() {
  std::vector<std::string> out;
  auto root = assetsRootDir();
  std::vector<std::filesystem::path> dirs = {
    root / "images" / "pins",
    root / "images",
    root / "icons",
    root
  };
  auto isImg = [](const std::filesystem::path& p){
    auto e = p.extension().string();
    std::transform(e.begin(), e.end(), e.begin(), ::tolower);
    return e == ".png" || e == ".jpg" || e == ".jpeg";
  };
  for (auto& d : dirs) {
    if (!std::filesystem::exists(d) || !std::filesystem::is_directory(d)) continue;
    for (auto& it : std::filesystem::directory_iterator(d)) {
      if (!it.is_regular_file()) continue;
      auto p = it.path();
      if (!isImg(p)) continue;
      std::string fn = p.filename().string();
      if (!(fn.rfind("pin",0)==0 || fn.rfind("marker",0)==0 || fn.rfind("icon",0)==0)) continue;

      std::filesystem::path rel;
      auto assets = assetsRootDir();
      std::error_code ec;
      rel = std::filesystem::relative(p, assets, ec);
      if (ec) continue;
      out.push_back(rel.generic_string());
    }
    if (!out.empty()) break;
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}


struct RadioStation {
  std::string name;
  std::vector<std::string> tracks;
  float freq = 0.0f;
};

static bool isAudioFile(const std::filesystem::path& p) {
  if (!std::filesystem::is_regular_file(p)) return false;
  std::string e = p.extension().string();
  std::transform(e.begin(), e.end(), e.begin(), [](unsigned char c){ return (char)std::tolower(c); });
  return e == ".mp3" || e == ".ogg" || e == ".wav" || e == ".flac" || e == ".mod" || e == ".xm";
}

static std::vector<std::string> listAudioFiles(const std::filesystem::path& dir) {
  std::vector<std::string> out;
  if (!std::filesystem::exists(dir) || !std::filesystem::is_directory(dir)) return out;
  for (auto& it : std::filesystem::directory_iterator(dir)) {
    if (!isAudioFile(it.path())) continue;
    out.push_back(it.path().string());
  }
  std::sort(out.begin(), out.end());
  return out;
}

static std::vector<RadioStation> buildRadioStations() {
  std::vector<RadioStation> st;
  auto root = assetsRootDir() / "sounds" / "radio" / "music";
  if (!std::filesystem::exists(root) || !std::filesystem::is_directory(root)) return st;

  std::vector<std::filesystem::path> dirs;
  for (auto& it : std::filesystem::directory_iterator(root)) {
    if (!it.is_directory()) continue;
    dirs.push_back(it.path());
  }
  std::sort(dirs.begin(), dirs.end(), [](const auto& a, const auto& b){
    return a.filename().string() < b.filename().string();
  });

  float f0 = 89.0f;
  float f1 = 107.0f;
  int n = (int)dirs.size();
  for (int i=0;i<n;i++){
    RadioStation rs;
    rs.name = dirs[i].filename().string();
    rs.tracks = listAudioFiles(dirs[i]);
    float t = (n <= 1) ? 0.5f : (float)i / (float)(n - 1);
    rs.freq = f0 + (f1 - f0) * t;
    if (!rs.tracks.empty()) st.push_back(std::move(rs));
  }
  return st;
}

static std::string basenameNoExt(const std::string& p) {
  std::filesystem::path pp(p);
  return pp.stem().string();
}

static std::atomic<bool> gRadioMusicFinished{false};
static void radioMusicFinishedCb() { gRadioMusicFinished.store(true); }

struct RadioRuntime {
  std::vector<RadioStation> stations;
  int stationIdx = 0;
  int trackIdx = 0;
  int playingStationIdx = -1;
  std::vector<int> trackByStation;
  std::vector<double> posByStation;
  std::vector<int> bgTrackByStation;
  std::vector<double> bgPosByStation;
  std::vector<std::vector<double>> durByStationTrack;
  uint32_t lastTick = 0;
  float tuneFreq = 90.0f;
  bool locked = false;
  bool hadLock = false;
  bool power = false;
  bool paused = false;
  int musicVol = 110;
  Mix_Music* music = nullptr;
  int staticChan = -1;
  int tapeChan = -1;
  Mix_Chunk* sStaticLp = nullptr;
  Mix_Chunk* sStaticTape = nullptr;
  Mix_Chunk* sFound = nullptr;
  Mix_Chunk* sLost = nullptr;
  std::string nowPlaying;

  void initStations() {
    trackByStation.assign(stations.size(), 0);
    posByStation.assign(stations.size(), 0.0);
    bgTrackByStation.assign(stations.size(), 0);
    bgPosByStation.assign(stations.size(), 0.0);
    durByStationTrack.clear();
    durByStationTrack.resize(stations.size());
    for (size_t i=0;i<stations.size();i++) durByStationTrack[i].assign(stations[i].tracks.size(), -1.0);
    if (!stations.empty()) {
      stationIdx = std::max(0, std::min((int)stations.size()-1, stationIdx));
      if (playingStationIdx < 0) playingStationIdx = stationIdx;
    }
    lastTick = SDL_GetTicks();
  }

  void stopMusic() {
    Mix_HookMusicFinished(nullptr);
    if (Mix_PlayingMusic()) Mix_HaltMusic();
    if (music) { Mix_FreeMusic(music); music = nullptr; }
  }

  void pauseMusicStream() {
    if (Mix_PlayingMusic()) Mix_PauseMusic();
  }

  void resumeMusicStream() {
    if (Mix_PausedMusic()) Mix_ResumeMusic();
  }

  void stopStatic() {
    if (staticChan >= 0) Mix_HaltChannel(staticChan);
    staticChan = -1;
  }

  void startStaticLoop(int vol) {
    if (!sStaticLp) return;
    if (staticChan < 0 || !Mix_Playing(staticChan)) staticChan = Mix_PlayChannel(-1, sStaticLp, -1);
    if (staticChan >= 0) Mix_Volume(staticChan, std::max(0, std::min(128, vol)));
  }

  void playTape() {
    if (!sStaticTape) return;
    tapeChan = Mix_PlayChannel(-1, sStaticTape, 0);
  }

  void playFound() { if (sFound) Mix_PlayChannel(-1, sFound, 0); }
  void playLost()  { if (sLost)  Mix_PlayChannel(-1, sLost, 0); }

  float curStationFreq() const {
    if (stations.empty()) return 95.0f;
    int i = std::max(0, std::min((int)stations.size()-1, stationIdx));
    return stations[i].freq;
  }

  const RadioStation* curStation() const {
    if (stations.empty()) return nullptr;
    int i = std::max(0, std::min((int)stations.size()-1, stationIdx));
    return &stations[i];
  }

  void syncPlayingToBg() {
#if defined(SDL_MIXER_VERSION_ATLEAST) && SDL_MIXER_VERSION_ATLEAST(2,6,0)
    if (playingStationIdx >= 0 && playingStationIdx < (int)stations.size() && music) {
      double p = Mix_GetMusicPosition(music);
      if (p >= 0.0) bgPosByStation[playingStationIdx] = p;
    }
#endif
    if (playingStationIdx >= 0 && playingStationIdx < (int)stations.size()) {
      bgTrackByStation[playingStationIdx] = trackIdx;
      trackByStation[playingStationIdx] = trackIdx;
      posByStation[playingStationIdx] = bgPosByStation[playingStationIdx];
    }
  }

  void loadStationNow(int idx) {
    if (stations.empty()) return;
    idx = std::max(0, std::min((int)stations.size()-1, idx));
    const auto& st = stations[idx];
    if (st.tracks.empty()) return;

    int tIdx = bgTrackByStation[idx];
    if (tIdx < 0) tIdx = 0;
    if (tIdx >= (int)st.tracks.size()) tIdx = 0;
    trackIdx = tIdx;

    stopMusic();
    const std::string& p = st.tracks[trackIdx];
    music = Mix_LoadMUS(p.c_str());
    if (!music) return;

    Mix_VolumeMusic(std::max(0, std::min(128, musicVol)));
    gRadioMusicFinished.store(false);
    Mix_HookMusicFinished(radioMusicFinishedCb);
    Mix_PlayMusic(music, 0);

#if defined(SDL_MIXER_VERSION_ATLEAST) && SDL_MIXER_VERSION_ATLEAST(2,0,0)
    double seek = bgPosByStation[idx];
    if (seek > 0.01) Mix_SetMusicPosition(seek);
#endif

    nowPlaying = basenameNoExt(p);
    playingStationIdx = idx;
    trackByStation[idx] = trackIdx;
    posByStation[idx] = bgPosByStation[idx];
  }

  void advanceBgBy(double dt) {
    if (stations.empty()) return;
    for (int i=0;i<(int)stations.size();i++) {
      if (i == playingStationIdx) continue;
      int tIdx = bgTrackByStation[i];
      if (tIdx < 0) tIdx = 0;
      if (tIdx >= (int)durByStationTrack[i].size()) tIdx = 0;
      double dur = durByStationTrack[i].empty() ? -1.0 : durByStationTrack[i][tIdx];
      if (dur <= 0.0) continue;
      double p = bgPosByStation[i] + dt;
      while (p >= dur && dur > 0.0 && !stations[i].tracks.empty()) {
        p -= dur;
        tIdx++;
        if (tIdx >= (int)stations[i].tracks.size()) tIdx = 0;
        bgTrackByStation[i] = tIdx;
        trackByStation[i] = tIdx;
        dur = durByStationTrack[i][tIdx];
        if (dur <= 0.0) { p = 0.0; break; }
      }
      bgPosByStation[i] = p;
      posByStation[i] = p;
    }
  }

  void nextTrackPlaying(bool recordDur) {
    if (playingStationIdx < 0 || playingStationIdx >= (int)stations.size()) return;
    auto& st = stations[playingStationIdx];
    if (st.tracks.empty()) return;

    int curT = trackIdx;
    if (recordDur && playingStationIdx >= 0 && playingStationIdx < (int)durByStationTrack.size()) {
      if (curT >= 0 && curT < (int)durByStationTrack[playingStationIdx].size()) {
        double d = bgPosByStation[playingStationIdx];
        if (d > 0.1 && durByStationTrack[playingStationIdx][curT] < 0.0) durByStationTrack[playingStationIdx][curT] = d;
      }
    }

    int nt = curT + 1;
    if (nt >= (int)st.tracks.size()) nt = 0;
    bgTrackByStation[playingStationIdx] = nt;
    trackByStation[playingStationIdx] = nt;
    bgPosByStation[playingStationIdx] = 0.0;
    posByStation[playingStationIdx] = 0.0;
    loadStationNow(playingStationIdx);
  }

  void skipIfCustomSelected() {
    auto st = curStation();
    if (!st) return;
    std::string n = st->name;
    std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c){ return (char)std::tolower(c); });
    if (n != "custom") return;
    if (stationIdx < 0 || stationIdx >= (int)stations.size()) return;
    bgTrackByStation[stationIdx] = bgTrackByStation[stationIdx] + 1;
    if (!stations[stationIdx].tracks.empty() && bgTrackByStation[stationIdx] >= (int)stations[stationIdx].tracks.size()) bgTrackByStation[stationIdx] = 0;
    trackByStation[stationIdx] = bgTrackByStation[stationIdx];
    bgPosByStation[stationIdx] = 0.0;
    posByStation[stationIdx] = 0.0;
    if (playingStationIdx == stationIdx) loadStationNow(stationIdx);
  }

  void selectStation(int idx) {
    if (stations.empty()) return;
    syncPlayingToBg();
    stationIdx = std::max(0, std::min((int)stations.size()-1, idx));
    if (power && !paused) {
      playingStationIdx = stationIdx;
      loadStationNow(playingStationIdx);
      playTape();
    } else {
      nowPlaying.clear();
    }
    hadLock = locked;
    locked = false;
  }

  void update(float) {
    if (stations.empty()) return;

    uint32_t now = SDL_GetTicks();
    double dt = 0.0;
    if (lastTick != 0 && now >= lastTick) dt = (double)(now - lastTick) / 1000.0;
    lastTick = now;

    if (!power) {
      stopStatic();
      pauseMusicStream();
      return;
    }

    if (paused) {
      stopStatic();
      pauseMusicStream();
      return;
    }

    resumeMusicStream();

    if (playingStationIdx != stationIdx) {
      syncPlayingToBg();
      playingStationIdx = stationIdx;
      loadStationNow(playingStationIdx);
    } else {
      if (!Mix_PlayingMusic() && !Mix_PausedMusic()) loadStationNow(playingStationIdx);
    }

    if (playingStationIdx >= 0 && playingStationIdx < (int)stations.size()) {
      bgPosByStation[playingStationIdx] += dt;
      posByStation[playingStationIdx] = bgPosByStation[playingStationIdx];
    }
    advanceBgBy(dt);

    float target = curStationFreq();
    float diff = std::fabs(tuneFreq - target);
    float lockTh = 0.25f;
    float fullTh = 1.50f;

    locked = diff <= lockTh;
    float sig = 1.0f - clamp01((diff - lockTh) / (fullTh - lockTh));
    int mv  = (int)std::lround((float)musicVol * sig);

    if (locked) {
      stopStatic();
      Mix_VolumeMusic(std::max(0, std::min(128, mv)));
    } else {
      Mix_VolumeMusic(0);
      startStaticLoop(std::max(0, std::min(128, musicVol)));
    }

    if (locked && !hadLock) playFound();
    if (!locked && hadLock) playLost();
    hadLock = locked;

    if (gRadioMusicFinished.exchange(false)) nextTrackPlaying(true);
    else if (Mix_PlayingMusic() == 0 && !Mix_PausedMusic()) nextTrackPlaying(false);

    syncPlayingToBg();
  }
};



enum class MapBtnId { ZPLUS, ZMINUS, HOME, SAVE, GO, DEL, CANCEL, CONFIRM };

struct Btn {
  SDL_Rect rc;
  MapBtnId id;
};

static std::vector<Btn> mapBtns;

static void computeLayoutRects(int W,int H, SDL_Rect& screen, SDL_Rect& content, SDL_Rect& left, SDL_Rect& right, SDL_Rect& mapArea, SDL_Rect& bottom) {
  int padX = std::max(44, W / 18);
  int padY = std::max(34, H / 18);

  screen = { padX, padY, W - padX*2, H - padY*2 };

  int topY = screen.y + 10;
  int subY = topY + 28;

  int bottomH = (H >= 720) ? 44 : 30;
  bottom = { screen.x + 10, screen.y + screen.h - (bottomH + 10), screen.w - 20, bottomH };

  int contentTop = subY + 28;
  int contentBottom = bottom.y - 10;

  content = { screen.x + 10, contentTop, screen.w - 20, std::max(40, contentBottom - contentTop) };

  left = { content.x, content.y, (int)(content.w * 0.62f), content.h };
  right = { left.x + left.w + 10, left.y, content.x + content.w - (left.x + left.w + 10), content.h };

  mapArea = { left.x + 12, left.y + 44, left.w - 24, std::max(40, left.h - 56) };
}

static SDL_Rect radioTuneRect(const SDL_Rect& right) {
  return SDL_Rect{ right.x + 18, right.y + 92, right.w - 36, 18 };
}

static void radioDetailButtons(const SDL_Rect& right, SDL_Rect& playBtn, SDL_Rect& volBtn) {
  int pad = 18;
  int gap = 10;
  int h = 28;
  int y = right.y + right.h - (h + 14);
  int w = (right.w - pad*2 - gap) / 2;
  playBtn = SDL_Rect{ right.x + pad, y, w, h };
  volBtn  = SDL_Rect{ right.x + pad + w + gap, y, w, h };
}

static void radioDetailButtonsEx(const SDL_Rect& right, bool showSkip, SDL_Rect& playBtn, SDL_Rect& volBtn, SDL_Rect& skipBtn) {
  int pad = 18;
  int gap = 10;
  int h = 28;
  int y = right.y + right.h - (h + 14);
  if (!showSkip) {
    int w = (right.w - pad*2 - gap) / 2;
    playBtn = SDL_Rect{ right.x + pad, y, w, h };
    volBtn  = SDL_Rect{ right.x + pad + w + gap, y, w, h };
    skipBtn = SDL_Rect{0,0,0,0};
    return;
  }
  int w = (right.w - pad*2 - gap*2) / 3;
  playBtn = SDL_Rect{ right.x + pad, y, w, h };
  skipBtn = SDL_Rect{ right.x + pad + w + gap, y, w, h };
  volBtn  = SDL_Rect{ right.x + pad + (w + gap)*2, y, w, h };
}

static bool isCustomStationName(const std::string& s) {
  std::string n = s;
  std::transform(n.begin(), n.end(), n.begin(), [](unsigned char c){ return (char)std::tolower(c); });
  return n == "custom";
}

static void buildTopSubHitboxes(TTF_Font* fontM, TTF_Font* fontS, const SDL_Rect& screen, TopTab tab, int subTab, std::vector<SDL_Rect>& topHit, std::vector<SDL_Rect>& subHit) {
  topHit.assign(5, SDL_Rect{0,0,0,0});
  int topY = screen.y + 10;
  int x = screen.x + 14;
  for (int i=0; i<5; i++) {
    TopTab t = (TopTab)i;
    bool active = (tab == t);
    std::string label = topTabName(t);
    std::string shown = active ? ("[" + label + "]") : label;
    int w = textW(fontM, shown);
    SDL_Rect hit { x - 6, topY - 4, w + 12, 28 };
    topHit[i] = hit;
    x += textW(fontM, "[" + label + "]") + 18;
  }

  auto subs = subTabsFor(tab);
  subHit.assign(subs.size(), SDL_Rect{0,0,0,0});
  int subY = topY + 28;
  int sx = screen.x + 14;
  for (int i=0; i<(int)subs.size(); i++) {
    bool active = (subTab == i);
    std::string shown = active ? ("[" + subs[i] + "]") : subs[i];
    int w = textW(fontS, shown);
    SDL_Rect hit { sx - 6, subY - 3, w + 12, 22 };
    subHit[i] = hit;
    sx += textW(fontS, "[" + subs[i] + "]") + 12;
  }
}

static int clampZoomForMode(int subTab, int z){
  if (subTab == 0) return std::max(2, std::min(7, z));
  return std::max(10, std::min(18, z));
}

struct UIState {
  TopTab tab = TopTab::STAT;
  int subTab = 0;

  int listIndex = 0;
  int listTarget = 0;
  float hiY = 0.0f;

  int level = 14;
  int hp = 86, hpMax = 100;
  int ap = 62, apMax = 100;
  int xp = 3800, xpMax = 5000;

  int carry = 171, carryMax = 245;
  int caps = 639;
  int ammo = 8;

  std::vector<std::string> invWeapons = {"10mm Pistol","Pipe Revolver Rifle","Short Laser Musket","Shock Baton","Molotov Cocktail (4)","Fragmentation Grenade (6)","Short Double-Barrel Shotgun"};
  std::vector<std::string> invApparel = {"Vault Suit","Leather Left Arm","Gas Mask","Road Leathers","Tattered Dress"};
  std::vector<std::string> invAid = {"Stimpak (7)","RadAway (3)","Purified Water (10)","Mutt Chops (2)","Jet (1)"};
  std::vector<std::string> invMisc = {"Bobby Pin (34)","Stealth Boy","Holotape (2)","Fuses (3)"};
  std::vector<std::string> invJunk = {"Wonderglue (2)","Duct Tape","Screwdriver","Aluminum Can (6)","Circuitry"};

  std::vector<std::string> dataQuests = {"When Freedom Calls","The First Step","Jewel of the Commonwealth","Reunions","The Molecular Level"};
  std::vector<std::string> dataWorkshops = {"Sanctuary","Red Rocket Truck Stop","Abernathy Farm"};
  std::vector<std::string> dataStats = {"Locations Discovered: 18","People Killed: 42","Locks Picked: 7","Hacks: 5","Caps Found: 639"};
  std::vector<std::string> dataSettings = {"Display","Audio","Controls","Gameplay"};

  std::vector<std::string> perks = {"Gunslinger","Rifleman","Toughness","Lone Wanderer","Cap Collector","Local Leader"};

  double homeLat = 25.6866;
  double homeLon = -100.3161;

  double worldLat = 25.6866;
  double worldLon = -100.3161;

  double localLat = 25.6866;
  double localLon = -100.3161;

  int zoomWorld = 5;
  int zoomLocal = 15;

  std::vector<Bookmark> bookmarks;
  std::vector<std::string> bookmarksNames;
  int bmIndex = 0;
  int bmTarget = 0;
  float bmHiY = 0.0f;

  std::vector<std::string> pinIcons;
  int pinPickIndex = 0;
  bool pinPickerOpen = false;
  double pinLat = 0, pinLon = 0;
  int pinZW = 5, pinZL = 15;
  std::string pinName;

  std::vector<std::string> radioStations;
  int radioStationIndex = 0;
  float radioTuneFreq = 95.0f;
  std::string radioNowPlaying;
  bool radioLocked = false;
  bool radioPower = false;
  int radioVol = 110;

  bool mapDragging = false;
  int dragLastX = 0;
  int dragLastY = 0;

  float pinchAccum = 0.0f;

  int clockSec = -1;
  std::string clockHHMM;
  std::string clockFull;
  std::string clockTZ;
};

static SDL_Rect gRadioPlayBtn{0,0,0,0};
static SDL_Rect gRadioVolBtn{0,0,0,0};
static SDL_Rect gRadioSkipBtn{0,0,0,0};
static bool gRadioShowSkip = false;


static double& curLat(UIState& ui) { return (ui.subTab == 0) ? ui.worldLat : ui.localLat; }
static double& curLon(UIState& ui) { return (ui.subTab == 0) ? ui.worldLon : ui.localLon; }
static int& curZoom(UIState& ui)   { return (ui.subTab == 0) ? ui.zoomWorld : ui.zoomLocal; }
static void updateClock(UIState& ui) {
  std::time_t now = std::time(nullptr);
  std::tm tmv{};
#if defined(_WIN32)
  localtime_s(&tmv, &now);
#else
  localtime_r(&now, &tmv);
#endif
  if (ui.clockSec == tmv.tm_sec && !ui.clockHHMM.empty()) return;
  ui.clockSec = tmv.tm_sec;

  char buf[64];
  std::snprintf(buf, sizeof(buf), "%02d:%02d", tmv.tm_hour, tmv.tm_min);
  ui.clockHHMM = buf;

  std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M", &tmv);
  ui.clockFull = buf;

  std::strftime(buf, sizeof(buf), "%Z", &tmv);
  ui.clockTZ = buf;
  if (ui.clockTZ.empty()) ui.clockTZ = "LOCAL";
}


static std::vector<std::string>& currentList(UIState& s) {
  static std::vector<std::string> empty;
  if (s.tab == TopTab::INV) {
    if (s.subTab == 0) return s.invWeapons;
    if (s.subTab == 1) return s.invApparel;
    if (s.subTab == 2) return s.invAid;
    if (s.subTab == 3) return s.invMisc;
    if (s.subTab == 4) return s.invJunk;
  }
  if (s.tab == TopTab::DATA) {
    if (s.subTab == 1) return s.dataQuests;
    if (s.subTab == 2) return s.dataWorkshops;
    if (s.subTab == 3) return s.dataStats;
    if (s.subTab == 4) return s.dataSettings;
  }
  if (s.tab == TopTab::STAT) {
    if (s.subTab == 3) return s.perks;
  }
  if (s.tab == TopTab::RADIO) {
    return s.radioStations;
  }
  return empty;
}

static void refreshBookmarkNames(UIState& s) {
  s.bookmarksNames.clear();
  for (auto& b : s.bookmarks) s.bookmarksNames.push_back(b.name);
  if (s.bookmarksNames.empty()) { s.bmIndex = 0; s.bmTarget = 0; s.bmHiY = 0; }
  else {
    s.bmTarget = std::max(0, std::min((int)s.bookmarksNames.size()-1, s.bmTarget));
    s.bmIndex  = std::max(0, std::min((int)s.bookmarksNames.size()-1, s.bmIndex));
  }
}

static void clampSub(UIState& s) {
  auto subs = subTabsFor(s.tab);
  if (subs.empty()) s.subTab = 0;
  else s.subTab = std::max(0, std::min((int)subs.size()-1, s.subTab));
}

static void clampList(UIState& s) {
  auto& list = currentList(s);
  if (list.empty()) { s.listIndex = 0; s.listTarget = 0; return; }
  s.listTarget = std::max(0, std::min((int)list.size()-1, s.listTarget));
  s.listIndex = std::max(0, std::min((int)list.size()-1, s.listIndex));
}

static void clampBookmarks(UIState& s) {
  if (s.bookmarksNames.empty()) { s.bmIndex = 0; s.bmTarget = 0; return; }
  s.bmTarget = std::max(0, std::min((int)s.bookmarksNames.size()-1, s.bmTarget));
  s.bmIndex  = std::max(0, std::min((int)s.bookmarksNames.size()-1, s.bmIndex));
}

static void setTab(UIState& s, TopTab t) {
  s.tab = t;
  s.subTab = 0;
  s.listIndex = 0;
  s.listTarget = 0;
  s.hiY = 0.0f;
  s.mapDragging = false;
  s.pinPickerOpen = false;
}

static void setSub(UIState& s, int idx) {
  auto subs = subTabsFor(s.tab);
  if (subs.empty()) { s.subTab = 0; return; }
  s.subTab = std::max(0, std::min((int)subs.size()-1, idx));
  s.listIndex = 0;
  s.listTarget = 0;
  s.hiY = 0.0f;
  s.mapDragging = false;
  s.pinPickerOpen = false;
}

static void nextSub(UIState& s, int dir) {
  auto subs = subTabsFor(s.tab);
  if (subs.empty()) return;
  s.subTab += dir;
  if (s.subTab < 0) s.subTab = (int)subs.size() - 1;
  if (s.subTab >= (int)subs.size()) s.subTab = 0;
  s.listIndex = 0;
  s.listTarget = 0;
  s.hiY = 0.0f;
  s.mapDragging = false;
  s.pinPickerOpen = false;
}

static void drawList(SDL_Renderer* r, TTF_Font* font, const SDL_Rect& panel, const std::vector<std::string>& list, int selectedIndex, float hiY, SDL_Color on) {
  int rowH = 26;
  int visible = std::max(1, panel.h / rowH);
  int center = visible / 2;
  int start = std::max(0, selectedIndex - center);
  start = std::min(start, std::max(0, (int)list.size() - visible));

  SDL_Rect hi{panel.x + 10, (int)hiY, panel.w - 20, 18};
  fillRect(r, hi, SDL_Color{on.r,on.g,on.b,190});

  for (int i=0; i<visible; i++) {
    int idx = start + i;
    if (idx >= (int)list.size()) break;
    int y = panel.y + 8 + i*rowH;
    bool sel = (idx == selectedIndex);
    SDL_Color tc = sel ? SDL_Color{0,0,0,255} : SDL_Color{on.r,on.g,on.b,220};
    drawText(r, font, list[idx], panel.x + 16, y, tc);
  }

  drawRect(r, panel, SDL_Color{on.r,on.g,on.b,65});
}

static void drawMapTiles(SDL_Renderer* r, TileCache& cache, const SDL_Rect& mapArea, double centerLat, double centerLon, int zoom, SDL_Color pipGreen) {
  const int tileSize = 256;

  int cx, cy;
  double fx, fy;
  latLonToTile(centerLat, centerLon, zoom, cx, cy, fx, fy);

  double centerPx = fx * tileSize;
  double centerPy = fy * tileSize;

  int tilesX = (mapArea.w + tileSize - 1) / tileSize + 3;
  int tilesY = (mapArea.h + tileSize - 1) / tileSize + 3;

  int startX = cx - tilesX/2;
  int startY = cy - tilesY/2;

  int offsetX = mapArea.x + mapArea.w/2 - tileSize/2 - (int)std::round(centerPx);
  int offsetY = mapArea.y + mapArea.h/2 - tileSize/2 - (int)std::round(centerPy);

  SDL_RenderSetClipRect(r, &mapArea);

  for (int iy=0; iy<tilesY; iy++) {
    int ty = clampTileY(startY + iy, zoom);
    for (int ix=0; ix<tilesX; ix++) {
      int tx = wrapTileX(startX + ix, zoom);

      int dx = offsetX + (startX + ix - cx) * tileSize;
      int dy = offsetY + (startY + iy - cy) * tileSize;

      SDL_Rect dst{ dx, dy, tileSize, tileSize };
      SDL_Rect clip = dst;

      if (clip.x < mapArea.x) { clip.w -= (mapArea.x - clip.x); clip.x = mapArea.x; }
      if (clip.y < mapArea.y) { clip.h -= (mapArea.y - clip.y); clip.y = mapArea.y; }
      if (clip.x + clip.w > mapArea.x + mapArea.w) clip.w = (mapArea.x + mapArea.w) - clip.x;
      if (clip.y + clip.h > mapArea.y + mapArea.h) clip.h = (mapArea.y + mapArea.h) - clip.y;
      if (clip.w <= 0 || clip.h <= 0) continue;

      TileTex tt = cache.get(zoom, tx, ty);
      if (!tt.tex) {
        // Fallback: try parent tile (zoom-1) and scale its quadrant. This reduces visible "holes"
        // when higher-zoom tiles are still downloading or are rate-limited.
        bool drewFallback = false;
        if (zoom > 0) {
          int pz = zoom - 1;
          int ptx = wrapTileX(tx / 2, pz);
          int pty = clampTileY(ty / 2, pz);
          TileTex pt = cache.get(pz, ptx, pty);
          if (pt.tex) {
            SDL_Rect srcChild{ clip.x - dst.x, clip.y - dst.y, clip.w, clip.h };
            int qx = tx & 1;
            int qy = ty & 1;
            int psx = qx * 128 + (srcChild.x / 2);
            int psy = qy * 128 + (srcChild.y / 2);
            int psw = std::max(1, (srcChild.w + 1) / 2);
            int psh = std::max(1, (srcChild.h + 1) / 2);
            SDL_Rect srcP{ psx, psy, psw, psh };

            SDL_SetTextureColorMod(pt.tex, pipGreen.r, pipGreen.g, pipGreen.b);
            SDL_SetTextureAlphaMod(pt.tex, 200);
            SDL_SetTextureBlendMode(pt.tex, SDL_BLENDMODE_BLEND);
            SDL_RenderCopy(r, pt.tex, &srcP, &clip);
            drewFallback = true;
          }
        }

        if (!drewFallback) {
          SDL_Rect ph = clip;
          fillRect(r, ph, SDL_Color{0,0,0,90});
          drawRect(r, ph, SDL_Color{pipGreen.r,pipGreen.g,pipGreen.b,110});
        }
        continue;
      }

      SDL_Rect src{ clip.x - dst.x, clip.y - dst.y, clip.w, clip.h };

      SDL_SetTextureColorMod(tt.tex, pipGreen.r, pipGreen.g, pipGreen.b);
      SDL_SetTextureAlphaMod(tt.tex, 220);
      SDL_SetTextureBlendMode(tt.tex, SDL_BLENDMODE_BLEND);

      SDL_RenderCopy(r, tt.tex, &src, &clip);
    }
  }

  SDL_RenderSetClipRect(r, nullptr);
}

static void buildMapButtons(const UIState& ui, const SDL_Rect& right, std::vector<Btn>& outBtns) {
  outBtns.clear();
  int bx = right.x + 12;
  int by = right.y + 46;
  int bw = right.w - 24;

  if (!ui.pinPickerOpen) {
    SDL_Rect bZoomP{ bx, by, bw, 22 };
    SDL_Rect bZoomM{ bx, by + 24, bw, 22 };
    SDL_Rect bHome { bx, by + 48, bw, 22 };
    SDL_Rect bSave { bx, by + 72, bw, 22 };
    SDL_Rect bGo   { bx, by + 96, bw, 22 };
    SDL_Rect bDel  { bx, by + 120, bw, 22 };

    outBtns.push_back({bZoomP, MapBtnId::ZPLUS});
    outBtns.push_back({bZoomM, MapBtnId::ZMINUS});
    outBtns.push_back({bHome,  MapBtnId::HOME});
    outBtns.push_back({bSave,  MapBtnId::SAVE});
    outBtns.push_back({bGo,    MapBtnId::GO});
    outBtns.push_back({bDel,   MapBtnId::DEL});
  } else {
    SDL_Rect bConfirm{ bx, by, bw, 22 };
    SDL_Rect bCancel { bx, by + 24, bw, 22 };
    outBtns.push_back({bConfirm, MapBtnId::CONFIRM});
    outBtns.push_back({bCancel,  MapBtnId::CANCEL});
  }
}

static bool hitButtons(const std::vector<Btn>& btns, int mx, int my, MapBtnId& outId) {
  for (auto& b : btns) {
    if (ptIn(b.rc, mx, my)) { outId = b.id; return true; }
  }
  return false;
}

static void pinPickerOpen(UIState& ui) {
  ui.pinPickerOpen = true;
  ui.pinPickIndex = 0;
  ui.pinLat = curLat(ui);
  ui.pinLon = curLon(ui);
  ui.pinZW = ui.zoomWorld;
  ui.pinZL = ui.zoomLocal;
  ui.pinName = "PIN " + std::to_string((int)ui.bookmarks.size() + 1);
  if (ui.pinIcons.empty()) ui.pinIcons = listPinIconsRel();
  if (ui.pinPickIndex < 0) ui.pinPickIndex = 0;
  if (!ui.pinIcons.empty()) ui.pinPickIndex = std::min(ui.pinPickIndex, (int)ui.pinIcons.size()-1);
}

static void pinPickerConfirm(UIState& ui) {
  Bookmark b;
  b.lat = ui.pinLat;
  b.lon = ui.pinLon;
  b.zoomWorld = ui.pinZW;
  b.zoomLocal = ui.pinZL;

  std::string chosen = "";
  if (!ui.pinIcons.empty() && ui.pinPickIndex >= 0 && ui.pinPickIndex < (int)ui.pinIcons.size()) {
    chosen = ui.pinIcons[ui.pinPickIndex];
  }
  b.iconRel = chosen;

  std::string label = ui.pinName;
  if (!chosen.empty()) {
    std::string stem = fileStem(chosen);
    std::string nice = stem;
    for (auto& ch : nice) if (ch == '_' || ch == '-') ch = ' ';
    if (!nice.empty()) label = nice;
  }
  b.name = label;

  ui.bookmarks.push_back(b);
  saveBookmarks(ui.bookmarks);
  refreshBookmarkNames(ui);
  ui.bmTarget = ui.bmIndex = (int)ui.bookmarks.size() - 1;
  ui.pinPickerOpen = false;
}

static void mapAction(UIState& ui, MapBtnId id, const SDL_Rect& mapArea, int mx, int my) {
  if (id == MapBtnId::HOME) {
    curLat(ui) = ui.homeLat;
    curLon(ui) = ui.homeLon;
    return;
  }
  if (id == MapBtnId::SAVE) {
    pinPickerOpen(ui);
    return;
  }
  if (id == MapBtnId::CANCEL) {
    ui.pinPickerOpen = false;
    return;
  }
  if (id == MapBtnId::CONFIRM) {
    pinPickerConfirm(ui);
    return;
  }
  if (id == MapBtnId::DEL) {
    if (!ui.bookmarks.empty() && ui.bmIndex >= 0 && ui.bmIndex < (int)ui.bookmarks.size()) {
      ui.bookmarks.erase(ui.bookmarks.begin() + ui.bmIndex);
      saveBookmarks(ui.bookmarks);
      refreshBookmarkNames(ui);
    }
    return;
  }
  if (id == MapBtnId::GO) {
    if (!ui.bookmarks.empty() && ui.bmIndex >= 0 && ui.bmIndex < (int)ui.bookmarks.size()) {
      auto& b = ui.bookmarks[ui.bmIndex];
      curLat(ui) = b.lat;
      curLon(ui) = b.lon;
      ui.zoomWorld = b.zoomWorld;
      ui.zoomLocal = b.zoomLocal;
    }
    return;
  }
  if (id == MapBtnId::ZPLUS || id == MapBtnId::ZMINUS) {
    int z0 = curZoom(ui);
    int z1 = z0 + (id == MapBtnId::ZPLUS ? 1 : -1);
    z1 = clampZoomForMode(ui.subTab, z1);
    applyZoomAtCursor(curLat(ui), curLon(ui), curZoom(ui), mapArea, mx, my, z1);
    return;
  }
}

int main(int argc, char** argv) {
  (void)argc; (void)argv;

  curl_global_init(CURL_GLOBAL_DEFAULT);

  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_EVENTS) != 0) die("SDL_Init");
  if (!(IMG_Init(IMG_INIT_PNG | IMG_INIT_JPG))) die("IMG_Init");
  if (TTF_Init() != 0) die("TTF_Init");

  int initFlags = MIX_INIT_MP3 | MIX_INIT_OGG;
int initted = Mix_Init(initFlags);
if ((initted & initFlags) != initFlags) {
  SDL_Log("Mix_Init missing codecs: %s", Mix_GetError());
}

if (Mix_OpenAudio(44100, MIX_DEFAULT_FORMAT, 2, 1024) != 0) {
  SDL_Log("Mix_OpenAudio: %s", Mix_GetError());
}
Mix_AllocateChannels(16);
Mix_VolumeMusic(110);

  int logicalW = 800;
  int logicalH = 480;

  SDL_Window* win = SDL_CreateWindow(
    "PipBoy Remake",
    SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
    logicalW, logicalH,
    SDL_WINDOW_SHOWN | SDL_WINDOW_ALLOW_HIGHDPI | SDL_WINDOW_RESIZABLE
  );
  if (!win) die("CreateWindow");

  SDL_Renderer* r = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
  if (!r) die("CreateRenderer");

  SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "nearest");
  SDL_RenderSetLogicalSize(r, logicalW, logicalH);

  Texture overlay = loadTexture(r, assetPath("images/overlay.png"));
  Texture border  = loadTexture(r, assetPath("images/border.png"));
  Texture vaultboy = loadTexture(r, assetPath("images/pipboy.png"));

  SDL_SetTextureBlendMode(overlay.tex, SDL_BLENDMODE_BLEND);
  SDL_SetTextureBlendMode(border.tex, SDL_BLENDMODE_BLEND);
  SDL_SetTextureBlendMode(vaultboy.tex, SDL_BLENDMODE_BLEND);

  std::string fontPath = assetPath("monofonto.ttf");
  TTF_Font* fontS = TTF_OpenFont(fontPath.c_str(), 18);
  TTF_Font* fontM = TTF_OpenFont(fontPath.c_str(), 22);
  if (!fontS || !fontM) die(std::string("TTF_OpenFont ") + fontPath);

  TTF_Font* fontL  = TTF_OpenFont(fontPath.c_str(), 56);
  if (!fontL) fontL = fontM;
  TTF_Font* fontXL = TTF_OpenFont(fontPath.c_str(), 88);
  if (!fontXL) fontXL = fontL;

  Uint32 uiFadeStart = 0;
  int uiFadeInMs = 0;

  static bool didBoot = false;
  if (!didBoot) {
    didBoot = true;

    BootSequenceConfig bc;
    bc.pipboyMs  = 6000;
    bc.endHoldMs = 500;

    bc.bootAMs   = 5000;
    bc.bootBMs   = 4500;

    bc.lineHoldMs = 0;

    bc.startX = 46;
    bc.startY = 60;
    bc.lineH  = 18;

    bc.exitFadeMs = 260;
    bc.uiFadeInMs = 450;

    bc.skipOnInput = true;
    bc.musicVolume = 110;

    bc.audioAPath = assetPath("sounds/boot_a.mp3");
    bc.audioBPath = assetPath("sounds/boot_b.mp3");

    TTF_Font* fontBig = TTF_OpenFont(fontPath.c_str(), 78);
    if (!fontBig) fontBig = fontM;

    BootSequence boot(win, r, logicalW, logicalH, fontBig, fontS, bc);
    boot.run();

    uiFadeStart = SDL_GetTicks();
    uiFadeInMs = bc.uiFadeInMs;

    if (fontBig && fontBig != fontM) TTF_CloseFont(fontBig);

    SDL_Event ev;
    while (SDL_PollEvent(&ev)) {}
  }


  Mix_Chunk* sDial = Mix_LoadWAV(assetPath("sounds/dial_move.ogg").c_str());
  Mix_Chunk* sMod  = Mix_LoadWAV(assetPath("sounds/module_change.ogg").c_str());
  Mix_Chunk* sSub  = Mix_LoadWAV(assetPath("sounds/submodule_change.ogg").c_str());
  auto play = [&](Mix_Chunk* c){ if (c) Mix_PlayChannel(-1, c, 0); };

  

  Mix_Chunk* sRadioStaticLp = Mix_LoadWAV(assetPath("sounds/radio/ui_radio_static_lp.mp3").c_str());
  Mix_Chunk* sRadioStaticTape = Mix_LoadWAV(assetPath("sounds/radio/ui_radio_statictape.mp3").c_str());
  Mix_Chunk* sRadioFound = Mix_LoadWAV(assetPath("sounds/radio/ui_radio_signalfound2.mp3").c_str());
  Mix_Chunk* sRadioLost  = Mix_LoadWAV(assetPath("sounds/radio/ui_radio_signallost.mp3").c_str());

  RadioRuntime radio;
  radio.sStaticLp = sRadioStaticLp;
  radio.sStaticTape = sRadioStaticTape;
  radio.sFound = sRadioFound;
  radio.sLost = sRadioLost;
  radio.power = false;
  radio.paused = false;
  radio.musicVol = 110;
  radio.stations = buildRadioStations();
  radio.initStations();
  if (!radio.stations.empty()) {
    radio.stationIdx = 0;
    radio.trackIdx = 0;
    radio.playingStationIdx = 0;
    radio.tuneFreq = radio.stations[0].freq;
    radio.locked = true;
    radio.hadLock = false;
  }
UIState ui;
  if (!radio.stations.empty()) {
    ui.radioStations.clear();
    for (auto& st : radio.stations) ui.radioStations.push_back(st.name);
    ui.radioStationIndex = radio.stationIdx;
    ui.radioTuneFreq = radio.tuneFreq;
  }
  ui.radioPower = false;
  ui.radioVol = 110;
  if (radio.sStaticTape) Mix_VolumeChunk(radio.sStaticTape, ui.radioVol);
  if (radio.sFound) Mix_VolumeChunk(radio.sFound, ui.radioVol);
  if (radio.sLost) Mix_VolumeChunk(radio.sLost, ui.radioVol);
  loadBookmarks(ui.bookmarks);
  refreshBookmarkNames(ui);
  ui.pinIcons = listPinIconsRel();

  std::filesystem::path tilesDir = assetsRootDir() / "cache" / "tiles";
  TileCache tileCache(r, tilesDir);

  bool running = true;
  Uint32 lastTick = SDL_GetTicks();
  float phase = 0.0f;

  std::vector<SDL_Rect> topHit(5);
  std::vector<SDL_Rect> subHit;

  while (running) {
    int ww=0, wh=0;
    SDL_GetWindowSize(win, &ww, &wh);
    logicalW = std::max(800, ww);
    logicalH = std::max(480, wh);
    SDL_RenderSetLogicalSize(r, logicalW, logicalH);

    SDL_Rect screen, content, left, right, mapArea, bottom;
    computeLayoutRects(logicalW, logicalH, screen, content, left, right, mapArea, bottom);

    buildTopSubHitboxes(fontM, fontS, screen, ui.tab, ui.subTab, topHit, subHit);
    if (ui.tab == TopTab::MAP) buildMapButtons(ui, right, mapBtns); else mapBtns.clear();

    updateClock(ui);

    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_QUIT) running = false;

      if (e.type == SDL_WINDOWEVENT && (e.window.event == SDL_WINDOWEVENT_RESIZED || e.window.event == SDL_WINDOWEVENT_SIZE_CHANGED)) {
        ui.mapDragging = false;
      }

      if (e.type == SDL_MULTIGESTURE) {
        if (ui.tab == TopTab::MAP && e.mgesture.numFingers >= 2) {
          int mx=0,my=0;
          windowNormToLogical(win, logicalW, logicalH, e.mgesture.x, e.mgesture.y, mx, my);
          if (ptIn(mapArea, mx, my)) {
            ui.pinchAccum += e.mgesture.dDist;
            float th = 0.035f;
            while (ui.pinchAccum >= th) {
              int z1 = clampZoomForMode(ui.subTab, curZoom(ui) + 1);
              applyZoomAtCursor(curLat(ui), curLon(ui), curZoom(ui), mapArea, mx, my, z1);
              ui.pinchAccum -= th;
            }
            while (ui.pinchAccum <= -th) {
              int z1 = clampZoomForMode(ui.subTab, curZoom(ui) - 1);
              applyZoomAtCursor(curLat(ui), curLon(ui), curZoom(ui), mapArea, mx, my, z1);
              ui.pinchAccum += th;
            }
          }
        }
      }

      if (e.type == SDL_FINGERDOWN) {
        int mx=0,my=0;
        windowNormToLogical(win, logicalW, logicalH, e.tfinger.x, e.tfinger.y, mx, my);

        bool handled = false;

        for (int i=0;i<5;i++){
          if (ptIn(topHit[i], mx, my)) { setTab(ui, (TopTab)i); play(sMod); handled = true; break; }
        }
        if (!handled) {
          for (int i=0;i<(int)subHit.size();i++){
            if (ptIn(subHit[i], mx, my)) { setSub(ui, i); play(sSub); handled = true; break; }
          }
        }

        if (ui.tab == TopTab::RADIO && !handled) {
          if (ptIn(gRadioPlayBtn, mx, my)) {
            if (!radio.power) {
              radio.power = true;
              radio.paused = false;
            } else {
              radio.paused = !radio.paused;
              if (radio.paused) radio.syncPlayingToBg();
            }
            ui.radioPower = (radio.power && !radio.paused);
            handled = true;
          } else if (gRadioShowSkip && ptIn(gRadioSkipBtn, mx, my)) {
            radio.skipIfCustomSelected();
            play(sSub);
            handled = true;
          } else if (ptIn(gRadioVolBtn, mx, my)) {
            int v = ui.radioVol;
            if (v <= 48) v = 80;
            else if (v <= 80) v = 110;
            else if (v <= 110) v = 128;
            else v = 48;
            ui.radioVol = v;
            radio.musicVol = v;
            Mix_VolumeMusic(v);
            if (radio.sStaticTape) Mix_VolumeChunk(radio.sStaticTape, std::max(0, std::min(128, v)));
            if (radio.sFound) Mix_VolumeChunk(radio.sFound, std::max(0, std::min(128, v)));
            if (radio.sLost) Mix_VolumeChunk(radio.sLost, std::max(0, std::min(128, v)));
            handled = true;
          }
        }

        if (ui.tab == TopTab::RADIO && !handled) {
          SDL_Rect tr = radioTuneRect(right);
          if (ptIn(tr, mx, my)) {
            float t = (tr.w <= 0) ? 0.0f : (float)(mx - tr.x) / (float)tr.w;
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            ui.radioTuneFreq = 87.5f + (108.0f - 87.5f) * t;
            radio.tuneFreq = ui.radioTuneFreq;
            handled = true;
          }
        }

        if (ui.tab == TopTab::MAP && !handled) {
          MapBtnId id;
          if (hitButtons(mapBtns, mx, my, id)) { mapAction(ui, id, mapArea, mx, my); play(sSub); handled = true; }
        }

        if (ui.tab == TopTab::MAP && !handled) {
          if (ptIn(mapArea, mx, my)) {
            ui.mapDragging = true;
            ui.dragLastX = mx;
            ui.dragLastY = my;
            handled = true;
          }
        }

        if (ui.tab == TopTab::MAP && !handled) {
          if (!ui.pinPickerOpen) {
            SDL_Rect placesPanel{ right.x + 10, right.y + 194, right.w - 20, std::max(70, std::min(220, right.h - 270)) };
            if (ptIn(placesPanel, mx, my) && !ui.bookmarksNames.empty()) {
              int rowH = 26;
              int relY = my - (placesPanel.y + 8);
              if (relY >= 0) {
                int visible = std::max(1, placesPanel.h / rowH);
                int start = std::max(0, ui.bmIndex - visible/2);
                start = std::min(start, std::max(0, (int)ui.bookmarksNames.size() - visible));
                int idx = start + (relY / rowH);
                if (idx >= 0 && idx < (int)ui.bookmarksNames.size()) {
                  ui.bmTarget = ui.bmIndex = idx;
                  play(sDial);
                }
              }
            }
          } else {
            SDL_Rect iconsPanel{ right.x + 10, right.y + 120, right.w - 20, std::max(70, right.h - 160) };
            if (ptIn(iconsPanel, mx, my) && !ui.pinIcons.empty()) {
              int rowH = 26;
              int relY = my - (iconsPanel.y + 8);
              if (relY >= 0) {
                int visible = std::max(1, iconsPanel.h / rowH);
                int start = std::max(0, ui.pinPickIndex - visible/2);
                start = std::min(start, std::max(0, (int)ui.pinIcons.size() - visible));
                int idx = start + (relY / rowH);
                if (idx >= 0 && idx < (int)ui.pinIcons.size()) {
                  ui.pinPickIndex = idx;
                  play(sDial);
                }
              }
            }
          }
        }

        if (!handled) {
          auto& L = currentList(ui);
          if (!L.empty() && ptIn(left, mx, my) && (ui.tab == TopTab::INV || ui.tab == TopTab::DATA || (ui.tab == TopTab::STAT && ui.subTab == 3) || ui.tab == TopTab::RADIO)) {
            int rowH = 26;
            int visible = std::max(1, left.h / rowH);
            int start = std::max(0, ui.listIndex - visible/2);
            start = std::min(start, std::max(0, (int)L.size() - visible));
            int relY = my - (left.y + 8);
            if (relY >= 0) {
              int idx = start + (relY / rowH);
              if (idx >= 0 && idx < (int)L.size()) {
                ui.listTarget = ui.listIndex = idx;
                play(sDial);
              }
            }
          }
        }
      }

      if (e.type == SDL_FINGERUP) ui.mapDragging = false;

      if (e.type == SDL_FINGERMOTION) {
        if (ui.tab == TopTab::MAP && ui.mapDragging) {
          int mx=0,my=0;
          windowNormToLogical(win, logicalW, logicalH, e.tfinger.x, e.tfinger.y, mx, my);
          int dx = mx - ui.dragLastX;
          int dy = my - ui.dragLastY;
          ui.dragLastX = mx;
          ui.dragLastY = my;

          int z = curZoom(ui);
          double px, py;
          latLonToPixel(curLat(ui), curLon(ui), z, px, py);
          px -= (double)dx;
          py -= (double)dy;

          double n = (double)(1 << z) * 256.0;
          px = std::fmod(px, n);
          if (px < 0) px += n;

          double worldPx = (double)(1 << z) * 256.0;
          if (py < 0.0) py = 0.0;
          if (py > worldPx - 1.0) py = worldPx - 1.0;

          pixelToLatLon(px, py, z, curLat(ui), curLon(ui));
        }
      }


      if (e.type == SDL_MOUSEWHEEL) {
        if (ui.tab == TopTab::RADIO) {
          float step = 0.20f;
          ui.radioTuneFreq += (float)e.wheel.y * step;
          if (ui.radioTuneFreq < 87.5f) ui.radioTuneFreq = 87.5f;
          if (ui.radioTuneFreq > 108.0f) ui.radioTuneFreq = 108.0f;
          radio.tuneFreq = ui.radioTuneFreq;
        }
      }

      if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
        int mx=0,my=0;
        toLogical(win, logicalW, logicalH, e.button.x, e.button.y, mx, my);

        bool handled = false;

        for (int i=0;i<5;i++){
          if (ptIn(topHit[i], mx, my)) { setTab(ui, (TopTab)i); play(sMod); handled = true; break; }
        }
        if (!handled) {
          for (int i=0;i<(int)subHit.size();i++){
            if (ptIn(subHit[i], mx, my)) { setSub(ui, i); play(sSub); handled = true; break; }
          }
        }

        if (ui.tab == TopTab::RADIO && !handled) {
          if (ptIn(gRadioPlayBtn, mx, my)) {
            if (!radio.power) {
              radio.power = true;
              radio.paused = false;
            } else {
              radio.paused = !radio.paused;
              if (radio.paused) radio.syncPlayingToBg();
            }
            ui.radioPower = (radio.power && !radio.paused);
            play(sSub);
            handled = true;
          } else if (gRadioShowSkip && ptIn(gRadioSkipBtn, mx, my)) {
            radio.skipIfCustomSelected();
            play(sSub);
            handled = true;
          } else if (ptIn(gRadioVolBtn, mx, my)) {
            int v = ui.radioVol;
            if (v <= 48) v = 80;
            else if (v <= 80) v = 110;
            else if (v <= 110) v = 128;
            else v = 48;
            ui.radioVol = v;
            radio.musicVol = v;
            Mix_VolumeMusic(v);
            if (radio.sStaticTape) Mix_VolumeChunk(radio.sStaticTape, std::max(0, std::min(128, v)));
            if (radio.sFound) Mix_VolumeChunk(radio.sFound, std::max(0, std::min(128, v)));
            if (radio.sLost) Mix_VolumeChunk(radio.sLost, std::max(0, std::min(128, v)));
            play(sSub);
            handled = true;
          }
        }

        if (ui.tab == TopTab::RADIO && !handled) {
          SDL_Rect tr = radioTuneRect(right);
          if (ptIn(tr, mx, my)) {
            float t = (tr.w <= 0) ? 0.0f : (float)(mx - tr.x) / (float)tr.w;
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;
            ui.radioTuneFreq = 87.5f + (108.0f - 87.5f) * t;
            radio.tuneFreq = ui.radioTuneFreq;
            play(sDial);
            handled = true;
          }
        }

        if (ui.tab == TopTab::MAP && !handled) {
          MapBtnId id;
          if (hitButtons(mapBtns, mx, my, id)) { mapAction(ui, id, mapArea, mx, my); play(sSub); handled = true; }
        }

        if (ui.tab == TopTab::MAP && !handled) {
          if (ptIn(mapArea, mx, my)) {
            ui.mapDragging = true;
            ui.dragLastX = mx;
            ui.dragLastY = my;
            handled = true;
          }
        }

        if (ui.tab == TopTab::MAP && !handled) {
          if (!ui.pinPickerOpen) {
            SDL_Rect placesPanel{ right.x + 10, right.y + 194, right.w - 20, std::max(70, std::min(220, right.h - 270)) };
            if (ptIn(placesPanel, mx, my) && !ui.bookmarksNames.empty()) {
              int rowH = 26;
              int relY = my - (placesPanel.y + 8);
              if (relY >= 0) {
                int visible = std::max(1, placesPanel.h / rowH);
                int start = std::max(0, ui.bmIndex - visible/2);
                start = std::min(start, std::max(0, (int)ui.bookmarksNames.size() - visible));
                int idx = start + (relY / rowH);
                if (idx >= 0 && idx < (int)ui.bookmarksNames.size()) {
                  ui.bmTarget = ui.bmIndex = idx;
                  play(sDial);
                }
              }
            }
          } else {
            SDL_Rect iconsPanel{ right.x + 10, right.y + 120, right.w - 20, std::max(70, right.h - 160) };
            if (ptIn(iconsPanel, mx, my) && !ui.pinIcons.empty()) {
              int rowH = 26;
              int relY = my - (iconsPanel.y + 8);
              if (relY >= 0) {
                int visible = std::max(1, iconsPanel.h / rowH);
                int start = std::max(0, ui.pinPickIndex - visible/2);
                start = std::min(start, std::max(0, (int)ui.pinIcons.size() - visible));
                int idx = start + (relY / rowH);
                if (idx >= 0 && idx < (int)ui.pinIcons.size()) {
                  ui.pinPickIndex = idx;
                  play(sDial);
                }
              }
            }
          }
        }

        if (!handled) {
          auto& L = currentList(ui);
          if (!L.empty() && ptIn(left, mx, my) && (ui.tab == TopTab::INV || ui.tab == TopTab::DATA || (ui.tab == TopTab::STAT && ui.subTab == 3) || ui.tab == TopTab::RADIO)) {
            int rowH = 26;
            int visible = std::max(1, left.h / rowH);
            int start = std::max(0, ui.listIndex - visible/2);
            start = std::min(start, std::max(0, (int)L.size() - visible));
            int relY = my - (left.y + 8);
            if (relY >= 0) {
              int idx = start + (relY / rowH);
              if (idx >= 0 && idx < (int)L.size()) {
                ui.listTarget = ui.listIndex = idx;
                play(sDial);
              }
            }
          }
        }
      }

      if (e.type == SDL_MOUSEBUTTONUP && e.button.button == SDL_BUTTON_LEFT) ui.mapDragging = false;

      if (e.type == SDL_MOUSEMOTION) {
        if (ui.tab == TopTab::MAP && ui.mapDragging) {
          int mx=0,my=0;
          toLogical(win, logicalW, logicalH, e.motion.x, e.motion.y, mx, my);
          int dx = mx - ui.dragLastX;
          int dy = my - ui.dragLastY;
          ui.dragLastX = mx;
          ui.dragLastY = my;

          int z = curZoom(ui);
          double px, py;
          latLonToPixel(curLat(ui), curLon(ui), z, px, py);
          px -= (double)dx;
          py -= (double)dy;

          double n = (double)(1 << z) * 256.0;
          px = std::fmod(px, n);
          if (px < 0) px += n;

          double worldPx = (double)(1 << z) * 256.0;
          if (py < 0.0) py = 0.0;
          if (py > worldPx - 1.0) py = worldPx - 1.0;

          pixelToLatLon(px, py, z, curLat(ui), curLon(ui));
        }
      }

      if (e.type == SDL_KEYDOWN && !e.key.repeat) {
        SDL_Keycode k = e.key.keysym.sym;

        if (k == SDLK_ESCAPE) running = false;

        if (k == SDLK_1) { setTab(ui, TopTab::STAT);  play(sMod); }
        if (k == SDLK_2) { setTab(ui, TopTab::INV);   play(sMod); }
        if (k == SDLK_3) { setTab(ui, TopTab::DATA);  play(sMod); }
        if (k == SDLK_4) { setTab(ui, TopTab::MAP);   play(sMod); }
        if (k == SDLK_5) { setTab(ui, TopTab::RADIO); play(sMod); }

        if (k == SDLK_6) { setSub(ui, 0); play(sSub); }
        if (k == SDLK_7) { setSub(ui, 1); play(sSub); }
        if (k == SDLK_8) { setSub(ui, 2); play(sSub); }
        if (k == SDLK_9) { setSub(ui, 3); play(sSub); }
        if (k == SDLK_0) { setSub(ui, 4); play(sSub); }

        if (k == SDLK_a) { int v=(int)ui.tab-1; if(v<0)v=4; setTab(ui,(TopTab)v); play(sMod); }
        if (k == SDLK_d) { int v=(int)ui.tab+1; if(v>4)v=0; setTab(ui,(TopTab)v); play(sMod); }

        if (k == SDLK_q) { nextSub(ui, -1); play(sSub); }
        if (k == SDLK_e) { nextSub(ui, +1); play(sSub); }

        if (ui.tab == TopTab::MAP) {
          if (!ui.pinPickerOpen) {
            if (k == SDLK_w) { ui.bmTarget = std::max(0, ui.bmTarget - 1); play(sDial); }
            if (k == SDLK_s) { ui.bmTarget = std::min((int)ui.bookmarksNames.size()-1, ui.bmTarget + 1); play(sDial); }
          } else {
            if (k == SDLK_w) { ui.pinPickIndex = std::max(0, ui.pinPickIndex - 1); play(sDial); }
            if (k == SDLK_s) { ui.pinPickIndex = std::min((int)ui.pinIcons.size()-1, ui.pinPickIndex + 1); play(sDial); }
            if (k == SDLK_RETURN || k == SDLK_KP_ENTER) { pinPickerConfirm(ui); play(sSub); }
            if (k == SDLK_BACKSPACE || k == SDLK_DELETE || k == SDLK_ESCAPE) { ui.pinPickerOpen = false; play(sSub); }
          }

          if (k == SDLK_LEFT)  curLon(ui) -= (ui.subTab==0 ? 1.0 : 0.01);
          if (k == SDLK_RIGHT) curLon(ui) += (ui.subTab==0 ? 1.0 : 0.01);
          if (k == SDLK_UP)    curLat(ui) += (ui.subTab==0 ? 1.0 : 0.01);
          if (k == SDLK_DOWN)  curLat(ui) -= (ui.subTab==0 ? 1.0 : 0.01);

          curLat(ui) = clampLat(curLat(ui));
          if (curLon(ui) > 180.0) curLon(ui) -= 360.0;
          if (curLon(ui) < -180.0) curLon(ui) += 360.0;

          if (k == SDLK_EQUALS || k == SDLK_KP_PLUS) {
            int mxRaw=0,myRaw=0; SDL_GetMouseState(&mxRaw,&myRaw);
            int mx=0,my=0; toLogical(win, logicalW, logicalH, mxRaw, myRaw, mx, my);
            int z1 = clampZoomForMode(ui.subTab, curZoom(ui)+1);
            applyZoomAtCursor(curLat(ui), curLon(ui), curZoom(ui), mapArea, mx, my, z1);
          }
          if (k == SDLK_MINUS || k == SDLK_KP_MINUS) {
            int mxRaw=0,myRaw=0; SDL_GetMouseState(&mxRaw,&myRaw);
            int mx=0,my=0; toLogical(win, logicalW, logicalH, mxRaw, myRaw, mx, my);
            int z1 = clampZoomForMode(ui.subTab, curZoom(ui)-1);
            applyZoomAtCursor(curLat(ui), curLon(ui), curZoom(ui), mapArea, mx, my, z1);
          }
          if (k == SDLK_r) {
            curLat(ui) = ui.homeLat;
            curLon(ui) = ui.homeLon;
          }
        } else {
          if (ui.tab == TopTab::RADIO) {
            float prev = ui.radioTuneFreq;
            if (k == SDLK_LEFT)  ui.radioTuneFreq -= 0.15f;
            if (k == SDLK_RIGHT) ui.radioTuneFreq += 0.15f;
            if (ui.radioTuneFreq < 87.5f) ui.radioTuneFreq = 87.5f;
            if (ui.radioTuneFreq > 108.0f) ui.radioTuneFreq = 108.0f;
            if (ui.radioTuneFreq != prev) {
              radio.tuneFreq = ui.radioTuneFreq;
            }
          }
          if (k == SDLK_w) { ui.listTarget = std::max(0, ui.listTarget - 1); play(sDial); }
          if (k == SDLK_s) { ui.listTarget = std::min((int)currentList(ui).size()-1, ui.listTarget + 1); play(sDial); }
        }

        if (k == SDLK_f) {
          Uint32 flags = SDL_GetWindowFlags(win);
          bool fs = (flags & SDL_WINDOW_FULLSCREEN_DESKTOP) != 0;
          SDL_SetWindowFullscreen(win, fs ? 0 : SDL_WINDOW_FULLSCREEN_DESKTOP);
        }
      }
    }

    clampSub(ui);
    clampList(ui);
    clampBookmarks(ui);

    Uint32 now = SDL_GetTicks();
    float dt = (now - lastTick) / 1000.0f;
    lastTick = now;

    phase += dt * 3.0f;
    radio.update(dt);
    if (ui.tab == TopTab::RADIO) {
      ui.radioTuneFreq = radio.tuneFreq;
      ui.radioNowPlaying = radio.nowPlaying;
      ui.radioLocked = radio.locked;
      ui.radioPower = (radio.power && !radio.paused);
      ui.radioVol = radio.musicVol;
    }
    float flick = 0.02f * SDL_sinf(phase * 2.1f) + 0.01f * SDL_sinf(phase * 7.0f);
    float a = 1.0f - std::exp(-18.0f * dt);

    auto& list = currentList(ui);
    if (!list.empty()) ui.listIndex = ui.listTarget;

    if (ui.tab == TopTab::RADIO) {
      if (!ui.radioStations.empty()) {
        int sel = ui.listIndex;
        if (sel != ui.radioStationIndex) {
          ui.radioStationIndex = sel;
          radio.selectStation(sel);
        }
        ui.radioTuneFreq = radio.tuneFreq;
        ui.radioNowPlaying = radio.nowPlaying;
        ui.radioLocked = radio.locked;
        ui.radioPower = (radio.power && !radio.paused);
        ui.radioVol = radio.musicVol;
      }
    }
    if (!ui.bookmarksNames.empty()) ui.bmIndex = ui.bmTarget;

    SDL_Color bg { 3, 10, 6, 255 };
    SDL_Color on { 120, 255, 160, 235 };
    SDL_Color off{ 120, 255, 160, 120 };
    SDL_Color white{ 220, 255, 230, 235 };

    SDL_SetRenderDrawColor(r, bg.r, bg.g, bg.b, 255);
    SDL_RenderClear(r);

    computeLayoutRects(logicalW, logicalH, screen, content, left, right, mapArea, bottom);

    fillRect(r, screen, SDL_Color{0,0,0,140});
    drawRect(r, screen, SDL_Color{on.r,on.g,on.b,75});

    int topY = screen.y + 10;
    int x = screen.x + 14;
    for (int i=0; i<5; i++) {
      TopTab t = (TopTab)i;
      bool active = (ui.tab == t);
      std::string label = topTabName(t);
      std::string shown = active ? ("[" + label + "]") : label;
      drawText(r, fontM, shown, x, topY, active ? on : off);
      x += textW(fontM, "[" + label + "]") + 18;
    }

    int subY = topY + 28;
    auto subs = subTabsFor(ui.tab);
    int sx = screen.x + 14;
    for (int i=0; i<(int)subs.size(); i++) {
      bool active = (ui.subTab == i);
      std::string shown = active ? ("[" + subs[i] + "]") : subs[i];
      drawText(r, fontS, shown, sx, subY, active ? white : off);
      sx += textW(fontS, "[" + subs[i] + "]") + 12;
    }

    drawLine(r, content.x, content.y - 6, content.x + content.w, content.y - 6, on, 80);
    drawRect(r, left, SDL_Color{on.r,on.g,on.b,65});
    drawRect(r, right, SDL_Color{on.r,on.g,on.b,65});

    if (ui.tab == TopTab::STAT && ui.subTab == 0) {
      drawText(r, fontM, "STATUS", left.x + 12, left.y + 10, on);
      SDL_Rect imgBox { left.x + 22, left.y + 52, left.w - 44, left.h - 74 };
      Uint8 imgA = (Uint8)std::clamp((int)(215 + 15 * std::fabs(flick)), 190, 230);
      renderImageFit(r, vaultboy.tex, vaultboy.w, vaultboy.h, imgBox, imgA);

      drawTextRight(r, fontM, "LVL " + std::to_string(ui.level), right.x + right.w - 12, right.y + 10, on);

      drawText(r, fontS, "HP", right.x + 12, right.y + 54, off);
      drawBar(r, SDL_Rect{right.x + 52, right.y + 56, right.w - 64, 16}, (float)ui.hp, (float)ui.hpMax, on, on);
      drawText(r, fontS, "AP", right.x + 12, right.y + 86, off);
      drawBar(r, SDL_Rect{right.x + 52, right.y + 88, right.w - 64, 16}, (float)ui.ap, (float)ui.apMax, on, on);
      drawText(r, fontS, "XP", right.x + 12, right.y + 118, off);
      drawBar(r, SDL_Rect{right.x + 52, right.y + 120, right.w - 64, 16}, (float)ui.xp, (float)ui.xpMax, on, on);
    } else {


if (ui.tab == TopTab::DATA && ui.subTab == 0) {
  SDL_Rect clockPanel{ left.x + 10, left.y + 44, left.w - 20, left.h - 56 };
  fillRect(r, clockPanel, SDL_Color{0,0,0,85});
  drawRect(r, clockPanel, SDL_Color{on.r,on.g,on.b,70});

  int tw=0, th=0;
  SDL_Texture* tTime = renderText(r, fontXL, ui.clockHHMM, on, tw, th);
  int tx = clockPanel.x + (clockPanel.w - tw)/2;
  int ty = clockPanel.y + (clockPanel.h - th)/2 - 10;
  SDL_Rect td{ tx, ty, tw, th };
  SDL_RenderCopy(r, tTime, nullptr, &td);
  SDL_DestroyTexture(tTime);

  drawText(r, fontS, ui.clockFull, clockPanel.x + 12, clockPanel.y + clockPanel.h - 26, off);
}

      if (!list.empty() && (ui.tab == TopTab::INV || ui.tab == TopTab::DATA || (ui.tab == TopTab::STAT && ui.subTab == 3) || ui.tab == TopTab::RADIO)) {
        int rowH = 26;
        int visible = std::max(1, left.h / rowH);
        int start = std::max(0, ui.listIndex - visible/2);
        start = std::min(start, std::max(0, (int)list.size() - visible));
        float targetY = (float)(left.y + 8 + (ui.listIndex - start) * rowH);
        ui.hiY = (ui.hiY == 0.0f) ? targetY : lerp(ui.hiY, targetY, a);
        drawList(r, fontS, left, list, ui.listIndex, ui.hiY, on);
      }

      drawText(r, fontM, (ui.tab == TopTab::MAP ? "MAP" : "DETAILS"), right.x + 12, right.y + 10, on);

      if (ui.tab == TopTab::MAP) {
        fillRect(r, mapArea, SDL_Color{0,0,0,110});
        drawRect(r, mapArea, SDL_Color{on.r,on.g,on.b,80});

        int z = curZoom(ui);
        tileCache.pump(2);
        drawMapTiles(r, tileCache, mapArea, curLat(ui), curLon(ui), z, on);

        int cx = mapArea.x + mapArea.w/2;
        int cy = mapArea.y + mapArea.h/2;
        fillRect(r, SDL_Rect{cx-3, cy-3, 6, 6}, SDL_Color{on.r,on.g,on.b,220});
        drawRect(r, SDL_Rect{cx-12, cy-12, 24, 24}, SDL_Color{on.r,on.g,on.b,120});

        buildMapButtons(ui, right, mapBtns);

        for (auto& b : mapBtns) {
          fillRect(r, b.rc, SDL_Color{0,0,0,60});
          drawRect(r, b.rc, SDL_Color{on.r,on.g,on.b,90});
          std::string label;
          if (!ui.pinPickerOpen) {
            if (b.id == MapBtnId::ZPLUS) label = "[+] ZOOM";
            if (b.id == MapBtnId::ZMINUS) label = "[-] ZOOM";
            if (b.id == MapBtnId::HOME) label = "HOME";
            if (b.id == MapBtnId::SAVE) label = "SAVE PIN";
            if (b.id == MapBtnId::GO) label = "GO PIN";
            if (b.id == MapBtnId::DEL) label = "DEL PIN";
          } else {
            if (b.id == MapBtnId::CONFIRM) label = "CONFIRM";
            if (b.id == MapBtnId::CANCEL) label = "CANCEL";
          }
          drawText(r, fontS, label, b.rc.x + 8, b.rc.y + 3, off);
        }

        if (!ui.pinPickerOpen) {
          drawText(r, fontS, "BOOKMARKS", right.x + 12, right.y + 172, off);

          int placesH = std::max(70, std::min(220, right.h - 270));
          SDL_Rect placesPanel{ right.x + 10, right.y + 194, right.w - 20, placesH };
          fillRect(r, placesPanel, SDL_Color{0,0,0,85});
          drawRect(r, placesPanel, SDL_Color{on.r,on.g,on.b,70});
          if (!ui.bookmarksNames.empty()) {
            int rowH = 26;
            int visible = std::max(1, placesPanel.h / rowH);
            int start = std::max(0, ui.bmIndex - visible/2);
            start = std::min(start, std::max(0, (int)ui.bookmarksNames.size() - visible));
            float targetY = (float)(placesPanel.y + 8 + (ui.bmIndex - start) * rowH);
            ui.bmHiY = (ui.bmHiY == 0.0f) ? targetY : lerp(ui.bmHiY, targetY, a);
            drawList(r, fontS, placesPanel, ui.bookmarksNames, ui.bmIndex, ui.bmHiY, on);
          } else {
            drawText(r, fontS, "NO PINS", placesPanel.x + 12, placesPanel.y + 10, off);
          }

          int infoY = placesPanel.y + placesPanel.h + 12;
          drawText(r, fontS, "PINCH ZOOM (2F)", right.x + 12, infoY, off);
          drawText(r, fontS, "DRAG PAN (1F)",   right.x + 12, infoY + 24, off);

          char buf[128];
          std::snprintf(buf, sizeof(buf), "ZOOM %d", curZoom(ui));
          drawText(r, fontS, buf, right.x + 12, infoY + 54, off);
          std::snprintf(buf, sizeof(buf), "LAT %.4f", curLat(ui));
          drawText(r, fontS, buf, right.x + 12, infoY + 78, off);
          std::snprintf(buf, sizeof(buf), "LON %.4f", curLon(ui));
          drawText(r, fontS, buf, right.x + 12, infoY + 102, off);
          drawText(r, fontS, (ui.subTab==0) ? "MODE WORLD" : "MODE LOCAL", right.x + 12, infoY + 126, off);
        } else {
          drawText(r, fontS, "CHOOSE PIN TYPE", right.x + 12, right.y + 80, off);

          SDL_Rect iconsPanel{ right.x + 10, right.y + 120, right.w - 20, std::max(70, right.h - 160) };
          fillRect(r, iconsPanel, SDL_Color{0,0,0,85});
          drawRect(r, iconsPanel, SDL_Color{on.r,on.g,on.b,70});

          if (!ui.pinIcons.empty()) {
            std::vector<std::string> names;
            names.reserve(ui.pinIcons.size());
            for (auto& rel : ui.pinIcons) names.push_back(fileStem(rel));
            int rowH = 26;
            int visible = std::max(1, iconsPanel.h / rowH);
            int start = std::max(0, ui.pinPickIndex - visible/2);
            start = std::min(start, std::max(0, (int)names.size() - visible));
            float hiY = (float)(iconsPanel.y + 8 + (ui.pinPickIndex - start) * rowH);
            drawList(r, fontS, iconsPanel, names, ui.pinPickIndex, hiY, on);

            std::string sel = names[ui.pinPickIndex];
            drawText(r, fontS, ("SELECTED: " + sel), right.x + 12, right.y + right.h - 34, off);
          } else {
            drawText(r, fontS, "NO ICONS FOUND", iconsPanel.x + 12, iconsPanel.y + 10, off);
          }
        }
      } else {
        if (ui.tab == TopTab::RADIO) {
          drawText(r, fontS, "TUNING", right.x + 12, right.y + 46, off);
          char buf[128];
          std::snprintf(buf, sizeof(buf), "%.1f MHz", ui.radioTuneFreq);
          int fx = right.x + 12 + textW(fontS, "TUNING") + 18;
          drawText(r, fontM, buf, fx, right.y + 42, on);

          SDL_Rect tr = radioTuneRect(right);
          SDL_Color tuneBg = {0,0,0,95};
          SDL_Color tuneBr = {on.r,on.g,on.b,90};
          fillRect(r, tr, tuneBg);
          drawRect(r, tr, tuneBr);

          float t = (ui.radioTuneFreq - 87.5f) / (108.0f - 87.5f);
          if (t < 0.0f) t = 0.0f;
          if (t > 1.0f) t = 1.0f;
          int px = tr.x + (int)std::lround(t * (float)tr.w);
          drawLine(r, px, tr.y - 8, px, tr.y + tr.h + 8, on, 220);

          float target = radio.curStationFreq();
          float tt = (target - 87.5f) / (108.0f - 87.5f);
          if (tt < 0.0f) tt = 0.0f;
          if (tt > 1.0f) tt = 1.0f;
          int tx = tr.x + (int)std::lround(tt * (float)tr.w);
          drawLine(r, tx, tr.y - 6, tx, tr.y + tr.h + 6, off, 170);
          std::snprintf(buf, sizeof(buf), "STATION %0.1f", target);
          int sw = textW(fontS, buf);
          int sx = tx - sw/2;
          if (sx < right.x + 12) sx = right.x + 12;
          if (sx + sw > right.x + right.w - 12) sx = right.x + right.w - 12 - sw;
          drawText(r, fontS, buf, sx, tr.y + 46, off);

          drawText(r, fontS, ui.radioLocked ? "SIGNAL LOCK" : "NO SIGNAL", right.x + 12, tr.y + 72, ui.radioLocked ? on : off);

          SDL_Rect sigRc{ right.x + 12, tr.y + 96, right.w - 24, 14 };
          drawRect(r, sigRc, SDL_Color{on.r,on.g,on.b,70});
          float sig = ui.radioLocked ? 1.0f : 0.0f;
          if (!ui.radioLocked) {
            float diff = std::fabs(ui.radioTuneFreq - target);
            float lockTh = 0.25f;
            float fullTh = 1.50f;
            sig = 1.0f - clamp01((diff - lockTh) / (fullTh - lockTh));
          }
          SDL_Rect fill{ sigRc.x + 2, sigRc.y + 2, (int)std::lround((sigRc.w - 4) * sig), sigRc.h - 4 };
          fillRect(r, fill, SDL_Color{on.r,on.g,on.b,190});

          if (!ui.radioStations.empty() && ui.radioStationIndex >= 0 && ui.radioStationIndex < (int)ui.radioStations.size()) {
            drawText(r, fontS, "NOW PLAYING", right.x + 12, sigRc.y + 30, off);
            std::string stn = ui.radioStations[ui.radioStationIndex];
            drawText(r, fontM, stn, right.x + 12, sigRc.y + 56, on);
            if (!ui.radioNowPlaying.empty()) {
              drawText(r, fontS, ui.radioNowPlaying, right.x + 12, sigRc.y + 88, on);
            } else {
              drawText(r, fontS, ui.radioLocked ? "LOADING..." : "TUNE TO HEAR", right.x + 12, sigRc.y + 88, off);
            }
          }

          SDL_Rect playBtn{}, volBtn{}, skipBtn{};
          bool showSkip = (!ui.radioStations.empty() && ui.radioStationIndex >= 0 && ui.radioStationIndex < (int)ui.radioStations.size() && isCustomStationName(ui.radioStations[ui.radioStationIndex]));
          radioDetailButtonsEx(right, showSkip, playBtn, volBtn, skipBtn);
          gRadioPlayBtn = playBtn;
          gRadioVolBtn  = volBtn;
          gRadioSkipBtn = skipBtn;
          gRadioShowSkip = showSkip;

          SDL_Color btnBg = {0,0,0,70};
          SDL_Color btnBr = {on.r,on.g,on.b,95};

          fillRect(r, playBtn, btnBg);
          drawRect(r, playBtn, btnBr);
          std::string pLabel = ui.radioPower ? "PAUSE" : "PLAY";
          drawText(r, fontS, pLabel, playBtn.x + 10, playBtn.y + 5, off);

          if (showSkip) {
            fillRect(r, skipBtn, btnBg);
            drawRect(r, skipBtn, btnBr);
            drawText(r, fontS, "NEXT", skipBtn.x + 10, skipBtn.y + 5, off);
          }
fillRect(r, volBtn, btnBg);
          drawRect(r, volBtn, btnBr);
          int pct = (int)std::lround((float)ui.radioVol * 100.0f / 128.0f);
          if (pct < 0) pct = 0;
          if (pct > 100) pct = 100;
          char vbuf[64];
          std::snprintf(vbuf, sizeof(vbuf), "VOL %d%%", pct);
          drawText(r, fontS, vbuf, volBtn.x + 10, volBtn.y + 5, off);
        } else if (ui.tab == TopTab::DATA && ui.subTab == 0) {
          drawText(r, fontS, "LOCAL TIME", right.x + 12, right.y + 46, off);
          drawText(r, fontM, ui.clockHHMM, right.x + 12, right.y + 74, on);
          drawText(r, fontS, ui.clockFull, right.x + 12, right.y + 118, off);
          drawText(r, fontS, ui.clockTZ, right.x + 12, right.y + 142, off);
        } else {
          if (!list.empty()) drawText(r, fontS, list[ui.listIndex], right.x + 12, right.y + 46, on);
        }
      }
    }

    drawRect(r, bottom, SDL_Color{on.r,on.g,on.b,85});
    drawText(r, fontS, std::to_string(ui.carry) + "/" + std::to_string(ui.carryMax), bottom.x + 10, bottom.y + 6, on);
    drawText(r, fontS, std::to_string(ui.caps), bottom.x + 120, bottom.y + 6, on);
    drawTextRight(r, fontS, "INSPECT   DROP   FAV   SORT   PERK CHART", bottom.x + bottom.w - 12, bottom.y + 6, off);
    drawTextRight(r, fontS, "AMMO " + std::to_string(ui.ammo), bottom.x + bottom.w - 12, bottom.y - 18, on);

    SDL_Rect full {0,0,logicalW,logicalH};
    SDL_SetTextureAlphaMod(overlay.tex, (Uint8)std::clamp((int)(60 + 30 * std::fabs(flick)), 45, 90));
    SDL_RenderCopy(r, overlay.tex, nullptr, &full);

    SDL_SetTextureAlphaMod(border.tex, 255);
    SDL_RenderCopy(r, border.tex, nullptr, &full);

    drawScanlines(r, logicalW, logicalH, 3, 18);

    int vignetteAlpha = std::clamp(6 + (int)(24.0f * std::fabs(flick)), 6, 22);
    fillRect(r, SDL_Rect{0,0,logicalW,logicalH}, SDL_Color{0,0,0,(Uint8)vignetteAlpha});


    if (uiFadeInMs > 0) {
      Uint32 tnow = SDL_GetTicks();
      int elapsed = (int)(tnow - uiFadeStart);
      if (elapsed < uiFadeInMs) {
        float p = (float)elapsed / (float)uiFadeInMs;
        if (p < 0.0f) p = 0.0f;
        if (p > 1.0f) p = 1.0f;

        Uint8 a2 = (Uint8)std::clamp((int)std::lround((1.0f - p) * 255.0f), 0, 255);

        SDL_SetRenderDrawBlendMode(r, SDL_BLENDMODE_BLEND);
        SDL_SetRenderDrawColor(r, 0, 0, 0, a2);
        SDL_Rect full2{0,0,logicalW,logicalH};
        SDL_RenderFillRect(r, &full2);
      } else {
        uiFadeInMs = 0;
      }
    }

    SDL_RenderPresent(r);
  }

  radio.stopMusic();
  radio.stopStatic();

  
  if (sRadioStaticLp) Mix_FreeChunk(sRadioStaticLp);
  if (sRadioStaticTape) Mix_FreeChunk(sRadioStaticTape);
  if (sRadioFound) Mix_FreeChunk(sRadioFound);
  if (sRadioLost) Mix_FreeChunk(sRadioLost);

  if (sDial) Mix_FreeChunk(sDial);
  if (sMod)  Mix_FreeChunk(sMod);
  if (sSub)  Mix_FreeChunk(sSub);

  if (fontXL && fontXL != fontL) TTF_CloseFont(fontXL);
  if (fontL && fontL != fontM) TTF_CloseFont(fontL);
  TTF_CloseFont(fontS);
  TTF_CloseFont(fontM);

  SDL_DestroyRenderer(r);
  SDL_DestroyWindow(win);

  Mix_CloseAudio();
  TTF_Quit();
  IMG_Quit();
  SDL_Quit();

  curl_global_cleanup();
  return 0;
}
