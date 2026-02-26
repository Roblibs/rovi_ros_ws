// Capture a single OpenNI2 depth frame and write it as a 16-bit PGM (millimeters).
//
// Intended as a direct (non-ROS) sanity check when the depth camera seems broken.
// See tools/depth/readme.md and docs/depth_camera_astra_stereo_s_u3.md for OpenNI2 SDK setup.
//
// Build (expects Orbbec OpenNI2 SDK at $HOME/OpenNI/OpenNI_2.3.0 and NiViewer redistributable):
//   g++ -std=c++17 openni2_snapshot_depth.cpp \
//     -I$HOME/OpenNI/OpenNI_2.3.0/sdk/Include -L$HOME/OpenNI/OpenNI_2.3.0/tools/NiViewer \
//     -lOpenNI2 -Wl,-rpath,$HOME/OpenNI/OpenNI_2.3.0/tools/NiViewer \
//     -O2 -o openni2_snapshot_depth

#include <OpenNI.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

static std::string nowStamp() {
  using clock = std::chrono::system_clock;
  const auto t = clock::to_time_t(clock::now());
  std::tm tm {};
#if defined(_WIN32)
  localtime_s(&tm, &t);
#else
  localtime_r(&t, &tm);
#endif
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y%m%d_%H%M%S");
  return oss.str();
}

static void writePgm16Be(const fs::path &path, int width, int height, const std::vector<uint16_t> &mm) {
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    throw std::runtime_error("Failed to open output file: " + path.string());
  }
  out << "P5\n" << width << " " << height << "\n65535\n";
  for (uint16_t v : mm) {
    const uint8_t hi = static_cast<uint8_t>((v >> 8) & 0xFF);
    const uint8_t lo = static_cast<uint8_t>(v & 0xFF);
    out.put(static_cast<char>(hi));
    out.put(static_cast<char>(lo));
  }
}

static void writePgm8(const fs::path &path, int width, int height, const std::vector<uint8_t> &img) {
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    throw std::runtime_error("Failed to open output file: " + path.string());
  }
  out << "P5\n" << width << " " << height << "\n255\n";
  out.write(reinterpret_cast<const char *>(img.data()), static_cast<std::streamsize>(img.size()));
}

struct Stats {
  uint64_t total = 0;
  uint64_t zeros = 0;
  uint16_t min_nonzero = 0;
  uint16_t max_val = 0;
};

static Stats statsU16(const std::vector<uint16_t> &mm) {
  Stats s;
  s.total = mm.size();
  bool has_nonzero = false;
  for (uint16_t v : mm) {
    if (v == 0) {
      s.zeros++;
      continue;
    }
    if (!has_nonzero) {
      s.min_nonzero = v;
      has_nonzero = true;
    } else if (v < s.min_nonzero) {
      s.min_nonzero = v;
    }
    if (v > s.max_val) {
      s.max_val = v;
    }
  }
  return s;
}

static std::vector<uint8_t> depthViz8(const std::vector<uint16_t> &mm, uint16_t clip_max_mm) {
  std::vector<uint8_t> out(mm.size());
  for (size_t i = 0; i < mm.size(); i++) {
    uint16_t v = mm[i];
    if (v == 0) {
      out[i] = 0;
      continue;
    }
    if (v > clip_max_mm) {
      v = clip_max_mm;
    }
    out[i] = static_cast<uint8_t>((static_cast<uint32_t>(v) * 255u) / clip_max_mm);
  }
  return out;
}

struct Args {
  std::string out_dir = "output/openni2_snapshot";
  std::string uri;
  int warmup_frames = 5;
  int timeout_ms = 3000;
  std::optional<int> width;
  std::optional<int> height;
  std::optional<int> fps;
  uint16_t viz_clip_max_mm = 5000;
};

static void printUsage(const char *argv0) {
  std::cerr
      << "Usage: " << argv0 << " [--out-dir DIR] [--uri URI] [--warmup-frames N] [--timeout-ms MS]\n"
      << "              [--width W --height H --fps FPS] [--viz-clip-max-mm MM]\n"
      << "\n"
      << "Writes:\n"
      << "  depth_<stamp>.pgm      (16-bit PGM, millimeters)\n"
      << "  depth_<stamp>_viz.pgm  (8-bit PGM quick view)\n";
}

static std::optional<std::string> argValue(int &i, int argc, char **argv) {
  if (i + 1 >= argc) {
    return std::nullopt;
  }
  i++;
  return std::string(argv[i]);
}

static std::optional<int> parseInt(const std::string &s) {
  try {
    size_t pos = 0;
    const int v = std::stoi(s, &pos, 10);
    if (pos != s.size()) {
      return std::nullopt;
    }
    return v;
  } catch (...) {
    return std::nullopt;
  }
}

int main(int argc, char **argv) {
  Args args;
  for (int i = 1; i < argc; i++) {
    const std::string a(argv[i]);
    if (a == "--help" || a == "-h") {
      printUsage(argv[0]);
      return 0;
    }
    if (a == "--out-dir") {
      const auto v = argValue(i, argc, argv);
      if (!v) {
        std::cerr << "[error] --out-dir requires a value\n";
        return 2;
      }
      args.out_dir = *v;
      continue;
    }
    if (a == "--uri") {
      const auto v = argValue(i, argc, argv);
      if (!v) {
        std::cerr << "[error] --uri requires a value\n";
        return 2;
      }
      args.uri = *v;
      continue;
    }
    if (a == "--warmup-frames") {
      const auto v = argValue(i, argc, argv);
      if (!v) {
        std::cerr << "[error] --warmup-frames requires a value\n";
        return 2;
      }
      const auto parsed = parseInt(*v);
      if (!parsed || *parsed < 0) {
        std::cerr << "[error] Invalid --warmup-frames: " << *v << "\n";
        return 2;
      }
      args.warmup_frames = *parsed;
      continue;
    }
    if (a == "--timeout-ms") {
      const auto v = argValue(i, argc, argv);
      if (!v) {
        std::cerr << "[error] --timeout-ms requires a value\n";
        return 2;
      }
      const auto parsed = parseInt(*v);
      if (!parsed || *parsed <= 0) {
        std::cerr << "[error] Invalid --timeout-ms: " << *v << "\n";
        return 2;
      }
      args.timeout_ms = *parsed;
      continue;
    }
    if (a == "--width") {
      const auto v = argValue(i, argc, argv);
      if (!v) {
        std::cerr << "[error] --width requires a value\n";
        return 2;
      }
      const auto parsed = parseInt(*v);
      if (!parsed || *parsed <= 0) {
        std::cerr << "[error] Invalid --width: " << *v << "\n";
        return 2;
      }
      args.width = *parsed;
      continue;
    }
    if (a == "--height") {
      const auto v = argValue(i, argc, argv);
      if (!v) {
        std::cerr << "[error] --height requires a value\n";
        return 2;
      }
      const auto parsed = parseInt(*v);
      if (!parsed || *parsed <= 0) {
        std::cerr << "[error] Invalid --height: " << *v << "\n";
        return 2;
      }
      args.height = *parsed;
      continue;
    }
    if (a == "--fps") {
      const auto v = argValue(i, argc, argv);
      if (!v) {
        std::cerr << "[error] --fps requires a value\n";
        return 2;
      }
      const auto parsed = parseInt(*v);
      if (!parsed || *parsed <= 0) {
        std::cerr << "[error] Invalid --fps: " << *v << "\n";
        return 2;
      }
      args.fps = *parsed;
      continue;
    }
    if (a == "--viz-clip-max-mm") {
      const auto v = argValue(i, argc, argv);
      if (!v) {
        std::cerr << "[error] --viz-clip-max-mm requires a value\n";
        return 2;
      }
      const auto parsed = parseInt(*v);
      if (!parsed || *parsed <= 0 || *parsed > 65535) {
        std::cerr << "[error] Invalid --viz-clip-max-mm: " << *v << "\n";
        return 2;
      }
      args.viz_clip_max_mm = static_cast<uint16_t>(*parsed);
      continue;
    }

    std::cerr << "[error] Unknown arg: " << a << "\n";
    printUsage(argv[0]);
    return 2;
  }

  const auto rc = openni::OpenNI::initialize();
  if (rc != openni::STATUS_OK) {
    std::cerr << "[error] OpenNI init failed: " << openni::OpenNI::getExtendedError() << "\n";
    return 1;
  }

  openni::Device device;
  const auto open_rc = args.uri.empty() ? device.open(openni::ANY_DEVICE) : device.open(args.uri.c_str());
  if (open_rc != openni::STATUS_OK) {
    std::cerr << "[error] OpenNI open failed: " << openni::OpenNI::getExtendedError() << "\n";
    openni::OpenNI::shutdown();
    return 1;
  }

  if (!device.hasSensor(openni::SENSOR_DEPTH)) {
    std::cerr << "[error] Device has no depth sensor\n";
    device.close();
    openni::OpenNI::shutdown();
    return 1;
  }

  openni::VideoStream depth;
  const auto create_rc = depth.create(device, openni::SENSOR_DEPTH);
  if (create_rc != openni::STATUS_OK) {
    std::cerr << "[error] Failed to create depth stream: " << openni::OpenNI::getExtendedError() << "\n";
    device.close();
    openni::OpenNI::shutdown();
    return 1;
  }

  if (args.width && args.height && args.fps) {
    openni::VideoMode mode = depth.getVideoMode();
    mode.setResolution(*args.width, *args.height);
    mode.setFps(*args.fps);
    mode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
    const auto mode_rc = depth.setVideoMode(mode);
    if (mode_rc != openni::STATUS_OK) {
      std::cerr << "[warn] Failed to set requested depth mode; continuing with default: "
                << openni::OpenNI::getExtendedError() << "\n";
    }
  }

  const auto start_rc = depth.start();
  if (start_rc != openni::STATUS_OK) {
    std::cerr << "[error] Failed to start depth stream: " << openni::OpenNI::getExtendedError() << "\n";
    depth.destroy();
    device.close();
    openni::OpenNI::shutdown();
    return 1;
  }

  const auto stamp = nowStamp();
  fs::path out_dir(args.out_dir);
  std::error_code ec;
  fs::create_directories(out_dir, ec);
  if (ec) {
    std::cerr << "[error] Failed to create out dir " << out_dir.string() << ": " << ec.message() << "\n";
    depth.stop();
    depth.destroy();
    device.close();
    openni::OpenNI::shutdown();
    return 1;
  }

  openni::VideoFrameRef frame;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(args.timeout_ms);
  int captured = 0;
  int ignored = 0;
  while (std::chrono::steady_clock::now() < deadline) {
    int ready = 0;
    openni::VideoStream *streams[] = {&depth};
    const auto wait_rc = openni::OpenNI::waitForAnyStream(streams, 1, &ready, 200);
    if (wait_rc != openni::STATUS_OK) {
      continue;
    }
    if (ready != 0) {
      continue;
    }
    const auto read_rc = depth.readFrame(&frame);
    if (read_rc != openni::STATUS_OK || !frame.isValid()) {
      continue;
    }
    if (ignored < args.warmup_frames) {
      ignored++;
      continue;
    }
    captured++;
    break;
  }

  if (captured <= 0) {
    std::cerr << "[error] No depth frame captured within timeout (" << args.timeout_ms << " ms)\n";
    depth.stop();
    depth.destroy();
    device.close();
    openni::OpenNI::shutdown();
    return 3;
  }

  const int width = frame.getWidth();
  const int height = frame.getHeight();
  const auto pf = frame.getVideoMode().getPixelFormat();
  const auto *src = static_cast<const openni::DepthPixel *>(frame.getData());
  const size_t count = static_cast<size_t>(width) * static_cast<size_t>(height);

  std::vector<uint16_t> mm(count);
  if (pf == openni::PIXEL_FORMAT_DEPTH_1_MM) {
    for (size_t i = 0; i < count; i++) {
      mm[i] = static_cast<uint16_t>(src[i]);
    }
  } else if (pf == openni::PIXEL_FORMAT_DEPTH_100_UM) {
    for (size_t i = 0; i < count; i++) {
      mm[i] = static_cast<uint16_t>(static_cast<uint32_t>(src[i]) / 10u);
    }
  } else {
    std::cerr << "[warn] Unexpected pixel format " << static_cast<int>(pf) << "; writing raw values as mm\n";
    for (size_t i = 0; i < count; i++) {
      mm[i] = static_cast<uint16_t>(src[i]);
    }
  }

  const Stats s = statsU16(mm);
  if (s.zeros == s.total) {
    std::cerr << "[stats] depth: ALL ZERO (" << s.total << " px)\n";
  } else {
    std::cerr << "[stats] depth: total=" << s.total << " zeros=" << s.zeros << " min_nonzero=" << s.min_nonzero
              << " max=" << s.max_val << " (mm)\n";
  }

  const fs::path depth_path = out_dir / ("depth_" + stamp + ".pgm");
  const fs::path viz_path = out_dir / ("depth_" + stamp + "_viz.pgm");
  try {
    writePgm16Be(depth_path, width, height, mm);
    const auto viz = depthViz8(mm, args.viz_clip_max_mm);
    writePgm8(viz_path, width, height, viz);
  } catch (const std::exception &e) {
    std::cerr << "[error] Failed writing output: " << e.what() << "\n";
    depth.stop();
    depth.destroy();
    device.close();
    openni::OpenNI::shutdown();
    return 1;
  }

  std::cout << "depth -> " << depth_path.string() << "\n";
  std::cout << "viz   -> " << viz_path.string() << "\n";

  depth.stop();
  depth.destroy();
  device.close();
  openni::OpenNI::shutdown();
  return 0;
}
