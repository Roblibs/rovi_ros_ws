// List OpenNI2 video modes for each sensor (depth/IR/color).
// Uses the Orbbec OpenNI2 SDK (see docs/depth_camera_astra_stereo_s_u3.md for install).

#include <OpenNI.h>

#include <iomanip>
#include <iostream>
#include <string>

static const char *pixelFormatToString(openni::PixelFormat fmt) {
  switch (fmt) {
    case openni::PIXEL_FORMAT_DEPTH_1_MM:
      return "Depth 1mm";
    case openni::PIXEL_FORMAT_DEPTH_100_UM:
      return "Depth 100um";
    case openni::PIXEL_FORMAT_SHIFT_9_2:
      return "Shift 9.2";
    case openni::PIXEL_FORMAT_SHIFT_9_3:
      return "Shift 9.3";
    case openni::PIXEL_FORMAT_RGB888:
      return "RGB888";
    case openni::PIXEL_FORMAT_YUV422:
      return "YUV422";
    case openni::PIXEL_FORMAT_GRAY8:
      return "Gray8";
    case openni::PIXEL_FORMAT_GRAY16:
      return "Gray16";
    case openni::PIXEL_FORMAT_JPEG:
      return "JPEG";
    default:
      return "Unknown";
  }
}

static void printSensorModes(openni::Device &device, openni::SensorType sensor,
                             const char *label) {
  const auto n = device.getSensorInfo(sensor);
  if (!n) {
    std::cout << label << ": (not available)\n";
    return;
  }
  std::cout << label << ":\n";
  const auto &modes = n->getSupportedVideoModes();
  for (int i = 0; i < modes.getSize(); ++i) {
    const auto &vm = modes[i];
    std::cout << "  - " << vm.getResolutionX() << "x" << vm.getResolutionY() << "@"
              << vm.getFps() << "Hz"
              << " format=" << pixelFormatToString(vm.getPixelFormat()) << "\n";
  }
}

int main(int argc, char **argv) {
  const std::string uri = (argc >= 2) ? argv[1] : std::string();

  const auto rc = openni::OpenNI::initialize();
  if (rc != openni::STATUS_OK) {
    std::cerr << "OpenNI init failed: " << openni::OpenNI::getExtendedError() << "\n";
    return 2;
  }

  openni::Device device;
  const auto open_rc = uri.empty() ? device.open(openni::ANY_DEVICE) : device.open(uri.c_str());
  if (open_rc != openni::STATUS_OK) {
    std::cerr << "OpenNI open failed: " << openni::OpenNI::getExtendedError() << "\n";
    openni::OpenNI::shutdown();
    return 2;
  }

  const auto info = device.getDeviceInfo();
  std::cout << "Device: " << info.getName() << " (vendor=" << info.getVendor()
            << ", uri=" << info.getUri() << ")\n";

  printSensorModes(device, openni::SENSOR_DEPTH, "Depth sensor video modes");
  printSensorModes(device, openni::SENSOR_IR, "IR sensor video modes");
  printSensorModes(device, openni::SENSOR_COLOR, "Color sensor video modes");

  device.close();
  openni::OpenNI::shutdown();
  return 0;
}
