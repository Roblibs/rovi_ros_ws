#include <iostream>
#include <string>
#include <libobsensor/hpp/Device.hpp>
#include <libobsensor/hpp/Context.hpp>
#include <libobsensor/hpp/Version.hpp>

int main(int argc, char **argv) {
  const std::string default_config_path =
      "/opt/ros/jazzy/share/orbbec_camera/config/OrbbecSDKConfig_v2.0.xml";
  const std::string config_path = (argc >= 2) ? argv[1] : default_config_path;

  std::cout << "OpenOrbbecSDK version: "
            << ob::Version::getMajor() << "."
            << ob::Version::getMinor() << "."
            << ob::Version::getPatch()
            << " (" << ob::Version::getStageVersion() << ")\n";
  std::cout << "Config path: " << config_path << "\n";

  ob::Context ctx(config_path.c_str());
  auto list = ctx.queryDeviceList();
  std::cout << "deviceCount=" << list->deviceCount() << "\n";
  for (uint32_t i = 0; i < list->deviceCount(); ++i) {
    std::cout << "i=" << i
              << " name=" << list->name(i)
              << " vid=0x" << std::hex << list->vid(i)
              << " pid=0x" << std::hex << list->pid(i) << std::dec
              << " uid=" << list->uid(i)
              << " sn=" << list->serialNumber(i)
              << " conn=" << list->connectionType(i)
              << "\n";
  }
}
