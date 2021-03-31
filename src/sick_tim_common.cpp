/*
 * Copyright (C) 2013, Osnabrück University
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Osnabrück University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: 24.05.2012
 *
 *      Authors:
 *         Jochen Sprickerhof <jochen@sprickerhof.de>
 *         Martin Günther <mguenthe@uos.de>
 *
 * Based on the TiM communication example by SICK AG.
 *
 */

#include <sick_tim/sick_tim_common.h>

#include <cstdio>
#include <cstring>

namespace sick_tim
{

SickTimCommon::SickTimCommon(AbstractParser* parser, rclcpp::Node::SharedPtr node, std::shared_ptr<diagnostic_updater::Updater> diagnostics) :
    diagnosticPub_(NULL), expectedFrequency_(15.0), parser_(parser), diagnostics_(diagnostics)
    // FIXME All Tims have 15Hz?
{
  node_ = node;

  node_->set_on_parameters_set_callback(
      std::bind(&SickTimCommon::onParameterEvent, this, std::placeholders::_1));

  // datagram publisher (only for debug)
  publish_datagram_ = node_->get_parameter("publish_datagram").as_bool();

  // Declare Sick Tim Parameters
  config_.min_ang = node_->get_parameter("min_ang").as_double();
  config_.max_ang = node_->get_parameter("max_ang").as_double();
  config_.intensity = node_->get_parameter("intensity").as_bool();
  config_.skip = node_->get_parameter("skip").as_int();
  config_.frame_id = node_->get_parameter("frame_id").as_string();
  config_.time_offset = node_->get_parameter("time_offset").as_double();
  config_.auto_reboot = node_->get_parameter("auto_reboot").as_bool();
  
  if (publish_datagram_)
  {
    datagram_pub_ = node_->create_publisher<example_interfaces::msg::String>("datagram", 1000);
  }

  // scan publisher

  pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>("scan", 1000);
  diagnosticPub_ = new diagnostic_updater::DiagnosedPublisher<sensor_msgs::msg::LaserScan>(pub_, *diagnostics_,
          // frequency should be target +- 10%.
          diagnostic_updater::FrequencyStatusParam(&expectedFrequency_, &expectedFrequency_, 0.1, 10),
          // timestamp delta can be from 0.0 to 1.3x what it ideally is.
          diagnostic_updater::TimeStampStatusParam(-1, 1.3 * 1.0/expectedFrequency_ - config_.time_offset));
  assert(diagnosticPub_ != NULL);
}

rcl_interfaces::msg::SetParametersResult SickTimCommon::onParameterEvent(
        const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto &parameter : parameters)
  {
    if (parameter.get_name() == "min_ang")
    {
      double value = parameter.as_double();
      if (value < -0.75 * M_PI || value > 0.75 * M_PI){
        result.successful = false;
        result.reason = "Minimum angle outside limits: [-0.75*pi,0.75*pi]. Parameter left unchanged";
        return result;
      }
      RCLCPP_INFO(node_->get_logger(), "Parameter 'min_ang' changed to %f", value);
    }
    if (parameter.get_name() == "max_ang")
    {
      double value = parameter.as_double();
      if (value < -0.75 * M_PI || value > 0.75 * M_PI){
        result.successful = false;
        result.reason = "Maximum angle outside limits: [-0.75*pi,0.75*pi]. Parameter left unchanged";
        return result;
      }
      RCLCPP_INFO(node_->get_logger(), "Parameter 'max_ang' changed to %f", value);
    }
    if (parameter.get_name() == "intensity")
    {
      bool value = parameter.as_bool();
      RCLCPP_INFO(node_->get_logger(), "Parameter 'intensity' changed to %s ", value ? "True" : "False");
    }
    if (parameter.get_name() == "skip")
    {
      int value = parameter.as_int();
      if (value < 0 || value > 9){
        result.successful = false;
        result.reason = "Skip outside limits: [0,9]. Parameter left unchanged";
        return result;
      }
      RCLCPP_INFO(node_->get_logger(), "Parameter 'skip' changed to %i", value);
    }
    if (parameter.get_name() == "frame_id")
    {
      std::string value = parameter.as_string();
      RCLCPP_INFO(node_->get_logger(), "Parameter 'frame_id' changed to %s ", value);
    }
    if (parameter.get_name() == "time_offset")
    {
      double value = parameter.as_double();
      if (value < -0.25 || value > 0.25){
        result.successful = false;
        result.reason = "Time offset outside limits: [-0.25,0.25]. Parameter left unchanged";
        return result;
      }
      RCLCPP_INFO(node_->get_logger(), "Parameter 'time_offset' changed to %f", value);
    }
    if (parameter.get_name() == "auto_reboot")
    {
      bool value = parameter.as_bool();
      RCLCPP_INFO(node_->get_logger(), "Parameter 'auto_reboot' changed to %s ", value ? "True" : "False");
    }
  }

  return result;
}

int SickTimCommon::stop_scanner()
{
  /*
   * Stop streaming measurements
   */
  const char requestScanData0[] = {"\x02sEN LMDscandata 0\x03\0"};
  int result = sendSOPASCommand(requestScanData0, NULL);
  if (result != 0)
    // use printf because we cannot use ROS_ERROR from the destructor
    printf("\nSOPAS - Error stopping streaming scan data!\n");
  else
    printf("\nSOPAS - Stopped streaming scan data.\n");

  return result;
}

bool SickTimCommon::rebootScanner()
{
  /*
   * Set Maintenance access mode to allow reboot to be sent
   */
  std::vector<unsigned char> access_reply;
  int result = sendSOPASCommand("\x02sMN SetAccessMode 03 F4724744\x03\0", &access_reply);
  if (result != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "SOPAS - Error setting access mode");
    diagnostics_->broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "SOPAS - Error setting access mode.");
    return false;
  }
  std::string access_reply_str = replyToString(access_reply);
  if (access_reply_str != "sAN SetAccessMode 1")
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "SOPAS - Error setting access mode, unexpected response : " << access_reply_str);
    diagnostics_->broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "SOPAS - Error setting access mode.");
    return false;
  }

  /*
   * Send reboot command
   */
  std::vector<unsigned char> reboot_reply;
  result = sendSOPASCommand("\x02sMN mSCreboot\x03\0", &reboot_reply);
  if (result != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "SOPAS - Error rebooting scanner");
    diagnostics_->broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "SOPAS - Error rebooting device.");
    return false;
  }
  std::string reboot_reply_str = replyToString(reboot_reply);
  if (reboot_reply_str != "sAN mSCreboot")
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "SOPAS - Error rebooting scanner, unexpected response : " << reboot_reply_str);
    diagnostics_->broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "SOPAS - Error setting access mode.");
    return false;
  }

  RCLCPP_INFO(node_->get_logger(), "SOPAS - Rebooted scanner");

  // Wait a few seconds after rebooting
  rclcpp::sleep_for(std::chrono::seconds(15));

  return true;
}

SickTimCommon::~SickTimCommon()
{
  bool removed = diagnostics_->removeByName("/scan topic status");
  //RCLCPP_INFO(rclcpp::get_logger(""), "Removed Diagnostic:[%d]", removed);
  
  delete diagnosticPub_;

  printf("sick_tim driver exiting.\n");
}


int SickTimCommon::init()
{
  int result = init_device();
  if(result != 0) {
      RCLCPP_FATAL(node_->get_logger(), "Failed to init device: %d", result);
      return result;
  }
  result = init_scanner();
  if(result != 0) {
      RCLCPP_FATAL(node_->get_logger(), "Failed to init scanner: %d", result);
  }
  return result;
}

int SickTimCommon::init_scanner()
{
  /*
   * Read the SOPAS variable 'DeviceIdent' by index.
   */
  const char requestDeviceIdent[] = "\x02sRI0\x03\0";
  std::vector<unsigned char> identReply;
  int result = sendSOPASCommand(requestDeviceIdent, &identReply);
  if (result != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "SOPAS - Error reading variable 'DeviceIdent'.");
    diagnostics_->broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "SOPAS - Error reading variable 'DeviceIdent'.");
  }

  /*
   * Read the SOPAS variable 'SerialNumber' by name.
   */
  const char requestSerialNumber[] = "\x02sRN SerialNumber\x03\0";
  std::vector<unsigned char> serialReply;
  result = sendSOPASCommand(requestSerialNumber, &serialReply);
  if (result != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "SOPAS - Error reading variable 'SerialNumber'.");
    diagnostics_->broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "SOPAS - Error reading variable 'SerialNumber'.");
  }

  // set hardware ID based on DeviceIdent and SerialNumber
  std::string identStr = replyToString(identReply);
  std::string serialStr = replyToString(serialReply);
  diagnostics_->setHardwareID(identStr + " " + serialStr);

  if (!isCompatibleDevice(identStr))
    return ExitFatal;

  /*
   * Read the SOPAS variable 'FirmwareVersion' by name.
   */
  const char requestFirmwareVersion[] = {"\x02sRN FirmwareVersion\x03\0"};
  result = sendSOPASCommand(requestFirmwareVersion, NULL);
  if (result != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "SOPAS - Error reading variable 'FirmwareVersion'.");
    diagnostics_->broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "SOPAS - Error reading variable 'FirmwareVersion'.");
  }

  /*
   * Read Device State
   */
  const char requestDeviceState[] = {"\x02sRN SCdevicestate\x03\0"};
  std::vector<unsigned char> deviceStateReply;
  result = sendSOPASCommand(requestDeviceState, &deviceStateReply);
  if (result != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "SOPAS - Error reading variable 'devicestate'.");
    diagnostics_->broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "SOPAS - Error reading variable 'devicestate'.");
  }
  std::string deviceStateReplyStr = replyToString(deviceStateReply);

  /*
   * Process device state, 0=Busy, 1=Ready, 2=Error
   * If configuration parameter is set, try resetting device in error state
   */
  if (deviceStateReplyStr == "sRA SCdevicestate 0")
  {
    RCLCPP_WARN(node_->get_logger(), "Laser is busy");
  }
  else if (deviceStateReplyStr == "sRA SCdevicestate 1")
  {
    RCLCPP_DEBUG(node_->get_logger(), "Laser is ready");
  }
  else if (deviceStateReplyStr == "sRA SCdevicestate 2")
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Laser reports error state : " << deviceStateReplyStr);
    if (config_.auto_reboot)
    {
      rebootScanner();
    }
  }
  else
  {
    RCLCPP_WARN_STREAM(node_->get_logger(), "Laser reports unknown devicestate : " << deviceStateReplyStr);
  }

  /*
   * Start streaming 'LMDscandata'.
   */
  const char requestScanData[] = {"\x02sEN LMDscandata 1\x03\0"};
  result = sendSOPASCommand(requestScanData, NULL);
  if (result != 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "SOPAS - Error starting to stream 'LMDscandata'.");
    diagnostics_->broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "SOPAS - Error starting to stream 'LMDscandata'.");
    return ExitError;
  }

  return ExitSuccess;
}

std::string sick_tim::SickTimCommon::replyToString(const std::vector<unsigned char> &reply)
{
  std::string reply_str;
  for (std::vector<unsigned char>::const_iterator it = reply.begin(); it != reply.end(); it++)
  {
    if (*it > 13) // filter control characters for display
    {
      reply_str.push_back(*it);
    }
  }
  return reply_str;
}

bool sick_tim::SickTimCommon::isCompatibleDevice(const std::string identStr) const
{
  char device_string[7];
  int version_major = -1;
  int version_minor = -1;

  if (sscanf(identStr.c_str(), "sRA 0 6 %6s E V%d.%d", device_string,
             &version_major, &version_minor) == 3
      && strncmp("TiM3", device_string, 4) == 0
      && version_major >= 2 && version_minor >= 50)
  {
    RCLCPP_ERROR(node_->get_logger(), "This scanner model/firmware combination does not support ranging output!");
    RCLCPP_ERROR(node_->get_logger(), "Supported scanners: TiM5xx: all firmware versions; TiM3xx: firmware versions < V2.50.");
    RCLCPP_ERROR(node_->get_logger(), "This is a %s, firmware version %d.%d", device_string, version_major, version_minor);

    return false;
  }
  return true;
}

int SickTimCommon::loopOnce()
{

  diagnostics_->force_update();

  unsigned char receiveBuffer[65536];
  int actual_length = 0;
  static unsigned int iteration_count = 0;

  int result = get_datagram(receiveBuffer, 65536, &actual_length);
  if (result != 0)
  {
      RCLCPP_ERROR(node_->get_logger(), "Read Error when getting datagram: %i.", result);
      diagnostics_->broadcast(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Read Error when getting datagram.");
      return ExitError; // return failure to exit node
  }
  if(actual_length <= 0)
      return ExitSuccess; // return success to continue looping

  // ----- if requested, skip frames
  if (iteration_count++ % (config_.skip + 1) != 0)
    return ExitSuccess;

  if (publish_datagram_)
  {
    example_interfaces::msg::String datagram_msg;
    datagram_msg.data = std::string(reinterpret_cast<char*>(receiveBuffer));
    datagram_pub_->publish(datagram_msg);
  }

  sensor_msgs::msg::LaserScan msg;

  /*
   * datagrams are enclosed in <STX> (0x02), <ETX> (0x03) pairs
   */
  char* buffer_pos = (char*)receiveBuffer;
  char *dstart, *dend;
  while( (dstart = strchr(buffer_pos, 0x02)) && (dend = strchr(dstart + 1, 0x03)) )
  {
    size_t dlength = dend - dstart;
    *dend = '\0';
    dstart++;
    int success = parser_->parse_datagram(dstart, dlength, config_, msg);
    if (success == ExitSuccess)
      diagnosticPub_->publish(msg);
    buffer_pos = dend + 1;
  }

  return ExitSuccess; // return success to continue looping
}

} /* namespace sick_tim */
