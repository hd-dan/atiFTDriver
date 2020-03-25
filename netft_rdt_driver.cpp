/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "netft_rdt_driver.h"
#include <stdint.h>
#include <exception>

using boost::asio::ip::udp;

namespace netft_rdt_driver
{

struct RDTRecord
{
  uint32_t rdt_sequence_;
  uint32_t ft_sequence_;
  uint32_t status_;
  int32_t fx_;
  int32_t fy_;
  int32_t fz_;
  int32_t tx_;
  int32_t ty_;
  int32_t tz_;

  enum {RDT_RECORD_SIZE = 36};
  void unpack(const uint8_t *buffer);
  static uint32_t unpack32(const uint8_t *buffer);
};

uint32_t RDTRecord::unpack32(const uint8_t *buffer)
{
  return
    ( uint32_t(buffer[0]) << 24) |     
    ( uint32_t(buffer[1]) << 16) |     
    ( uint32_t(buffer[2]) << 8 ) |     
    ( uint32_t(buffer[3]) << 0 ) ;
}

void RDTRecord::unpack(const uint8_t *buffer)
{
  rdt_sequence_ = unpack32(buffer + 0);
  ft_sequence_  = unpack32(buffer + 4);
  status_       = unpack32(buffer + 8);
  fx_ = int32_t(unpack32(buffer + 12));
  fy_ = int32_t(unpack32(buffer + 16));
  fz_ = int32_t(unpack32(buffer + 20));
  tx_ = int32_t(unpack32(buffer + 24));
  ty_ = int32_t(unpack32(buffer + 28));
  tz_ = int32_t(unpack32(buffer + 32));
}


struct RDTCommand{
  uint16_t command_header_;
  uint16_t command_;
  uint32_t sample_count_;

  RDTCommand() : command_header_(HEADER){
    // empty
  }

  enum {HEADER=0x1234};

  // Possible values for command_
  enum {
    CMD_STOP_STREAMING=0, 
    CMD_START_HIGH_SPEED_STREAMING=2,
    // More command values are available but are not used by this driver
  };

  // Special values for sample count
  enum { INFINITE_SAMPLES=0 };

  enum {RDT_COMMAND_SIZE = 8};

  //!Packet structure into buffer for network transport
  //  Buffer should be RDT_COMMAND_SIZE
  void pack(uint8_t *buffer) const;
};

void RDTCommand::pack(uint8_t *buffer) const
{
  // Data is big-endian
  buffer[0] = (command_header_ >> 8) & 0xFF;
  buffer[1] = (command_header_ >> 0) & 0xFF;
  buffer[2] = (command_ >> 8) & 0xFF;
  buffer[3] = (command_ >> 0) & 0xFF;
  buffer[4] = (sample_count_ >> 8) & 0xFF;
  buffer[5] = (sample_count_ >> 0) & 0xFF;
  buffer[6] = (sample_count_ >> 8) & 0xFF;
  buffer[7] = (sample_count_ >> 0) & 0xFF;
}


//NetFTRDTDriver::NetFTRDTDriver(const std::string &address) :
//  address_(address),
NetFTRDTDriver::NetFTRDTDriver(const std::string &configPath) :
  configPath_(configPath),fCali_(0),
  socket_(io_service_),
  stop_recv_thread_(false),
  recv_thread_running_(false),
  packet_count_(0),
  lost_packets_(0),
  out_of_order_count_(0),
  seq_counter_(0),
  diag_packet_count_(0),
  last_rdt_sequence_(0),
  system_status_(0){

  NetFTRDTDriver::setConfig(configPath);
}

NetFTRDTDriver::~NetFTRDTDriver(){
  // TODO stop transmission, 
  // stop thread
  stop_recv_thread_ = true;
  if (!recv_thread_.timed_join(boost::posix_time::time_duration(0,0,1,0)))
  {
    printf("Interrupting recv thread\n");
    recv_thread_.interrupt();
    if (!recv_thread_.timed_join(boost::posix_time::time_duration(0,0,1,0)))
    {
      printf("Failed second join to recv thread\n");
    }
  }
  socket_.close();
}


NetFTRDTDriver::NetFTRDTDriver() :
  fCali_(0),
  socket_(io_service_),
  stop_recv_thread_(false),
  recv_thread_running_(false),
  packet_count_(0),
  lost_packets_(0),
  out_of_order_count_(0),
  seq_counter_(0),
  diag_packet_count_(0),
  last_rdt_sequence_(0),
  system_status_(0){

}

void NetFTRDTDriver::setConfig(std::string configPath){
    configPath_= configPath;

    double forceCount,torqueCount;
    bias_= std::vector<double>(6,0);
    slope_= std::vector<double>(6,0);
    calibration_com_= {0.2,0.2,0.5};
    configPath_= NetFTRDTDriver::processPath(configPath_);
    if (!std::ifstream(configPath_).good()){
        address_= "192.168.1.108";
        forceCount = 10000000;
        torqueCount = 10000000;
        slope_= std::vector<double>(6,1);
        bias_= std::vector<double>(6,0);
        calibration_com_= std::vector<double>(3,0);

        printf("Config File not exist!\n"
               "Default ft ip= %s\n"
               "Count_per_force: %.0f | Count_per_torque: %.0f\n",
               address_.c_str(),forceCount,torqueCount);
    }else{
        configFile_= xml_file(configPath_);
        address_= configFile_.getXmlVal<std::string>("ft.ip");
        forceCount= configFile_.getXmlVal<double>("ft.forceCount",1e6);
        torqueCount= configFile_.getXmlVal<double>("ft.torqueCount",1e6);

        if (configFile_.checkElementExist("ft.bias"))
            bias_= configFile_.getXmlVect<double>("ft.bias");
        else
            bias_= std::vector<double>(6,0);
        for (int i=int(bias_.size());i<6;i++)
            bias_.push_back(0);

        if (configFile_.checkElementExist("ft.slope"))
            slope_= configFile_.getXmlVect<double>("ft.slope");
        else
            slope_= std::vector<double>(6,1);
        for (int i=int(slope_.size());i<6;i++)
            slope_.push_back(1);

        if (configFile_.checkElementExist("ft.com4cal"))
            calibration_com_= configFile_.getXmlVect<double>("ft.com4cal");
        for (int i=int(calibration_com_.size());i<3;i++)
            calibration_com_.push_back(0);

        printf("Read Ft Config File\n");
        printf("Slope: [");
        for (unsigned int i=0;i<slope_.size();i++)
            printf("%.2f, ",slope_.at(i));
        printf("]\n");
        printf("Bias: [");
        for (unsigned int i=0;i<bias_.size();i++)
            printf("%.2f, ",bias_.at(i));
        printf("]\n");
  //      printf("Force Count: %.f | Torque Count: %.f\n",forceCount, torqueCount);
    }

    // Construct UDP socket
    udp::endpoint netft_endpoint( boost::asio::ip::address_v4::from_string(address_), RDT_PORT);
    socket_.open(udp::v4());
    socket_.connect(netft_endpoint);

    // TODO : Get/Set Force/Torque scale for device
    // Force/Sclae is based on counts per force/torque value from device
    // these value are manually read from device webserver, but in future they
    // may be collected using http get requests
  //  static const double counts_per_force = 1000000;
  //  static const double counts_per_torque = 1000000;

    static const double counts_per_force = forceCount;
    static const double counts_per_torque = torqueCount;
    force_scale_ = 1.0 / counts_per_force;
    torque_scale_ = 1.0 / counts_per_torque;
    printf("force_scale: %.9f | torque_scale: %.9f\n",force_scale_,torque_scale_);

    // Start receive thread
    recv_thread_ = boost::thread(&NetFTRDTDriver::recvThreadFunc, this);

    // Since start steaming command is sent with UDP packet,
    // the packet could be lost, retry startup 10 times before giving up
    for (int i=0; i<10; ++i){
      startStreaming();
      if (waitForNewData())
        break;
    }
    { boost::unique_lock<boost::mutex> lock(mutex_);
      if (packet_count_ == 0){
        throw std::runtime_error("No data received from NetFT device");
      }
    }
}

std::string NetFTRDTDriver::processPath(std::string path){
    if (path.at(0)=='~')
        path= getenv("HOME")+path.substr(1);
    if (path.find("..")==0){
//        std::string pwd= getenv("PWD");
        char tuh[PATH_MAX];
        std::string pwd=getcwd(tuh,sizeof(tuh));
        do{
            pwd= pwd.substr(0,pwd.rfind("/"));
            path= path.substr(path.find("..")+2);
        }while(path.find("..")<2);
        path= pwd+path;

    }else if(path.at(0)=='.')
        path= getenv("PWD")+path.substr(1);
    if (path.find("/")==path.npos)
        path= getenv("PWD")+std::string("/")+path;
    configPath_= path;
    NetFTRDTDriver::processDirectory(configPath_);
    return configPath_;
}
void NetFTRDTDriver::processDirectory(std::string dir){
    dir= dir.substr(0,dir.rfind('/'));
    struct stat st;
    if (stat(dir.c_str(),&st)!=0){
        NetFTRDTDriver::processDirectory(dir);
        mkdir(dir.c_str(),S_IRWXU);
    }
    return;
}

std::vector<double> NetFTRDTDriver::getCom(){
    return calibration_com_;
}

std::vector<double> NetFTRDTDriver::getScale(){
    return std::vector<double>{force_scale_,torque_scale_};
}

std::vector<double> NetFTRDTDriver::getBias(){
    return bias_;
}

//void NetFTRDTDriver::setBias(std::vector<double> slope, std::vector<double> bias, bool calMode){
//    if (calMode){
//        for (unsigned int i=0; i<bias.size();i++){
//            double bias_i= bias.at(i);
//            if (i<3) bias_.at(i)= bias_i;
//            else bias_.at(i)= bias_i;
//        }
//    }else{
//        for (unsigned int i=0; i<bias.size();i++){
//            double bias_i= bias.at(i);
//            bias_.at(i)= bias_.at(i)-bias_i;
//        }
//    }

//    return;
//} // <---------Need

void NetFTRDTDriver::setCalibration(std::vector<std::vector<double> > mc, bool calMode){
    if (calMode){
        slope_= mc.at(0);
        bias_= mc.at(1);
    }else{
        for (unsigned int i=0;i<bias_.size();i++)
            bias_.at(i)+= slope_.at(i)*mc.at(1).at(i);
        for (unsigned int i=0;i<slope_.size();i++)
            slope_.at(i)*= mc.at(0).at(i);
    }
}

void NetFTRDTDriver::saveCalibration(){
    boost::property_tree::ptree pt= configFile_.getPtree();
    std::vector<std::string> strPool= {"x","y","z","tx","ty","tz"};
    for (unsigned int i=0; i<bias_.size();i++){
        pt.put("ft.bias."+strPool.at(i), bias_.at(i) );
    }
    for (unsigned int i=0; i<slope_.size();i++){
        pt.put("ft.slope."+strPool.at(i), slope_.at(i) );
    }
    for (unsigned int i=0; i<calibration_com_.size();i++){
        pt.put("ft.com4cal."+strPool.at(i), calibration_com_.at(i) );
    }
    pt.put("ft.ip",address_);
    pt.put("ft.forceCount",int(1./force_scale_));
    pt.put("ft.torqueCount",int(1./torque_scale_));
    configFile_.writeFile(configPath_,pt);
}

void NetFTRDTDriver::saveCalibration(std::vector<std::vector<double> > mc){
    NetFTRDTDriver::setCalibration(mc);
    NetFTRDTDriver::saveCalibration();
    return;
}

bool NetFTRDTDriver::waitForNewData(){
  // Wait upto 100ms for new data
  bool got_new_data = false;
  {
    boost::mutex::scoped_lock lock(mutex_);
    unsigned current_packet_count = packet_count_;
    condition_.timed_wait(lock, boost::posix_time::milliseconds(100));    
    got_new_data = packet_count_ != current_packet_count;
  }

  return got_new_data;
}


void NetFTRDTDriver::startStreaming(void){
  // Command NetFT to start data transmission
  RDTCommand start_transmission; 
  start_transmission.command_ = RDTCommand::CMD_START_HIGH_SPEED_STREAMING;
  start_transmission.sample_count_ = RDTCommand::INFINITE_SAMPLES;
  // TODO change buffer into boost::array
  uint8_t buffer[RDTCommand::RDT_COMMAND_SIZE];
  start_transmission.pack(buffer);
  socket_.send(boost::asio::buffer(buffer, RDTCommand::RDT_COMMAND_SIZE)); 
}


void NetFTRDTDriver::setCalibrateMode(bool calMode){
    fCali_= calMode;
    return;
}

void NetFTRDTDriver::recvThreadFunc(){
  try{
    recv_thread_running_ = true;
    RDTRecord rdt_record;
    std::vector<double> tmp_data(6,0);
    uint8_t buffer[RDTRecord::RDT_RECORD_SIZE+1];
    while (!stop_recv_thread_){
      size_t len = socket_.receive(boost::asio::buffer(buffer, RDTRecord::RDT_RECORD_SIZE+1));
      if (len != RDTRecord::RDT_RECORD_SIZE){
        printf("Receive size of %d bytes does not match expected size of %d\n",
                 int(len), int(RDTRecord::RDT_RECORD_SIZE));
      }
      else{
        rdt_record.unpack(buffer);
        if (rdt_record.status_ != 0){
          // Latch any system status error code
          boost::unique_lock<boost::mutex> lock(mutex_);
          system_status_ = rdt_record.status_;
        }
        int32_t seqdiff = int32_t(rdt_record.rdt_sequence_ - last_rdt_sequence_);
        last_rdt_sequence_ = rdt_record.rdt_sequence_;
        if (seqdiff < 1){
            boost::unique_lock<boost::mutex> lock(mutex_);
          // Don't use data that is old
          ++out_of_order_count_;
        }
        else{

            if (fCali_){
                tmp_data.at(0)= rdt_record.fx_*force_scale_;
                tmp_data.at(1)= rdt_record.fy_*force_scale_;
                tmp_data.at(2)= rdt_record.fz_*force_scale_;
                tmp_data.at(3)= rdt_record.tx_*torque_scale_;
                tmp_data.at(4)= rdt_record.ty_*torque_scale_;
                tmp_data.at(5)= rdt_record.tz_*torque_scale_;
            }else{
                tmp_data.at(0)= ( force_scale_*double(rdt_record.fx_) + bias_.at(0) ) *slope_.at(0);
                tmp_data.at(1)= ( force_scale_*double(rdt_record.fy_) + bias_.at(1) ) *slope_.at(1);
                tmp_data.at(2)= ( force_scale_*double(rdt_record.fz_) + bias_.at(2) ) *slope_.at(2);
                tmp_data.at(3)= ( torque_scale_*double(rdt_record.tx_) + bias_.at(3) ) *slope_.at(3);
                tmp_data.at(4)= ( torque_scale_*double(rdt_record.ty_) + bias_.at(4) ) *slope_.at(4);
                tmp_data.at(5)= ( torque_scale_*double(rdt_record.tz_) + bias_.at(5) ) *slope_.at(5);
            }

          { boost::unique_lock<boost::mutex> lock(mutex_);
            new_data_ = tmp_data;
            lost_packets_ += unsigned(seqdiff - 1);
            ++packet_count_;
            condition_.notify_all();
          }
        }
      }
    } // end while
  }
  catch (std::exception &e)
  {    
    recv_thread_running_ = false;
    { boost::unique_lock<boost::mutex> lock(mutex_);
      recv_thread_error_msg_ = e.what();
    }
  }
}


std::vector<double> NetFTRDTDriver::getData()
{
  std::vector<double> data;
  { boost::unique_lock<boost::mutex> lock(mutex_);
    data = new_data_;
  }  
  return data;
}


//void NetFTRDTDriver::diagnostics(diagnostic_updater::DiagnosticStatusWrapper &d)
//{
//  // Publish diagnostics
//  d.name = "NetFT RDT Driver : " + address_;
  
//  d.summary(d.OK, "OK");
//  d.hardware_id = "0";

//  if (diag_packet_count_ == packet_count_)
//  {
//    d.mergeSummary(d.ERROR, "No new data in last second");
//  }

//  if (!recv_thread_running_)
//  {
//    d.mergeSummaryf(d.ERROR, "Receive thread has stopped : %s", recv_thread_error_msg_.c_str());
//  }

//  if (system_status_ != 0)
//  {
//    d.mergeSummaryf(d.ERROR, "NetFT reports error 0x%08x", system_status_);
//  }

//  ros::Time current_time(ros::Time::now());
//  double recv_rate = double(int32_t(packet_count_ - diag_packet_count_)) / (current_time - last_diag_pub_time_).toSec();
    
//  d.clear();
//  d.addf("IP Address", "%s", address_.c_str());
//  d.addf("System status", "0x%08x", system_status_);
//  d.addf("Good packets", "%u", packet_count_);
//  d.addf("Lost packets", "%u", lost_packets_);
//  d.addf("Out-of-order packets", "%u", out_of_order_count_);
//  d.addf("Recv rate (pkt/sec)", "%.2f", recv_rate);
//  d.addf("Force scale (N/bit)", "%f", force_scale_);
//  d.addf("Torque scale (Nm/bit)", "%f", torque_scale_);

//  geometry_msgs::WrenchStamped data;
//  getData(data);
//  d.addf("Force X (N)",   "%f", data.wrench.force.x);
//  d.addf("Force Y (N)",   "%f", data.wrench.force.y);
//  d.addf("Force Z (N)",   "%f", data.wrench.force.z);
//  d.addf("Torque X (Nm)", "%f", data.wrench.torque.x);
//  d.addf("Torque Y (Nm)", "%f", data.wrench.torque.y);
//  d.addf("Torque Z (Nm)", "%f", data.wrench.torque.z);

//  last_diag_pub_time_ = current_time;
//  diag_packet_count_ = packet_count_;
//}


} // end namespace netft_rdt_driver

