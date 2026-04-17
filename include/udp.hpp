// Copyright (c) 2023, Wei Jian, Co.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef WLR716_UDP_UDP_HPP_
#define WLR716_UDP_UDP_HPP_

#include <iostream>
#include <condition_variable>
#include <mutex>
#include <stdio.h>
#include <time.h>
#include <string>
#include <unistd.h>
#include <pthread.h>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>

#include "rclcpp/rclcpp.hpp"
#include "protocol.hpp"


using namespace std;
using boost::asio::ip::udp;
using namespace boost::asio;

namespace wlr
{

/**
 * @class wlr:UDPTrans
 */
class UDPTrans : public rclcpp::Node
{
public:
  /**
   * @brief Constructor for wlr::Lidar
   */
  UDPTrans(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
    UDPTrans(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
   */
  ~UDPTrans();

  sensor_msgs::msg::LaserScan* get_scan(void);
  void set_scan_data(unsigned int findex, float *range, float *intens);

private:

  void init_param(void);
  void init_scan(void);
  ip::udp::socket* sock(io_service *srv, int port);
  int send(ip::udp::socket *sock, unsigned char *buffer, int size);
  void recv(ip::udp::socket *sock, Protocol *proto);
  void callback(ip::udp::socket *sock, Protocol *proto, const boost::system::error_code& error, std::size_t count);
  void run(io_service *srv, ip::udp::socket *sock);
  int set_rcvbuf_size(ip::udp::socket* sock, int size);
  void monitor(ip::udp::socket *sock);
  void query_status(ip::udp::socket *sock);
  void scan(ip::udp::socket *sock);
  void publish(void);
  void cv_notify(void);
  sensor_msgs::msg::LaserScan* get_scan_data(void);

  std::string ip_addr_;

  std::cv_status wait_until(int ms);
  void set_freq(float freq);
  float get_freq(void);
  pthread_t get_pthread(void);
  void set_pthread(pthread_t pthread);

  float freq_;
  pthread_t pthread_;
  std::thread monitor_;
  std::thread worker_;
  std::thread pub_;
  ip::udp::socket* sock_;

  std::condition_variable cv_;
  std::mutex mutex_;
  std::mutex freq_mutex_;
  std::mutex pthread_mutex_;
  std::mutex scan_mutex_;

  int resizeNum_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_msg_pub_;
  sensor_msgs::msg::LaserScan scan_;
  int index_start_;
  int index_end_;

  unsigned char buf_[1500];
  std::atomic<bool> updated_;
};

}  // namespace wlr

#endif // WLR716_UDP_UDP_HPP_
