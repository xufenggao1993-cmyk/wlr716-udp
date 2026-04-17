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

#include "rclcpp/rclcpp.hpp"
#include "udp.hpp"
#include "utils.hpp"

namespace wlr
{

#define CMD_PORT  6050
#define DATA_PORT 6060

UDPTrans::UDPTrans()
: Node("frontlidar_ctrl_node"),
  freq_(20.0)
{
  setup_signal();
  /* Must before Protocol() constructor */
  init_scan();
  Protocol *proto = new Protocol(this);
  io_service *srv = new io_service();
  sock_ = sock(srv, DATA_PORT);
  recv(sock_, proto);
  worker_ = std::thread(std::bind(&UDPTrans::run, this, srv, sock_));
  monitor_ = std::thread(std::bind(&UDPTrans::monitor, this, sock_));
  pub_ = std::thread(std::bind(&UDPTrans::publish, this));
}

UDPTrans::~UDPTrans()
{
  if (worker_.joinable())
    worker_.join();
  if (monitor_.joinable())
    monitor_.join();
}

void
UDPTrans::set_freq(float freq)
{
  std::lock_guard<std::mutex> lock(freq_mutex_);
  freq_ = freq;
}

float
UDPTrans::get_freq(void)
{
  std::lock_guard<std::mutex> lock(freq_mutex_);
  float res;
  res = freq_;
  return res;
}

sensor_msgs::msg::LaserScan *
UDPTrans::get_scan(void)
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  return &scan_;
}

sensor_msgs::msg::LaserScan *
UDPTrans::get_scan_data(void)
{
  std::lock_guard<std::mutex> lock(scan_mutex_);
  if(!updated_.load()) {
    return nullptr;
  }
  updated_.store(false);
  return &scan_;
}

void
UDPTrans::set_scan_data(unsigned int findex, float *range, float *intens)
{
  {
    std::lock_guard<std::mutex> lock(scan_mutex_);
    /*
     *  In ROS, angles are measured around
     *  the positive Z axis (counterclockwise, if Z is up)
     *  with zero angle being forward along the x axis
     */
    for (int j = index_start_ - 1, i = 0; j >= index_end_; --j, i++) {
      scan_.ranges[i] = range[j];
      scan_.intensities[i] = intens[j];
    }

    scan_.header.stamp = this->now();
    // Navigation software stack need a scan loop count number.
    // So give it a ture number that is from lidar hardware,
    // or make a fake squence number.
    // There is no spared fild in LaserScan message.
    // So I have make use of header.stamp.nanosec.
    // Hope there is no side effect.
    scan_.header.stamp.nanosec = findex;
    updated_.store(true);
  }

  (void)cv_notify();
}

void
UDPTrans::set_pthread(pthread_t pthread)
{
  std::lock_guard<std::mutex> lock(pthread_mutex_);
  pthread_ = pthread;
}

pthread_t
UDPTrans::get_pthread(void)
{
  std::lock_guard<std::mutex> lock(pthread_mutex_);
  pthread_t res;
  res = pthread_;
  return res;
}

/* Send query device status command to force scan index to 1 */
void
UDPTrans::query_status(ip::udp::socket *sock)
{
  unsigned char query[] = {
    0xFF,0xAA,0x00,0x1E,0x00,0x00,0x00,0x00,
    0x00,0x00,0x01,0x01,0x00,0x05,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x01,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1F,
    0xEE,0xEE
  };
  int len = sizeof(query)/sizeof(unsigned char);
  send(sock, query, len);
}

void
UDPTrans::scan(ip::udp::socket *sock)
{
  query_status(sock);
}

void
UDPTrans::monitor(ip::udp::socket *sock)
{
  int timeout; //ms
  while(rclcpp::ok()) {
    timeout = (int)(1000.0 / get_freq());
    //wait about 200ms plus interval.
    timeout = timeout + 200;
    //wait at least 1s.
    timeout = timeout < 1000 ? 1000 : timeout;

    if (std::cv_status::timeout == wait_until(timeout)) {
      RCLCPP_INFO(get_logger(), "Lidar data lost!");
      scan(sock);
    } else {
      sleep(2);
    }
  }
}

void
UDPTrans::init_scan(void)
{
  laser_msg_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/frontlidar", 10);
  scan_.header.frame_id = "laser_link";
  scan_.angle_min = -2.35619449;
  scan_.angle_max = 2.35619449;
  //scan_.range_min = 0.05;
  scan_.range_min = 0.0;
  scan_.range_max = 25;

  scan_.angle_increment = 0.017453 / 3;
  scan_.time_increment = 1 / 15.00000000 / 1081;
  //adjust angle_min to min_ang config param
  //index_start = 811 - ((scan.angle_min + 2.35619449) / scan.angle_increment);
  index_start_ = 811;
  //adjust angle_max to max_ang config param
  //index_end = (2.35619449 - scan.angle_max) / scan.angle_increment;
  index_end_ = 0;
  //resizeNum = index_start - index_end;
  resizeNum_ = index_start_ - index_end_ + 1;
  RCLCPP_INFO(get_logger(), "index_start: %d", index_start_);
  RCLCPP_INFO(get_logger(), "index_end: %d", index_end_);
  RCLCPP_INFO(get_logger(), "resize: %d", resizeNum_);

  scan_.ranges.resize(resizeNum_);
  scan_.intensities.resize(resizeNum_);
  RCLCPP_INFO(get_logger(), "frame_id: %s", scan_.header.frame_id.c_str());
  RCLCPP_INFO(get_logger(), "angle_min: %f", scan_.angle_min);
  RCLCPP_INFO(get_logger(), "angle_max: %f", scan_.angle_max);
  RCLCPP_INFO(get_logger(), "angle_increment: %f", scan_.angle_increment);
  RCLCPP_INFO(get_logger(), "time_increment: %f", scan_.time_increment);
  RCLCPP_INFO(get_logger(), "range_min: %f", scan_.range_min);
  RCLCPP_INFO(get_logger(), "range_max: %f", scan_.range_max);
}

void
UDPTrans::publish(void)
{
  static struct timespec start;
  struct timespec req;
  long sleep_ns;
  sensor_msgs::msg::LaserScan *msg;

  set_pthread(pthread_self());
  while(rclcpp::ok()) {
    clock_gettime(CLOCK_MONOTONIC, &start);
    sleep_ns = (long)(1000000000.0 / get_freq());
    req = timespec_add(start, sleep_ns);
    /* return Null point if there is not updated scan */
    msg = get_scan_data();
    if(msg)
      laser_msg_pub_->publish(*msg);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &req, 0);
  }
}

std::cv_status
UDPTrans::wait_until(int ms)
{
  std::cv_status res;
  std::unique_lock<std::mutex> lock(mutex_);
  auto now = std::chrono::system_clock::now();
  res = cv_.wait_until(lock, now + ms*1ms);
  return res;
}

void
UDPTrans::cv_notify(void)
{
  std::lock_guard<std::mutex> lock(mutex_);
  cv_.notify_one();
}

int
UDPTrans::set_rcvbuf_size(ip::udp::socket* sock, int size)
{
  int res;
  socket_base::receive_buffer_size opt(size);
  sock->set_option(opt);
  sock->get_option(opt);
  res = opt.value();
  RCLCPP_INFO(get_logger(), "Socket receive buffer size: %d bytes.", res);
  return res;
}

ip::udp::socket*
UDPTrans::sock(io_service *srv, int port)
{
  ip::udp::socket *res = nullptr;
  ip::udp::endpoint ep(ip::udp::v4(), port);
  try {
    res = new ip::udp::socket(*srv, ep);
  } catch (std::exception& e) {
    RCLCPP_ERROR(get_logger(), "socket error: %s", e.what());
  }
  return res;
}	

void
UDPTrans::run(io_service *srv, ip::udp::socket *sock)
{
  srv->run();
  scan(sock);
}

void
UDPTrans::recv(ip::udp::socket *sock, Protocol *proto)
{
  ip::udp::endpoint ep;
  sock->async_receive_from(boost::asio::buffer(buf_), ep,
    boost::bind(&UDPTrans::callback, this,
    sock,
    proto,
    boost::asio::placeholders::error,
    boost::asio::placeholders::bytes_transferred));
}

void
UDPTrans::callback(ip::udp::socket *sock, Protocol *proto, const boost::system::error_code &error, std::size_t count)
{
  (void) error;
  proto->dataProcess(buf_, count);
  recv(sock, proto);
}

int
UDPTrans::send(ip::udp::socket *sock, unsigned char *buffer, int size)
{
  int res = -1;
  ip::udp::endpoint rmt_ep;
  rmt_ep.address(ip::address::from_string("192.168.0.2"));
  rmt_ep.port(CMD_PORT);

  try {
    res = sock->send_to(boost::asio::buffer(buffer, size), rmt_ep);
  } catch (std::exception& e) {
    RCLCPP_ERROR(get_logger(), "send error: %s", e.what());
  }

  return res;
}

}  // namespace wlr
