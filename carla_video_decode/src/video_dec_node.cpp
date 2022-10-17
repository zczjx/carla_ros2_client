#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "video_decoder.hpp"

#include <queue>
#include <thread>

namespace video_dec_node
{

class VideoDecNode : public rclcpp::Node
{
public:

  VideoDecNode()
  : Node("video_dec_node"), m_stopSignal(false)
  {
    // Create a callback function for when messages are received.
    // Variations of this function also exist using, for example UniquePtr for zero-copy transport.
    std::string codec_name("h264_cuvid");
    m_decoder = std::make_shared<VideoDecoder>(codec_name);
    m_pub = create_publisher<sensor_msgs::msg::Image>("/carla/video_dec/image_bgra", 10);
    m_decodeThread = std::make_unique<std::thread>([this]() { doDecode(); });
    m_pubThread = std::make_unique<std::thread>([this]() { doPublish(); });

    auto callback = [this](const std::shared_ptr<sensor_msgs::msg::Image> image){ subCallback(image); };
    m_sub = create_subscription<sensor_msgs::msg::Image>("/carla/video_enc/image_h264", 10, callback);
  }

  ~VideoDecNode();

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_sub;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub;
  std::shared_ptr<VideoDecoder> m_decoder;

  std::queue<std::shared_ptr<sensor_msgs::msg::Image>> m_buffer;
  std::mutex m_bufferMutex;

  std::queue<std::shared_ptr<sensor_msgs::msg::Image>> m_decodeBuffer;
  std::mutex m_decodeBufferMutex;

  std::queue<std::shared_ptr<sensor_msgs::msg::Image>> m_rgbaPixelBuffer;
  std::mutex m_pixelBufferMutex;

  std::unique_ptr<std::thread> m_pubThread;
  std::unique_ptr<std::thread> m_decodeThread;
  std::atomic<bool> m_stopSignal;
  void doPublish();
  void doDecode();
  void subCallback(const std::shared_ptr<sensor_msgs::msg::Image> image);
};

void VideoDecNode::doDecode()
{
  RCLCPP_INFO(this->get_logger(), "video doDecode thread started");

  while (!m_stopSignal && rclcpp::ok())
  {
    std::shared_ptr<sensor_msgs::msg::Image> tmp_image;
    if(m_buffer.empty())
        continue;
    {
      std::lock_guard<std::mutex> lock(m_bufferMutex);
      // RCLCPP_INFO(this->get_logger(), "m_buffer.size: [%d]", m_buffer.size());
      tmp_image = m_buffer.front();
      m_buffer.pop();
    }
    auto image_msg = std::make_shared<sensor_msgs::msg::Image>(*tmp_image);
    m_decoder->parseFrame(image_msg, image_msg->step);
    m_decoder->decode(m_decodeBuffer);
  }

}

void VideoDecNode::doPublish()
{
  RCLCPP_INFO(this->get_logger(), "video doPublish thread started");

  while (!m_stopSignal && rclcpp::ok())
  {
    if(m_decodeBuffer.empty())
      continue;

    auto bgra_msg = m_decodeBuffer.front();
    m_decodeBuffer.pop();
    m_pub->publish(std::move(*bgra_msg));
  }
}

void VideoDecNode::subCallback(const std::shared_ptr<sensor_msgs::msg::Image> image)
{
  std::lock_guard<std::mutex> lock(m_bufferMutex);
  m_buffer.push(image);
}

VideoDecNode::~VideoDecNode()
{
  if (m_decodeThread->joinable())
  {
    RCLCPP_INFO(this->get_logger(), "join m_decodeThread");
    m_stopSignal = true;
    m_decodeThread->join();
  }

  if (m_pubThread->joinable())
  {
    RCLCPP_INFO(this->get_logger(), "join m_pubThread");
    m_stopSignal = true;
    m_pubThread->join();
  }
}

}


int main(int argc, char **argv)
{
    //init client
    rclcpp::init(argc, argv);
    //create new node

    rclcpp::spin(std::make_shared<video_dec_node::VideoDecNode>());

    //shutdown
    rclcpp::shutdown();

    return 0;
}