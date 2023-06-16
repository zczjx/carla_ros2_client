#include <gst/gst.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <thread>
#include <queue>

#include "sensor_msgs/msg/image.hpp"

namespace video_dec_node
{

class gstVideoDec
{
public:
    gstVideoDec();

    int start_gst_pipeline();

    void stop_gst_pipeline();

    GstFlowReturn pushFrame(std::shared_ptr<sensor_msgs::msg::Image> image_msg, int pts_idx);

    GstFlowReturn getFrame(std::shared_ptr<sensor_msgs::msg::Image> outFrame);

    virtual ~gstVideoDec();

private:
    std::shared_ptr<GstBus> m_gst_bus;
    std::shared_ptr<GstMessage> m_gst_msg;
    std::shared_ptr<GstElement> m_gst_pipeline;

    std::shared_ptr<GstElement> m_gst_appsrc;
    std::shared_ptr<GstElement> m_gst_queue;
    std::shared_ptr<GstElement> m_gst_h264parse;
    std::shared_ptr<GstElement> m_gst_nvv4l2decoder;
    std::shared_ptr<GstElement> m_gst_appsink;

    std::queue<std::shared_ptr<GstBuffer>> m_gstSinkBuf;
    std::mutex m_sinkBufMutex;

    GstFlowReturn gstSinkSample(GstElement *appsink, void *data);
    std::shared_ptr<GstBuffer> sensorMsgtoGstBuffer(std::shared_ptr<sensor_msgs::msg::Image> image);
    int gstBuffertoSensorMsg(std::shared_ptr<GstBuffer> buffer, std::shared_ptr<sensor_msgs::msg::Image> image);

};

}