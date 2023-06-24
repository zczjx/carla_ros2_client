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

struct BufferQueue
{
    std::queue<std::shared_ptr<sensor_msgs::msg::Image>> m_SinkBuf;
    std::mutex m_sinkBufMutex;
};

class gstVideoDec
{
public:
    gstVideoDec();

    int start_gst_pipeline();

    void stop_gst_pipeline();

    GstFlowReturn pushFrame(std::shared_ptr<sensor_msgs::msg::Image> image_msg, int pts_idx);

    std::shared_ptr<sensor_msgs::msg::Image> getFrame();

    void printVideoOutputFormat(GstElement *element, gchar *padName);

    virtual ~gstVideoDec();

private:
    std::shared_ptr<GstBus> m_gst_bus;
    guint m_gst_bus_watch_id;
    std::shared_ptr<GstMessage> m_gst_msg;
    std::shared_ptr<GstElement> m_gst_pipeline;
    std::shared_ptr<GstElement> m_gst_appsrc;
    std::shared_ptr<GstElement> m_gst_h264parse;
    std::shared_ptr<GstElement> m_gst_nvv4l2decoder;
    std::shared_ptr<GstElement> m_gst_nvvidconv;
    std::shared_ptr<GstElement> m_gst_appsink;

    struct BufferQueue m_sinkBufferQueue;

    GstBuffer* sensorMsgtoGstBuffer(std::shared_ptr<sensor_msgs::msg::Image> image);
    int gstBuffertoSensorMsg(GstBuffer *buffer, std::shared_ptr<sensor_msgs::msg::Image> image);

};

}