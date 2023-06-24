#ifndef GST_C_CALLBACK_HPP_
#define GST_C_CALLBACK_HPP_

#include <gst/gst.h>

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <thread>
#include <queue>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct BufferQueue
{
    std::queue<std::shared_ptr<sensor_msgs::msg::Image>> m_SinkBuf;
    std::mutex m_sinkBufMutex;
} BufferQueue;

extern GstFlowReturn gstSinkSample(GstElement *appsink, gpointer sinkBufferQueue);

extern gboolean print_field (GQuark field, const GValue * value, gpointer pfx);

extern void print_caps(const GstCaps * caps, const gchar * pfx);

extern gboolean gst_bus_callback(GstBus *bus, GstMessage *message, gpointer data);

extern GstBuffer* sensorMsgtoGstBuffer(std::shared_ptr<sensor_msgs::msg::Image> image);

extern GstFlowReturn gstBuffertoSensorMsg(GstBuffer *buffer, std::shared_ptr<sensor_msgs::msg::Image> image);

#ifdef __cplusplus
} //extern C
#endif



#endif //GST_C_CALLBACK_HPP_