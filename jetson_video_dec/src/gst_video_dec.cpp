#include "rclcpp/rclcpp.hpp"
#include "gst_video_dec.hpp"
#include "gst_c_callback.hpp"
#include <iostream>

#include <gst/gst.h>
namespace video_dec_node
{

gstVideoDec::gstVideoDec()
{
    int ret;

    // gst_debug_set_active(TRUE);
    // gst_debug_set_default_threshold(GST_LEVEL_WARNING);
    gst_init(NULL, NULL);

    m_gst_appsrc.reset(gst_element_factory_make("appsrc", "m_gst_appsrc"));

    // m_gst_appsrc.reset(gst_element_factory_make("filesrc", "m_gst_appsrc"));

    if (nullptr == m_gst_appsrc)
    {
        std::cerr << "m_gst_appsrc: " << " could not be created" << std::endl;
        exit(1);
    }

    m_gst_h264parse.reset(gst_element_factory_make("h264parse", "m_gst_h264parse"));

    if (nullptr == m_gst_h264parse)
    {
        std::cerr << "m_gst_h264parse: " << " could not be created" << std::endl;
        exit(1);
    }

    m_gst_nvv4l2decoder.reset(gst_element_factory_make("nvv4l2decoder", "m_gst_nvv4l2decoder"));

    if (nullptr == m_gst_nvv4l2decoder)
    {
        std::cerr << "m_gst_nvv4l2decoder: " << " could not be created" << std::endl;
        exit(1);
    }

    m_gst_nvvidconv.reset(gst_element_factory_make("nvvidconv", "m_gst_nvvidconv"));

    if (nullptr == m_gst_nvvidconv)
    {
        std::cerr << "m_gst_nvvidconv: " << " could not be created" << std::endl;
        exit(1);
    }


    m_gst_appsink.reset(gst_element_factory_make("appsink", "m_gst_appsink"));

    // m_gst_appsink.reset(gst_element_factory_make("nv3dsink", "m_gst_appsink"));

    if (nullptr == m_gst_appsink)
    {
        std::cerr << "m_gst_appsink: " << " could not be created" << std::endl;
        exit(1);
    }

    m_gst_pipeline.reset(gst_pipeline_new ("run_pipeline"));

    if (nullptr == m_gst_pipeline)
    {
        std::cerr << "m_gst_pipeline: " << " could not be created" << std::endl;
        exit(1);
    }

    gst_bin_add_many(GST_BIN(m_gst_pipeline.get()), m_gst_appsrc.get(), m_gst_h264parse.get(),
        m_gst_nvv4l2decoder.get(), m_gst_nvvidconv.get(), m_gst_appsink.get(), NULL);

    if(gst_element_link_many(m_gst_appsrc.get(), m_gst_h264parse.get(), m_gst_nvv4l2decoder.get(),
        m_gst_nvvidconv.get(), NULL) != TRUE)
    {
        std::cerr << "Elements could not be linked." << std::endl;
        gst_object_unref(m_gst_pipeline.get());
        exit(1);
    }

    /* setup nvv4l2decoder */
    g_object_set(G_OBJECT (m_gst_nvv4l2decoder.get()), "enable-max-performance", TRUE, NULL);
    // g_object_set(G_OBJECT (m_gst_nvv4l2decoder.get()), "output-io-mode", 2, NULL);

    /* setup nvvidconv and appsink caps */
    GstCaps *filtercaps = gst_caps_new_simple ("video/x-raw", "format", G_TYPE_STRING, "BGRx", NULL);
    gboolean link_ok = gst_element_link_filtered (m_gst_nvvidconv.get(), m_gst_appsink.get(), filtercaps);
    gst_caps_unref (filtercaps);
    if (!link_ok)
    {
        std::cerr << "Failed to link m_gst_nvvidconv and m_gst_appsink!" << std::endl;
    }

    /* setup appsrc */
    g_object_set(G_OBJECT (m_gst_appsrc.get()), "is-live", TRUE, NULL);

    /* setup appsink */
    g_object_set(m_gst_appsink.get(), "emit-signals", TRUE, NULL);
    g_signal_connect(m_gst_appsink.get(), "new-sample", G_CALLBACK(gstSinkSample), &m_sinkBufferQueue);
    std::cerr << "gstVideoDec: " << " pipeline build successfully!" << std::endl;

}

GstFlowReturn gstVideoDec::pushFrame(std::shared_ptr<sensor_msgs::msg::Image> image_msg, int pts_idx)
{
    GstFlowReturn ret;
    GstBuffer *buffer = sensorMsgtoGstBuffer(image_msg);

    /* Set its timestamp and duration */
    GST_BUFFER_PTS(buffer) = pts_idx;
    GST_BUFFER_DURATION (buffer) = gst_util_uint64_scale_int (1, GST_SECOND, 20);

    // Push the buffer into AppSrc
    g_signal_emit_by_name(m_gst_appsrc.get(), "push-buffer", buffer, &ret);
    gst_buffer_unref(buffer);

    if (ret != GST_FLOW_OK)
    {
        /* something wrong, stop pushing */
        g_printerr("appsrc: push-buffer fail");
        return ret;
    }

    return ret;
}

std::shared_ptr<sensor_msgs::msg::Image> gstVideoDec::getFrame()
{
    std::lock_guard<std::mutex> lock(m_sinkBufferQueue.m_sinkBufMutex);

    if(m_sinkBufferQueue.m_SinkBuf.empty())
    {
        // std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        return nullptr;
    }

    std::shared_ptr<sensor_msgs::msg::Image> bgra_msg = m_sinkBufferQueue.m_SinkBuf.front();
    m_sinkBufferQueue.m_SinkBuf.pop();

    return bgra_msg;
}

int gstVideoDec::start_gst_pipeline()
{
    m_gst_bus.reset(gst_element_get_bus(m_gst_pipeline.get()));
    m_gst_bus_watch_id = gst_bus_add_watch (m_gst_bus.get(), gst_bus_callback, NULL);
    GstStateChangeReturn ret = gst_element_set_state(m_gst_pipeline.get(), GST_STATE_PLAYING);

    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        std::cerr << "Unable to set the pipeline to the playing state." << std::endl;
        gst_object_unref (m_gst_pipeline.get());
        return -1;
    }

}

void gstVideoDec::stop_gst_pipeline()
{
    gst_object_unref (m_gst_bus.get());
    gst_element_set_state (m_gst_pipeline.get(), GST_STATE_NULL);
    gst_object_unref (m_gst_pipeline.get());
    g_source_remove (m_gst_bus_watch_id);
}

gstVideoDec::~gstVideoDec()
{
    stop_gst_pipeline();
}

}