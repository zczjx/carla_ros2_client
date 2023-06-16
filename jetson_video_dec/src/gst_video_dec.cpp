#include "rclcpp/rclcpp.hpp"
#include "gst_video_dec.hpp"
#include <iostream>

namespace video_dec_node
{

gstVideoDec::gstVideoDec()
{
    int ret;

    m_gst_pipeline.reset(gst_pipeline_new ("run_pipeline"));

    if (nullptr == m_gst_pipeline)
    {
        std::cerr << "m_gst_pipeline: " << " could not be created" << std::endl;
        exit(1);
    }

    m_gst_appsrc.reset(gst_element_factory_make("appsrc", "m_gst_appsrc"));

    if (nullptr == m_gst_appsrc)
    {
        std::cerr << "m_gst_appsrc: " << " could not be created" << std::endl;
        exit(1);
    }

    m_gst_queue.reset(gst_element_factory_make("queue", "m_gst_queue"));

    if (nullptr == m_gst_queue)
    {
        std::cerr << "m_gst_queue: " << " could not be created" << std::endl;
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

    m_gst_appsink.reset(gst_element_factory_make("appsink", "m_gst_appsink"));

    if (nullptr == m_gst_appsink)
    {
        std::cerr << "m_gst_appsink: " << " could not be created" << std::endl;
        exit(1);
    }

    gst_bin_add_many(GST_BIN(m_gst_pipeline.get()), m_gst_appsrc.get(), m_gst_queue.get(),
        m_gst_h264parse.get(), m_gst_nvv4l2decoder.get(), m_gst_appsink.get(), NULL);

    if(gst_element_link_many(m_gst_appsrc.get(), m_gst_queue.get(), m_gst_h264parse.get(),
        m_gst_nvv4l2decoder.get(), m_gst_appsink.get(), NULL) != TRUE)
    {
        std::cerr << "Elements could not be linked." << std::endl;
        gst_object_unref(m_gst_pipeline.get());
        exit(1);
    }

    /* setup appsrc */
    g_object_set(G_OBJECT (m_gst_appsrc.get()), "is-live", TRUE, "format", GST_FORMAT_TIME, NULL);
    /* setup appsink */
    g_object_set(m_gst_appsink.get(), "emit-signals", TRUE, NULL);
    g_signal_connect(m_gst_appsink.get(), "new-sample", G_CALLBACK(gstSinkSample), NULL);

}

GstFlowReturn gstVideoDec::gstSinkSample(GstElement *appsink, void *data)
{
    GstSample *sample;
    GstFlowReturn ret = GST_FLOW_OK;

    /* Retrieve the buffer */
    g_signal_emit_by_name(appsink, "pull-sample", &sample);

    if (sample)
    {
        std::lock_guard<std::mutex> lock(m_sinkBufMutex);
        /* The only thing we do in this example is print a * to indicate a received buffer */
        std::shared_ptr<GstBuffer> buffer;
        buffer.reset(gst_sample_get_buffer(sample));
        m_gstSinkBuf.push(buffer);

        std::cerr << "appsink get new_sample data!" << std::endl;
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    return GST_FLOW_ERROR;
}

GstFlowReturn gstVideoDec::pushFrame(std::shared_ptr<sensor_msgs::msg::Image> image_msg, int pts_idx)
{
    GstFlowReturn ret;
    std::shared_ptr<GstBuffer> buffer = sensorMsgtoGstBuffer(image_msg);

    /* Set its timestamp and duration */
    GST_BUFFER_PTS(buffer.get()) = pts_idx;

    // Push the buffer into AppSrc
    g_signal_emit_by_name(m_gst_appsrc.get(), "push-buffer", buffer.get(), &ret);
    g_printerr("m_gst_appsrc put new data!\n");

    if (ret != GST_FLOW_OK)
    {
        /* something wrong, stop pushing */
        g_printerr("appsrc: push-buffer fail");
        return ret;
    }

    return ret;
}

GstFlowReturn gstVideoDec::getFrame(std::shared_ptr<sensor_msgs::msg::Image> outFrame)
{
    std::lock_guard<std::mutex> lock(m_sinkBufMutex);

    if(m_gstSinkBuf.empty())
    {
        std::cerr << "m_gstSinkBuf is empty!" << std::endl;
        return GST_FLOW_CUSTOM_ERROR;
    }

    std::shared_ptr<GstBuffer> buffer = m_gstSinkBuf.front();
    m_gstSinkBuf.pop();
    gstBuffertoSensorMsg(buffer, outFrame);

    return GST_FLOW_OK;
}

std::shared_ptr<GstBuffer> gstVideoDec::sensorMsgtoGstBuffer(std::shared_ptr<sensor_msgs::msg::Image> image)
{
    auto buffer = std::make_shared<GstBuffer>();

    buffer.reset(gst_buffer_new_and_alloc(image->data.size()));
    gst_buffer_fill(buffer.get(), 0, image->data.data(), image->d0ata.size());

    return buffer;
}

int gstBuffertoSensorMsg(std::shared_ptr<GstBuffer> buffer, std::shared_ptr<sensor_msgs::msg::Image> image)
{
    GstMapInfo map;

    gst_buffer_map (buffer.get(), &map, GST_MAP_READ);

    if((NULL == map.data) || (map.size <= 0))
    {
        return -1;
    }
    image->data.insert(image->data.begin(), map.data[0], (map.data[0] + map.size));
    gst_buffer_unmap (buffer.get(), &map);

    return 0;
}

int gstVideoDec::start_gst_pipeline()
{
    GstStateChangeReturn ret = gst_element_set_state(m_gst_pipeline.get(), GST_STATE_PLAYING);

    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        std::cerr << "Unable to set the pipeline to the playing state." << std::endl;
        gst_object_unref (run_pipeline.get());
        return -1;
    }

}

void gstVideoDec::stop_gst_pipeline()
{
    m_gst_bus.reset(gst_element_get_bus(m_gst_pipeline));
    m_gst_msg.reset(gst_bus_timed_pop_filtered (m_gst_bus.get(), GST_CLOCK_TIME_NONE, (GstMessageType) (GST_MESSAGE_ERROR | GST_MESSAGE_EOS)));
    if(m_gst_msg != nullptr)
        gst_message_unref(m_gst_msg.get());
    gst_object_unref (m_gst_bus.get());
    gst_element_set_state (m_gst_pipeline.get(), GST_STATE_NULL);
    gst_object_unref (m_gst_pipeline.get());
}

gstVideoDec::~gstVideoDec()
{
    stop_gst_pipeline();
}

}