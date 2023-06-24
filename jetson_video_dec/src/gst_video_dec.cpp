#include "rclcpp/rclcpp.hpp"
#include "gst_video_dec.hpp"
#include <iostream>

#include <gst/gst.h>
namespace video_dec_node
{

extern "C"{

static GstFlowReturn gstSinkSample(GstElement *appsink, gpointer sinkBufferQueue)
{
    GstSample *sample = NULL;
    GstFlowReturn ret = GST_FLOW_OK;
    struct BufferQueue *ptr_buffer_queue = (struct BufferQueue *) sinkBufferQueue;

    /* Retrieve the buffer */
    g_signal_emit_by_name(appsink, "pull-sample", &sample);

    if (sample)
    {
        std::lock_guard<std::mutex> lock(ptr_buffer_queue->m_sinkBufMutex);
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;
        /* The only thing we do in this example is print a * to indicate a received buffer */
        // std::shared_ptr<GstBuffer> buffer;
        // buffer.reset(gst_sample_get_buffer(sample));
        g_printerr ("buffer pts: %ld\n", GST_BUFFER_PTS(buffer));
        g_printerr ("buffer duration: %ld\n", GST_BUFFER_DURATION(buffer));

        gst_buffer_map (buffer, &map, GST_MAP_READ);

        g_printerr ("output buffer map.size: %ld\n", map.size);
        g_printerr ("output buffer map.maxsize: %ld\n", map.maxsize);
        std::shared_ptr<sensor_msgs::msg::Image> bgra_msg = std::make_shared<sensor_msgs::msg::Image>();
        // gstBuffertoSensorMsg(buffer, bgra_msg);
        bgra_msg->header.frame_id = "video_dec/image_bgra";
        bgra_msg->encoding = "bgra8";
        bgra_msg->width = 800;
        bgra_msg->height = 600;
        bgra_msg->step = GST_BUFFER_PTS(buffer);
        bgra_msg->data.clear();
        bgra_msg->data.insert(bgra_msg->data.end(), map.data, (map.data + map.size));
        g_printerr ("bgra_msg->data.size(): %ld\n", bgra_msg->data.size());


        gst_buffer_unmap(buffer, &map);
        // gst_buffer_unref(buffer);
        gst_sample_unref(sample);
        ptr_buffer_queue->m_SinkBuf.push(bgra_msg);

        std::cerr << "gstSinkSample: appsink get new_sample data!" << std::endl;
        return GST_FLOW_OK;
    }

    return GST_FLOW_ERROR;
}

static gboolean print_field (GQuark field, const GValue * value, gpointer pfx)
{
  gchar *str = gst_value_serialize (value);

  g_print ("%s  %15s: %s\n", (gchar *) pfx, g_quark_to_string (field), str);
  g_free (str);
  return TRUE;
}

static void print_caps(const GstCaps * caps, const gchar * pfx)
{
  guint i;

  g_return_if_fail (caps != NULL);

  if (gst_caps_is_any (caps))
  {
    g_print ("%sANY\n", pfx);
    return;
  }

  if (gst_caps_is_empty (caps))
  {
    g_print ("%sEMPTY\n", pfx);
    return;
  }

  for(i = 0; i < gst_caps_get_size (caps); i++)
  {
    GstStructure *structure = gst_caps_get_structure (caps, i);

    g_print ("%s%s\n", pfx, gst_structure_get_name (structure));
    gst_structure_foreach (structure, print_field, (gpointer) pfx);
  }
}

static gboolean my_bus_callback(GstBus *bus, GstMessage *message, gpointer data)
{
    g_printerr("Got %s message\n", GST_MESSAGE_TYPE_NAME (message));
    return TRUE;
}


}

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
        // std::cerr << "m_gstSinkBuf is empty!" << std::endl;
        // std::this_thread::sleep_for(std::chrono::milliseconds(10000));
        return nullptr;
    }
    printVideoOutputFormat(m_gst_nvvidconv.get(), "src");
    std::shared_ptr<sensor_msgs::msg::Image> bgra_msg = m_sinkBufferQueue.m_SinkBuf.front();
    m_sinkBufferQueue.m_SinkBuf.pop();

    return bgra_msg;
}

void gstVideoDec::printVideoOutputFormat(GstElement *element, gchar *padName)
{
    GstPad *pad = NULL;
    GstCaps *caps = NULL;
    gchar * pad_name = padName;

    /* Retrieve pad */
    pad = gst_element_get_static_pad (element, pad_name);
    if (!pad) {
        g_printerr ("Could not retrieve pad '%s'\n", pad_name);
        return;
    }

    /* Retrieve negotiated caps (or acceptable caps if negotiation is not finished yet) */
    caps = gst_pad_get_current_caps (pad);
    if (!caps)
        caps = gst_pad_query_caps (pad, NULL);

    /* Print and free */
    g_print ("Caps for the %s pad:\n", pad_name);
    print_caps (caps, "      ");
    gst_caps_unref (caps);
    gst_object_unref (pad);

}

GstBuffer* gstVideoDec::sensorMsgtoGstBuffer(std::shared_ptr<sensor_msgs::msg::Image> image)
{
    GstBuffer *buffer;

    buffer = gst_buffer_new_and_alloc(image->data.size());
    gst_buffer_fill(buffer, 0, image->data.data(), image->data.size());
    // g_printerr("input push data size: %ld\n", image->data.size());

    return buffer;
}

int gstVideoDec::gstBuffertoSensorMsg(GstBuffer *buffer, std::shared_ptr<sensor_msgs::msg::Image> image)
{
    GstMapInfo map;

    gst_buffer_map (buffer, &map, GST_MAP_READ);

    if((NULL == map.data) || (map.size <= 0))
    {
        return -1;
    }
    image->data.clear();
    image->data.insert(image->data.begin(), map.data[0], (map.data[0] + map.size));
    g_printerr ("output buffer map.size: %ld\n", map.size);
    g_printerr ("output buffer map.maxsize: %ld\n", map.maxsize);
    gst_buffer_unmap (buffer, &map);

    return 0;
}

int gstVideoDec::start_gst_pipeline()
{
    m_gst_bus.reset(gst_element_get_bus(m_gst_pipeline.get()));
    m_gst_bus_watch_id = gst_bus_add_watch (m_gst_bus.get(), my_bus_callback, NULL);
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
    // m_gst_msg.reset(gst_bus_timed_pop_filtered (m_gst_bus.get(), GST_CLOCK_TIME_NONE, (GstMessageType) (GST_MESSAGE_ERROR | GST_MESSAGE_EOS)));
    // if(m_gst_msg != nullptr)
    //    gst_message_unref(m_gst_msg.get());
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