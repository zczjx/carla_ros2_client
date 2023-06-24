#include "gst_c_callback.hpp"

#ifdef __cplusplus
extern "C" {
#endif

GstFlowReturn gstSinkSample(GstElement *appsink, gpointer sinkBufferQueue)
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
        std::shared_ptr<sensor_msgs::msg::Image> bgra_msg = std::make_shared<sensor_msgs::msg::Image>();

        if(GST_FLOW_OK != gstBuffertoSensorMsg(buffer, bgra_msg))
        {
          gst_sample_unref(sample);
          goto err_exit;
        }

        bgra_msg->header.frame_id = "video_dec/image_bgra";
        bgra_msg->encoding = "bgra8";
        bgra_msg->width = 800;
        bgra_msg->height = 600;
        bgra_msg->step = GST_BUFFER_PTS(buffer);
        gst_sample_unref(sample);
        ptr_buffer_queue->m_SinkBuf.push(bgra_msg);
        return GST_FLOW_OK;
    }

err_exit:
    return GST_FLOW_ERROR;
}

gboolean print_field (GQuark field, const GValue * value, gpointer pfx)
{
  gchar *str = gst_value_serialize (value);

  g_print ("%s  %15s: %s\n", (gchar *) pfx, g_quark_to_string (field), str);
  g_free (str);
  return TRUE;
}

void print_caps(const GstCaps * caps, const gchar * pfx)
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

gboolean gst_bus_callback(GstBus *bus, GstMessage *message, gpointer data)
{
    g_printerr("Got %s message\n", GST_MESSAGE_TYPE_NAME (message));
    return TRUE;
}

GstBuffer* sensorMsgtoGstBuffer(std::shared_ptr<sensor_msgs::msg::Image> image)
{
    GstBuffer *buffer;

    buffer = gst_buffer_new_and_alloc(image->data.size());
    gst_buffer_fill(buffer, 0, image->data.data(), image->data.size());

    return buffer;
}

GstFlowReturn gstBuffertoSensorMsg(GstBuffer *buffer, std::shared_ptr<sensor_msgs::msg::Image> image)
{
    GstMapInfo map;

    gst_buffer_map (buffer, &map, GST_MAP_READ);

    if((NULL == map.data) || (map.size <= 0))
    {
        return GST_FLOW_ERROR;
    }

    image->data.clear();
    image->data.insert(image->data.end(), map.data, (map.data + map.size));
    gst_buffer_unmap (buffer, &map);

    return GST_FLOW_OK;
}


#ifdef __cplusplus
} //extern C
#endif