#include "gst_c_callback.hpp"

#ifdef __cplusplus
extern "C" {
#endif

GstStructure* gstGetPadCapStructure(GstElement *element, gchar *padName)
{
  GstPad *pad = NULL;
  GstCaps *caps = NULL;
  gchar * pad_name = padName;

  /* Retrieve pad */
  pad = gst_element_get_static_pad (element, pad_name);
  if (!pad)
  {
    g_printerr ("Could not retrieve pad '%s'\n", pad_name);
    return NULL;
  }

  /* Retrieve negotiated caps (or acceptable caps if negotiation is not finished yet) */
  caps = gst_pad_get_current_caps (pad);
  if (!caps)
    caps = gst_pad_query_caps (pad, NULL);

  return gst_caps_get_structure (caps, 0);

}

GstFlowReturn gstSinkSample(GstElement *appsink, gpointer sinkBufferQueue)
{
    GstSample *sample = NULL;
    GstFlowReturn ret = GST_FLOW_OK;
    struct BufferQueue *ptr_buffer_queue = (struct BufferQueue *) sinkBufferQueue;
    GstStructure *structure = NULL;

    /* Retrieve the buffer */
    g_signal_emit_by_name(appsink, "pull-sample", &sample);

    if (sample)
    {
        std::lock_guard<std::mutex> lock(ptr_buffer_queue->m_sinkBufMutex);
        // printVideoOutputFormat(appsink, "sink");

        GstBuffer *buffer = gst_sample_get_buffer(sample);
        std::shared_ptr<sensor_msgs::msg::Image> bgra_msg = std::make_shared<sensor_msgs::msg::Image>();

        if(GST_FLOW_OK != gstBuffertoSensorMsg(buffer, bgra_msg))
        {
          gst_sample_unref(sample);
          goto err_exit;
        }

        bgra_msg->header.frame_id = "video_dec/image_bgra";
        bgra_msg->step = GST_BUFFER_PTS(buffer);
        structure = gstGetPadCapStructure(appsink, "sink");

        if(NULL != structure)
        {
          gint width, height;

          gst_structure_get_int(structure, "width", &width);
          gst_structure_get_int(structure, "height", &height);
          bgra_msg->width = width;
          bgra_msg->height = height;
          bgra_msg->encoding = gst_structure_get_string(structure, "format");
        }

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

    g_print ("i: %d %s%s\n", i, pfx, gst_structure_get_name (structure));
    gint width, height;
    gst_structure_get_int(structure, "width", &width);
    gst_structure_get_int(structure, "height", &height);
    const gchar *format = gst_structure_get_string(structure, "format");
    g_print ("%s width: %d\n", pfx, width);
    g_print ("%s height: %d\n", pfx, height);
    g_print ("%s format: %s\n", pfx, format);
    //gst_structure_foreach (structure, print_field, (gpointer) pfx);
  }
}

void printVideoOutputFormat(GstElement *element, gchar *padName)
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