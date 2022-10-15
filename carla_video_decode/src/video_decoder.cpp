#include "rclcpp/rclcpp.hpp"
#include "video_decoder.hpp"
#include <iostream>

namespace video_dec_node
{

VideoDecoder::VideoDecoder(std::string &codec_name)
    : m_codec_name(codec_name)
{
    int ret;

    m_pkt.reset(av_packet_alloc());
    if (nullptr == m_pkt)
        exit(1);

    /* find the mpeg1video encoder */
    m_codec.reset(avcodec_find_decoder_by_name(m_codec_name.c_str()));

    if (nullptr == m_codec)
    {
        std::cerr << "Codec: " << m_codec_name << " not found" << std::endl;
        exit(1);
    }

    m_parser.reset(av_parser_init(m_codec->id));

    if (nullptr == m_parser)
    {
        std::cerr << "parser not found" << std::endl;
        exit(1);
    }

    m_ctx.reset(avcodec_alloc_context3(m_codec.get()));

    if (nullptr == m_ctx)
    {
        std::cerr << "Could not allocate video codec context" << std::endl;
        exit(1);
    }

    /* open it */
    ret = avcodec_open2(m_ctx.get(), m_codec.get(), NULL);

    if (ret < 0)
    {
        std::cerr << "Could not open codec: " << m_codec_name << std::endl;
        exit(1);
    }

    m_frame.reset(av_frame_alloc());
    if (nullptr == m_frame)
    {
        std::cerr << "Could not allocate video frame" << std::endl;
        exit(1);
    }
}

int VideoDecoder::parseFrame(std::shared_ptr<sensor_msgs::msg::Image> image_msg, int pts_idx)
{
    int used_bytes = av_parser_parse2(m_parser.get(), m_ctx.get(), &m_pkt->data, &m_pkt->size,
            image_msg->data.data(), image_msg->data.size(), pts_idx, AV_NOPTS_VALUE, 0);

    if (used_bytes < 0)
        std::cerr << "Error while parsing" << std::endl;

    return used_bytes;

}

void VideoDecoder::flushDecode()
{
    // decode(outfile);
    // fclose(outfile.get());
    ;
}


void VideoDecoder::decode(std::queue<std::shared_ptr<sensor_msgs::msg::Image>> &out_buffer)
{
    int ret;

    if (m_pkt->size <= 0)
    {
        std::cerr << "empty packet to decode m_pkt->size: " << m_pkt->size << std::endl;
        return;
    }

    std::cerr << "got correct packet to decode" << std::endl;
    ret = avcodec_send_packet(m_ctx.get(), m_pkt.get());

    if (ret < 0)
    {
        std::cerr << "Error sending a frame for encoding" << std::endl;
        exit(1);
    }

    while (ret >= 0)
    {
        ret = avcodec_receive_frame(m_ctx.get(), m_frame.get());

        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
        {
            std::cerr << "decode fail or finished" << std::endl;
            return;
        }

        else if (ret < 0)
        {
            std::cerr << "Error during encoding" << std::endl;
            exit(1);
        }

        printf("saving frame %3d\n", m_ctx->frame_number);
        printf("image pixformat: %d\n", m_frame->format);
        auto decode_msg = std::make_shared<sensor_msgs::msg::Image>();
        decode_msg->header.frame_id = "video_dec/image_bgra";
        decode_msg->encoding = m_codec_name;
        decode_msg->width = m_frame->width;
        decode_msg->height = m_frame->height;
        decode_msg->data.insert(decode_msg->data.end(), m_frame->data[0], (m_frame->data[0] + m_frame->linesize[0]));
        out_buffer.push(decode_msg);
    }
}

/******
void VideoDecoder::decode(std::shared_ptr<FILE> outfile)
{
    int ret;

    ret = avcodec_send_frame(m_ctx.get(), frame.get());

    if (ret < 0)
    {
        std::cerr << "Error sending a frame for encoding" << std::endl;
        exit(1);
    }

    while (ret >= 0)
    {
        ret = avcodec_receive_packet(m_ctx.get(), m_pkt.get());

        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
            return;
        else if (ret < 0)
        {
            std::cerr << "Error during encoding" << std::endl;
            exit(1);
        }

        printf("Write packet %3"PRId64" (size=%5d)\n", m_pkt->pts, m_pkt->size);
        fwrite(m_pkt->data, 1, m_pkt->size, outfile.get());
        av_packet_unref(m_pkt.get());
    }

}
***************/
VideoDecoder::~VideoDecoder()
{
    AVCodecContext *ctx = m_ctx.get();
    AVFrame *frame = m_frame.get();
    AVPacket *pkt = m_pkt.get();

    av_parser_close(m_parser.get());
    avcodec_free_context(&ctx);
    av_frame_free(&frame);
    av_packet_free(&pkt);
}

}