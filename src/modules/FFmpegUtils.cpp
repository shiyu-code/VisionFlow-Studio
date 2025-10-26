#include "FFmpegUtils.h"

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavcodec/version.h>
#include <libavutil/avutil.h>
}

#include <QString>
#include <QByteArray>

namespace {
    QString codecName(AVCodecParameters* par) {
        const char* name = avcodec_get_name(par->codec_id);
        return QString::fromUtf8(name ? name : "unknown");
    }
}

namespace FFmpegUtils {

QString probeMedia(const QString& path) {
    AVFormatContext* fmt = nullptr;
    QByteArray utf8 = path.toUtf8();
    const char* cpath = utf8.constData();

    if (avformat_open_input(&fmt, cpath, nullptr, nullptr) != 0) {
        return QString("FFmpeg: failed to open %1").arg(path);
    }
    if (avformat_find_stream_info(fmt, nullptr) < 0) {
        avformat_close_input(&fmt);
        return QString("FFmpeg: failed to read stream info");
    }

    QString summary = QString("%1 | streams=%2 | duration=%3s")
        .arg(QString::fromUtf8(fmt->iformat->name))
        .arg(static_cast<int>(fmt->nb_streams))
        .arg(fmt->duration > 0 ? static_cast<int>(fmt->duration / AV_TIME_BASE) : 0);

    for (unsigned i = 0; i < fmt->nb_streams; ++i) {
        AVStream* st = fmt->streams[i];
        AVCodecParameters* par = st->codecpar;
        if (par->codec_type == AVMEDIA_TYPE_VIDEO) {
            summary += QString(" | [video %1x%2 %3]")
                .arg(par->width).arg(par->height).arg(codecName(par));
        } else if (par->codec_type == AVMEDIA_TYPE_AUDIO) {
            int nb_channels = 0;
#if LIBAVCODEC_VERSION_MAJOR >= 60
            nb_channels = par->ch_layout.nb_channels;
#else
            nb_channels = par->channels;
#endif
            summary += QString(" | [audio %1ch %2Hz %3]")
                .arg(nb_channels).arg(par->sample_rate).arg(codecName(par));
        } else if (par->codec_type == AVMEDIA_TYPE_SUBTITLE) {
            summary += QString(" | [subs %1]").arg(codecName(par));
        }
    }

    avformat_close_input(&fmt);
    return summary;
}

} // namespace FFmpegUtils