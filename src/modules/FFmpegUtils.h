#pragma once

#include <QString>

namespace FFmpegUtils {
    // Probe media with FFmpeg and return human-readable summary
    QString probeMedia(const QString& path);
}