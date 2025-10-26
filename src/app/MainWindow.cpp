#include "MainWindow.h"

#include <QMenuBar>
#include <QFileDialog>
#include <QStatusBar>
#include <QImage>
#include <QPixmap>
#include <QMessageBox>
#include <QPainter>
#include <QDialog>
#include <QFormLayout>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QSlider>
#include <QDialogButtonBox>
#include <QPushButton>
#include <QSettings>
#include <QCheckBox>
#include <QDateTime>
#include <QColorDialog>
#include <QEvent>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QElapsedTimer>
#include <algorithm>
#include <thread>
 
 // OpenCV
 #include <opencv2/opencv.hpp>

// VTK + Qt
#include <QVTKOpenGLNativeWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vtkImageData.h>
#include <vtkImageActor.h>
#include <vtkImageProperty.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkGlyph2D.h>
#include <vtkArrowSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkLookupTable.h>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkPolyDataMapper2D.h>
#include <vtkActor2D.h>
#include <vtkProperty2D.h>
#include <vtkCellArray.h>

// FFmpeg helper
#include "FFmpegUtils.h"
#include "../modules/piv/PivEngine.h"
#include "../modules/rdic/RdicEngine.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent) {
    splitter_ = new QSplitter(this);
    videoLabel_ = new QLabel("Open a video to start", splitter_);
    videoLabel_->setAlignment(Qt::AlignCenter);
    videoLabel_->setMinimumSize(640, 480);

    vtkWidget_ = new QVTKOpenGLNativeWidget(splitter_);
    vtkWidget_->setMinimumSize(400, 300);

    setCentralWidget(splitter_);

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MainWindow::onFrameTick);

    setupMenus();
    setupVTKScene();
    statusBar()->showMessage("Ready");
    loadSettings();
    videoLabel_->setMouseTracking(true);
    videoLabel_->installEventFilter(this);
}

MainWindow::~MainWindow() {
    stop();
}

void MainWindow::setupMenus() {
    auto *fileMenu = menuBar()->addMenu("&File");
    auto *openAct = fileMenu->addAction("Open Video...");
    connect(openAct, &QAction::triggered, this, &MainWindow::openVideo);

    auto *playAct = fileMenu->addAction("Play");
    connect(playAct, &QAction::triggered, this, &MainWindow::play);

    auto *stopAct = fileMenu->addAction("Stop");
    connect(stopAct, &QAction::triggered, this, &MainWindow::stop);

    fileMenu->addSeparator();
    auto *exitAct = fileMenu->addAction("Exit");
    connect(exitAct, &QAction::triggered, this, &QWidget::close);

    auto *analysisMenu = menuBar()->addMenu("&Analysis");
    auto *runPivAct = analysisMenu->addAction("Run PIV (two frames)");
    connect(runPivAct, &QAction::triggered, this, &MainWindow::runPIVAnalysis);
    auto *toggleOverlayAct = analysisMenu->addAction("Toggle PIV Overlay");
    connect(toggleOverlayAct, &QAction::triggered, this, &MainWindow::togglePivOverlay);
    auto *toggleLinkAct = analysisMenu->addAction("Toggle Link Views");
    connect(toggleLinkAct, &QAction::triggered, this, &MainWindow::toggleLinkViews);
    auto *pivSettingsAct = analysisMenu->addAction("PIV Settings...");
    connect(pivSettingsAct, &QAction::triggered, this, &MainWindow::openPivSettings);
    auto *exportPivCsvAct = analysisMenu->addAction("Export PIV CSV...");
    connect(exportPivCsvAct, &QAction::triggered, this, &MainWindow::exportPivCSV);
    auto *exportRdicCsvAct = analysisMenu->addAction("Export RDIC CSV...");
    connect(exportRdicCsvAct, &QAction::triggered, this, &MainWindow::exportRdicCSV);
    // RDIC menu actions
    auto *runRdicAct = analysisMenu->addAction("Run RDIC (two frames)");
    connect(runRdicAct, &QAction::triggered, this, &MainWindow::runRDICAnalysis);
    auto *toggleRdicAct = analysisMenu->addAction("Toggle RDIC Overlay");
    connect(toggleRdicAct, &QAction::triggered, this, &MainWindow::toggleRdicOverlay);
    auto *rdicSettingsAct = analysisMenu->addAction("RDIC Settings...");
    connect(rdicSettingsAct, &QAction::triggered, this, &MainWindow::openRdicSettings);
    // New: ROI overlay toggle
    auto *toggleRoiAct = analysisMenu->addAction("Toggle ROI Overlay");
    connect(toggleRoiAct, &QAction::triggered, this, &MainWindow::toggleRoiOverlay);
    toggleRoiAct->setShortcut(QKeySequence("Ctrl+R"));
    // New: Edit ROI mode toggle
    editRoiAct_ = analysisMenu->addAction("Edit ROI");
    editRoiAct_->setCheckable(true);
    editRoiAct_->setShortcut(QKeySequence("Ctrl+E"));
    connect(editRoiAct_, &QAction::toggled, this, &MainWindow::setEditRoiMode);
}

void MainWindow::setupVTKScene() {
    auto colors = vtkSmartPointer<vtkNamedColors>::New();
    vtkRenderer_ = vtkSmartPointer<vtkRenderer>::New();
    vtkRenderer_->SetBackground(colors->GetColor3d("SlateGray").GetData());

    vtkImage_ = vtkSmartPointer<vtkImageData>::New();
    vtkImageActor_ = vtkSmartPointer<vtkImageActor>::New();
    vtkImageActor_->SetVisibility(false);
    vtkImageActor_->InterpolateOff();

    // RDIC heatmap pipeline
    vtkImageRdic_ = vtkSmartPointer<vtkImageData>::New();
    vtkImageActorRdic_ = vtkSmartPointer<vtkImageActor>::New();
    vtkImageActorRdic_->SetVisibility(false);
    vtkImageActorRdic_->GetProperty()->SetOpacity(0.6);
    vtkImageActorRdic_->InterpolateOff();

    // PIV glyph pipeline
    pivPolyData_ = vtkSmartPointer<vtkPolyData>::New();
    auto arrow = vtkSmartPointer<vtkArrowSource>::New();
    pivGlyph_ = vtkSmartPointer<vtkGlyph2D>::New();
    pivGlyph_->SetSourceConnection(arrow->GetOutputPort());
    pivGlyph_->SetInputData(pivPolyData_);
    pivGlyph_->SetVectorModeToUseVector();
    pivGlyph_->SetScaleModeToScaleByVector();
    pivGlyph_->SetScaleFactor(1.0);

    pivGlyphMapper_ = vtkSmartPointer<vtkPolyDataMapper>::New();
    pivGlyphMapper_->SetInputConnection(pivGlyph_->GetOutputPort());
    pivGlyphActor_ = vtkSmartPointer<vtkActor>::New();
    pivGlyphActor_->SetMapper(pivGlyphMapper_);
    pivGlyphActor_->GetProperty()->SetColor(colors->GetColor3d("Tomato").GetData());
    pivGlyphActor_->SetVisibility(false);

    // PIV ROI rectangle pipeline
    roiPolyData_ = vtkSmartPointer<vtkPolyData>::New();
    roiMapper_ = vtkSmartPointer<vtkPolyDataMapper>::New();
    roiMapper_->SetInputData(roiPolyData_);
    roiActor_ = vtkSmartPointer<vtkActor>::New();
    roiActor_->SetMapper(roiMapper_);
    applyRoiStyle();
    roiActor_->SetPosition(0, 0, 0.1); // draw above image actor
    roiActor_->SetVisibility(false);

    vtkRenderer_->AddActor(vtkImageActor_);
    vtkRenderer_->AddActor(vtkImageActorRdic_); // draw RDIC heatmap on top of video
    vtkRenderer_->AddActor(pivGlyphActor_);
    vtkRenderer_->AddActor(roiActor_); // draw ROI rectangle on top (3D actor)

    vtkWindow_ = vtkGenericOpenGLRenderWindow::New();
    vtkWidget_->setRenderWindow(vtkWindow_);
    vtkWidget_->renderWindow()->AddRenderer(vtkRenderer_);
    vtkRenderer_->GetActiveCamera()->ParallelProjectionOn();
}

void MainWindow::openVideo() {
    const QString file = QFileDialog::getOpenFileName(this, "Open Video", QString(), "Video Files (*.mp4 *.avi *.mkv *.mov);;All Files (*.*)");
    if (file.isEmpty()) return;

    stop();

    // Diagnostic information
    QFileInfo fileInfo(file);
    QString diagnostics = QString("File diagnostics:\n");
    diagnostics += QString("- Path: %1\n").arg(file);
    diagnostics += QString("- Exists: %1\n").arg(fileInfo.exists() ? "Yes" : "No");
    diagnostics += QString("- Size: %1 bytes\n").arg(fileInfo.size());
    diagnostics += QString("- Readable: %1\n").arg(fileInfo.isReadable() ? "Yes" : "No");
    diagnostics += QString("- Extension: %1\n").arg(fileInfo.suffix());
    
    // Check for non-ASCII characters
    bool hasNonAscii = false;
    for (const QChar& c : file) {
        if (c.unicode() > 127) {
            hasNonAscii = true;
            break;
        }
    }
    diagnostics += QString("- Contains non-ASCII: %1\n\n").arg(hasNonAscii ? "Yes" : "No");

    // Use UTF-8 path to avoid Windows ANSI encoding issues with non-ASCII filenames
    const std::string path = file.toUtf8().toStdString();

    // Also try with native Windows path for comparison
    const std::string nativePath = file.toLocal8Bit().toStdString();

    // Prefer FFmpeg backend for broader codec/container support; fallback to MSMF and DirectShow
    std::unique_ptr<cv::VideoCapture> cap;
    std::vector<std::tuple<int, const char*, std::string>> backends = {
        {cv::CAP_FFMPEG, "FFmpeg", path},
        {cv::CAP_FFMPEG, "FFmpeg(native)", nativePath},
        {cv::CAP_MSMF,   "MediaFoundation", path},
        {cv::CAP_MSMF,   "MediaFoundation(native)", nativePath},
        {cv::CAP_DSHOW,  "DirectShow", path},
        {cv::CAP_DSHOW,  "DirectShow(native)", nativePath}
    };

    QString tried;
    for (const auto& b : backends) {
        const int backend = std::get<0>(b);
        const char* name = std::get<1>(b);
        const std::string& testPath = std::get<2>(b);
        
        tried += QString("%1 ").arg(name);
        cap = std::make_unique<cv::VideoCapture>(testPath, backend);
        if (cap->isOpened()) {
            diagnostics += QString("Success with: %1\n").arg(name);
            break;
        }
    }

    if (!cap || !cap->isOpened()) {
        QMessageBox::critical(this, "Video Open Failed",
                             diagnostics +
                             QString("Failed to open video with all backends: %1\n\n")
                                 .arg(tried.trimmed()) +
                             "Possible solutions:\n" +
                             "1. Move file to a path with only English characters (e.g., C:\\temp\\)\n" +
                             "2. Ensure the file is not corrupted or in use by another program\n" +
                             "3. Try a different video format (H.264 MP4 works best)\n" +
                             "4. Check file permissions and antivirus software"
        );
        return;
    }

    cap_ = std::move(cap);

    // Probe media using FFmpeg for richer info
    const QString info = FFmpegUtils::probeMedia(file);
    statusBar()->showMessage(info.left(200)); // show a short snippet

    // Determine FPS for timer
    double fps_read = cap_->get(cv::CAP_PROP_FPS);
    fps_ = (fps_read > 1.0 && fps_read < 240.0) ? static_cast<int>(fps_read) : 30;

    play();
}

void MainWindow::play() {
    if (!cap_ || !cap_->isOpened()) return;
    const int interval_ms = std::max(1, static_cast<int>(1000.0 / std::max(1, fps_)));
    timer_->start(interval_ms);
    statusBar()->showMessage(QString("Playing (%1 FPS)").arg(fps_));
}

void MainWindow::stop() {
    timer_->stop();
    if (cap_) {
        cap_->release();
        cap_.reset();
    }
    statusBar()->showMessage("Stopped");
}

void MainWindow::onFrameTick() {
    if (!cap_ || !cap_->isOpened()) return;
    cv::Mat frame;
    if (!cap_->read(frame) || frame.empty()) {
        stop();
        return;
    }
    lastVideoW_ = frame.cols;
    lastVideoH_ = frame.rows;
    cv::Mat display = frame;
    if (showRdic_ && !rdicResult_.mag.empty()) {
        cv::Mat heat = computeRdicHeatmapCurrent();
        if (!heat.empty()) {
            cv::Mat heatResized;
            cv::resize(heat, heatResized, frame.size(), 0, 0, cv::INTER_LINEAR);
            cv::addWeighted(frame, 1.0 - rdicGuiAlpha_, heatResized, rdicGuiAlpha_, 0.0, display);
        }
    }
    QImage img = matToQImage(display);
    if (showPivVectors_ && !pivVectors_.empty()) {
        QPainter painter(&img);
        painter.setRenderHint(QPainter::Antialiasing, true);
        QPen pen(QColor(255, 80, 80));
        pen.setWidth(2);
        painter.setPen(pen);
        for (const auto& v : pivVectors_) {
            QPointF p(v.position.x, v.position.y);
            QPointF q(v.position.x + v.displacement.x, v.position.y + v.displacement.y);
            painter.drawLine(p, q);
            const QPointF dir = q - p;
            const double L = std::sqrt(dir.x()*dir.x() + dir.y()*dir.y());
            if (L > 1.0) {
                QPointF u = dir / L;
                QPointF left = q - 6*u + QPointF(-3*u.y(), 3*u.x());
                QPointF right = q - 6*u + QPointF(3*u.y(), -3*u.x());
                painter.drawLine(q, left);
                painter.drawLine(q, right);
            }
        }
    }
    // Draw ROI overlay in Qt view for consistency with VTK overlay
    if (showRoiOverlay_ && pivParams_.roi.width > 0 && pivParams_.roi.height > 0) {
        QPainter roiPainter(&img);
        roiPainter.setRenderHint(QPainter::Antialiasing, true);
        QColor c = currentRoiQColor();
        QColor edge = c;
        edge.setAlpha(255);
        QPen pen(edge);
        pen.setWidth(2);
        roiPainter.setPen(pen);
        QColor fill = c;
        fill.setAlpha(static_cast<int>(std::clamp(roiOverlayAlpha_, 0.0, 1.0) * 255.0 + 0.5));
        roiPainter.setBrush(QBrush(fill));
        const QRect roiRect(pivParams_.roi.x, pivParams_.roi.y, pivParams_.roi.width, pivParams_.roi.height);
        roiPainter.drawRect(roiRect);
        // Draw edit handles when in edit mode
        if (editRoiMode_) {
            QPen handlePen(Qt::black);
            handlePen.setWidth(1);
            roiPainter.setPen(handlePen);
            roiPainter.setBrush(QBrush(Qt::white));
            const int hs = std::max(3, roiHandleSize_);
            const int x1 = roiRect.left();
            const int y1 = roiRect.top();
            const int x2 = roiRect.right();
            const int y2 = roiRect.bottom();
            const int xm = (x1 + x2) / 2;
            const int ym = (y1 + y2) / 2;
            const std::vector<QPoint> pts = {
                QPoint(x1, y1), QPoint(xm, y1), QPoint(x2, y1),
                QPoint(x1, ym),                QPoint(x2, ym),
                QPoint(x1, y2), QPoint(xm, y2), QPoint(x2, y2)
            };
            for (const auto& pt : pts) {
                roiPainter.drawRect(QRect(pt.x() - hs/2, pt.y() - hs/2, hs, hs));
            }
        }
    }
    videoLabel_->setPixmap(QPixmap::fromImage(img).scaled(videoLabel_->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    if (linkViews_) {
        updateVTKVideoImage(frame);
        updateVTKRoiRectangle(); // Update ROI rectangle overlay
        pivGlyphActor_->SetVisibility(showPivVectors_ && !pivVectors_.empty());
        if (showRdic_ && !rdicResult_.mag.empty()) {
            cv::Mat heat = computeRdicHeatmapCurrent();
            if (!heat.empty()) {
                updateVTKRdicHeatmap(heat);
                vtkImageActorRdic_->GetProperty()->SetOpacity(rdicOverlayAlpha_);
                vtkImageActorRdic_->SetVisibility(true);
            } else {
                vtkImageActorRdic_->SetVisibility(false);
            }
        } else {
            vtkImageActorRdic_->SetVisibility(false);
        }
        vtkWidget_->renderWindow()->Render();
    }
}

QImage MainWindow::matToQImage(const cv::Mat& mat) {
    switch (mat.type()) {
        case CV_8UC3: {
            cv::Mat rgb;
            cv::cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
            return QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888).copy();
        }
        case CV_8UC1: {
            return QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_Grayscale8).copy();
        }
        case CV_8UC4: {
            return QImage(mat.data, mat.cols, mat.rows, static_cast<int>(mat.step), QImage::Format_ARGB32).copy();
        }
        default:
            // Fallback: convert to RGB
            cv::Mat rgb;
            mat.convertTo(rgb, CV_8UC3);
            return QImage(rgb.data, rgb.cols, rgb.rows, static_cast<int>(rgb.step), QImage::Format_RGB888).copy();
    }
}

void MainWindow::runPIVAnalysis() {
    if (!cap_ || !cap_->isOpened()) {
        QMessageBox::warning(this, "PIV", "Please open a video first.");
        return;
    }
    const bool wasRunning = timer_->isActive();
    if (wasRunning) timer_->stop();

    cv::Mat f0, f1;
    if (!cap_->read(f0) || f0.empty()) {
        QMessageBox::warning(this, "PIV", "Failed to read frame 0.");
        if (wasRunning) play();
        return;
    }
    if (!cap_->read(f1) || f1.empty()) {
        QMessageBox::warning(this, "PIV", "Failed to read frame 1.");
        if (wasRunning) play();
        return;
    }

    // Use existing PIV parameters from settings, only set defaults if not already configured
    if (pivParams_.windowSize <= 0) pivParams_.windowSize = 32;
    if (pivParams_.overlap < 0) pivParams_.overlap = 16;
    if (pivParams_.searchRadius < 0) pivParams_.searchRadius = 8;
    // Keep existing ROI settings from user configuration

    // Crop to ROI to reduce computation if set and valid
    cv::Rect roi = pivParams_.roi;
    auto validRoi = [&](const cv::Mat& m){ return roi.width > 0 && roi.height > 0 && roi.x >= 0 && roi.y >= 0 && roi.x + roi.width <= m.cols && roi.y + roi.height <= m.rows; };
    PivParams params = pivParams_; // make a local copy to avoid data races
    if (validRoi(f0) && validRoi(f1)) {
        f0 = f0(roi).clone();
        f1 = f1(roi).clone();
        // After cropping, ROI should cover the entire cropped frames
        params.roi = cv::Rect(0, 0, f0.cols, f0.rows);
    }

    // Run analysis asynchronously to keep UI responsive
    setCursor(Qt::BusyCursor);
    statusBar()->showMessage("Running PIV (two frames)...");
    std::thread([this, wasRunning, f0 = std::move(f0), f1 = std::move(f1), params]() mutable {
        QElapsedTimer t; t.start();
        PivResult res;
        const bool ok = PivEngine::analyze(f0, f1, params, res);
        const qint64 ms = t.elapsed();
        QMetaObject::invokeMethod(this, [this, ok, res = std::move(res), wasRunning, ms]() mutable {
            unsetCursor();
            if (!ok) {
                QMessageBox::warning(this, "PIV", "PIV analysis failed.");
                showPivVectors_ = false;
                if (wasRunning) play();
                return;
            }
            pivVectors_ = std::move(res.vectors);
            showPivVectors_ = true;
            statusBar()->showMessage(QString("PIV computed in %1 ms, vectors: %2").arg(ms).arg(pivVectors_.size()), 3000);
            updateVTKPivGlyphs();
            if (wasRunning) play();
        }, Qt::QueuedConnection);
    }).detach();
}

void MainWindow::togglePivOverlay() {
    showPivVectors_ = !showPivVectors_;
    onFrameTick();
}

void MainWindow::toggleLinkViews() {
    linkViews_ = !linkViews_;
}

void MainWindow::toggleRdicOverlay() {
    showRdic_ = !showRdic_;
    onFrameTick();
}

void MainWindow::runRDICAnalysis() {
    if (!cap_ || !cap_->isOpened()) {
        QMessageBox::warning(this, "RDIC", "Please open a video first.");
        return;
    }
    const bool wasRunning = timer_->isActive();
    if (wasRunning) timer_->stop();

    cv::Mat f0, f1;
    if (!cap_->read(f0) || f0.empty()) {
        QMessageBox::warning(this, "RDIC", "Failed to read frame 0.");
        if (wasRunning) play();
        return;
    }
    if (!cap_->read(f1) || f1.empty()) {
        QMessageBox::warning(this, "RDIC", "Failed to read frame 1.");
        if (wasRunning) play();
        return;
    }

    // Use persisted parameters if available; otherwise set defaults
    if (rdicParams_.windowSize <= 0) rdicParams_.windowSize = 21;
    if (rdicParams_.gridStep <= 0) rdicParams_.gridStep = 8;
    if (rdicParams_.smoothSigma <= 0.0) rdicParams_.smoothSigma = 1.0;

    // Crop to ROI to significantly reduce computation if ROI is valid
    cv::Rect roi = pivParams_.roi;
    auto validRoi = [&](const cv::Mat& m){ return roi.width > 0 && roi.height > 0 && roi.x >= 0 && roi.y >= 0 && roi.x + roi.width <= m.cols && roi.y + roi.height <= m.rows; };
    if (validRoi(f0) && validRoi(f1)) {
        f0 = f0(roi).clone();
        f1 = f1(roi).clone();
    }

    // Run analysis asynchronously to keep UI responsive
    setCursor(Qt::BusyCursor);
    statusBar()->showMessage("Running RDIC (two frames)...");
    RdicParams rdicParamsLocal = rdicParams_; // copy to avoid data races
    std::thread([this, wasRunning, f0 = std::move(f0), f1 = std::move(f1), rdicParamsLocal]() mutable {
        QElapsedTimer t; t.start();
        RdicResult res;
        const bool ok = RdicEngine::analyze(f0, f1, rdicParamsLocal, res);
        const qint64 ms = t.elapsed();
        QMetaObject::invokeMethod(this, [this, ok, res = std::move(res), wasRunning, ms]() mutable {
            unsetCursor();
            if (!ok) {
                QMessageBox::warning(this, "RDIC", "RDIC analysis failed.");
                showRdic_ = false;
                if (wasRunning) play();
                return;
            }
            rdicResult_ = std::move(res);
            showRdic_ = true;
            statusBar()->showMessage(QString("RDIC computed in %1 ms").arg(ms), 3000);
            if (linkViews_) {
                cv::Mat heat = computeRdicHeatmapCurrent();
                updateVTKRdicHeatmap(heat);
                vtkImageActorRdic_->GetProperty()->SetOpacity(rdicOverlayAlpha_);
                vtkImageActorRdic_->SetVisibility(true);
                vtkWidget_->renderWindow()->Render();
            }
            if (wasRunning) play();
        }, Qt::QueuedConnection);
    }).detach();
}

void MainWindow::updateVTKVideoImage(const cv::Mat& frame) {
    if (frame.empty()) return;
    cv::Mat rgb;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, rgb, cv::COLOR_BGR2RGB);
    } else if (frame.channels() == 1) {
        cv::cvtColor(frame, rgb, cv::COLOR_GRAY2RGB);
    } else if (frame.channels() == 4) {
        cv::cvtColor(frame, rgb, cv::COLOR_BGRA2RGB);
    } else {
        frame.convertTo(rgb, CV_8UC3);
    }
    const int w = rgb.cols;
    const int h = rgb.rows;
    vtkImage_->SetDimensions(w, h, 1);
    vtkImage_->SetOrigin(0.0, 0.0, 0.0);
    vtkImage_->SetSpacing(1.0, 1.0, 1.0);
    vtkImage_->AllocateScalars(VTK_UNSIGNED_CHAR, 3);
    unsigned char* ptr = static_cast<unsigned char*>(vtkImage_->GetScalarPointer());
    const size_t stride = static_cast<size_t>(rgb.step);
    // Write rows in reverse order to match VTK's bottom-left origin
    for (int y = 0; y < h; ++y) {
        const unsigned char* srcRow = rgb.ptr<unsigned char>(y);
        std::memcpy(ptr + static_cast<size_t>(h - 1 - y) * stride, srcRow, stride);
    }
    vtkImageActor_->SetInputData(vtkImage_);
    vtkImageActor_->SetVisibility(true);

    if (!vtkCameraInit_ || w != vtkImageW_ || h != vtkImageH_) {
        vtkRenderer_->ResetCamera();
        vtkImageW_ = w;
        vtkImageH_ = h;
        vtkCameraInit_ = true;
    }

    // Ensure ROI has a sensible default within bounds when unset or invalid
    if (pivParams_.roi.width <= 0 || pivParams_.roi.height <= 0 ||
        pivParams_.roi.x < 0 || pivParams_.roi.y < 0 ||
        pivParams_.roi.x + pivParams_.roi.width > w ||
        pivParams_.roi.y + pivParams_.roi.height > h) {
        const int roiW = std::max(1, static_cast<int>(w * 0.6));
        const int roiH = std::max(1, static_cast<int>(h * 0.6));
        const int roiX = (w - roiW) / 2;
        const int roiY = (h - roiH) / 2;
        pivParams_.roi = cv::Rect(roiX, roiY, roiW, roiH);
    }
}

void MainWindow::updateVTKRdicHeatmap(const cv::Mat& heatmap) {
    if (heatmap.empty()) return;
    cv::Mat rgb;
    if (heatmap.channels() == 3) {
        cv::cvtColor(heatmap, rgb, cv::COLOR_BGR2RGB);
    } else {
        cv::cvtColor(heatmap, rgb, cv::COLOR_GRAY2RGB);
    }
    const int w = rgb.cols;
    const int h = rgb.rows;
    vtkImageRdic_->SetDimensions(w, h, 1);
    vtkImageRdic_->SetOrigin(0.0, 0.0, 0.0);
    vtkImageRdic_->SetSpacing(1.0, 1.0, 1.0);
    vtkImageRdic_->AllocateScalars(VTK_UNSIGNED_CHAR, 3);
    unsigned char* ptr = static_cast<unsigned char*>(vtkImageRdic_->GetScalarPointer());
    const size_t stride = static_cast<size_t>(rgb.step);
    // Write rows in reverse order to match VTK's bottom-left origin
    for (int y = 0; y < h; ++y) {
        const unsigned char* srcRow = rgb.ptr<unsigned char>(y);
        std::memcpy(ptr + static_cast<size_t>(h - 1 - y) * stride, srcRow, stride);
    }
    vtkImageActorRdic_->SetInputData(vtkImageRdic_);
}

void MainWindow::updateVTKPivGlyphs() {
    auto points = vtkSmartPointer<vtkPoints>::New();
    auto vectors = vtkSmartPointer<vtkDoubleArray>::New();
    vectors->SetNumberOfComponents(3);
    vectors->SetName("vectors");

    double minMag = std::numeric_limits<double>::max();
    double maxMag = std::numeric_limits<double>::lowest();
    auto mags = vtkSmartPointer<vtkDoubleArray>::New();
    mags->SetNumberOfComponents(1);
    mags->SetName("magnitude");

    for (const auto& v : pivVectors_) {
        points->InsertNextPoint(v.position.x, v.position.y, 0.0);
        vectors->InsertNextTuple3(v.displacement.x, v.displacement.y, 0.0);
        const double m = std::sqrt(v.displacement.x*v.displacement.x + v.displacement.y*v.displacement.y);
        mags->InsertNextValue(m);
        minMag = std::min(minMag, m);
        maxMag = std::max(maxMag, m);
    }
    pivPolyData_->SetPoints(points);
    pivPolyData_->GetPointData()->SetVectors(vectors);
    pivPolyData_->GetPointData()->SetScalars(mags);

    // Color map by magnitude
    auto lut = vtkSmartPointer<vtkLookupTable>::New();
    lut->SetNumberOfTableValues(256);
    lut->Build();
    pivGlyphMapper_->ScalarVisibilityOn();
    pivGlyphMapper_->SetScalarModeToUsePointData();
    pivGlyphMapper_->SetColorModeToMapScalars();
    pivGlyphMapper_->SetLookupTable(lut);
    if (minMag < maxMag) {
        pivGlyphMapper_->SetScalarRange(minMag, maxMag);
    }
}

void MainWindow::openRdicSettings() {
    QDialog dlg(this);
    dlg.setWindowTitle("RDIC Settings");
    auto form = new QFormLayout(&dlg);

    auto winSize = new QSpinBox(&dlg);
    winSize->setRange(5, 101);
    winSize->setSingleStep(2);
    winSize->setValue(rdicParams_.windowSize > 0 ? rdicParams_.windowSize : 21);

    auto gridStep = new QSpinBox(&dlg);
    gridStep->setRange(1, 64);
    gridStep->setValue(rdicParams_.gridStep > 0 ? rdicParams_.gridStep : 8);

    auto smoothSigma = new QDoubleSpinBox(&dlg);
    smoothSigma->setRange(0.0, 10.0);
    smoothSigma->setDecimals(2);
    smoothSigma->setSingleStep(0.1);
    smoothSigma->setValue(rdicParams_.smoothSigma > 0.0 ? rdicParams_.smoothSigma : 1.0);

    auto heatType = new QComboBox(&dlg);
    heatType->addItems({"Von-Mises","exx","eyy","exy"});
    heatType->setCurrentIndex(static_cast<int>(rdicHeatType_));

    auto colorMap = new QComboBox(&dlg);
    colorMap->addItems({"Jet","Turbo","Viridis"});
    colorMap->setCurrentIndex(static_cast<int>(rdicColorMap_));

    auto overlayAlpha = new QSlider(Qt::Horizontal, &dlg);
    overlayAlpha->setRange(0, 100);
    overlayAlpha->setValue(static_cast<int>(rdicOverlayAlpha_ * 100.0));

    auto guiAlpha = new QSlider(Qt::Horizontal, &dlg);
    guiAlpha->setRange(0, 100);
    guiAlpha->setValue(static_cast<int>(rdicGuiAlpha_ * 100.0));

    auto rangeAuto = new QCheckBox("Auto Range", &dlg);
    rangeAuto->setChecked(rdicRangeAuto_);

    auto rangeMin = new QDoubleSpinBox(&dlg);
    rangeMin->setRange(-1e9, 1e9);
    rangeMin->setDecimals(4);
    rangeMin->setSingleStep(0.01);
    rangeMin->setValue(rdicRangeMin_);
    rangeMin->setEnabled(!rdicRangeAuto_);

    auto rangeMax = new QDoubleSpinBox(&dlg);
    rangeMax->setRange(-1e9, 1e9);
    rangeMax->setDecimals(4);
    rangeMax->setSingleStep(0.01);
    rangeMax->setValue(rdicRangeMax_);
    rangeMax->setEnabled(!rdicRangeAuto_);

    auto invert = new QCheckBox("Invert Colormap", &dlg);
    invert->setChecked(rdicInvert_);

    form->addRow("Window Size", winSize);
    form->addRow("Grid Step", gridStep);
    form->addRow("Smooth Sigma", smoothSigma);
    form->addRow("Heat Type", heatType);
    form->addRow("Color Map", colorMap);
    form->addRow("VTK Overlay Opacity", overlayAlpha);
    form->addRow("GUI Overlay Alpha", guiAlpha);
    form->addRow(rangeAuto);
    form->addRow("Range Min", rangeMin);
    form->addRow("Range Max", rangeMax);
    form->addRow(invert);

    QObject::connect(rangeAuto, &QCheckBox::toggled, [&](bool autoOn){
        rangeMin->setEnabled(!autoOn);
        rangeMax->setEnabled(!autoOn);
    });

    auto btns = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    form->addRow(btns);
    QObject::connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    QObject::connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    if (dlg.exec() == QDialog::Accepted) {
        rdicParams_.windowSize = winSize->value();
        rdicParams_.gridStep = gridStep->value();
        rdicParams_.smoothSigma = smoothSigma->value();
        rdicHeatType_ = static_cast<RdicHeatType>(heatType->currentIndex());
        rdicColorMap_ = static_cast<RdicColorMap>(colorMap->currentIndex());
        rdicOverlayAlpha_ = overlayAlpha->value() / 100.0;
        rdicGuiAlpha_ = guiAlpha->value() / 100.0;
        rdicRangeAuto_ = rangeAuto->isChecked();
        rdicRangeMin_ = rangeMin->value();
        rdicRangeMax_ = rangeMax->value();
        rdicInvert_ = invert->isChecked();
        vtkImageActorRdic_->GetProperty()->SetOpacity(rdicOverlayAlpha_);
        saveSettings();
        onFrameTick();
    }
}

static int toCvColormap(int m) {
    switch (m) {
        case 1: return cv::COLORMAP_TURBO;
        case 2: return cv::COLORMAP_VIRIDIS;
        case 0:
        default: return cv::COLORMAP_JET;
    }
}

static cv::Mat makeHeat(const cv::Mat& src, int colormap, bool autoRange, double minPref, double maxPref, bool invert) {
    if (src.empty()) return {};
    cv::Mat mag; src.convertTo(mag, CV_32F);
    double minVal=minPref, maxVal=maxPref;
    if (autoRange) {
        cv::minMaxLoc(mag, &minVal, &maxVal);
    }
    double range = std::max(maxVal - minVal, 1e-9);
    cv::Mat norm = (mag - static_cast<float>(minVal)) * (1.0f/static_cast<float>(range));
    if (invert) norm = 1.0f - norm;
    cv::Mat u8; norm.convertTo(u8, CV_8U, 255.0);
    cv::Mat heat; cv::applyColorMap(u8, heat, colormap);
    return heat;
}

cv::Mat MainWindow::computeRdicHeatmapCurrent() const {
    int cmap;
    switch (rdicColorMap_) {
        case RdicColorMap::Turbo: cmap = cv::COLORMAP_TURBO; break;
        case RdicColorMap::Viridis: cmap = cv::COLORMAP_VIRIDIS; break;
        case RdicColorMap::Jet:
        default: cmap = cv::COLORMAP_JET; break;
    }
    switch (rdicHeatType_) {
        case RdicHeatType::VonMises:
            return makeHeat(rdicResult_.mag, cmap, rdicRangeAuto_, rdicRangeMin_, rdicRangeMax_, rdicInvert_);
        case RdicHeatType::Exx:
            return makeHeat(rdicResult_.exx, cmap, rdicRangeAuto_, rdicRangeMin_, rdicRangeMax_, rdicInvert_);
        case RdicHeatType::Eyy:
            return makeHeat(rdicResult_.eyy, cmap, rdicRangeAuto_, rdicRangeMin_, rdicRangeMax_, rdicInvert_);
        case RdicHeatType::Exy:
            return makeHeat(rdicResult_.exy, cmap, rdicRangeAuto_, rdicRangeMin_, rdicRangeMax_, rdicInvert_);
        default:
            return makeHeat(rdicResult_.mag, cmap, rdicRangeAuto_, rdicRangeMin_, rdicRangeMax_, rdicInvert_);
    }
}

void MainWindow::applyRoiStyle() {
    auto colors = vtkSmartPointer<vtkNamedColors>::New();
    if (roiColorCustom_) {
        roiActor_->GetProperty()->SetColor(roiOverlayRGB_[0], roiOverlayRGB_[1], roiOverlayRGB_[2]);
        roiActor_->GetProperty()->SetEdgeVisibility(true);
        roiActor_->GetProperty()->SetEdgeColor(roiOverlayRGB_[0], roiOverlayRGB_[1], roiOverlayRGB_[2]);
    } else {
        roiActor_->GetProperty()->SetColor(colors->GetColor3d(roiOverlayColor_.c_str()).GetData());
        roiActor_->GetProperty()->SetEdgeVisibility(true);
        roiActor_->GetProperty()->SetEdgeColor(colors->GetColor3d(roiOverlayColor_.c_str()).GetData());
    }
    roiActor_->GetProperty()->SetLineWidth(2.0);
    roiActor_->GetProperty()->SetOpacity(roiOverlayAlpha_);
}

void MainWindow::loadSettings() {
    QSettings s("VisionFlowStudio", "VisionFlowStudio");
    // PIV settings
    pivParams_.windowSize = s.value("piv/windowSize", 32).toInt();
    pivParams_.overlap = s.value("piv/overlap", 16).toInt();
    pivParams_.searchRadius = s.value("piv/searchRadius", 8).toInt();
    pivParams_.useCLAHE = s.value("piv/useCLAHE", true).toBool();
    pivParams_.calibrationScale = s.value("piv/calibrationScale", 1.0).toDouble();
    pivParams_.roi.x = s.value("piv/roiX", 0).toInt();
    pivParams_.roi.y = s.value("piv/roiY", 0).toInt();
    pivParams_.roi.width = s.value("piv/roiW", 0).toInt();
    pivParams_.roi.height = s.value("piv/roiH", 0).toInt();

    // RDIC settings
    rdicParams_.windowSize = s.value("rdic/windowSize", 21).toInt();
    rdicParams_.gridStep = s.value("rdic/gridStep", 8).toInt();
    rdicParams_.smoothSigma = s.value("rdic/smoothSigma", 1.0).toDouble();
    rdicHeatType_ = static_cast<RdicHeatType>(s.value("rdic/heatType", static_cast<int>(RdicHeatType::VonMises)).toInt());
    rdicColorMap_ = static_cast<RdicColorMap>(s.value("rdic/colorMap", static_cast<int>(RdicColorMap::Jet)).toInt());
    rdicOverlayAlpha_ = s.value("rdic/overlayAlpha", 0.6).toDouble();
    rdicGuiAlpha_ = s.value("rdic/guiAlpha", 0.3).toDouble();
    rdicRangeAuto_ = s.value("rdic/rangeAuto", true).toBool();
    rdicRangeMin_ = s.value("rdic/rangeMin", 0.0).toDouble();
    rdicRangeMax_ = s.value("rdic/rangeMax", 1.0).toDouble();
    rdicInvert_ = s.value("rdic/invert", false).toBool();

    // ROI overlay settings
    showRoiOverlay_ = s.value("roi/show", true).toBool();
    roiOverlayAlpha_ = s.value("roi/alpha", 0.25).toDouble();
    roiOverlayColor_ = s.value("roi/color", "Yellow").toString().toStdString();
    const QString rgb = s.value("roi/colorRGB", "").toString();
    if (!rgb.isEmpty()) {
        const auto parts = rgb.split(",");
        if (parts.size() == 3) {
            bool ok1=false, ok2=false, ok3=false;
            const int r = parts[0].toInt(&ok1);
            const int g = parts[1].toInt(&ok2);
            const int b = parts[2].toInt(&ok3);
            if (ok1 && ok2 && ok3) {
                roiColorCustom_ = true;
                roiOverlayRGB_[0] = std::clamp(r, 0, 255) / 255.0;
                roiOverlayRGB_[1] = std::clamp(g, 0, 255) / 255.0;
                roiOverlayRGB_[2] = std::clamp(b, 0, 255) / 255.0;
            }
        }
    }
    applyRoiStyle();
}

void MainWindow::saveSettings() const {
    QSettings s("VisionFlowStudio", "VisionFlowStudio");
    // PIV settings
    s.setValue("piv/windowSize", pivParams_.windowSize);
    s.setValue("piv/overlap", pivParams_.overlap);
    s.setValue("piv/searchRadius", pivParams_.searchRadius);
    s.setValue("piv/useCLAHE", pivParams_.useCLAHE);
    s.setValue("piv/calibrationScale", pivParams_.calibrationScale);
    s.setValue("piv/roiX", pivParams_.roi.x);
    s.setValue("piv/roiY", pivParams_.roi.y);
    s.setValue("piv/roiW", pivParams_.roi.width);
    s.setValue("piv/roiH", pivParams_.roi.height);

    // RDIC settings
    s.setValue("rdic/windowSize", rdicParams_.windowSize);
    s.setValue("rdic/gridStep", rdicParams_.gridStep);
    s.setValue("rdic/smoothSigma", rdicParams_.smoothSigma);
    s.setValue("rdic/heatType", static_cast<int>(rdicHeatType_));
    s.setValue("rdic/colorMap", static_cast<int>(rdicColorMap_));
    s.setValue("rdic/overlayAlpha", rdicOverlayAlpha_);
    s.setValue("rdic/guiAlpha", rdicGuiAlpha_);
    s.setValue("rdic/rangeAuto", rdicRangeAuto_);
    s.setValue("rdic/rangeMin", rdicRangeMin_);
    s.setValue("rdic/rangeMax", rdicRangeMax_);
    s.setValue("rdic/invert", rdicInvert_);

    // ROI overlay settings
    s.setValue("roi/show", showRoiOverlay_);
    s.setValue("roi/alpha", roiOverlayAlpha_);
    if (roiColorCustom_) {
        const int r = static_cast<int>(roiOverlayRGB_[0] * 255.0 + 0.5);
        const int g = static_cast<int>(roiOverlayRGB_[1] * 255.0 + 0.5);
        const int b = static_cast<int>(roiOverlayRGB_[2] * 255.0 + 0.5);
        s.setValue("roi/color", "Custom");
        s.setValue("roi/colorRGB", QString("%1,%2,%3").arg(r).arg(g).arg(b));
    } else {
        s.setValue("roi/color", QString::fromStdString(roiOverlayColor_));
        s.setValue("roi/colorRGB", "");
    }
}

void MainWindow::openPivSettings() {
    QDialog dlg(this);
    dlg.setWindowTitle("PIV Settings");
    auto form = new QFormLayout(&dlg);

    auto winSize = new QSpinBox(&dlg);
    winSize->setRange(8, 256);
    winSize->setSingleStep(2);
    winSize->setValue(pivParams_.windowSize > 0 ? pivParams_.windowSize : 32);

    auto overlap = new QSpinBox(&dlg);
    overlap->setRange(0, 255);
    overlap->setValue(pivParams_.overlap >= 0 ? pivParams_.overlap : 16);

    auto searchRadius = new QSpinBox(&dlg);
    searchRadius->setRange(0, 64);
    searchRadius->setValue(pivParams_.searchRadius >= 0 ? pivParams_.searchRadius : 8);

    auto useClahe = new QCheckBox("Use CLAHE", &dlg);
    useClahe->setChecked(pivParams_.useCLAHE);

    auto calibScale = new QDoubleSpinBox(&dlg);
    calibScale->setRange(0.01, 100.0);
    calibScale->setDecimals(3);
    calibScale->setSingleStep(0.01);
    calibScale->setValue(pivParams_.calibrationScale > 0.0 ? pivParams_.calibrationScale : 1.0);

    auto roiX = new QSpinBox(&dlg);
    auto roiY = new QSpinBox(&dlg);
    auto roiW = new QSpinBox(&dlg);
    auto roiH = new QSpinBox(&dlg);
    roiX->setRange(0, 10000); roiY->setRange(0, 10000);
    roiW->setRange(0, 10000); roiH->setRange(0, 10000);
    roiX->setValue(pivParams_.roi.x); roiY->setValue(pivParams_.roi.y);
    roiW->setValue(pivParams_.roi.width); roiH->setValue(pivParams_.roi.height);

    // New: ROI overlay style controls
    auto showRoi = new QCheckBox("Show ROI Overlay", &dlg);
    showRoi->setChecked(showRoiOverlay_);

    auto roiAlpha = new QSlider(Qt::Horizontal, &dlg);
    roiAlpha->setRange(0, 100);
    roiAlpha->setValue(static_cast<int>(roiOverlayAlpha_ * 100.0));

    auto roiColor = new QComboBox(&dlg);
    roiColor->addItems({"Yellow","Lime","Cyan","Magenta","Red","White","Custom..."});
    int colorIndex = roiColor->findText(QString::fromStdString(roiOverlayColor_));
    if (roiColorCustom_) {
        roiColor->setCurrentText("Custom...");
    } else {
        roiColor->setCurrentIndex(colorIndex >= 0 ? colorIndex : 0);
    }

    form->addRow("Window Size", winSize);
    form->addRow("Overlap", overlap);
    form->addRow("Search Radius", searchRadius);
    form->addRow(useClahe);
    form->addRow("Calibration Scale", calibScale);
    form->addRow("ROI X", roiX);
    form->addRow("ROI Y", roiY);
    form->addRow("ROI W", roiW);
    form->addRow("ROI H", roiH);
    form->addRow("Show ROI", showRoi);
    form->addRow("ROI Opacity", roiAlpha);
    form->addRow("ROI Color", roiColor);

    auto buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    form->addRow(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    // Preview ROI overlay without closing the dialog
    QPushButton* previewBtn = buttons->addButton("Preview ROI", QDialogButtonBox::ActionRole);
    connect(previewBtn, &QPushButton::clicked, [=]() {
        pivParams_.roi = cv::Rect(roiX->value(), roiY->value(), roiW->value(), roiH->value());
        showRoiOverlay_ = showRoi->isChecked();
        roiOverlayAlpha_ = roiAlpha->value() / 100.0;
        if (roiColor->currentText() != "Custom...") {
            roiColorCustom_ = false;
            roiOverlayColor_ = roiColor->currentText().toStdString();
        }
        applyRoiStyle();
        saveSettings();
        updateVTKRoiRectangle();
        if (linkViews_) vtkWidget_->renderWindow()->Render();
        onFrameTick();
        statusBar()->showMessage("Previewed ROI overlay", 1000);
    });

    if (dlg.exec() == QDialog::Accepted) {
        pivParams_.windowSize = winSize->value();
        pivParams_.overlap = overlap->value();
        pivParams_.searchRadius = searchRadius->value();
        pivParams_.useCLAHE = useClahe->isChecked();
        pivParams_.calibrationScale = calibScale->value();
        pivParams_.roi = cv::Rect(roiX->value(), roiY->value(), roiW->value(), roiH->value());

        // Apply ROI overlay style
        showRoiOverlay_ = showRoi->isChecked();
        roiOverlayAlpha_ = roiAlpha->value() / 100.0;
        const bool wantsCustom = (roiColor->currentText() == "Custom...");
        if (wantsCustom) {
            // Use current custom color as default if available
            QColor defaultColor = roiColorCustom_ ? QColor(static_cast<int>(roiOverlayRGB_[0]*255.0),
                                                           static_cast<int>(roiOverlayRGB_[1]*255.0),
                                                           static_cast<int>(roiOverlayRGB_[2]*255.0))
                                                  : QColor("yellow");
            const QColor c = QColorDialog::getColor(defaultColor, this, "Pick ROI Color");
            if (c.isValid()) {
                roiColorCustom_ = true;
                roiOverlayRGB_[0] = c.red() / 255.0;
                roiOverlayRGB_[1] = c.green() / 255.0;
                roiOverlayRGB_[2] = c.blue() / 255.0;
            } // if user cancels, keep previous color state
        } else {
            roiColorCustom_ = false;
            roiOverlayColor_ = roiColor->currentText().toStdString();
        }
        applyRoiStyle();
        saveSettings();
        updateVTKRoiRectangle(); // Update ROI rectangle after settings change
        if (linkViews_) {
            vtkWidget_->renderWindow()->Render();
        }
    }
}

void MainWindow::exportPivCSV() {
    if (pivVectors_.empty()) {
        QMessageBox::warning(this, "Export CSV", "No PIV vectors to export. Run PIV first.");
        return;
    }
    // Generate default filename with timestamp and parameters
    const QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    const QString defaultName = QString("PIV_ws%1_ol%2_sr%3_%4.csv")
        .arg(pivParams_.windowSize)
        .arg(pivParams_.overlap)
        .arg(pivParams_.searchRadius)
        .arg(timestamp);
    const QString path = QFileDialog::getSaveFileName(this, "Save PIV CSV", defaultName, "CSV Files (*.csv)");
    if (path.isEmpty()) return;

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(this, "Export CSV", "Failed to open file for writing.");
        return;
    }
    QTextStream out(&file);
    out << "x,y,dx,dy,score\n";
    for (const auto& v : pivVectors_) {
        out << v.position.x << "," << v.position.y << ","
            << v.displacement.x << "," << v.displacement.y << ","
            << v.score << "\n";
    }
    file.close();
    statusBar()->showMessage(QString("Exported %1 vectors to CSV").arg(pivVectors_.size()), 3000);
}

void MainWindow::exportRdicCSV() {
    if (rdicResult_.mag.empty()) {
        QMessageBox::warning(this, "Export CSV", "No RDIC results to export. Run RDIC first.");
        return;
    }
    
    // Ask user whether to export current heat type only or all types
    QMessageBox msgBox;
    msgBox.setWindowTitle("RDIC CSV Export");
    msgBox.setText("Choose export format:");
    msgBox.setInformativeText("Export current heat type only or all heat map types?");
    QPushButton *currentBtn = msgBox.addButton("Current Type Only", QMessageBox::ActionRole);
    QPushButton *allBtn = msgBox.addButton("All Types", QMessageBox::ActionRole);
    QPushButton *cancelBtn = msgBox.addButton(QMessageBox::Cancel);
    msgBox.setDefaultButton(allBtn);
    msgBox.exec();
    
    QAbstractButton* clicked = msgBox.clickedButton();
    if (clicked->text() == "Cancel") return;
    
    // Generate default filename with timestamp and type information
    const QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    QString defaultName;
    if (clicked->text() == "Current Type Only") {
        QString typeName;
        switch (rdicHeatType_) {
            case RdicHeatType::VonMises: typeName = "VonMises"; break;
            case RdicHeatType::Exx: typeName = "Exx"; break;
            case RdicHeatType::Eyy: typeName = "Eyy"; break;
            case RdicHeatType::Exy: typeName = "Exy"; break;
        }
        defaultName = QString("RDIC_%1_ws%2_gs%3_%4.csv")
            .arg(typeName)
            .arg(rdicParams_.windowSize)
            .arg(rdicParams_.gridStep)
            .arg(timestamp);
    } else {
        defaultName = QString("RDIC_AllTypes_ws%1_gs%2_%3.csv")
            .arg(rdicParams_.windowSize)
            .arg(rdicParams_.gridStep)
            .arg(timestamp);
    }
    const QString path = QFileDialog::getSaveFileName(this, "Save RDIC CSV", defaultName, "CSV Files (*.csv)");
    if (path.isEmpty()) return;

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(this, "Export CSV", "Failed to open file for writing.");
        return;
    }
    QTextStream out(&file);
    
    if (clicked->text() == "Current Type Only") {
        // Export current heat type only
        cv::Mat src;
        QString typeName;
        switch (rdicHeatType_) {
            case RdicHeatType::VonMises: src = rdicResult_.mag; typeName = "VonMises"; break;
            case RdicHeatType::Exx: src = rdicResult_.exx; typeName = "Exx"; break;
            case RdicHeatType::Eyy: src = rdicResult_.eyy; typeName = "Eyy"; break;
            case RdicHeatType::Exy: src = rdicResult_.exy; typeName = "Exy"; break;
            default: src = rdicResult_.mag; typeName = "VonMises"; break;
        }
        cv::Mat f; src.convertTo(f, CV_32F);
        out << "x,y," << typeName << "\n";
        for (int y = 0; y < f.rows; ++y) {
            const float* row = f.ptr<float>(y);
            for (int x = 0; x < f.cols; ++x) {
                out << x << "," << y << "," << row[x] << "\n";
            }
        }
        statusBar()->showMessage(QString("Exported %1 %2 values to CSV").arg(f.cols * f.rows).arg(typeName), 3000);
    } else {
        // Export all heat types
        out << "x,y,VonMises,Exx,Eyy,Exy\n";
        cv::Mat magF, exxF, eyyF, exyF;
        rdicResult_.mag.convertTo(magF, CV_32F);
        rdicResult_.exx.convertTo(exxF, CV_32F);
        rdicResult_.eyy.convertTo(eyyF, CV_32F);
        rdicResult_.exy.convertTo(exyF, CV_32F);
        
        const int rows = magF.rows;
        const int cols = magF.cols;
        for (int y = 0; y < rows; ++y) {
            const float* magRow = magF.ptr<float>(y);
            const float* exxRow = exxF.ptr<float>(y);
            const float* eyyRow = eyyF.ptr<float>(y);
            const float* exyRow = exyF.ptr<float>(y);
            for (int x = 0; x < cols; ++x) {
                out << x << "," << y << "," << magRow[x] << "," << exxRow[x] << "," << eyyRow[x] << "," << exyRow[x] << "\n";
            }
        }
        statusBar()->showMessage(QString("Exported %1 values (all types) to RDIC CSV").arg(rows * cols), 3000);
    }
    file.close();
}

void MainWindow::toggleRoiOverlay() {
    showRoiOverlay_ = !showRoiOverlay_;
    updateVTKRoiRectangle();
    if (linkViews_) vtkWidget_->renderWindow()->Render();
}
void MainWindow::updateVTKRoiRectangle() {
    if (vtkImageW_ <= 0 || vtkImageH_ <= 0 || !showRoiOverlay_) {
        roiActor_->SetVisibility(false);
        return;
    }

    // Get ROI parameters
    const int roiX = pivParams_.roi.x;
    const int roiY = pivParams_.roi.y;
    const int roiW = pivParams_.roi.width;
    const int roiH = pivParams_.roi.height;

    // Validate ROI bounds
    if (roiW <= 0 || roiH <= 0 || roiX < 0 || roiY < 0 || 
        roiX + roiW > vtkImageW_ || roiY + roiH > vtkImageH_) {
        roiActor_->SetVisibility(false);
        return;
    }

    // Create rectangle points (VTK uses bottom-left origin, but we need to flip Y)
    auto points = vtkSmartPointer<vtkPoints>::New();
    const double x1 = roiX;
    const double y1 = vtkImageH_ - roiY - roiH; // Flip Y coordinate
    const double x2 = roiX + roiW;
    const double y2 = vtkImageH_ - roiY; // Flip Y coordinate

    points->InsertNextPoint(x1, y1, 0); // Bottom-left
    points->InsertNextPoint(x2, y1, 0); // Bottom-right
    points->InsertNextPoint(x2, y2, 0); // Top-right
    points->InsertNextPoint(x1, y2, 0); // Top-left

    // Create rectangle lines
    auto lines = vtkSmartPointer<vtkCellArray>::New();
    lines->InsertNextCell(5); // 5 points to close the rectangle
    lines->InsertCellPoint(0);
    lines->InsertCellPoint(1);
    lines->InsertCellPoint(2);
    lines->InsertCellPoint(3);
    lines->InsertCellPoint(0); // Close the rectangle

    // Create rectangle polygon (for semi-transparent fill)
    auto polys = vtkSmartPointer<vtkCellArray>::New();
    polys->InsertNextCell(4);
    polys->InsertCellPoint(0);
    polys->InsertCellPoint(1);
    polys->InsertCellPoint(2);
    polys->InsertCellPoint(3);

    roiPolyData_->SetPoints(points);
    roiPolyData_->SetLines(lines);
    roiPolyData_->SetPolys(polys);
    roiPolyData_->Modified();

    roiActor_->SetVisibility(true);
}

QColor MainWindow::currentRoiQColor() const {
    if (roiColorCustom_) {
        return QColor(static_cast<int>(roiOverlayRGB_[0] * 255.0 + 0.5),
                      static_cast<int>(roiOverlayRGB_[1] * 255.0 + 0.5),
                      static_cast<int>(roiOverlayRGB_[2] * 255.0 + 0.5));
    }
    QColor named(QString::fromStdString(roiOverlayColor_));
    if (!named.isValid()) return QColor("yellow");
    return named;
}

QPoint MainWindow::mapLabelPosToImage(const QPoint& pos) const {
    if (!videoLabel_) return QPoint(-1, -1);
    const QPixmap pm = videoLabel_->pixmap();
    if (pm.isNull()) return QPoint(-1, -1);
    const QSize pmSize = pm.size();
    if (pmSize.width() <= 0 || pmSize.height() <= 0 || lastVideoW_ <= 0 || lastVideoH_ <= 0) return QPoint(-1, -1);
    const int mx = (videoLabel_->width() - pmSize.width()) / 2;
    const int my = (videoLabel_->height() - pmSize.height()) / 2;
    const int x = std::clamp(pos.x() - mx, 0, pmSize.width());
    const int y = std::clamp(pos.y() - my, 0, pmSize.height());
    const double nx = static_cast<double>(x) / pmSize.width();
    const double ny = static_cast<double>(y) / pmSize.height();
    const int ix = std::clamp(static_cast<int>(nx * lastVideoW_), 0, std::max(0, lastVideoW_ - 1));
    const int iy = std::clamp(static_cast<int>(ny * lastVideoH_), 0, std::max(0, lastVideoH_ - 1));
    return QPoint(ix, iy);
}

MainWindow::DragMode MainWindow::hitTestRoi(const QPoint& imgPt) const {
    const cv::Rect r = pivParams_.roi;
    if (r.width <= 0 || r.height <= 0) return DragMode::None;
    const int hs = std::max(3, roiHandleSize_);
    const int x1 = r.x; const int y1 = r.y;
    const int x2 = r.x + r.width; const int y2 = r.y + r.height;
    const auto inBox = [&](int cx, int cy){ return std::abs(imgPt.x() - cx) <= hs && std::abs(imgPt.y() - cy) <= hs; };
    if (inBox(x1, y1)) return DragMode::TopLeft;
    if (inBox(x2, y1)) return DragMode::TopRight;
    if (inBox(x1, y2)) return DragMode::BottomLeft;
    if (inBox(x2, y2)) return DragMode::BottomRight;
    // Edges
    if (std::abs(imgPt.x() - x1) <= hs && imgPt.y() >= y1 && imgPt.y() <= y2) return DragMode::Left;
    if (std::abs(imgPt.x() - x2) <= hs && imgPt.y() >= y1 && imgPt.y() <= y2) return DragMode::Right;
    if (std::abs(imgPt.y() - y1) <= hs && imgPt.x() >= x1 && imgPt.x() <= x2) return DragMode::Top;
    if (std::abs(imgPt.y() - y2) <= hs && imgPt.x() >= x1 && imgPt.x() <= x2) return DragMode::Bottom;
    // Inside
    if (imgPt.x() >= x1 && imgPt.x() <= x2 && imgPt.y() >= y1 && imgPt.y() <= y2) return DragMode::Move;
    return DragMode::None;
}

void MainWindow::clampRoiToBounds() {
    auto& r = pivParams_.roi;
    r.x = std::clamp(r.x, 0, std::max(0, lastVideoW_ - 1));
    r.y = std::clamp(r.y, 0, std::max(0, lastVideoH_ - 1));
    r.width = std::clamp(r.width, 1, std::max(1, lastVideoW_ - r.x));
    r.height = std::clamp(r.height, 1, std::max(1, lastVideoH_ - r.y));
}

void MainWindow::keyPressEvent(QKeyEvent* event) {
    if (editRoiMode_) {
        int step = (event->modifiers() & Qt::ShiftModifier) ? 10 : 1;
        auto& r = pivParams_.roi;
        bool changed = false;
        switch (event->key()) {
            case Qt::Key_Left:
                if (event->modifiers() & Qt::AltModifier) { r.width = std::max(1, r.width - step); }
                else { r.x -= step; }
                changed = true; break;
            case Qt::Key_Right:
                if (event->modifiers() & Qt::AltModifier) { r.width += step; }
                else { r.x += step; }
                changed = true; break;
            case Qt::Key_Up:
                if (event->modifiers() & Qt::AltModifier) { r.height = std::max(1, r.height - step); }
                else { r.y -= step; }
                changed = true; break;
            case Qt::Key_Down:
                if (event->modifiers() & Qt::AltModifier) { r.height += step; }
                else { r.y += step; }
                changed = true; break;
            default: break;
        }
        if (changed) {
            clampRoiToBounds();
            updateVTKRoiRectangle();
            if (linkViews_) vtkWidget_->renderWindow()->Render();
            onFrameTick();
            saveSettings();
            statusBar()->showMessage("ROI adjusted", 1500);
            return;
        }
    }
    QMainWindow::keyPressEvent(event);
}

void MainWindow::setEditRoiMode(bool on) {
    editRoiMode_ = on;
    if (editRoiMode_) {
        statusBar()->showMessage("Edit ROI: Click-drag in video to set ROI", 3000);
        showRoiOverlay_ = true; // ensure overlay visible during editing
    } else {
        statusBar()->showMessage("Edit ROI: off", 2000);
    }
}

bool MainWindow::eventFilter(QObject* obj, QEvent* event) {
    if (obj == videoLabel_) {
        if (editRoiMode_) {
            if (event->type() == QEvent::MouseButtonPress) {
                auto* me = static_cast<QMouseEvent*>(event);
                if (me->button() == Qt::LeftButton) {
                    QPoint p = mapLabelPosToImage(me->pos());
                    if (p.x() >= 0) {
                        roiDragStart_ = p;
                        roiDragOrig_ = pivParams_.roi;
                        roiDragMode_ = hitTestRoi(p);
                        if (roiDragMode_ == DragMode::None) {
                            // Start a new ROI from this point
                            roiDragMode_ = DragMode::TopLeft;
                            pivParams_.roi = cv::Rect(p.x(), p.y(), 1, 1);
                            roiDragOrig_ = pivParams_.roi;
                        }
                        editingRoiActive_ = true;
                        return true;
                    }
                }
            } else if (event->type() == QEvent::MouseMove) {
                auto* me = static_cast<QMouseEvent*>(event);
                if (editingRoiActive_) {
                    QPoint p = mapLabelPosToImage(me->pos());
                    if (p.x() >= 0) {
                        const int dx = p.x() - roiDragStart_.x();
                        const int dy = p.y() - roiDragStart_.y();
                        cv::Rect r = roiDragOrig_;
                        switch (roiDragMode_) {
                            case DragMode::Move:
                                r.x += dx; r.y += dy; break;
                            case DragMode::Left: {
                                int newX = std::min(p.x(), r.x + r.width - 1);
                                int newW = r.x + r.width - newX;
                                r.x = newX; r.width = std::max(1, newW); break;
                            }
                            case DragMode::Right: {
                                int newW = std::max(1, p.x() - r.x);
                                r.width = newW; break;
                            }
                            case DragMode::Top: {
                                int newY = std::min(p.y(), r.y + r.height - 1);
                                int newH = r.y + r.height - newY;
                                r.y = newY; r.height = std::max(1, newH); break;
                            }
                            case DragMode::Bottom: {
                                int newH = std::max(1, p.y() - r.y);
                                r.height = newH; break;
                            }
                            case DragMode::TopLeft: {
                                int newX = std::min(p.x(), r.x + r.width - 1);
                                int newY = std::min(p.y(), r.y + r.height - 1);
                                int newW = r.x + r.width - newX;
                                int newH = r.y + r.height - newY;
                                r.x = newX; r.y = newY; r.width = std::max(1, newW); r.height = std::max(1, newH);
                                break;
                            }
                            case DragMode::TopRight: {
                                int newY = std::min(p.y(), r.y + r.height - 1);
                                int newH = r.y + r.height - newY;
                                int newW = std::max(1, p.x() - r.x);
                                r.y = newY; r.height = std::max(1, newH); r.width = newW; break;
                            }
                            case DragMode::BottomLeft: {
                                int newX = std::min(p.x(), r.x + r.width - 1);
                                int newW = r.x + r.width - newX;
                                int newH = std::max(1, p.y() - r.y);
                                r.x = newX; r.width = std::max(1, newW); r.height = newH; break;
                            }
                            case DragMode::BottomRight: {
                                int newW = std::max(1, p.x() - r.x);
                                int newH = std::max(1, p.y() - r.y);
                                r.width = newW; r.height = newH; break;
                            }
                            default: {
                                // Fallback to selection rectangle
                                const int x1 = std::min(roiDragStart_.x(), p.x());
                                const int y1 = std::min(roiDragStart_.y(), p.y());
                                const int x2 = std::max(roiDragStart_.x(), p.x());
                                const int y2 = std::max(roiDragStart_.y(), p.y());
                                r = cv::Rect(x1, y1, std::max(1, x2 - x1), std::max(1, y2 - y1));
                                break;
                            }
                        }
                        pivParams_.roi = r;
                        clampRoiToBounds();
                        updateVTKRoiRectangle();
                        if (linkViews_) vtkWidget_->renderWindow()->Render();
                        onFrameTick();
                        return true;
                    }
                }
            } else if (event->type() == QEvent::MouseButtonRelease) {
                auto* me = static_cast<QMouseEvent*>(event);
                if (editingRoiActive_ && me->button() == Qt::LeftButton) {
                    editingRoiActive_ = false;
                    saveSettings();
                    updateVTKRoiRectangle();
                    if (linkViews_) vtkWidget_->renderWindow()->Render();
                    onFrameTick();
                    statusBar()->showMessage("ROI updated", 2000);
                    return true;
                }
            }
        }
    }
    return QMainWindow::eventFilter(obj, event);
}