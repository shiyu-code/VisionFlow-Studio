#pragma once

#include <QMainWindow>
#include <QLabel>
#include <QTimer>
#include <QSplitter>
#include <QString>
#include <memory>
#include <vector>
#include <vtkSmartPointer.h>
#include <string>

// Forward declarations
class QVTKOpenGLNativeWidget;
class vtkGenericOpenGLRenderWindow;
class vtkRenderer;
class vtkImageData;
class vtkImageActor;
class vtkPolyData;
class vtkGlyph2D;
class vtkPolyDataMapper;
class vtkActor;
class vtkPolyDataMapper2D;
class vtkActor2D;

namespace cv { class Mat; class VideoCapture; }

#include "../modules/piv/PivEngine.h"
#include "../modules/rdic/RdicEngine.h"

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow() override;

protected:
    bool eventFilter(QObject* obj, QEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

private slots:
    void openVideo();
    void play();
    void stop();
    void onFrameTick();
    void runPIVAnalysis();
    void togglePivOverlay();
    void toggleLinkViews();
    void runRDICAnalysis();
    void toggleRdicOverlay();
    void openRdicSettings();
    void openPivSettings();
    void exportPivCSV();
    void exportRdicCSV();

private:
    void setupMenus();
    void setupVTKScene();
    QImage matToQImage(const cv::Mat& mat);
    void updateVTKVideoImage(const cv::Mat& frame);
    void updateVTKPivGlyphs();
    void updateVTKRdicHeatmap(const cv::Mat& heatmap);
    void updateVTKRoiRectangle();
    cv::Mat computeRdicHeatmapCurrent() const;
    void loadSettings();
    void saveSettings() const;
    void applyRoiStyle();
    void toggleRoiOverlay();

    // ROI editing helpers
    QPoint mapLabelPosToImage(const QPoint& pos) const;
    QColor currentRoiQColor() const;
    void setEditRoiMode(bool on);
    enum class DragMode { None, Move, Left, Right, Top, Bottom, TopLeft, TopRight, BottomLeft, BottomRight };
    DragMode hitTestRoi(const QPoint& imgPt) const;
    void clampRoiToBounds();

private:
    QSplitter* splitter_ {nullptr};
    QLabel* videoLabel_ {nullptr};
    QTimer* timer_ {nullptr};

    std::unique_ptr<cv::VideoCapture> cap_;
    int fps_ {30};

    QVTKOpenGLNativeWidget* vtkWidget_ {nullptr};
    vtkGenericOpenGLRenderWindow* vtkWindow_ {nullptr};
    vtkSmartPointer<vtkRenderer> vtkRenderer_;

    bool showPivVectors_ {false};
    std::vector<PivVector> pivVectors_;
    PivParams pivParams_;

    bool linkViews_ {true};
    vtkSmartPointer<vtkImageData> vtkImage_;
    vtkSmartPointer<vtkImageActor> vtkImageActor_;
    vtkSmartPointer<vtkPolyData> pivPolyData_;
    vtkSmartPointer<vtkGlyph2D> pivGlyph_;
    vtkSmartPointer<vtkPolyDataMapper> pivGlyphMapper_;
    vtkSmartPointer<vtkActor> pivGlyphActor_;
    
    // PIV ROI rectangle overlay (use 3D actor to align with image actor coords)
    vtkSmartPointer<vtkPolyData> roiPolyData_;
    vtkSmartPointer<vtkPolyDataMapper> roiMapper_;
    vtkSmartPointer<vtkActor> roiActor_;

    int vtkImageW_ {0};
    int vtkImageH_ {0};
    bool vtkCameraInit_ {false};
    // RDIC overlay state and VTK actors
    bool showRdic_ {false};
    RdicParams rdicParams_;
    RdicResult rdicResult_;
    vtkSmartPointer<vtkImageData> vtkImageRdic_;
    vtkSmartPointer<vtkImageActor> vtkImageActorRdic_;
    enum class RdicHeatType { VonMises, Exx, Eyy, Exy };
    enum class RdicColorMap { Jet, Turbo, Viridis };
    RdicHeatType rdicHeatType_ {RdicHeatType::VonMises};
    RdicColorMap rdicColorMap_ {RdicColorMap::Jet};
    double rdicOverlayAlpha_ {0.6};
    double rdicGuiAlpha_ {0.3};
    bool rdicRangeAuto_ {true};
    double rdicRangeMin_ {0.0};
    double rdicRangeMax_ {1.0};
    bool rdicInvert_ {false};
    bool showRoiOverlay_ = true;
    double roiOverlayAlpha_ = 0.25;
    std::string roiOverlayColor_ = "Yellow"; // named color fallback
    bool roiColorCustom_ = false;             // if true, use RGB instead of named color
    double roiOverlayRGB_[3] = {1.0, 1.0, 0.0}; // normalized [0,1], default yellow

    // Interactive ROI editing state
    bool editRoiMode_ {false};
    bool editingRoiActive_ {false};
    QPoint roiDragStart_;
    cv::Rect roiDragOrig_;
    int lastVideoW_ {0};
    int lastVideoH_ {0};
    QAction* editRoiAct_ {nullptr};
    int roiHandleSize_ {6};
    DragMode roiDragMode_ {DragMode::None};
};