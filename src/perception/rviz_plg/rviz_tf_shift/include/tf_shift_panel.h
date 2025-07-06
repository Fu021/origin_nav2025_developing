#ifndef TF_SHIFT_PANEL_HPP
#define TF_SHIFT_PANEL_HPP
#include <QSharedMemory>
#include <QWidget>
#include <QPainter>
#include <QPainterPath>
#include <QMouseEvent>
#include <QVBoxLayout>
#include <rviz_common/panel.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <QApplication>
namespace tf_shift_panel
{

    class DirectionalControlWidget : public QWidget
    {
        Q_OBJECT
    public:
        explicit DirectionalControlWidget(QWidget *parent = nullptr);
        virtual ~DirectionalControlWidget();
    signals:
        void directionChanged();

    protected:
        void paintEvent(QPaintEvent *event) override;
        void mousePressEvent(QMouseEvent *event) override;
        void mouseReleaseEvent(QMouseEvent *event) override;
        void keyPressEvent(QKeyEvent *event) override;
        void keyReleaseEvent(QKeyEvent *event) override;
        bool eventFilter(QObject *obj, QEvent *event) override;

    private:
        enum class PressBtnType
        {
            None,
            CCW, // Counter Clockwise
            CW,  // Clockwise
            Up,
            Down,
            Left,
            Right
        };

        PressBtnType pressedBtn_ = PressBtnType::None;

        QRectF drawRect_;
        QRectF centerRect_;
        QPainterPath transShaped[4];
        QPainterPath ccwPath_;
        QPainterPath cwPath_;

        double directionArray[3] = {0.88, -0.816, 0.0}; // x y rot
        int QSharedMemoryLen = 3;
        QSharedMemory XYR_memory_list;
        void createTwist(int switchXYR, double delta);

        bool isPointInCircle(const QPoint &point, const QRect &rect);
        bool isPointInPath(const QPoint &point, const QPainterPath &path);
        QPainterPath gradientArc(double startAngle, double angleLength, double arcHeight);
        QPainterPath createSemiCircle(bool left);
    };

    class Rviz2Panel : public rviz_common::Panel
    {
        Q_OBJECT
    public:
        explicit Rviz2Panel(QWidget *parent = nullptr);
        virtual ~Rviz2Panel();
        void onInitialize() override;

    private:
        DirectionalControlWidget *controlWidget_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdVelPub_;
    };

} // namespace tf_shift_panel

#endif // TF_SHIFT_PANEL_HPP
