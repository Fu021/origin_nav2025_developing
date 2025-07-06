#include "tf_shift_panel.h"
#include <pluginlib/class_list_macros.hpp>

namespace tf_shift_panel
{

    // DirectionalControlWidget implementation
    DirectionalControlWidget::DirectionalControlWidget(QWidget *parent)
        : QWidget(parent)
    {

        setFixedSize(200, 200);
        setMouseTracking(true);
        setFocusPolicy(Qt::StrongFocus);
        XYR_memory_list.setKey(QString::fromStdString("direction"));
        if (!XYR_memory_list.attach())
        {
            std::cerr << "Failed to create shared memory: " << XYR_memory_list.errorString().toStdString() << std::endl;
        }
    }

    DirectionalControlWidget::~DirectionalControlWidget()
    {
        if (XYR_memory_list.isAttached())
        {
            XYR_memory_list.detach();
        }
    }

    bool DirectionalControlWidget::eventFilter(QObject *obj, QEvent *event)
    {
        if (obj != this)
            return QWidget::eventFilter(obj, event);

        if (event->type() == QEvent::KeyPress)
        {
            QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
            keyPressEvent(keyEvent);
            return true;
        }
        else if (event->type() == QEvent::KeyRelease)
        {
            QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
            keyReleaseEvent(keyEvent);
            return true;
        }
        return QWidget::eventFilter(obj, event); 
    }

    void DirectionalControlWidget::paintEvent(QPaintEvent *event)
    {
        Q_UNUSED(event);
        QPainter painter(this);
        painter.setRenderHints(QPainter::Antialiasing | QPainter::SmoothPixmapTransform);

        QPoint centerPoint = rect().center();
        double radius = (std::min(width(), height()) - 20) / 2.0;
        drawRect_ = QRectF(centerPoint.x() - radius, centerPoint.y() - radius,
                           radius * 2, radius * 2);
        double arcHeight = radius / 2.0;

        // Draw base circle
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor("#EAEAEA"));
        painter.drawEllipse(drawRect_);

        // Draw directional sectors
        transShaped[0] = gradientArc(45.0, 90.0, arcHeight);  // Up
        transShaped[1] = gradientArc(225.0, 90.0, arcHeight); // Down
        transShaped[2] = gradientArc(135.0, 90.0, arcHeight); // Left
        transShaped[3] = gradientArc(315.0, 90.0, arcHeight); // Right

        painter.setBrush(QColor("#D0D0D0"));
        for (auto &path : transShaped)
        {
            painter.drawPath(path);
        }

        // Prepare rotation buttons
        double centerRadius = radius / 2.0;
        centerRect_ = QRectF(centerPoint.x() - centerRadius,
                             centerPoint.y() - centerRadius,
                             centerRadius * 2, centerRadius * 2);

        // Create semi-circle paths
        ccwPath_ = createSemiCircle(false); // Right half (CCW)
        cwPath_ = createSemiCircle(true);   // Left half (CW)

        // Draw rotation buttons
        painter.setBrush((pressedBtn_ == PressBtnType::CCW) ? QColor(0, 0, 0, 63) : QColor("#D0D0D0"));
        painter.drawPath(ccwPath_);

        painter.setBrush((pressedBtn_ == PressBtnType::CW) ? QColor(0, 0, 0, 63) : QColor("#D0D0D0"));
        painter.drawPath(cwPath_);

        // Draw symbols
        painter.setPen(QPen(Qt::black, 2));
        QFont font = painter.font();
        font.setPixelSize(16);
        painter.setFont(font);

        // Direction arrows
        painter.drawText(QRectF(drawRect_.x(), centerRect_.y(), radius / 2, radius),
                         Qt::AlignCenter, "〈");
        painter.drawText(QRectF(centerRect_.x(), drawRect_.y(), radius, radius / 2),
                         Qt::AlignCenter, "︿");
        painter.drawText(QRectF(centerRect_.topRight().x(), centerRect_.topRight().y(),
                                radius / 2, radius),
                         Qt::AlignCenter, "〉");
        painter.drawText(QRectF(centerRect_.bottomLeft().x(),
                                centerRect_.bottomLeft().y(), radius, radius / 2),
                         Qt::AlignCenter, "﹀");

        // Rotation arrows
        font.setPixelSize(24);
        painter.setFont(font);
        painter.drawText(cwPath_.boundingRect(), Qt::AlignCenter, "↻");
        painter.drawText(ccwPath_.boundingRect(), Qt::AlignCenter, "↺");

        // Highlight pressed area
        if (pressedBtn_ != PressBtnType::None &&
            (pressedBtn_ != PressBtnType::CCW && pressedBtn_ != PressBtnType::CW))
        {
            painter.setBrush(QColor(0, 0, 0, 63));
            int index = static_cast<int>(pressedBtn_) - 3;
            if (index >= 0 && index < 4)
            {
                painter.drawPath(transShaped[index]);
            }
        }
    }

    void DirectionalControlWidget::mousePressEvent(QMouseEvent *event)
    {
        if (event->button() == Qt::LeftButton)
        {
            QPoint point = event->pos();

            // Check rotation buttons first
            if (isPointInPath(point, ccwPath_))
            {
                pressedBtn_ = PressBtnType::CCW;
                createTwist(2, 0.017);
                emit directionChanged();
            }
            else if (isPointInPath(point, cwPath_))
            {
                pressedBtn_ = PressBtnType::CW;
                createTwist(2, -0.017);
                emit directionChanged();
            }
            else if (isPointInCircle(point, drawRect_.toRect()))
            {
                QPoint center = drawRect_.center().toPoint();
                double angle = atan2(point.y() - center.y(), point.x() - center.x());
                angle = -angle * (180.0 / M_PI);
                if (angle < 0)
                    angle += 360.0;

                if (angle < 45 || angle >= 315)
                {
                    pressedBtn_ = PressBtnType::Right;
                    createTwist(1, -0.05);
                    emit directionChanged();
                }
                else if (angle >= 45 && angle < 135)
                {
                    pressedBtn_ = PressBtnType::Up;
                    createTwist(0, 0.05);
                    emit directionChanged();
                }
                else if (angle >= 135 && angle < 225)
                {
                    pressedBtn_ = PressBtnType::Left;
                    createTwist(1, 0.05);
                    emit directionChanged();
                }
                else if (angle >= 225 && angle < 315)
                {
                    pressedBtn_ = PressBtnType::Down;
                    createTwist(0, -0.05);
                    emit directionChanged();
                }
            }
            update();
        }
        QWidget::mousePressEvent(event);
    }

    void DirectionalControlWidget::keyPressEvent(QKeyEvent *event)
    {
        switch (event->key())
        {
        case Qt::Key_W: // Up
            pressedBtn_ = PressBtnType::Up;
            createTwist(0, 0.05);
            emit directionChanged();
            break;
        case Qt::Key_S: // Down
            pressedBtn_ = PressBtnType::Down;
            createTwist(0, -0.05);
            emit directionChanged();
            break;
        case Qt::Key_A: // Left
            pressedBtn_ = PressBtnType::Left;
            createTwist(1, 0.05);
            emit directionChanged();
            break;
        case Qt::Key_D: // Right
            pressedBtn_ = PressBtnType::Right;
            createTwist(1, -0.05);
            emit directionChanged();
            break;
        case Qt::Key_Q: // Rotate CCW
            pressedBtn_ = PressBtnType::CCW;
            createTwist(2, 0.017);
            emit directionChanged();
            break;
        case Qt::Key_E: // Rotate CW
            pressedBtn_ = PressBtnType::CW;
            createTwist(2, -0.017);
            emit directionChanged();
            break;
        default:
            QWidget::keyPressEvent(event);
            return;
        }

        update(); // 刷新按钮高亮
    }

    void DirectionalControlWidget::mouseReleaseEvent(QMouseEvent *event)
    {
        if (pressedBtn_ != PressBtnType::None)
        {
            pressedBtn_ = PressBtnType::None;
            update();
        }
        QWidget::mouseReleaseEvent(event);
    }

    void DirectionalControlWidget::keyReleaseEvent(QKeyEvent *event)
    {
        Q_UNUSED(event);
        pressedBtn_ = PressBtnType::None;
        update();
    }
    bool DirectionalControlWidget::isPointInCircle(const QPoint &point, const QRect &rect)
    {
        QPoint center = rect.center();
        int dx = point.x() - center.x();
        int dy = point.y() - center.y();
        return (dx * dx + dy * dy) <= (rect.width() / 2 * rect.width() / 2);
    }

    bool DirectionalControlWidget::isPointInPath(const QPoint &point, const QPainterPath &path)
    {
        return path.contains(point);
    }

    QPainterPath DirectionalControlWidget::gradientArc(double startAngle, double angleLength, double arcHeight)
    {
        QPainterPath path;
        path.arcMoveTo(drawRect_, startAngle);
        path.arcTo(drawRect_, startAngle, angleLength);

        QPainterPath innerPath;
        innerPath.addEllipse(drawRect_.adjusted(arcHeight, arcHeight, -arcHeight, -arcHeight));

        return path - innerPath;
    }

    QPainterPath DirectionalControlWidget::createSemiCircle(bool left)
    {
        QPainterPath path;
        QRectF baseRect = centerRect_.adjusted(2, 2, -2, -2);

        if (left)
        {
            path.arcMoveTo(baseRect, 90);
            path.arcTo(baseRect, 90, 180);
        }
        else
        {
            path.arcMoveTo(baseRect, 270);
            path.arcTo(baseRect, 270, 180);
        }
        path.closeSubpath();
        return path;
    }

    void DirectionalControlWidget::createTwist(int switchXYR, double delta)
    {
        if (XYR_memory_list.isAttached())
        {

            XYR_memory_list.lock();

            double *data = static_cast<double *>(XYR_memory_list.data());
            if (data)
            {
                // 读取数组元素
                data[switchXYR] += delta;
            }
            XYR_memory_list.unlock();
        }
        else
        {
            std::cerr << "Failed to attach shared memory: " << XYR_memory_list.errorString().toStdString() << std::endl;
        }
    }

    // Rviz2Panel implementation
    Rviz2Panel::Rviz2Panel(QWidget *parent)
        : rviz_common::Panel(parent)
    {
        QVBoxLayout *layout = new QVBoxLayout(this);
        controlWidget_ = new DirectionalControlWidget();
        layout->addWidget(controlWidget_);
        controlWidget_->installEventFilter(controlWidget_);
        controlWidget_->setFocus(); // 主动获取焦点
    }

    Rviz2Panel::~Rviz2Panel()
    {
    }

    void Rviz2Panel::onInitialize()
    {
        // Initialization handled in constructor
    }

} // namespace tf_shift_panel

PLUGINLIB_EXPORT_CLASS(tf_shift_panel::Rviz2Panel, rviz_common::Panel)
