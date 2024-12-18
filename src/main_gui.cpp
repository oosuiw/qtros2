#include "qtros2/main_gui.hpp"

#include <QPushButton>
#include <QLabel>
#include <QBoxLayout>
#include <QTimer>
#include <QScreen>
#include <QGuiApplication>
#include <std_msgs/msg/float64.hpp>

MainGUI::MainGUI(const std::shared_ptr<Ros2Node>& ros2_node, QWidget* parent)
    : QMainWindow(parent), ros2_node(ros2_node), is_on(false), lateral_offset(0.0)
{
    main_widget = new QWidget(this);
    main_widget->setStyleSheet("background-color: #1F3347;");
    setCentralWidget(main_widget);

    QVBoxLayout* main_layout = new QVBoxLayout(main_widget);
    main_layout->setSpacing(20);
    main_layout->setMargin(20);

    status_label = createLabel("Lateral Offset: 0.0", 36, Qt::AlignCenter); // 36 -> font size
    status_label->setFixedWidth(320);  // 라벨의 너비 고정 //음수 값의 (-) 기호가 추가되면서 텍스트 길이가 늘어나고 라벨의 크기를 자동으로 확장
    main_layout->addWidget(status_label);

    toggle_button = createButton("STOP", "#CC0000", 260, 500, 72);
    main_layout->addWidget(toggle_button);

    QHBoxLayout* arrow_layout = new QHBoxLayout;
    QPushButton* left_arrow_button = createButton("\u2190", "#4A90E2", 130, 320, 60);
    QPushButton* right_arrow_button = createButton("\u2192", "#50E3C2", 130, 320, 60);

    arrow_layout->addWidget(left_arrow_button);
    arrow_layout->addWidget(right_arrow_button);
    main_layout->addLayout(arrow_layout);

    connect(toggle_button, &QPushButton::clicked, this, &MainGUI::toggle_operation_mode);
    connect(left_arrow_button, &QPushButton::clicked, this, [this]() {
        decrease_lateral_offset();
        status_label->setText(QString("Lateral Offset: %1").arg(lateral_offset, 0, 'f', 1));
    });
    connect(right_arrow_button, &QPushButton::clicked, this, [this]() {
        increase_lateral_offset();
        status_label->setText(QString("Lateral Offset: %1").arg(lateral_offset, 0, 'f', 1));
    });

    adjustFontSize();
}

QLabel* MainGUI::createLabel(const std::string& text, int font_size, Qt::Alignment alignment)
{
    QLabel* label = new QLabel(QString::fromStdString(text), this);
    label->setStyleSheet(QString("color: white; font-weight: bold; font-size: %1px;").arg(font_size));
    label->setAlignment(alignment);
    return label;
}

QPushButton* MainGUI::createButton(const std::string& text, const std::string& color, int width, int height, int font_size)
{
    QPushButton* button = new QPushButton(QString::fromStdString(text), this);
    button->setStyleSheet(QString(
        "QPushButton { background-color: %1; color: white; font-size: %2px; font-weight: bold; border-radius: 10px; }"
    ).arg(QString::fromStdString(color)).arg(font_size));
    
    button->setMinimumSize(width, height); // 버튼 크기 설정
    // button->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed); // 버튼 크기 고정
    
    return button;
}



void MainGUI::toggle_operation_mode()
{
    if (is_on) {
        ros2_node->operation_mode_req_off();
        toggle_button->setText("STOP");
        toggle_button->setStyleSheet("background-color: #CC0000; color: white;");
        is_on = false;
    } else {
        ros2_node->operation_mode_req_on();
        toggle_button->setText("START");
        toggle_button->setStyleSheet("background-color: #00CC66; color: white;");
        is_on = true;
    }
}

void MainGUI::decrease_lateral_offset()
{
    lateral_offset = std::max(-0.5, lateral_offset - 0.1);
    ros2_node->publish_lateral_offset(lateral_offset);
}

void MainGUI::increase_lateral_offset()
{
    lateral_offset = std::min(0.5, lateral_offset + 0.1);
    ros2_node->publish_lateral_offset(lateral_offset);
}

void MainGUI::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
    adjustFontSize();
}

void MainGUI::adjustFontSize()
{
    int fontSize = qMin(toggle_button->width(), toggle_button->height()) / 5;
    QFont font = toggle_button->font();
    font.setPointSize(fontSize);
    toggle_button->setFont(font);
}

QSize MainGUI::sizeHint() const
{
    return QSize(300, 400);
}

void MainGUI::showEvent(QShowEvent* event)
{
    QMainWindow::showEvent(event);
    QRect screenGeometry = QGuiApplication::primaryScreen()->geometry();
    setGeometry(screenGeometry.width() - 300, 0, 300, screenGeometry.height());
}

MainGUI::~MainGUI() {}
