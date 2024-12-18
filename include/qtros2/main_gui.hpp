#ifndef MAIN_GUI_HPP
#define MAIN_GUI_HPP

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QFrame>
#include <QTimer>
#include <memory>
#include "qtros2/ros2node.hpp"
#include <QShowEvent>

class MainGUI : public QMainWindow
{
  Q_OBJECT

public:
  explicit MainGUI(const std::shared_ptr<Ros2Node>& ros2_node, QWidget* parent = nullptr);
  ~MainGUI() override;

  void showEvent(QShowEvent* event) override;

protected:
  void resizeEvent(QResizeEvent* event) override;
  QSize sizeHint() const override;

private:
  // KMS_241217
  struct Frame_info
  {
      int x;
      int y;
      int width;
      int height;
  };

  struct Label_info
  {
      int x;
      int y;
      int width;
      int height;
      std::string text;
  };

  int program_x_;
  int program_y_;
  bool is_on;
  double lateral_offset;

  std::shared_ptr<Ros2Node> ros2_node;

  QLabel* status_label;
  QWidget* main_widget;
  QPushButton* toggle_button;

  std::vector<QFrame *> QFrame_vector;
  std::vector<QLabel *> QLabel_vector;
  std::vector<QPushButton *> QPushButton_vector;

  void createFrame(const Frame_info& info);
  QLabel* createLabel(const std::string& text, int font_size, Qt::Alignment alignment);
  QPushButton* createButton(const std::string& text, const std::string& color, int width, int height, int font_size);
  
  void adjustFontSize();
  void toggle_operation_mode();
  void decrease_lateral_offset();
  void increase_lateral_offset();
};

#endif // MAIN_GUI_HPP
