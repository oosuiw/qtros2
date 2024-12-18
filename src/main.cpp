#include <QApplication>
#include "qtros2/ros2node.hpp"
#include "qtros2/main_gui.hpp"
#include <csignal>

static void siginthandler(int /*param*/)
{
    QApplication::quit();
}

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);
    signal(SIGINT, siginthandler);

    rclcpp::init(argc, argv);

    auto ros2_node = std::make_shared<Ros2Node>();
    auto main_gui = std::make_shared<MainGUI>(ros2_node);

    main_gui->show();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(ros2_node);

    while (rclcpp::ok()) 
    {
        executor.spin_some();
        app.processEvents();
    }

    executor.remove_node(ros2_node);
    rclcpp::shutdown();

    return 0;
}
