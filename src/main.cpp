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
    auto gui_app = std::make_shared<MainGUI>(ros2_node);

    gui_app->show();  

    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(ros2_node);

    while (rclcpp::ok())
    {
        exec.spin_some();  
        app.processEvents(); 
    }

    exec.remove_node(ros2_node);
    rclcpp::shutdown();

    return 0;  
}
