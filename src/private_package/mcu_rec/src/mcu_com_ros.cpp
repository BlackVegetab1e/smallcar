/*************所有的头文件，全局变量，宏定义*************/
#include <mcu_rec/mcu_com.h>
/*******************都在这个头文件中********************/



mcu_com *mcuArray;

void mySignitHandler(int sig)
{
    ROS_ERROR("Mscu: Ctrl+c detected");
    mcuArray->shutdownMCU();
    ros::shutdown();

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "mcudata_rec");
    ros::NodeHandle n;

    signal(SIGINT, mySignitHandler);

    std::string serialPort;
    int baudRate;
    ros::param::get("Port",serialPort);
    ros::param::get("Baudrate",baudRate);
    mcu_com mcuOnBoard(serialPort, baudRate);
    mcuArray = &mcuOnBoard;
    mcuOnBoard.startMCU();
}

