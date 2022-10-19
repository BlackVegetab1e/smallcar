#include<serial/serial.h>

class arm_ctrl
{
    public:
        arm_ctrl(int bode, std::string port);
        void set_arm_angle(const int &level);
        void set_lock(const int &level);
        void shutdown_arm(void);


    private:
        int bode;
        std::string port;
        
        void init_port(const int &bode, const std::string &port);

        serial::Serial serialPort;
        serial::Timeout Timeout = serial::Timeout::simpleTimeout(100);

};