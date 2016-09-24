#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "ggb.h"
#include "geometry_msgs/Point32.h"

using namespace ros;
using namespace std;
serial::Serial* serial_port;

struct Command {
    double leftSpeed;
    double rightSpeed;
    double time;
};

// Ew, globals :)
vector<Command> commandList;
int commandNumber = 0;
ros::Timer timer;
NodeHandle* globalHandle = NULL;
bool doSerial = true;

ros::Publisher simulatorPub;

Command stop = {0.0, 0.0, 100.0};

void killRobot()
{
    serial_port->write("0\n");
    cout << "0" << endl;
}

void sendCommand(const Command& c_orig)
{
    Command c = c_orig;

    if (c.leftSpeed > 11) { c.leftSpeed = 11; cout << "Too fast!" << endl; }
    if (c.rightSpeed > 11) { c.rightSpeed = 11; cout << "Too fast!" << endl; }
    if (c.leftSpeed < -10) { c.leftSpeed = -10; cout << "Too fast!" << endl; }
    if (c.rightSpeed < -10) { c.rightSpeed = -10; cout << "Too fast!" << endl; }
    
    if (doSerial) {
        ostringstream strm;
        strm << (c.leftSpeed + 64) << "," << (c.rightSpeed + 64) << "\n";
        serial_port->write(strm.str().c_str());
    }

    // TODO: Methinks the motor speeds can currently be floats.
    // (Ar|Net)duino probably wants integers. We should eventually do
    // something about this ...

    geometry_msgs::Point32 p;
    p.x = c.leftSpeed;
    p.y = c.rightSpeed;
    simulatorPub.publish(p);

    cout << (c.leftSpeed + 64) << "," << (c.rightSpeed + 64) << "\n" << endl;
    //cout << "L " << c.leftSpeed << " R " << c.rightSpeed << " @ " << ros::Time::now() << endl;
    cout << "(" << c.time << ")" << endl;
}

void timerCallback(const ros::TimerEvent&)
{
    commandNumber++;
    if (commandNumber < commandList.size())
    {
        sendCommand(commandList[commandNumber]);

        const bool oneShot = true;
        timer = globalHandle->createTimer(ros::Duration(commandList[commandNumber].time), timerCallback, oneShot);
    }
    else
    {
        sendCommand(stop);
    }
}


void tx_netduino(const std_msgs::String::ConstPtr& msg)
{
    cout << "RECEIVED COMMAND LIST" << endl;
    
    commandList.clear();
    commandNumber = 0;

    string s = msg->data;

    while (s.find(";") != s.npos)
    {
        string cmd = s.substr(0, s.find(";"));

        istringstream strm(cmd);

        string leftSymbol, rightSymbol, timeSymbol;
        double leftSpeed, rightSpeed, time;

        strm >> leftSymbol >> leftSpeed >>
            rightSymbol >> rightSpeed >>
            timeSymbol >> time;

        if (strm.fail() || !strm.eof() || leftSymbol != "L" || rightSymbol != "R" || timeSymbol != "T")
        {
            cout << "WARNING: ignored malformed command: '" << cmd << "'" << endl;
        }
        else
        {
            Command c;
            c.leftSpeed = leftSpeed;
            c.rightSpeed = rightSpeed;
            c.time = time;

            commandList.push_back(c);
        }
        
        s = s.substr(s.find(";")+1);
    }

    timer.stop();

    if (commandList.empty())
    {
        cout << "WARNING: received command list with bad commands; robot will not move" << endl;
        killRobot();
    }
    else
    {
        sendCommand(commandList[0]);
        
        const bool oneShot = true;
        timer = globalHandle->createTimer(ros::Duration(commandList[0].time), timerCallback, oneShot);
    }

    cout << "DONE PROCESSING COMMAND LIST" << endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ggb_txrx");
    NodeHandle n;
    NodeHandle privateHandle("~");
    globalHandle = &n;

    string portName;
    privateHandle.param<string>("port", portName, "/dev/ttyUSB0");
    n.param<bool>("do_serial", doSerial, true);

    // Argument 1 is the port name
    //std::string port(argv[1]);

    // Argument 2 is the baudrate
    unsigned long baud = 115200;
    //sscanf(argv[2], "%lu", &baud);

    ros::Publisher pub;
    //pub = n.advertise<std_msgs::String>("path_planner", 1000);

    serial_port = NULL;
    if (doSerial) new serial::Serial(portName, baud, serial::Timeout::simpleTimeout(1000));

    if(serial_port && !serial_port->isOpen()){
        ROS_ERROR("Serial port could not open. Exiting...");
        return 1;
    }
    
    //ros::Subscriber sub = n.subscribe("path_planner", 1000, tx_netduino);       
    ros::Subscriber submotor = n.subscribe("motor_commands", 1000, tx_netduino);

    simulatorPub = n.advertise<geometry_msgs::Point32>("simulator_cmds", 1000);  

    ros::spin();

    // while(ros::ok()) {
    //     //TODO: Make sure that Netduino adds a new line character after each message.
    //     if(serial_port->available()) {
    //         std_msgs::String msg;
    //         msg.data = serial_port->readline(); 
    //         pub.publish(msg);   
    //     }
    //     ros::spinOnce();    
    // }

    if (serial_port) delete serial_port;
    return 0;
}

