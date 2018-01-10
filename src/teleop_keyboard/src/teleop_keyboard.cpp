// ouiyeah @ 2016-04-11

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <actionlib_msgs/GoalID.h>
#include <yocs_msgs/NavigationControl.h>
#include <std_msgs/String.h>

#include <signal.h>
#include <termios.h>

int fd_;
struct termios cooked_, raw_;

void quit(int sig)
{
    tcsetattr(fd_, TCSANOW, &cooked_);
    exit(0);
}

int xtest();

class teleop_keyboard
{
public:
    teleop_keyboard();

private:
    void read_key_spin();
    void cmd_vel_timer(const ros::TimerEvent& e);
    void show_help();
    
private:
    ros::NodeHandle nh_;
    ros::NodeHandle ph_;
    ros::Timer cmd_vel_timer_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher cancel_pub_;
    ros::Publisher nav_ctrl_pub_;
    ros::Publisher cmd_string_pub_;

    geometry_msgs::Twist cmd_vel_;
    actionlib_msgs::GoalID cancel_;
    yocs_msgs::NavigationControl nav_ctrl_;
    std_msgs::String cmd_string_;

    double linear_vel_base_;
    double angular_vel_base_;

    bool deadman_enabled_;
    bool deadman_pressed_;
    bool stop_published_;
};

teleop_keyboard::teleop_keyboard()
:ph_("~")
{
    double cmd_vel_freq;
    ph_.param("cmd_vel_freq", cmd_vel_freq, 5.0);
    cmd_vel_timer_ = nh_.createTimer(ros::Rate(cmd_vel_freq), &teleop_keyboard::cmd_vel_timer, this);
    cmd_vel_pub_ = ph_.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    cancel_pub_ = nh_.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 50);
    nav_ctrl_pub_ = nh_.advertise<yocs_msgs::NavigationControl>("/nav_ctrl", 50);
    cmd_string_pub_ = nh_.advertise<std_msgs::String>("/rosbridge_shell/cmd_string", 50);

    nav_ctrl_.control = yocs_msgs::NavigationControl::STOP;
    std::string shell_cmd_string_name;
    ph_.param("shell_cmd_string_name", shell_cmd_string_name, std::string("keyd"));
    cmd_string_.data = shell_cmd_string_name;

    ph_.param("linear_vel_base", linear_vel_base_, 0.5);
    ph_.param("angular_vel_base", angular_vel_base_, 0.5);

    deadman_enabled_ = true;
    deadman_pressed_ = false;
    stop_published_ = false;

    show_help();

    read_key_spin();
}

void teleop_keyboard::read_key_spin()
{    
    signal(SIGINT, quit);
    fd_ = 0;
    // get the console in raw mode
    tcgetattr(fd_, &cooked_);
    memcpy(&raw_, &cooked_, sizeof(struct termios));
    raw_.c_lflag &=~ (ICANON | ECHO);
    raw_.c_cc[VEOL] = 1; // set a new line
    raw_.c_cc[VEOF] = 2; // set end of file
    tcsetattr(fd_, TCSANOW, &raw_);

    enum KEYCODE
    {
        KEYCODE_NULL,
        KEYCODE_ESC   = 0x1b,
        KEYCODE_LEFT  = 0x25,
        KEYCODE_UP,
        KEYCODE_RIGHT,
        KEYCODE_DOWN,
        KEYCODE_A_CAP = 0x41,
        KEYCODE_B_CAP,
        KEYCODE_C_CAP,
        KEYCODE_D_CAP,
        KEYCODE_E_CAP,
        KEYCODE_Q_CAP = 0x51,
        KEYCODE_S_CAP = 0x53,
        KEYCODE_W_CAP = 0x57,
        KEYCODE_X_CAP = 0x58,
        KEYCODE_Z_CAP = 0x5a,
        KEYCODE_CSE   = 0x5b,
        KEYCODE_A     = 0x61,
        KEYCODE_C     = 0x63,
        KEYCODE_D     = 0x64,
        KEYCODE_E     = 0x65,
        KEYCODE_Q     = 0x71,
        KEYCODE_S     = 0x73,
        KEYCODE_W     = 0x77,
        KEYCODE_X     = 0x78,
        KEYCODE_Z     = 0x7a,
    } ch = KEYCODE_NULL;
    enum KEYENUM
    {
        KEYENUM_NULL,
        KEYENUM_ESC,
        KEYENUM_CSE,
    } key = KEYENUM_NULL;

    while (ros::ok())
    {
        ros::spinOnce();

        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_, &fds);
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 500000; // set 0.5s timeout
        if (select(fd_ + 1, &fds, NULL, NULL, &tv) > 0)
        {
            // get the next event from the keyboard
            if (read(fd_, &ch, 1) < 0) {
                ROS_ERROR("keyboard read fd: %d", fd_);
                exit(-1);
            }
    
            if (KEYENUM_NULL == key && KEYCODE_ESC == ch) {
                key = KEYENUM_ESC;
                continue;
            } else if (KEYENUM_ESC == key && KEYCODE_CSE == ch) {
                key = KEYENUM_CSE;
                continue;
            } else if (KEYENUM_CSE == key) {
                key = KEYENUM_NULL;
                if (KEYCODE_A_CAP == ch) {
                    ch = KEYCODE_UP;
                } else if (KEYCODE_B_CAP == ch) {
                    ch = KEYCODE_DOWN;
                } else if (KEYCODE_C_CAP == ch) {
                    ch = KEYCODE_RIGHT;
                } else if (KEYCODE_D_CAP == ch) {
                    ch = KEYCODE_LEFT;
                }
            }

            deadman_enabled_ = true;
            deadman_pressed_ = true;
            switch(ch)
            {
            case KEYCODE_Q_CAP:
            case KEYCODE_Q:
                cmd_vel_.linear.x = 0;
                cmd_vel_.angular.z = 0;
                break;
            case KEYCODE_Z_CAP:
            case KEYCODE_Z:
                cancel_pub_.publish(cancel_);
                nav_ctrl_pub_.publish(nav_ctrl_);
                deadman_pressed_ = false;
                break;
            case KEYCODE_C_CAP:
                cmd_string_pub_.publish(cmd_string_);
                deadman_pressed_ = false;
                break;
            case KEYCODE_C:
                show_help();
                deadman_pressed_ = false;
                break;
            case KEYCODE_E_CAP:
                linear_vel_base_ = linear_vel_base_ * 2;
                angular_vel_base_ = angular_vel_base_ * 2;
                break;
            case KEYCODE_E:
                linear_vel_base_ = linear_vel_base_ / 2;
                angular_vel_base_ = angular_vel_base_ / 2;
                break;
            case KEYCODE_W_CAP:
                deadman_enabled_ = false;
            case KEYCODE_W:
            case KEYCODE_UP:
                cmd_vel_.linear.x = linear_vel_base_;
                cmd_vel_.angular.z = 0;
                break;
            case KEYCODE_X_CAP:
                // xtest();
                // break;
            case KEYCODE_S_CAP:
                deadman_enabled_ = false;
            case KEYCODE_X:
            case KEYCODE_S:
            case KEYCODE_DOWN:
                cmd_vel_.linear.x = - linear_vel_base_;
                cmd_vel_.angular.z = 0;
                break;
            case KEYCODE_A_CAP:
                deadman_enabled_ = false;
            case KEYCODE_A:
            case KEYCODE_LEFT:
                cmd_vel_.linear.x = 0;
                cmd_vel_.angular.z = angular_vel_base_;
                break;
            case KEYCODE_D_CAP:
                deadman_enabled_ = false;
            case KEYCODE_D:
            case KEYCODE_RIGHT:
                cmd_vel_.linear.x = 0;
                cmd_vel_.angular.z = - angular_vel_base_;
                break;
            default:
                deadman_pressed_ = false;
                break;
            }
        } else {
            deadman_pressed_ = false;
        }
    }
}

void teleop_keyboard::cmd_vel_timer(const ros::TimerEvent& e)
{
    if (deadman_pressed_)
    {
        cmd_vel_pub_.publish(cmd_vel_);
        stop_published_ = false;
    }
    else if(deadman_enabled_ && !deadman_pressed_ && !stop_published_)
    {
        cmd_vel_.linear.x = 0;
        cmd_vel_.angular.z = 0;
        cmd_vel_pub_.publish(cmd_vel_);
        stop_published_ = true;
    }
}

void teleop_keyboard::show_help()
{
    puts("------------------------------");
    puts("--  keyboard teleoperation  --");
    puts("------------------------------");
    puts("  ______     ____     ______  ");
    puts(" |  Q:  |   / W: \\   |  E:  | ");
    puts(" | stop |  |______|  |setvel| ");
    puts("  ______    ______    ______  ");
    puts(" /  A:  |  |  S:  |  |  D:  \\ ");
    puts(" \\______|   \\____/   |______/ ");
    puts("  ______    ______    ______  ");
    puts(" |  Z*  |  |  X:  |  |  C#  | ");
    puts(" |cancel|   \\todo/   | help | ");
    puts("------------------------------");
    puts("--  E  :  set vel up        --");
    puts("--  e  :  set vel down      --");
    puts("-- Z*z :  cancel, nav_ctrl  --");
    puts("-- C#  :  shell/cmd_string  --");
    puts("------------------------------");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_keyboard");
    teleop_keyboard teleop_keyboard;

    ros::spin();
}

#include <linux/input.h>
#include <fcntl.h>

#define KEY_RESERVED     0 
#define KEY_ESC  1 
#define KEY_1    2 
#define KEY_2    3 
#define KEY_3    4 
#define KEY_4    5 
#define KEY_5    6 
#define KEY_6    7 
#define KEY_7    8 
#define KEY_8    9 
#define KEY_9    10 
#define KEY_0    11 
#define KEY_MINUS    12 
#define KEY_EQUAL    13 
#define KEY_BACKSPACE    14 
#define KEY_TAB  15 
#define KEY_Q    16 
#define KEY_W    17
#define KEY_E    18 
#define KEY_R    19 
#define KEY_T    20

int xtest()
{
    int keys_fd;
    char ret[2];
    struct input_event t;

    keys_fd = open("/dev/input/event3", O_RDONLY);
    if (keys_fd <= 0)
    {
        ROS_ERROR("open /dev/input/event0 device error!\n");
        return 0;
    } else {
        ROS_ERROR("%d", keys_fd);
    }

    while (1)
    {
        if (read (keys_fd, &t, sizeof (t)) == sizeof (t))
        {
            if (t.type == EV_KEY)
            if (t.value == 0 || t.value == 1)
        {
            ROS_ERROR("key %d %s\n", t.code,
                        (t.value) ? "Pressed" : "Released");
            if(t.code==KEY_ESC)
                break;
        }
        }
    }
    close (keys_fd);

    return 0;
}
