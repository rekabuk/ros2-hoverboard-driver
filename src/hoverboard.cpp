#include "ros2-hoverboard-driver/hoverboard.hpp"

#define MY_DEBUG

Hoverboard::Hoverboard() 
: Node("hoverboard_driver_node") // Member initialization for ROS2 node
{
    // // Constructor implementation
    // hardware_interface::JointStateHandle left_wheel_state_handle("left_wheel",
	// 							 &joints[0].pos.data,
	// 							 &joints[0].vel.data,
	// 							 &joints[0].eff.data);
    // hardware_interface::JointStateHandle right_wheel_state_handle("right_wheel",
	// 							  &joints[1].pos.data,
	// 							  &joints[1].vel.data,
	// 							  &joints[1].eff.data);
    // joint_state_interface.registerHandle (left_wheel_state_handle);
    // joint_state_interface.registerHandle (right_wheel_state_handle);
    // registerInterface(&joint_state_interface);

    // hardware_interface::JointHandle left_wheel_vel_handle(
    //     joint_state_interface.getHandle("left_wheel"),
    //     &joints[0].cmd.data);
    // hardware_interface::JointHandle right_wheel_vel_handle(
    //     joint_state_interface.getHandle("right_wheel"),
    //     &joints[1].cmd.data);
    // velocity_joint_interface.registerHandle (left_wheel_vel_handle);
    // velocity_joint_interface.registerHandle (right_wheel_vel_handle);
    // registerInterface(&velocity_joint_interface);

    // These publishers are only for debugging purposes
    // TODO : understand this. I think that using this-> for creating subscribers and publishers, makes that those
    // publishers and subscribers are directly related to the created node. That makes sense for using spin_some(node), since
    // only the related tasks to "node" will be executed.
    vel_pub_[0]    = this->create_publisher<std_msgs::msg::Float64>("hoverboard/left_wheel/velocity", 10);
    vel_pub_[1]    = this->create_publisher<std_msgs::msg::Float64>("hoverboard/right_wheel/velocity", 10);
    cmd_pub_[0]    = this->create_publisher<std_msgs::msg::Float64>("hoverboard/left_wheel/cmd", 10);
    cmd_pub_[1]    = this->create_publisher<std_msgs::msg::Float64>("hoverboard/right_wheel/cmd", 10);
    voltage_pub_   = this->create_publisher<std_msgs::msg::Float64>("hoverboard/battery_voltage", 10);
    temp_pub_      = this->create_publisher<std_msgs::msg::Float64>("hoverboard/temperature", 10);

    // Create the subscriber to receive speed setpoints
    speeds_sub_   = this->create_subscription<wheel_msgs::msg::WheelSpeeds>("wheel_vel_setpoints",
                    10, std::bind(&Hoverboard::setpoint_callback, this, std::placeholders::_1));

    cmd_vel_pub_  = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 
                    10, std::bind(&Hoverboard::twist_callback, this, std::placeholders::_1));

    // Convert m/s to rad/s
    // TODO : take into account the way we send references to the hoverboard controller
    //max_velocity /= wheel_radius;

    if ((port_fd = open(PORT, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Cannot open serial port to hoverboard");
        exit(-1); // TODO : put this again
    }
    
    // CONFIGURE THE UART -- connecting to the board
    // The flags (defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html):
    // TODO : understand this shit
    struct termios options;
    tcgetattr(port_fd, &options);
    options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(port_fd, TCIFLUSH);
    tcsetattr(port_fd, TCSANOW, &options);
}

Hoverboard::~Hoverboard() { // Destructor implementation
    if (port_fd != -1) 
        close(port_fd);
}

void Hoverboard::setpoint_callback(wheel_msgs::msg::WheelSpeeds::UniquePtr msg)
{
    setpoint[0] = msg->right_wheel;
    setpoint[1] = msg->left_wheel;

    RCLCPP_INFO(this->get_logger(), "I heard wheelSpeeds: %f, %f", setpoint[0], setpoint[1]);
}

void Hoverboard::twist_callback(geometry_msgs::msg::Twist::UniquePtr msg)
{
   // setpoint[0] = msg->right_wheel;
   // setpoint[1] = msg->left_wheel;

    RCLCPP_INFO(this->get_logger(), "I heard twist: %f, %f", setpoint[0], setpoint[1]);
}

int32_t Hoverboard::read() {
    int32_t status = 0;

    if (port_fd != -1) {
        uint8_t c;
        int i = 0, r = 0;
#ifdef MY_DEBUG1
            rclcpp::Time last_read;
#endif
#ifdef MY_DEBUG2
           RCLCPP_INFO(this->get_logger(), "Start reading UART");
#endif           

        while ((r = ::read(port_fd, &c, 1)) > 0 && i++ < 1024){
#ifdef MY_DEBUG2
           RCLCPP_INFO(this->get_logger(), "Reading UART");
#endif           
            status = protocol_recv(c);
        }            

#ifdef MY_DEBUG1
        if (i > 0)
            last_read = rclcpp::Clock().now();
#endif
        if (r < 0 && errno != EAGAIN)
            RCLCPP_ERROR(this->get_logger(), "Reading from serial %s failed: %d", PORT, r);
    }
#ifdef MY_DEBUG1
    if ((rclcpp::Clock().now() - last_read).toSec() > 1) {
        RCLCPP_ERROR(this->get_logger(), "Timeout reading from serial %s failed", PORT);
     }
#endif   
    return status;
}

int32_t Hoverboard::protocol_recv (uint8_t byte) {
    int32_t   status = 0;

    start_frame = ((uint16_t)(byte) << 8) | prev_byte;
#ifdef MY_DEBUG1    
    RCLCPP_INFO(this->get_logger(), "Received a byte: %x",(uint8_t)byte);
     if ((uint8_t)byte == 0xAB && (uint8_t)prev_byte == 0xCD){
         RCLCPP_INFO(this->get_logger(), "Received Start frame: %x", start_frame);
         RCLCPP_INFO(this->get_logger(), "Received Start frame: %x %x", (byte) << 8, (uint8_t)prev_byte);
     }
#endif
    // Read the start frame
    if (start_frame == START_FRAME) {
        //RCLCPP_INFO(this->get_logger(), "Start frame recognised");
        p = (uint8_t*)&msg;
        *p++ = prev_byte;
        *p++ = byte;
        msg_len = 2;
    } else if (msg_len >= 2 && msg_len < sizeof(SerialFeedback)) {
        // Otherwise just read the message content until the end
        *p++ = byte;
        msg_len++;
    }

    if (msg_len == sizeof(SerialFeedback)) {
        uint16_t checksum = (uint16_t)(
            msg.start ^
            msg.cmd1 ^
            msg.cmd2 ^
            msg.speedR_meas ^
            msg.speedL_meas ^
            msg.wheelR_cnt ^
            msg.wheelL_cnt ^
            msg.batVoltage ^
            msg.boardTemp ^
            msg.cmdLed);

        if (msg.start == START_FRAME && msg.checksum == checksum) {
            std_msgs::msg::Float64 f;

            f.data = (double)msg.batVoltage/100.0;
            voltage_pub_->publish(f);

            f.data = (double)msg.boardTemp/10.0;
            temp_pub_->publish(f);

            f.data = (double)msg.speedL_meas;
            vel_pub_[0]->publish(f);
            f.data = (double)msg.speedR_meas;
            vel_pub_[1]->publish(f);

            f.data = (double)msg.cmd1;
            cmd_pub_[0]->publish(f);
            f.data = (double)msg.cmd2;
            cmd_pub_[1]->publish(f);

            status = 1;
        } else {
            RCLCPP_INFO(this->get_logger(), "Hoverboard checksum mismatch: %d vs %d", msg.checksum, checksum);
        }
        msg_len = 0;
    }
    prev_byte = byte;

    return status;
}

void Hoverboard::write() {
    if (port_fd == -1) {
        RCLCPP_ERROR(this->get_logger(), "Attempt to write on closed serial");
        return;
    }
    // Calculate steering from difference of left and right //TODO : change thi
    const double speed = (setpoint[0] + setpoint[1])/2.0;
    const double steer = (setpoint[0] - setpoint[1])*2.0;

    SerialCommand command;
    command.start = (uint16_t)START_FRAME;
    command.steer = (int16_t)steer;
    command.speed = (int16_t)speed;
    command.checksum = (uint16_t)(command.start ^ command.steer ^ command.speed);

    int rc = ::write(port_fd, (const void*)&command, sizeof(command));
    // if (rc < 0) {
    //     RCLCPP_ERROR(this->get_logger(), "Error writing to hoverboard serial port errno=%d, %s", 
    //                   errno, strerror(errno));
    // } else if (rc == sizeof(command)) {
    //     RCLCPP_ERROR(this->get_logger(), "1");
    // } else {
    //     RCLCPP_ERROR(this->get_logger(), "0.");
    // }
    
}

