#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "integer_publisher");
    ros::NodeHandle nh;

    // Create a ROS publisher for sending integer messages
    ros::Publisher int_pub = nh.advertise<std_msgs::Int32>("integer_topic", 1000);

    ros::Rate loop_rate(1);  // 1 Hz

    while (ros::ok())
    {
        // Prompt the user to enter an integer or 'q' to quit
        std::cout << "Enter an integer (or 'q' to quit): ";
        std::string user_input;
        std::cin >> user_input;

        // Check if the user wants to quit
        if (user_input == "q" || user_input == "Q")
        {
            break;  // Exit the loop
        }

        // Convert the user input to an integer
        int input_value;
        try
        {
            input_value = std::stoi(user_input);
        }
        catch (const std::invalid_argument& e)
        {
            std::cout << "Invalid input. Please enter an integer or 'q' to quit." << std::endl;
            continue;  // Skip the rest of the loop iteration
        }

        // Create a ROS message with the entered integer
        std_msgs::Int32 msg;
        msg.data = input_value;

        // Publish the message to the "integer_topic"
        int_pub.publish(msg);

        // Process other ROS callbacks
        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
