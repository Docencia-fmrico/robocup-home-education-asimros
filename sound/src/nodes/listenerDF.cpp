#include <dialog/DialogInterface.h>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

namespace ph = std::placeholders;

namespace gb_dialog
{
class ListenerDF: public DialogInterface
{
  public:
    ListenerDF(): nl_()
    {     
      this->registerCallback(
        std::bind(&ListenerDF::arrivedIntentCB, this, ph::_1),
        "Arrived");
      this->registerCallback(
        std::bind(&ListenerDF::sideIntentCB, this, ph::_1),
        "Side");
      this->registerCallback(std::bind(&ListenerDF::noIntentCB, this, ph::_1));

      sub_ = nl_.subscribe("/listen", 1, &ListenerDF::messageCallback, this);
      pub_ = nl_.advertise<std_msgs::String>("/answer", 1);
    }

    void messageCallback(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO("Message: [%s]", msg->data);
        listen();
    }

    void sideIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[ListenerDF] sideIntentCB: intent [%s]", result.intent.c_str());
      msg_.data = result.fulfillment_text;
      pub_.publish(msg_);
    }

    void noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[ListenerDF] noIntentCB: intent [%s]", result.intent.c_str());
      msg_.data = "false";
      pub_.publish(msg_);
    }

    void arrivedIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[ListenerDF] arrivedIntentCB: intent [%s]", result.intent.c_str());
      msg_.data = result.fulfillment_text;
      pub_.publish(msg_);
    }

  private:
    ros::NodeHandle nl_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std_msgs::String msg_;
};
}  // namespace gb_dialog

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener_df_node");
  gb_dialog::ListenerDF forwarder;
  ROS_INFO("Right or left?");
  ros::spin();
  return 0;
}