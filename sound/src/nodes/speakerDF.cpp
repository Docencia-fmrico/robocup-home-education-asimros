#include <dialog/DialogInterface.h>
#include <string>

#include "ros/ros.h"
#include "std_msgs/String.h"

namespace ph = std::placeholders;

namespace gb_dialog
{
class SpeakerDF: public DialogInterface
{
  public:
    SpeakerDF(): ns_()
    {     
      sub_ = ns_.subscribe("/speak", 1, &SpeakerDF::messageCallback, this);
      pub_ = ns_.advertise<std_msgs::String>("/listen", 1);
    }

    void messageCallback(const std_msgs::String::ConstPtr& msg)
    {
        ROS_INFO("Speak: [%s]", msg->data);
        speak(msg -> data);
        //msg_.data = "listen";
        //pub_.publish(msg_);
    }

  private:
    ros::NodeHandle ns_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    std_msgs::String msg_;
};
}  // namespace gb_dialog

int main(int argc, char** argv)
{
  ros::init(argc, argv, "speaker_df_node");
  gb_dialog::SpeakerDF forwarder;
  ROS_INFO("Right or left?");
  ros::spin();
  return 0;
}