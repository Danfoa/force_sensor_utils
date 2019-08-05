/**
 * @file wrench_transformer.cpp
 * @author Daniel Felipe Ordonez Apraez (daniels.ordonez@gmail.com)
 * @brief Node and class definition to perform TF transformations of WrenchStamped messages
 * @version 0.1
 * @date 2019-08-05
 * 
 * @copyright Copyright (c) 2019 INVITE GmbH 
 *  
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the `Software`), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions: 
 *  
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *  
 * THE SOFTWARE IS PROVIDED `AS IS`, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>

#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class WrenchTransformer {

  private:
    std::string target_frame_;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener tf2_listener_;
    ros::NodeHandle nh_;

    ros::Publisher wrench_pub_;
    message_filters::Subscriber<geometry_msgs::WrenchStamped> wrench_sub_;
    tf2_ros::MessageFilter<geometry_msgs::WrenchStamped> tf2_filter_;

  public:

    WrenchTransformer(std::string target_frame, std::string input_topic = "wrench_input") : 
                          tf2_listener_(buffer_), 
                          tf2_filter_(wrench_sub_, buffer_, target_frame, 10, 0) {
                            
      
      target_frame_ = target_frame;
      wrench_sub_.subscribe(nh_, input_topic, 5);
      tf2_filter_.registerCallback( boost::bind(&WrenchTransformer::wrenchUpdateCB, this, _1) );
      wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("wrench_transformed", 1);
      ROS_WARN("Transforming wrench messages on [%s] topic to target frame [%s]", 
                                                            wrench_sub_.getTopic().c_str(),
                                                            target_frame.c_str());
    }

    //  Callback to register with tf2_ros::MessageFilter to be called when transforms are available
    void wrenchUpdateCB(const geometry_msgs::WrenchStampedConstPtr& input_wrench_ptr) {
      // Configure output message 
      geometry_msgs::WrenchStamped output_wrench;
      output_wrench.header = input_wrench_ptr->header;
      output_wrench.header.frame_id = target_frame_;

      try { 
        // Get transformation data  
        geometry_msgs::TransformStamped transformation;
        transformation = buffer_.lookupTransform(target_frame_, 
                                                  input_wrench_ptr->header.frame_id, 
                                                  input_wrench_ptr->header.stamp);
        // Apply transformation to force and torque vectors                                         
        tf2::doTransform(input_wrench_ptr->wrench.force, output_wrench.wrench.force, transformation);
        tf2::doTransform(input_wrench_ptr->wrench.torque, output_wrench.wrench.torque, transformation);
        // Publish transformed wrench.
        wrench_pub_.publish(output_wrench);
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("Failure to transform wrench data: %s\n", ex.what()); //Print exception which was caught
      }
    }


};


int main(int argc, char ** argv) {
  ros::init(argc, argv, "wrench_transformer"); 
  ros::NodeHandle nh;

  std::string target_frame = "~";

  std::string param_name;

  ros::param::get("~target_frame", target_frame);
  if (target_frame == "~") {
    ROS_FATAL("Invalid target frame given");
    ros::requestShutdown();
  }

  WrenchTransformer wrench_transformer(target_frame, "wrench_input"); 
  ros::spin(); 
  return 0;
};