/*********************************************************************
Written for use in Deep Orange 13 for CAN messages management
 *********************************************************************/

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <deeporange13_control/DeepOrangeDbwCan.h>

namespace deeporange_dbw_ros
{

class CanNodelet : public nodelet::Nodelet
{
public:
  CanNodelet()
  {
  }
  ~CanNodelet()
  {
  }

  void onInit(void)
  {
    node_.reset(new DeepOrangeDbwCan(getNodeHandle(), getPrivateNodeHandle()));
  }

private:
  boost::shared_ptr<DeepOrangeDbwCan> node_;
};

} // deeporange_dbw_ros

// Register this plugin with pluginlib.  Names must match nodelets.xml.
//
// parameters: class type, base class type
PLUGINLIB_EXPORT_CLASS(deeporange_dbw_ros::CanNodelet, nodelet::Nodelet);
