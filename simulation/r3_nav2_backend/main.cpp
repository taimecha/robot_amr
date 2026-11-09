// Calls the installed Nav2 plugins. No drive/lift command publishers exist here.
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <pluginlib/class_loader.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_core/smoother.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_subscriber.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
using J = nlohmann::json;
geometry_msgs::msg::PoseStamped pose(const J & a) {
  geometry_msgs::msg::PoseStamped p; p.header.frame_id="odom";
  p.pose.position.x=a[0]; p.pose.position.y=a[1];
  double yaw=a[2]; p.pose.orientation.z=sin(yaw/2); p.pose.orientation.w=cos(yaw/2); return p;
}
J rows(const nav_msgs::msg::Path & p) {
  J out=J::array(); for (const auto & v:p.poses) out.push_back({v.pose.position.x,v.pose.position.y,tf2::getYaw(v.pose.orientation)}); return out;
}
int main(int argc,char **argv) {
  if(argc!=3) {std::cerr<<"usage: r3_nav2_backend request.json response.json\n"; return 2;}
  J req; std::ifstream(argv[1])>>req;
  rclcpp::init(0,nullptr); J out;
  try {
    // Loaders outlive nodes even when a planner/smoother exception unwinds.
    pluginlib::ClassLoader<nav2_core::GlobalPlanner> pl("nav2_core","nav2_core::GlobalPlanner");
    pluginlib::ClassLoader<nav2_core::Smoother> sl("nav2_core","nav2_core::Smoother");
    auto cm=std::make_shared<nav2_costmap_2d::Costmap2DROS>("r3_costmap");
    cm->set_parameter(rclcpp::Parameter("global_frame","odom"));
    cm->set_parameter(rclcpp::Parameter("plugins",std::vector<std::string>{}));
    cm->set_parameter(rclcpp::Parameter("footprint_padding",0.0));
    if(cm->on_configure(rclcpp_lifecycle::State())!=nav2_util::CallbackReturn::SUCCESS) throw std::runtime_error("costmap configure failed");
    auto grid=cm->getCostmap();
    double res=req.at("resolution"), ox=req.at("origin")[0],oy=req.at("origin")[1];
    unsigned width=req.contains("size")?req.at("size")[0].get<unsigned>():240;
    unsigned height=req.contains("size")?req.at("size")[1].get<unsigned>():240;
    if(width==0 || height==0)throw std::runtime_error("empty costmap");
    grid->resizeMap(width,height,res,ox,oy);
    const size_t cells=static_cast<size_t>(width)*height;
    std::fill(grid->getCharMap(),grid->getCharMap()+cells,0);
    for(const auto & r:req.at("obstacles")) {
      for(unsigned y=0;y<height;y++) for(unsigned x=0;x<width;x++) {
        double wx,wy; grid->mapToWorld(x,y,wx,wy);
        if(wx>=r[0]&&wx<=r[2]&&wy>=r[1]&&wy<=r[3]) grid->setCost(x,y,nav2_costmap_2d::LETHAL_OBSTACLE);
      }
    }
    std::vector<geometry_msgs::msg::Point> fp;
    for(const auto & a:req.at("footprint")) {geometry_msgs::msg::Point p;p.x=a[0];p.y=a[1];fp.push_back(p);} cm->setRobotFootprint(fp);
    auto node=std::make_shared<nav2_util::LifecycleNode>("r3_plugins");
    for(auto it=req.at("parameters").begin();it!=req.at("parameters").end();++it) {
      auto v=it.value(); if(v.is_boolean()) node->declare_parameter(it.key(),v.get<bool>());
      else if(v.is_number_integer()) node->declare_parameter(it.key(),v.get<int>());
      else if(v.is_number()) node->declare_parameter(it.key(),v.get<double>());
      else node->declare_parameter(it.key(),v.get<std::string>());
    }
    auto planner=pl.createSharedInstance("nav2_smac_planner::SmacPlannerLattice");
    auto smoother=sl.createSharedInstance("nav2_constrained_smoother/ConstrainedSmoother");
    planner->configure(node,"Lattice",nullptr,cm); planner->activate();
    auto sub=std::make_shared<nav2_costmap_2d::CostmapSubscriber>(node,"/r3_private_costmap");
    auto msg=std::make_shared<nav2_msgs::msg::Costmap>(); msg->header.frame_id="odom";
    msg->metadata.size_x=width; msg->metadata.size_y=height; msg->metadata.resolution=res;
    msg->metadata.origin.position.x=ox;msg->metadata.origin.position.y=oy;msg->metadata.origin.orientation.w=1.;
    msg->data.assign(grid->getCharMap(),grid->getCharMap()+cells); sub->costmapCallback(msg);
    smoother->configure(node,"Constrained",nullptr,sub,nullptr); smoother->activate();
    auto path=planner->createPlan(pose(req.at("start")),pose(req.at("goal")),[](){return false;});
    if(path.poses.size()<2) throw std::runtime_error("lattice returned fewer than two poses");
    out["raw"]=rows(path);
    // Preserve continuous start/goal when the discrete lattice starts or
    // ends with an in-place rotation, not just translating segments.
    auto first=path.poses.front().pose.position;
    auto last=path.poses.back().pose.position;
    auto exact_start=pose(req.at("start"));auto exact_goal=pose(req.at("goal"));
    for(auto & v:path.poses) {
      if(hypot(v.pose.position.x-first.x,v.pose.position.y-first.y)>1e-5)break;
      v.pose.position=exact_start.pose.position;
    }
    for(auto it=path.poses.rbegin();it!=path.poses.rend();++it) {
      if(hypot(it->pose.position.x-last.x,it->pose.position.y-last.y)>1e-5)break;
      it->pose.position=exact_goal.pose.position;
    }
    path.poses.front().pose.orientation=exact_start.pose.orientation;
    path.poses.back().pose.orientation=exact_goal.pose.orientation;
    // Snap only the translating segment's endpoints to the requested continuous
    // poses BEFORE constrained optimization. The final line is never optimized.
    // A coincident-XY/yaw jump is a preserved rotation phase, never smoothed.
    J phases=J::array(); size_t begin=0;
    auto add_translation=[&](size_t end) {
      if(end<=begin)return;
      nav_msgs::msg::Path seg; seg.header=path.header;
      seg.poses.assign(path.poses.begin()+begin,path.poses.begin()+end+1);
      if(begin==0)seg.poses.front()=pose(req.at("start"));
      if(end==path.poses.size()-1)seg.poses.back()=pose(req.at("goal"));
      // Jazzy ConstrainedSmoother fixes the first TWO and last TWO XYs.
      // Set those direction holders on the exact endpoint rays before solve;
      // setting quaternion alone does not constrain its XY tangent.
      if(seg.poses.size()>=4) {
        auto & first=seg.poses.front().pose;auto & next=seg.poses[1].pose;
        auto & last=seg.poses.back().pose;auto & prev=seg.poses[seg.poses.size()-2].pose;
        double d0=hypot(next.position.x-first.position.x,next.position.y-first.position.y);
        double d1=hypot(last.position.x-prev.position.x,last.position.y-prev.position.y);
        double h0=tf2::getYaw(first.orientation),h1=tf2::getYaw(last.orientation);
        next.position.x=first.position.x+d0*cos(h0);next.position.y=first.position.y+d0*sin(h0);
        prev.position.x=last.position.x-d1*cos(h1);prev.position.y=last.position.y-d1*sin(h1);
      }
      if(!smoother->smooth(seg,rclcpp::Duration::from_seconds(2.0)))throw std::runtime_error("constrained smoothing failed");
      phases.push_back({{"type","APPROACH"},{"poses",rows(seg)}});
    };
    for(size_t i=1;i<path.poses.size();i++) {
      const auto & a=path.poses[i-1].pose;const auto & b=path.poses[i].pose;
      if(hypot(b.position.x-a.position.x,b.position.y-a.position.y)<1e-5 && fabs(tf2::getYaw(b.orientation)-tf2::getYaw(a.orientation))>1e-4) {
        add_translation(i-1);
        nav_msgs::msg::Path rot; rot.poses={path.poses[i-1],path.poses[i]};
        phases.push_back({{"type","ALIGN_YAW"},{"poses",rows(rot)}});begin=i;
      }
    }
    add_translation(path.poses.size()-1);
    out["phases"]=phases;out["status"]="OK";
    smoother->deactivate();smoother->cleanup();planner->deactivate();planner->cleanup();
    cm->on_cleanup(rclcpp_lifecycle::State());
    // Node callbacks may reference plugin code: destroy the node before the
    // loaders unload their libraries (also required on the installed Jazzy).
    smoother.reset();planner.reset();sub.reset();node.reset();cm.reset();
  } catch(const std::exception &e) {out["status"]="ERROR";out["reason"]=e.what();}
  std::ofstream(argv[2])<<out.dump(2)<<'\n';rclcpp::shutdown(); return out["status"]=="OK"?0:1;
}
