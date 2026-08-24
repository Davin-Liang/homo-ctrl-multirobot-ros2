#include "homo_multirobot_formation_control/formation_control_node_6d_artstein_disc_hocbf.hpp"

#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using formation_control::hocbf::Circle;

FormationController6DArtsteinDiscHocbf::FormationController6DArtsteinDiscHocbf()
: Node("formation_control_node_6d_artstein_disc_hocbf")
{
  leader_ns_ = declare_parameter("leader_ns", "/robot1");
  follower_ns_ = declare_parameter("follower_ns", "/robot2");
  const double radius = declare_parameter("radius", 2.0), mass = declare_parameter("mass", 2.0), inertia = declare_parameter("I", 1.0);
  rate_ = declare_parameter("control_rate", 20.0); tau_ = declare_parameter("tau", .43); tau_yaw_ = declare_parameter("tau_yaw", tau_); Td_ = declare_parameter("Td", .22);
  vmax_ = declare_parameter("max_linear_vel", 1.0); wmax_ = declare_parameter("max_angular_vel", .5); amax_ = declare_parameter("max_linear_accel", 2.0);
  follower_radius_ = declare_parameter("follower_radius", .15); clearance_ = declare_parameter("clearance", .10); perception_margin_ = declare_parameter("perception_margin", .15); scan_timeout_ = declare_parameter("scan_timeout", .30);
  cluster_tolerance_ = declare_parameter("cluster_tolerance", .10); min_cluster_points_ = declare_parameter("min_cluster_points", 5); max_obstacles_ = declare_parameter("max_obstacles", 10);
  min_cylinder_radius_ = declare_parameter("min_cylinder_radius", .03); max_cylinder_radius_ = declare_parameter("max_cylinder_radius", .60); max_fit_residual_ = declare_parameter("max_fit_residual", .03);
  const double dt = 1.0 / rate_;
  Eigen::Matrix4d A4 = Eigen::Matrix4d::Zero(); A4(0,2)=1; A4(1,3)=1; A4(2,2)=A4(3,3)=-1.0/tau_;
  Eigen::Matrix<double,4,2> B4 = Eigen::Matrix<double,4,2>::Zero(); B4(2,0)=B4(3,1)=1.0/tau_;
  trans_.build(A4, B4, tau_, Td_, dt);
  Eigen::Matrix2d A2; A2 << 0,1,0,-1.0/tau_yaw_; Eigen::Matrix<double,2,1> B2; B2 << 0,1.0/tau_yaw_; yaw_.build(A2,B2,tau_yaw_,Td_,dt);
  ctrl_ = std::make_unique<formation_control::LpcController6DArtsteinDisc>(radius,mass,inertia,declare_parameter("m_p",4),declare_parameter("tol",.1),declare_parameter("use_hpc",true),dt,declare_parameter("hpc_c_min",.5),declare_parameter("initial_min_lambda",1.0),declare_parameter("switch_min_lambda",4.0),declare_parameter("hpc_vel_threshold",.3),declare_parameter("hpc_yaw_threshold",.3),declare_parameter("stability_margin",.01));
  constraint_ = formation_control::KinematicConstraint(declare_parameter("wheel_radius",.03),declare_parameter("base_radius",.11),declare_parameter("wheel_max_omega",20.0),declare_parameter("max_linear_accel",2.0),declare_parameter("max_angular_accel",4.0));
  tf_ = std::make_unique<tf2_ros::Buffer>(get_clock()); tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
  auto qos=rclcpp::SensorDataQoS();
  leader_sub_=create_subscription<nav_msgs::msg::Odometry>(leader_ns_+"/odometry/filtered",qos,[this](nav_msgs::msg::Odometry::SharedPtr m){leader_odom_=m;});
  follower_sub_=create_subscription<nav_msgs::msg::Odometry>(follower_ns_+"/odometry/filtered",qos,[this](nav_msgs::msg::Odometry::SharedPtr m){follower_odom_=m;});
  scan_sub_=create_subscription<sensor_msgs::msg::LaserScan>(declare_parameter("scan_topic","scan"),qos,[this](sensor_msgs::msg::LaserScan::SharedPtr m){scan_cb(m);});
  pub_=create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
  timer_=create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000/rate_)),[this]{timer_cb();});
}

double FormationController6DArtsteinDiscHocbf::yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q) { return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)); }
Eigen::Vector2d FormationController6DArtsteinDiscHocbf::body_to_map(double yaw,const Eigen::Vector2d& v) { const double c=std::cos(yaw),s=std::sin(yaw); return {c*v.x()-s*v.y(),s*v.x()+c*v.y()}; }
Eigen::Vector2d FormationController6DArtsteinDiscHocbf::map_to_body(double yaw,const Eigen::Vector2d& v) { const double c=std::cos(yaw),s=std::sin(yaw); return {c*v.x()+s*v.y(),-s*v.x()+c*v.y()}; }

bool FormationController6DArtsteinDiscHocbf::odom_to_state(const std::string& ns,const nav_msgs::msg::Odometry::SharedPtr& odom,State& state) {
  if (!odom) return false; std::string frame=ns; if (!frame.empty()&&frame.front()=='/') frame.erase(0,1); frame+="_odom";
  try { const auto t=tf_->lookupTransform("map",frame,tf2::TimePoint()); const double ty=yaw_from_quaternion(t.transform.rotation), ey=yaw_from_quaternion(odom->pose.pose.orientation); state.x(0)=t.transform.translation.x+odom->pose.pose.position.x*std::cos(ty)-odom->pose.pose.position.y*std::sin(ty); state.x(1)=t.transform.translation.y+odom->pose.pose.position.x*std::sin(ty)+odom->pose.pose.position.y*std::cos(ty); state.x(2)=std::atan2(std::sin(ty+ey),std::cos(ty+ey)); state.x(3)=odom->twist.twist.linear.x; state.x(4)=odom->twist.twist.linear.y; state.x(5)=odom->twist.twist.angular.z; state.v_map=body_to_map(state.x(2),state.x.segment<2>(3)); return true; } catch(const tf2::TransformException&) { return false; }
}

Eigen::VectorXd FormationController6DArtsteinDiscHocbf::predict_leader(const Eigen::VectorXd& x,double horizon) const { Eigen::VectorXd p=x; p(0)+=body_to_map(x(2),x.segment<2>(3)).x()*horizon; p(1)+=body_to_map(x(2),x.segment<2>(3)).y()*horizon; p(2)=std::atan2(std::sin(x(2)+x(5)*horizon),std::cos(x(2)+x(5)*horizon)); return p; }
Eigen::VectorXd FormationController6DArtsteinDiscHocbf::predict_follower(const State& measured) { Eigen::VectorXd x4(4); x4<<measured.x(0),measured.x(1),measured.v_map; auto p4=trans_.predict(x4+trans_.integral(v_history_),last_map_cmd_); Eigen::VectorXd x2(2); x2<<measured.x(2),measured.x(5); Eigen::VectorXd w(1);w<<last_wcmd_; auto p2=yaw_.predict(x2+yaw_.integral(w_history_),w); Eigen::VectorXd out(6); const double theta=std::atan2(std::sin(p2(0)),std::cos(p2(0))); const auto vb=map_to_body(theta,p4.tail<2>()); out<<p4(0),p4(1),theta,vb.x(),vb.y(),p2(1); return out; }

void FormationController6DArtsteinDiscHocbf::scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::vector<std::vector<Eigen::Vector2d>> clusters; std::vector<Eigen::Vector2d> current;
  for(size_t i=0;i<msg->ranges.size();++i) { const double r=msg->ranges[i]; if(!std::isfinite(r)||r<msg->range_min||r>msg->range_max) { if((int)current.size()>=min_cluster_points_) clusters.push_back(current); current.clear(); continue; } const double a=msg->angle_min+i*msg->angle_increment; Eigen::Vector2d p(r*std::cos(a),r*std::sin(a)); if(!current.empty()&&(p-current.back()).norm()>cluster_tolerance_) { if((int)current.size()>=min_cluster_points_) clusters.push_back(current); current.clear(); } current.push_back(p); }
  if((int)current.size()>=min_cluster_points_) clusters.push_back(current);
  try { const auto t=tf_->lookupTransform("map",msg->header.frame_id,rclcpp::Time(msg->header.stamp)); const double yaw=yaw_from_quaternion(t.transform.rotation); std::vector<Circle> detected; for(const auto& cluster:clusters) { auto fitted=formation_control::hocbf::fit_circle(cluster,max_fit_residual_); if(!fitted||fitted->radius<min_cylinder_radius_||fitted->radius>max_cylinder_radius_) continue; fitted->center=body_to_map(yaw,fitted->center)+Eigen::Vector2d(t.transform.translation.x,t.transform.translation.y); fitted->radius+=follower_radius_+clearance_+perception_margin_; detected.push_back(*fitted); } std::sort(detected.begin(),detected.end(),[](const Circle&a,const Circle&b){return a.center.squaredNorm()<b.center.squaredNorm();}); if((int)detected.size()>max_obstacles_) detected.resize(max_obstacles_); obstacles_=std::move(detected); last_scan_=rclcpp::Time(msg->header.stamp); } catch(const tf2::TransformException&) { RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"HOCBF scan TF unavailable"); }
}

void FormationController6DArtsteinDiscHocbf::timer_cb() {
  State leader,follower; if(!odom_to_state(leader_ns_,leader_odom_,leader)||!odom_to_state(follower_ns_,follower_odom_,follower)) return;
  if(!initialized_) { last_map_cmd_=follower.v_map; last_wcmd_=follower.x(5); for(int i=0;i<trans_.buffer_size();++i){Eigen::VectorXd v(2);v<<last_map_cmd_;v_history_.push_back(v);} for(int i=0;i<yaw_.buffer_size();++i){Eigen::VectorXd w(1);w<<last_wcmd_;w_history_.push_back(w);} ctrl_->controller_initial(predict_leader(leader.x,Td_+std::max(tau_,tau_yaw_)),predict_follower(follower)); initialized_=true; }
  const auto pred=predict_follower(follower); const auto nominal=ctrl_->lpc_calculate(predict_leader(leader.x,Td_+std::max(tau_,tau_yaw_)),pred); const auto nominal_map=body_to_map(follower.x(2),Eigen::Vector2d(nominal[0],nominal[1])); Eigen::Vector2d safe_map=Eigen::Vector2d::Zero(); bool safe=false;
  if((now()-last_scan_).seconds()<=scan_timeout_) { std::vector<formation_control::hocbf::Halfspace> constraints; Eigen::Vector4d state; const auto velocity_map=body_to_map(pred(2),pred.segment<2>(3)); state << pred(0), pred(1), velocity_map.x(), velocity_map.y(); for(const auto&o:obstacles_) constraints.push_back(formation_control::hocbf::hocbf_halfspace(state,o,tau_,2.,2.)); const auto result=formation_control::hocbf::solve_translation_qp(nominal_map,last_map_cmd_,constraints,vmax_,amax_,1.0/rate_); safe_map=result.command; safe=result.feasible; }
  if(!safe) RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),1000,"HOCBF stopping: stale scan or infeasible QP"); const auto body=map_to_body(follower.x(2),safe_map); geometry_msgs::msg::Twist cmd; cmd.linear.x=body.x();cmd.linear.y=body.y();cmd.angular.z=std::clamp(nominal[2],-wmax_,wmax_); constraint_.apply(cmd.linear.x,cmd.linear.y,cmd.angular.z,1.0/rate_); pub_->publish(cmd); last_map_cmd_=body_to_map(follower.x(2),Eigen::Vector2d(cmd.linear.x,cmd.linear.y));last_wcmd_=cmd.angular.z; Eigen::VectorXd v(2);v<<last_map_cmd_;v_history_.push_front(v);while((int)v_history_.size()>trans_.buffer_size())v_history_.pop_back();Eigen::VectorXd w(1);w<<last_wcmd_;w_history_.push_front(w);while((int)w_history_.size()>yaw_.buffer_size())w_history_.pop_back();
}
