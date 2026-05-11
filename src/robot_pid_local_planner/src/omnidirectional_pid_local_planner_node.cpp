#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_listener.h> // 必须包含 TF 监听器头文件

#include <robot_communication/localizationInfoBroadcast.h>
#include <nav_msgs/Odometry.h>

#include <cmath>
#include <algorithm>
#include <iostream>

using namespace std;

//把角度差归一化到 [-pi, pi]，避免“359° - 1° = 358°”这种跳变
static inline double wrap_to_pi(double a)
{
  while (a > M_PI) a -= 2.0*M_PI;
  while (a < -M_PI) a += 2.0*M_PI;
  return a;
}

//把值夹在范围内，速度限幅用
static inline double clamp(double x, double lo, double hi)
{
  return std::max(lo, std::min(hi, x));
}

struct PID
{
  double kp{0}, ki{0}, kd{0};
  double i{0}, prev_e{0};  // 上一次误差
  double i_limit{1.0};     // 积分限幅

  //给误差 e、时间间隔 dt，输出控制量u: kp*e + ki*i + kd*de
  double step(double e, double dt)
  {
    if (dt <= 1e-6) return kp*e;

    // I
    i += e*dt;
    i = clamp(i, -i_limit, i_limit);

    // D
    const double de = (e - prev_e)/dt;
    prev_e = e;

    return kp*e + ki*i + kd*de;
  }

  // 把积分和 prev_e 清零
  void reset()
  {
    i = 0;
    prev_e = 0;
  }
};

class OmnidirectionalPIDLocalPlanner
{
public:
  OmnidirectionalPIDLocalPlanner()
  : nh_(), pnh_("~")
  {
    nh_.param("IS_SIM", is_sim_, false);

    nh_.param("LOOKAHEAD_DIST", lookahead_dist_, 0.5);  //前瞻距离
    nh_.param("GOAL_TOL", goal_tol_, 0.25);

    nh_.param("MAX_VX", max_vx_, 0.4);
    nh_.param("MAX_VY", max_vy_, 0.4);
    nh_.param("MAX_WZ", max_wz_, 0.8);

    nh_.param("KP_X", pid_x_.kp, 0.9);
    nh_.param("KI_X", pid_x_.ki, 0.0);
    nh_.param("KD_X", pid_x_.kd, 0.0);

    nh_.param("KP_Y", pid_y_.kp, 0.9);
    nh_.param("KI_Y", pid_y_.ki, 0.0);
    nh_.param("KD_Y", pid_y_.kd, 0.0);

    nh_.param("KP_YAW", pid_yaw_.kp, 1.2);
    nh_.param("KI_YAW", pid_yaw_.ki, 0.0);
    nh_.param("KD_YAW", pid_yaw_.kd, 0.0);

    nh_.param("I_LIMIT_X", pid_x_.i_limit, 0.6);
    nh_.param("I_LIMIT_Y", pid_y_.i_limit, 0.6);
    nh_.param("I_LIMIT_YAW", pid_yaw_.i_limit, 1.0);

    nh_.param("CMD_TIMEOUT", cmd_timeout_, 0.5);
    nh_.param("PUB_HZ", pub_hz_, 30.0);

    nh_.param("IDLE_LATCH_ON_REACHED", idle_latch_on_reached_, true);
    nh_.param("PUBLISH_ZERO_WHEN_IDLE", publish_zero_when_idle_, true);
    nh_.param("STOP_V_TOL", stop_v_tol_, 0.03);
    nh_.param("STOP_W_TOL", stop_w_tol_, 0.05);
    nh_.param("USE_SPEED_CHECK", use_speed_check_, true);
    nh_.param("NEW_PATH_EPS", new_path_eps_, 0.02);
    
    // 【新增】终点朝向约束参数
    nh_.param("ENABLE_FINAL_YAW", enable_final_yaw_, true);  // 是否启用终点朝向约束
    nh_.param("YAW_TOL", yaw_tol_, 0.1);  // 朝向容差（弧度），默认 0.1 ≈ 5.7°
    nh_.param("YAW_ADJUST_TIMEOUT", yaw_adjust_timeout_, 5.0);  // 最长旋转时间
    nh_.param("CONST_WZ", const_wz_, 0.3);  // 终点朝向调整使用的恒定转速（rad/s）
    nh_.param("YAW_OFFSET", yaw_offset_, 0.0); // 【新增】朝向修正补偿（弧度）

    path_sub_ = nh_.subscribe("/opt_path", 1, &OmnidirectionalPIDLocalPlanner::pathCb, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &OmnidirectionalPIDLocalPlanner::goalCb, this);

    if (is_sim_)
    {
      odom_sim_sub_ = nh_.subscribe("/truth_pose_odom", 1, &OmnidirectionalPIDLocalPlanner::simOdomCb, this);
    }
    else
    {
      odom_carto_sub_ = nh_.subscribe("/carto_odom", 1, &OmnidirectionalPIDLocalPlanner::odomCb, this);
    }

    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_auto", 1);
    local_path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/local_goal", 1, true);

    last_cmd_time_ = ros::Time(0);

    timer_ = nh_.createTimer(ros::Duration(1.0/std::max(1.0, pub_hz_)),
                             &OmnidirectionalPIDLocalPlanner::onTimer, this);

    ROS_INFO("[robot_pid_local_planner] started. IS_SIM=%s, lookahead=%.2f, pub_hz=%.1f, yaw_offset=%.2f",
             is_sim_ ? "true":"false", lookahead_dist_, pub_hz_, yaw_offset_);
  }

private:
  void pathCb(const nav_msgs::PathConstPtr& msg)
  {
    path_ = *msg;
    has_path_ = !path_.poses.empty();

    bool new_path = false;
    const size_t sz = path_.poses.size();

    if (sz != last_path_size_) {
      if (std::abs((int)sz - (int)last_path_size_) >= 5) new_path = true; // 放宽长度变化要求
    }

    if (sz > 0) {
      // 1. 只检查末端点（终点）变化！
      const auto& last = path_.poses.back().pose.position;
      const double dlast = std::hypot(last.x - last_path_last_x_, last.y - last_path_last_y_);
      
      // 【关键修复】：把终点变化的阈值放大到 0.1 米（10厘米）。
      // 防止车身抖动导致目标点微小跳变，从而打破锁存状态。
      if (dlast > 0.1) new_path = true;

      // 【关键修复】：彻底删除对 dfirst（起点）的检查！
      // 原地旋转时起点一定会跟着雷达漂移，不能以此作为新路径的判断标准。

      // 更新记录
      last_path_last_x_ = last.x;
      last_path_last_y_ = last.y;
    }
    last_path_size_ = sz;

    if (has_path_ && new_path) {
      reached_latched_ = false;
      yaw_adjust_dir_ = 0;  // 【修复】新路径时重置转向方向
      yaw_adjust_start_time_ = ros::Time(0);  // 【修复】新路径时重置计时器
      path_id_counter_++;  
      pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
      ROS_INFO("[robot_pid_local_planner] True new path detected (goal changed) -> exit IDLE");
    }
  }
  
  void goalCb(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    goal_ = *msg;
    has_goal_ = true;
    reached_latched_ = false;
    yaw_adjust_start_time_ = ros::Time(0);  // 重置朝向调整超时计时器
    yaw_adjust_dir_ = 0;  // 【修复】重置转向方向标志
    pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
  }

  void simOdomCb(const robot_communication::localizationInfoBroadcastConstPtr& msg)
  {
    // 仿真模式下，直接获取位置
    x_ = msg->xPosition;
    y_ = msg->yPosition;
    yaw_ = msg->chassisAngle;

    vx_fb_ = msg->xSpeed;
    vy_fb_ = msg->ySpeed;
    wz_fb_ = msg->chassisGyro;

    has_odom_ = true;
  }

  void odomCb(const nav_msgs::OdometryConstPtr& msg)
  {
    // 实车模式下，只从 odom 获取速度
    vx_fb_ = msg->twist.twist.linear.x;
    vy_fb_ = msg->twist.twist.linear.y;
    wz_fb_ = msg->twist.twist.angular.z;

    has_odom_ = true;
  }

  bool computeLookaheadPoint(double& gx, double& gy, double& g_yaw_target)
  {
    if (!has_path_ || path_.poses.empty()) return false;

    int nearest = 0;
    double best_d2 = 1e100;
    for (int i = 0; i < (int)path_.poses.size(); ++i)
    {
      const double px = path_.poses[i].pose.position.x;
      const double py = path_.poses[i].pose.position.y;
      const double dx = px - x_;
      const double dy = py - y_;
      const double d2 = dx*dx + dy*dy;
      if (d2 < best_d2) { best_d2 = d2; nearest = i; }
    }

    double acc = 0.0;
    int idx = nearest;
    for (int i = nearest; i+1 < (int)path_.poses.size(); ++i)
    {
      const double x0 = path_.poses[i].pose.position.x;
      const double y0 = path_.poses[i].pose.position.y;
      const double x1 = path_.poses[i+1].pose.position.x;
      const double y1 = path_.poses[i+1].pose.position.y;
      const double seg = std::hypot(x1-x0, y1-y0);
      acc += seg;
      if (acc >= lookahead_dist_) { idx = i+1; break; }
      idx = i+1;
    }

    gx = path_.poses[idx].pose.position.x;
    gy = path_.poses[idx].pose.position.y;

    g_yaw_target = std::atan2(gy - y_, gx - x_);

    nav_msgs::Path local;
    local.header = path_.header;
    for (int i = nearest; i <= idx; ++i) local.poses.push_back(path_.poses[i]);
    local_path_pub_.publish(local);

    visualization_msgs::Marker mk;
    mk.header = path_.header;
    mk.ns = "pid_local_goal";
    mk.id = 1;
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.position.x = gx;
    mk.pose.position.y = gy;
    mk.pose.position.z = 0.1;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = 0.18;
    mk.scale.y = 0.18;
    mk.scale.z = 0.18;
    mk.color.a = 1.0;
    mk.color.r = 1.0;
    mk.color.g = 0.8;
    mk.color.b = 0.0;
    marker_pub_.publish(mk);

    return true;
  }

  bool isReachedGoal() const
  {
    if (!has_path_ || path_.poses.empty()) return false;

    // 【终极修复】：免死金牌放在最前面！
    // 只要已经进入了终点朝向调整阶段，无视任何位置漂移，直接锁定为“已到达”！
    if (!yaw_adjust_start_time_.isZero()) {
        return true; 
    }

    // 只有在没开始旋转的时候，才去判断距离
    const auto& last = path_.poses.back().pose.position;
    const double dist_goal = std::hypot(last.x - x_, last.y - y_);
    if (dist_goal > goal_tol_) return false;

    if (use_speed_check_)
    {
      const double v = std::hypot(vx_fb_, vy_fb_);
      if (v > stop_v_tol_) return false;
      if (std::fabs(wz_fb_) > stop_w_tol_) return false;
    }

    return true;
  }
  
  bool isYawReached(double yaw_target) const
  {
    const double e_yaw = wrap_to_pi(yaw_target - yaw_);
    const bool reached = std::fabs(e_yaw) < yaw_tol_;
    if (!reached)
    {
      ROS_DEBUG_THROTTLE(0.5, "[robot_pid_local_planner] Yaw error: %.3f rad (%.1f deg), target: %.3f, current: %.3f",
                         e_yaw, e_yaw*180.0/M_PI, yaw_target, yaw_);
    }
    return reached;
  }

  // 【核心修改处】在提取目标朝向时，加上 YAML 传进来的 yaw_offset_
  double getPathEndYaw() const
  {
    // 1. 首选：直接从 RViz 下发的全局目标 (goal_) 中获取鼠标拖拽的朝向
    if (has_goal_)
    {
      const auto& q = goal_.pose.orientation;
      if (!(std::abs(q.x) < 1e-6 && std::abs(q.y) < 1e-6 && 
            std::abs(q.z) < 1e-6 && std::abs(q.w) < 1e-6)) 
      {
        double target_yaw = tf::getYaw(q);
        target_yaw += yaw_offset_; // 【附加人为修正偏移量】
        target_yaw = wrap_to_pi(target_yaw);
        
        ROS_DEBUG_THROTTLE(1.0, "[robot_pid_local_planner] Using Goal Yaw with offset: %.3f rad", target_yaw);
        return target_yaw;
      }
    }

    // 2. 备选：如果因为某些原因没收到 goal，兜底尝试从路径末尾点获取
    if (has_path_ && !path_.poses.empty()) 
    {
      const auto& q = path_.poses.back().pose.orientation;
      if (!(std::abs(q.x) < 1e-6 && std::abs(q.y) < 1e-6 && 
            std::abs(q.z) < 1e-6 && std::abs(q.w) < 1e-6)) 
      {
        double target_yaw = tf::getYaw(q);
        target_yaw += yaw_offset_; // 【附加人为修正偏移量】
        return wrap_to_pi(target_yaw);
      }
    }

    ROS_WARN_THROTTLE(2.0, "[robot_pid_local_planner] No valid final yaw found. Keeping current yaw.");
    return yaw_; 
  }

  void publishZero()
  {
    geometry_msgs::Twist z;
    cmd_pub_.publish(z);
  }

  void onTimer(const ros::TimerEvent& ev)
  {
    if (!has_odom_)
    {
      publishZero();
      return;
    }

    if (!is_sim_ && has_path_) 
    {
      tf::StampedTransform transform;
      try {
        tf_listener_.lookupTransform(path_.header.frame_id, "base_link", ros::Time(0), transform);
        x_ = transform.getOrigin().x();
        y_ = transform.getOrigin().y();
        yaw_ = tf::getYaw(transform.getRotation());
      } catch (tf::TransformException &ex) {
        ROS_WARN_THROTTLE(1.0, "[robot_pid_local_planner] TF 获取失败等待中: %s", ex.what());
        publishZero();
        return;
      }
    }

    if (idle_latch_on_reached_ && reached_latched_)
    {
      pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
      if (publish_zero_when_idle_) publishZero();
      return;
    }

    if (idle_latch_on_reached_ && isReachedGoal())
    {
      if (enable_final_yaw_ && has_path_ && !path_.poses.empty())
      {
        double target_yaw = getPathEndYaw();
        const double e_yaw = wrap_to_pi(target_yaw - yaw_);
        
        if (std::fabs(e_yaw) < yaw_tol_)
        {
          publishZero();
          reached_latched_ = true;
          pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
          ROS_INFO_THROTTLE(1.0, "[robot_pid_local_planner] Final yaw reached. e_yaw=%.3f rad", e_yaw);
          return;
        }
        
        if (yaw_adjust_start_time_.isZero())
        {
          yaw_adjust_start_time_ = ros::Time::now();
          pid_yaw_.reset();
          // 【核心新增】：在开始旋转的第一瞬间，锁死旋转方向！
          yaw_adjust_dir_ = (e_yaw >= 0) ? 1 : -1;
          ROS_INFO("[robot_pid_local_planner] Entering yaw adjust. target=%.3f, current=%.3f, e_yaw=%.3f, dir=%d",
                   target_yaw, yaw_, e_yaw, yaw_adjust_dir_);
        }
        
        const double yaw_adjust_time = (ros::Time::now() - yaw_adjust_start_time_).toSec();
        if (yaw_adjust_time > yaw_adjust_timeout_)
        {
          ROS_WARN("[robot_pid_local_planner] YAW adjustment timeout (%.1f s). Force stop.", yaw_adjust_time);
          publishZero();
          reached_latched_ = true;
          pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
          yaw_adjust_start_time_ = ros::Time(0);
          yaw_adjust_dir_ = 0;  // 【修复】超时时也重置方向
          return;
        }
        
        const double dt = (last_cmd_time_.isZero()) ? (1.0/std::max(1.0, pub_hz_))
                                                    : (ros::Time::now() - last_cmd_time_).toSec();
        last_cmd_time_ = ros::Time::now();
        
        // 【关键修复】用恒定速度转动，而不是PID
        double wz = yaw_adjust_dir_ * const_wz_;

        // 【严格判定】：在目标前50%容差时就准备停止，避免过冲导致摆动
        const double early_stop_threshold = yaw_tol_ * 0.5;
        if (std::abs(e_yaw) < early_stop_threshold) 
        {
            publishZero();
            reached_latched_ = true;  
            pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
            yaw_adjust_start_time_ = ros::Time(0);
            yaw_adjust_dir_ = 0;  // 重置方向
            
            ROS_INFO("[robot_pid_local_planner] Yaw aligned (early stop). e_yaw=%.3f, latched!", e_yaw);
            return;
        }
        // 【过冲保护】：如果误差符号反向了，说明已经越过目标，也要停止
        if (e_yaw * yaw_adjust_dir_ < 0)
        {
            publishZero();
            reached_latched_ = true;  
            pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
            yaw_adjust_start_time_ = ros::Time(0);
            yaw_adjust_dir_ = 0;  // 重置方向
            
            ROS_INFO("[robot_pid_local_planner] Yaw overshoot detected. e_yaw=%.3f, stop. latched!", e_yaw);
            return;
        }

        // ==============================================================
        // 【新增功能】：在最后旋转的阶段，如果发生了物理漂移，用 PID 把它拉回原位！
        // ==============================================================
        // 1. 获取真正的终点坐标
        const double gx = path_.poses.back().pose.position.x;
        const double gy = path_.poses.back().pose.position.y;
        
        // 2. 计算当前位置与终点的偏差
        const double dx = gx - x_;
        const double dy = gy - y_;
        
        // 3. 把全图偏差转换到当前正在转动的小车坐标系下
        const double cy = std::cos(yaw_);
        const double sy = std::sin(yaw_);
        const double ex =  cy*dx + sy*dy;
        const double ey = -sy*dx + cy*dy;
        
        // 4. 让 PID 继续工作，计算出平移的补偿速度
        double vx = pid_x_.step(ex, dt);
        double vy = pid_y_.step(ey, dt);

        // ==========================================
        // 【核心修复 1】：加入位置死区（Deadzone）
        // 如果小车偏离终点小于 0.03 米（3厘米），忽略平移，把全部电机功率留给旋转！
        if (std::hypot(ex, ey) < 0.03) {
            vx = 0.0;
            vy = 0.0;
            // 清理积分，防止没动的时候误差偷偷累积
            pid_x_.reset(); 
            pid_y_.reset();
        }
        
        // 5. 限制一下修正速度（比如最大 0.05/s），让最后阶段以“旋转为主，平移微调为辅”
        double max_corr_v = 0.05;
        vx = clamp(vx, -max_corr_v, max_corr_v);
        vy = clamp(vy, -max_corr_v, max_corr_v);

        // ==============================================================

        geometry_msgs::Twist cmd;
        cmd.linear.x = vx;  // 【修改】：原本是 0，现在换成 PID 计算出的补偿速度
        cmd.linear.y = vy;  // 【修改】：原本是 0，现在换成 PID 计算出的补偿速度
        cmd.angular.z = wz;

        if (std::isnan(cmd.linear.x) || std::isnan(cmd.linear.y) || std::isnan(cmd.angular.z)) 
        {
          ROS_ERROR_THROTTLE(1.0, "[robot_pid_local_planner] NaN velocity detected in final yaw adjustment! Forcing to zero.");
          cmd.linear.x = 0.0;
          cmd.linear.y = 0.0;
          cmd.angular.z = 0.0;
        }

        cmd_pub_.publish(cmd);

        ROS_DEBUG_THROTTLE(0.5, "[robot_pid_local_planner] Adjusting yaw: e=%.3f, wz=%.3f (dir=%d*const_wz), time=%.2f/%.1f",
                           e_yaw, wz, yaw_adjust_dir_, yaw_adjust_time, yaw_adjust_timeout_);
        return;
      }

      reached_latched_ = true;
      pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
      yaw_adjust_start_time_ = ros::Time(0);
      publishZero();
      ROS_INFO_THROTTLE(1.0, "[robot_pid_local_planner] reached goal -> IDLE (latched).");
      return;
    }
    
    if (has_path_ && path_id_counter_ != last_path_id_)
    {
      yaw_adjust_start_time_ = ros::Time(0);
      last_path_id_ = path_id_counter_;
    }

    double gx, gy, yaw_target;
    if (!computeLookaheadPoint(gx, gy, yaw_target))
    {
      pid_x_.reset(); pid_y_.reset(); pid_yaw_.reset();
      publishZero();
      return;
    }

    const double dt = (last_cmd_time_.isZero()) ? (1.0/std::max(1.0, pub_hz_))
                                                : (ros::Time::now() - last_cmd_time_).toSec();
    last_cmd_time_ = ros::Time::now();

    const double dx = gx - x_;
    const double dy = gy - y_;

    const double cy = std::cos(yaw_);
    const double sy = std::sin(yaw_);
    const double ex =  cy*dx + sy*dy;
    const double ey = -sy*dx + cy*dy;

    const double e_yaw = wrap_to_pi(yaw_target - yaw_);

    double vx = pid_x_.step(ex, dt);
    double vy = pid_y_.step(ey, dt);
    double wz = pid_yaw_.step(e_yaw, dt);

    vx = clamp(vx, -max_vx_, max_vx_);
    vy = clamp(vy, -max_vy_, max_vy_);
    wz = clamp(wz, -max_wz_, max_wz_);

    geometry_msgs::Twist cmd;
    cmd.linear.x = vx;
    cmd.linear.y = vy;
    cmd.angular.z = wz;

    if (std::isnan(cmd.linear.x) || std::isnan(cmd.linear.y) || std::isnan(cmd.angular.z)) 
    {
      ROS_ERROR_THROTTLE(1.0, "[robot_pid_local_planner] NaN velocity detected in normal tracking! Forcing to zero.");
      cmd.linear.x = 0.0;
      cmd.linear.y = 0.0;
      cmd.angular.z = 0.0;
    }

    cmd_pub_.publish(cmd);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber path_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber odom_sim_sub_;
  ros::Subscriber odom_carto_sub_;

  ros::Publisher cmd_pub_;
  ros::Publisher local_path_pub_;
  ros::Publisher marker_pub_;

  ros::Timer timer_;
  ros::Time last_cmd_time_;

  nav_msgs::Path path_;
  geometry_msgs::PoseStamped goal_;

  tf::TransformListener tf_listener_;
  double last_path_first_x_{0.0};
  double last_path_first_y_{0.0};

  bool is_sim_{true};
  bool has_path_{false};
  bool has_goal_{false};
  bool has_odom_{false};

  bool idle_latch_on_reached_{true};
  bool publish_zero_when_idle_{true};
  bool reached_latched_{false};
  bool use_speed_check_{true};
  double stop_v_tol_{0.03};
  double stop_w_tol_{0.05};
  double new_path_eps_{0.02};
  
  uint64_t path_id_counter_{0};
  uint64_t last_path_id_{0};
  
  ros::Time last_path_stamp_{0};
  size_t last_path_size_{0};
  double last_path_last_x_{0.0}, last_path_last_y_{0.0};

  double lookahead_dist_{0.6};
  double goal_tol_{0.25};
  double max_vx_{0.4}, max_vy_{0.4}, max_wz_{0.8};
  double cmd_timeout_{0.5};
  double pub_hz_{30.0};
  
  double yaw_tol_{0.1};  
  bool enable_final_yaw_{true};  

  double x_{0}, y_{0}, yaw_{0};
  double vx_fb_{0}, vy_fb_{0}, wz_fb_{0};
  
  ros::Time yaw_adjust_start_time_;
  double yaw_adjust_timeout_{5.0};  
  double const_wz_{0.3};  
  double yaw_offset_{0.0}; // 【新增的类成员变量】
  int yaw_adjust_dir_{0};  // 【新增】锁死旋转方向，1 为向左(正)，-1 为向右(负)

  PID pid_x_, pid_y_, pid_yaw_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "omnidirectional_pid_local_planner");
  OmnidirectionalPIDLocalPlanner node;
  ros::spin();
  return 0;
}