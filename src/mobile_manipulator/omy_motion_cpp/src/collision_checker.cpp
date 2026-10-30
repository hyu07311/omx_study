#include "omy_motion_cpp/collision_checker.hpp"

#include <geometric_shapes/shapes.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <cmath>
#include <stdexcept>

namespace omy_motion
{

CollisionChecker::CollisionChecker(const moveit::core::RobotModelConstPtr & model, const std::string & group)
: scene_(std::make_shared<planning_scene::PlanningScene>(model)),
  jmg_(model->getJointModelGroup(group)),
  state_(model)
{
  if (!jmg_) {
    throw std::runtime_error("no joint model group " + group);
  }
  if (jmg_->getActiveVariableCount() != kDof) {
    throw std::runtime_error("group " + group + " must have 6 variables");
  }
  state_.setToDefaultValues();
  for (const auto * j : model->getJointModels()) {
    if (j->getName().rfind("rh_", 0) == 0) {
      gripper_joints_.push_back(j->getName());
    }
  }
  const auto & bounds = jmg_->getActiveJointModelsBounds();
  for (int i = 0; i < kDof; ++i) {
    lower_[i] = (*bounds[i])[0].min_position_;
    upper_[i] = (*bounds[i])[0].max_position_;
  }
  resolution_ = 0.005 * maximumExtent();
}

void CollisionChecker::addBox(const std::string & id, const Eigen::Vector3d & size, const Eigen::Vector3d & center,
                              double padding)
{
  moveit_msgs::msg::CollisionObject obj;
  obj.id = id;
  obj.header.frame_id = scene_->getPlanningFrame();
  obj.operation = moveit_msgs::msg::CollisionObject::ADD;
  obj.pose.position.x = center.x();
  obj.pose.position.y = center.y();
  obj.pose.position.z = center.z();
  obj.pose.orientation.w = 1.0;
  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {size.x() + 2 * padding, size.y() + 2 * padding, size.z() + 2 * padding};
  obj.primitives.push_back(box);
  geometry_msgs::msg::Pose identity;
  identity.orientation.w = 1.0;
  obj.primitive_poses.push_back(identity);
  if (scene_->getWorld()->hasObject(id)) {
    scene_->getWorldNonConst()->removeObject(id);
  }
  if (!scene_->processCollisionObjectMsg(obj)) {
    throw std::runtime_error("failed to add collision object " + id);
  }
}

void CollisionChecker::removeObject(const std::string & id)
{
  scene_->getWorldNonConst()->removeObject(id);
}

void CollisionChecker::attach(const std::string & id, const std::string & link,
                              const std::vector<std::string> & touch_links, const Vector6d & q)
{
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.link_name = link;
  aco.object.id = id;
  aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;
  aco.touch_links = touch_links;
  // world 에 있는 물체를 q 자세의 link 기준 상대 위치로 붙임 → scene 의 현재 상태를 q 로
  setState(q);
  scene_->setCurrentState(state_);
  if (!scene_->processAttachedCollisionObjectMsg(aco)) {
    throw std::runtime_error("failed to attach " + id);
  }
  // 붙인 물체는 scene 의 현재 상태에 들어감 → 검사용 상태도 그 복사본으로
  state_ = scene_->getCurrentState();
}

void CollisionChecker::detach(const std::string & id, const Vector6d & q)
{
  moveit_msgs::msg::AttachedCollisionObject aco;
  aco.object.id = id;
  aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  setState(q);
  scene_->setCurrentState(state_);
  // REMOVE 는 붙인 물체를 q 자세에서의 위치에 world 물체로 되돌림
  if (!scene_->processAttachedCollisionObjectMsg(aco)) {
    throw std::runtime_error("failed to detach " + id);
  }
  state_ = scene_->getCurrentState();
}

void CollisionChecker::allowCollision(const std::string & id, const std::vector<std::string> & links, bool allowed)
{
  auto & acm = scene_->getAllowedCollisionMatrixNonConst();
  for (const auto & l : links) {
    acm.setEntry(id, l, allowed);
  }
}

std::vector<std::string> CollisionChecker::gripperLinks() const
{
  std::vector<std::string> out;
  for (const auto * l : scene_->getRobotModel()->getLinkModelsWithCollisionGeometry()) {
    if (l->getName().rfind("rh_p12_rn", 0) == 0) {
      out.push_back(l->getName());
    }
  }
  return out;
}

void CollisionChecker::setState(const Vector6d & q) const
{
  state_.setJointGroupPositions(jmg_, q.data());
  for (const auto & j : gripper_joints_) {
    state_.setVariablePosition(j, gripper_);
  }
  state_.update();
}

bool CollisionChecker::isValid(const Vector6d & q) const
{
  ++checks_;
  if ((q.array() < lower_.array() - 1e-9).any() || (q.array() > upper_.array() + 1e-9).any()) {
    return false;
  }
  setState(q);
  return !scene_->isStateColliding(state_, jmg_->getName());
}

bool CollisionChecker::isMotionValid(const Vector6d & a, const Vector6d & b) const
{
  const int n = static_cast<int>(std::ceil(distance(a, b) / resolution_));
  if (!isValid(b)) {
    return false;
  }
  if (n <= 1) {
    return true;
  }
  // 구간 [1, n-1] 을 가운데부터 이분 순서로
  std::vector<std::pair<int, int>> queue{{1, n - 1}};
  for (size_t k = 0; k < queue.size(); ++k) {
    const auto [lo, hi] = queue[k];
    if (lo > hi) {
      continue;
    }
    const int mid = (lo + hi) / 2;
    if (!isValid(a + (b - a) * (static_cast<double>(mid) / n))) {
      return false;
    }
    queue.emplace_back(lo, mid - 1);
    queue.emplace_back(mid + 1, hi);
  }
  return true;
}

std::set<std::pair<std::string, std::string>> CollisionChecker::contacts(const Vector6d & q) const
{
  isValid(q);
  collision_detection::CollisionRequest req;
  req.group_name = jmg_->getName();
  req.contacts = true;
  req.max_contacts = 50;
  collision_detection::CollisionResult res;
  scene_->checkCollision(req, res, state_);
  std::set<std::pair<std::string, std::string>> out;
  for (const auto & [pair, _] : res.contacts) {
    out.insert(pair.first < pair.second ? pair : std::make_pair(pair.second, pair.first));
  }
  return out;
}

}  // namespace omy_motion
