// 평행 링크 손가락 플러그인 (Gazebo Classic ModelPlugin)
//
// RH-P12-RN 그리퍼의 손끝 마디는 평행 링크 기구라서 항상 "손끝 각도 = 첫 마디 각도" 가 유지된다.
// URDF 의 <mimic> 은 Gazebo Classic 이 무시하고, gazebo_ros2_control 의 mimic 은 effort 모드에서
// 같은 토크만 넣어 줄 뿐 각도를 맞춰 주지 않는다 (손끝이 관절 한계까지 꺾여 버림).
// 그래서 매 물리 스텝마다 따라가는 관절 각도를 기준 관절의 "실제" 각도로 맞춘다.
//
// 사용법 (URDF)
//   <gazebo>
//     <plugin name="parallel_finger" filename="libparallel_finger_plugin.so">
//       <pair><joint>rh_r2</joint><follow>rh_r1_joint</follow><multiplier>1</multiplier></pair>
//       ...
//     </plugin>
//   </gazebo>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Model.hh>

#include <string>
#include <vector>

namespace omy_gazebo_plugins
{

class ParallelFingerPlugin : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    for (auto pair = sdf->GetElement("pair"); pair; pair = pair->GetNextElement("pair")) {
      const auto joint_name = pair->Get<std::string>("joint");
      const auto follow_name = pair->Get<std::string>("follow");
      const double multiplier = pair->HasElement("multiplier") ? pair->Get<double>("multiplier") : 1.0;

      Pair p{model->GetJoint(joint_name), model->GetJoint(follow_name), multiplier};
      if (!p.joint || !p.follow) {
        gzerr << "[parallel_finger] joint not found: " << joint_name << " / " << follow_name << "\n";
        continue;
      }
      gzmsg << "[parallel_finger] " << joint_name << " = " << multiplier << " * " << follow_name << "\n";
      pairs_.push_back(p);
    }
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&ParallelFingerPlugin::OnUpdate, this));
  }

private:
  struct Pair
  {
    gazebo::physics::JointPtr joint;
    gazebo::physics::JointPtr follow;
    double multiplier;
  };

  void OnUpdate()
  {
    for (const auto & p : pairs_) {
      // preserveWorldVelocity=true - 위치만 맞추고 링크 속도는 유지 (접촉이 튀지 않게)
      p.joint->SetPosition(0, p.multiplier * p.follow->Position(0), true);
    }
  }

  std::vector<Pair> pairs_;
  gazebo::event::ConnectionPtr update_connection_;
};

GZ_REGISTER_MODEL_PLUGIN(ParallelFingerPlugin)

}  // namespace omy_gazebo_plugins
