//
// Copyright (c) 2025 INRIA
//

#include "pinocchio/math/fwd.hpp"
#include "pinocchio/multibody/joint/joints.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/aba.hpp"
#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"

#include <boost/test/unit_test.hpp>
#include <iostream>

using namespace pinocchio;

template<typename D>
void addJointAndBody(
  Model & model,
  const JointModelBase<D> & jmodel,
  const Model::JointIndex parent_id,
  const SE3 & joint_placement,
  const std::string & joint_name,
  const Inertia & Y)
{
  Model::JointIndex idx;

  idx = model.addJoint(parent_id, jmodel, joint_placement, joint_name);
  model.appendBodyToJoint(idx, Y);
}

BOOST_AUTO_TEST_SUITE(JointEllipsoid)

BOOST_AUTO_TEST_CASE(vsFreeFlyer)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef Eigen::Matrix<double, 6, 1> Vector6;
  typedef Eigen::Matrix<double, 7, 1> VectorFF;
  typedef SE3::Matrix3 Matrix3;

  Model modelEllipsoid, modelFreeflyer;

  Inertia inertia(1., Vector3(0.5, 0., 0.0), Matrix3::Identity());
  SE3 pos(1);
  pos.translation() = SE3::LinearType(1., 0., 0.);

  addJointAndBody(modelEllipsoid, JointModelEllipsoid(1, 2,3), 0, pos, "ellipsoid", inertia);
  addJointAndBody(modelFreeflyer, JointModelFreeFlyer(), 0, pos, "free-flyer", inertia);

  Data dataEllipsoid(modelEllipsoid);

  Eigen::VectorXd q = Eigen::VectorXd::Ones(modelEllipsoid.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Ones(modelEllipsoid.nv);

  forwardKinematics(modelEllipsoid, dataEllipsoid, q, v);

  Eigen::VectorXd tauEllipsoid = Eigen::VectorXd::Ones(modelEllipsoid.nv);

  Eigen::VectorXd aEllipsoid = Eigen::VectorXd::Ones(modelEllipsoid.nv);

  // ForwardDynamics == aba
  // Eigen::VectorXd aAbaEllipsoid =
  //  aba(modelEllipsoid, dataEllipsoid, q, v, tauEllipsoid, Convention::WORLD);

  // Calculer jdata.S().transpose() * data.f[i]
}
BOOST_AUTO_TEST_CASE(vsRandomForce)
{
  using namespace pinocchio;
  typedef SE3::Vector3 Vector3;
  typedef Eigen::Matrix<double, 6, 1> Vector6;
  typedef Eigen::Matrix<double, 7, 1> VectorFF;
  typedef SE3::Matrix3 Matrix3;

  JointModelEllipsoid jmodel(1, 2, 3);
  JointDataEllipsoid data = jmodel.createData();

  Eigen::Vector3d qEllipsoid = Eigen::Vector3d::Ones();
  jmodel.calc(data, qEllipsoid);

  Force frandom = Force::Random();
  std::cout << "Random force:\n" << frandom.toVector() << std::endl;
  std::cout << "jdata.S():\n" << data.S.matrix() << std::endl;
  std::cout << "jdata.S().transpose():\n" << data.S.matrix().transpose() << std::endl;
  data.S.matrix().transpose() * frandom.toVector();
  // addJointAndBody(modelEllipsoid, JointModelEllipsoid(1, 2,3), 0, pos, "ellipsoid", inertia);
  // addJointAndBody(modelFreeflyer, JointModelFreeFlyer(), 0, pos, "free-flyer", inertia);

  // Data dataEllipsoid(modelEllipsoid);



  // forwardKinematics(modelEllipsoid, dataEllipsoid, q, v);

  // Eigen::VectorXd tauEllipsoid = Eigen::VectorXd::Ones(modelEllipsoid.nv);

  // Eigen::VectorXd aEllipsoid = Eigen::VectorXd::Ones(modelEllipsoid.nv);

  // ForwardDynamics == aba
  // Eigen::VectorXd aAbaEllipsoid =
  //  aba(modelEllipsoid, dataEllipsoid, q, v, tauEllipsoid, Convention::WORLD);

  // Calculer jdata.S().transpose() * data.f[i]
}

BOOST_AUTO_TEST_SUITE_END()
