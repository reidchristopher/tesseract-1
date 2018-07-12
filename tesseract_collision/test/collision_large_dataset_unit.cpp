
#include "tesseract_collision/bullet/bullet_contact_checker.h"
#include <gtest/gtest.h>
#include <ros/ros.h>

TEST(TesseractCollisionLargeDataSetUnit, CollisionUnit)
{
  tesseract::BulletContactChecker checker;
  sleep(10);

  // Add Meshed Sphere to checker
  shapes::ShapePtr sphere(shapes::createMeshFromResource("package://tesseract_collision/test/sphere.stl"));
  double delta = 0.25;

  std::size_t t = 2;
  std::vector<std::string> link_names;
  tesseract::TransformMap location;
  for (std::size_t x = 0; x < t; ++x)
  {
    for (std::size_t y = 0; y < t; ++y)
    {
      for (std::size_t z = 0; z < t; ++z)
      {
        std::vector<shapes::ShapeConstPtr> obj3_shapes;
        EigenSTL::vector_Affine3d obj3_poses;
        tesseract::CollisionObjectTypeVector obj3_types;
        Eigen::Affine3d sphere_pose;
        sphere_pose.setIdentity();

        obj3_shapes.push_back(shapes::ShapePtr(sphere->clone()));
        obj3_poses.push_back(sphere_pose);
        obj3_types.push_back(tesseract::CollisionObjectType::ConvexHull);

        link_names.push_back("sphere_link_" + std::to_string(x) + std::to_string(y) + std::to_string(z));

        location[link_names.back()] = sphere_pose;
        location[link_names.back()].translation() = Eigen::Vector3d(x * delta, y * delta, z * delta);
        checker.addObject(link_names.back(), 0, obj3_shapes, obj3_poses, obj3_types);
      }
    }
  }

  // Check if they are in collision
  tesseract::ContactRequest req;
  req.link_names = link_names;
  req.contact_distance = 0.1;
  req.type = tesseract::ContactRequestType::FIRST;

  tesseract::ContactResultMap result;
  tesseract::ContactResultVector result_vector;
  checker.calcCollisionsDiscrete(req, location, result);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);
  ROS_ERROR_STREAM("Num Links: " << link_names.size());
  ROS_ERROR_STREAM("Contacts: " << result_vector.size());

  for (const auto& link_name : link_names)
  {
    ROS_ERROR_STREAM("Link Name: " << link_name);
  }

  for (const auto& contact_result : result_vector)
  {
    if (result.find(std::make_pair(contact_result.link_names[1], contact_result.link_names[0])) == result.end())
    {
      ROS_ERROR_STREAM("Duplicate Pair: [" << contact_result.link_names[0] << ", " << contact_result.link_names[1] << "]");
    }
  }


//  checker.calcCollisionsContinuous(req, location, location2, result);
//  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
