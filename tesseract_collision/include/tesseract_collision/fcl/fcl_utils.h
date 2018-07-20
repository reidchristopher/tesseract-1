/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#ifndef TESSERACT_COLLISION_FCL_UTILS_H
#define TESSERACT_COLLISION_FCL_UTILS_H

#include <fcl/broadphase/broadphase_collision_manager.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/distance.h>
#include <memory>
#include <set>
#include <tesseract_core/basic_types.h>
#include <tesseract_collision/contact_checker_common.h>
#include <geometric_shapes/mesh_operations.h>
#include <ros/console.h>

namespace tesseract
{
typedef std::shared_ptr<fcl::CollisionGeometryd> FCLCollisionGeometryPtr;

class FCLCollisionGeometryWrapper
{
public:
  FCLCollisionGeometryWrapper(const std::string& name,
                              const int& type_id,
                              const shapes::ShapeConstPtr& shape,
                              const Eigen::Affine3d& shape_pose,
                              const CollisionObjectType& collision_object_type);

  short int m_collisionFilterGroup;
  short int m_collisionFilterMask;
  bool m_enabled;

  const std::string& getName() const { return name_; }
  const int& getTypeID() const { return type_id_; }

  /** \brief Check if two objects point to the same source object */
  bool sameObject(const FCLCollisionGeometryWrapper& other) const
  {
    return name_ == other.name_ && type_id_ == other.type_id_ && &shape_ == &(other.shape_) &&
           &shape_pose_ == &(other.shape_pose_) && collision_geometry_ == other.collision_geometry_;
  }

  std::shared_ptr<FCLCollisionGeometryWrapper> clone()
  {
    std::shared_ptr<FCLCollisionGeometryWrapper> clone_cow(
        new FCLCollisionGeometryWrapper(name_, type_id_, shape_, shape_pose_, collision_object_type_, collision_geometry_));
    clone_cow->m_collisionFilterGroup = m_collisionFilterGroup;
    clone_cow->m_collisionFilterMask = m_collisionFilterMask;
    clone_cow->m_enabled = m_enabled;
    return clone_cow;
  }

protected:

  FCLCollisionGeometryWrapper(const std::string& name,
                              const int& type_id,
                              const shapes::ShapeConstPtr& shape,
                              const Eigen::Affine3d& shape_pose,
                              const CollisionObjectType& collision_object_type,
                              FCLCollisionGeometryPtr collision_geometry);

  std::string name_;  // name of the collision object
  int type_id_;       // user defined type id
  const shapes::ShapeConstPtr& shape_;
  const Eigen::Affine3d& shape_pose_;
  const CollisionObjectType& collision_object_type_;
  FCLCollisionGeometryPtr collision_geometry_;
};

fcl::CollisionGeometryd* createShapePrimitive(const shapes::ShapeConstPtr& geom,
                                              const CollisionObjectType& collision_object_type);

typedef std::shared_ptr<FCLCollisionGeometryWrapper> FCLCollisionGeometryWrapperPtr;
typedef std::shared_ptr<const FCLCollisionGeometryWrapper> FCLCollisionGeometryWrapperConstPtr;

typedef std::shared_ptr<fcl::CollisionObjectd> FCLCollisionObjectPtr;
typedef std::shared_ptr<const fcl::CollisionObjectd> FCLCollisionObjectConstPtr;

struct FCLObject
{
  void registerTo(fcl::BroadPhaseCollisionManagerd* manager);
  void unregisterFrom(fcl::BroadPhaseCollisionManagerd* manager);
  void clear();

  std::vector<FCLCollisionObjectPtr> collision_objects_;
  std::vector<FCLCollisionGeometryWrapperConstPtr> collision_geometry_;
};

struct FCLManager
{
  FCLObject object_;
  std::shared_ptr<fcl::BroadPhaseCollisionManagerd> manager_;
};

bool collisionCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data);

bool distanceDetailedCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data, double& min_dist);

inline void transform2fcl(const Eigen::Affine3d& b, fcl::Transform3d& f)
{
  f = b;
}

inline fcl::Transform3d transform2fcl(const Eigen::Affine3d& b)
{
  fcl::Transform3d t;
  transform2fcl(b, t);
  return t;
}

}
#endif // TESSERACT_COLLISION_FCL_UTILS_H
