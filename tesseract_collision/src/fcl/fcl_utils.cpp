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

/* Author: Ioan Sucan, Jia Pan */

#include <tesseract_collision/fcl/fcl_utils.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/bodies.h>
#include <fcl/geometry/bvh/BVH_model.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/cylinder.h>
#include <fcl/geometry/shape/convex.h>
#include <fcl/geometry/shape/plane.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/geometry/shape/cone.h>
#include <fcl/geometry/octree/octree.h>
#include <boost/thread/mutex.hpp>
#include <memory>

namespace tesseract
{
fcl::CollisionGeometryd* createShapePrimitive(const shapes::ShapeConstPtr& geom,
                                              const CollisionObjectType& collision_object_type)
{
  fcl::CollisionGeometryd* subshape = 0;

  switch (geom->type)
  {
    case shapes::PLANE:
    {
      assert(collision_object_type == CollisionObjectType::UseShapeType);
      const shapes::Plane* p = static_cast<const shapes::Plane*>(geom.get());
      subshape = new fcl::Planed(p->a, p->b, p->c, p->d);
    }
    case shapes::BOX:
    {
      assert(collision_object_type == CollisionObjectType::UseShapeType);
      const shapes::Box* s = static_cast<const shapes::Box*>(geom.get());
      const double* size = s->size;
      subshape = new fcl::Boxd(size[0], size[1], size[2]);
      return subshape;
    }
    case shapes::SPHERE:
    {
      assert(collision_object_type == CollisionObjectType::UseShapeType);
      const shapes::Sphere* s = static_cast<const shapes::Sphere*>(geom.get());
      subshape = new fcl::Sphered(s->radius);
      return subshape;
    }
    case shapes::CYLINDER:
    {
      assert(collision_object_type == CollisionObjectType::UseShapeType);
      const shapes::Cylinder* s = static_cast<const shapes::Cylinder*>(geom.get());
      subshape = new fcl::Cylinderd(s->radius, s->length);
      return subshape;
    }
    case shapes::CONE:
    {
      assert(collision_object_type == CollisionObjectType::UseShapeType);
      const shapes::Cone* s = static_cast<const shapes::Cone*>(geom.get());
      subshape = new fcl::Coned(s->radius, s->length);
      return subshape;
    }
    case shapes::MESH:
    {
      assert(collision_object_type == CollisionObjectType::UseShapeType ||
             collision_object_type == CollisionObjectType::ConvexHull ||
             collision_object_type == CollisionObjectType::SDF);

      const shapes::Mesh* mesh = static_cast<const shapes::Mesh*>(geom.get());
      bodies::ConvexMesh convex(geom.get());
      convex.correctVertexOrderFromPlanes();
      bool is_convex = true;
      for (unsigned i = 0; i < mesh->vertex_count; ++i)
      {
        Eigen::Vector3d pt(mesh->vertices[3 * i], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);
        if (!convex.containsPoint(pt))
        {
          is_convex = false;
          break;
        }
      }

      // convert the mesh to the assigned collision object type
      switch (collision_object_type)
      {
        case CollisionObjectType::ConvexHull:
        {
          const std::vector<unsigned int>& triangles = convex.getTriangles();
          const EigenSTL::vector_Vector3d& vertices = convex.getVertices();
          const EigenSTL::vector_Vector4d& planes = convex.getPlanes();

          int triangle_count = triangles.size()/3;
          Eigen::Vector3d* fcl_vertices = new Eigen::Vector3d[vertices.size()];
    //        Eigen::Vector3d* fcl_plane_normals = new Eigen::Vector3d[planes.size()];
    //        double* fcl_plane_dis = new double[planes.size()];
          Eigen::Vector3d* fcl_plane_normals = new Eigen::Vector3d[triangle_count];
          double* fcl_plane_dis = new double[triangle_count];
          int* polygons = new int[4 * triangle_count];

          for (unsigned i = 0; i < vertices.size(); ++i)
          {
            fcl_vertices[i] = vertices[i];
          }

          for(auto i = 0; i < triangle_count; ++i)
          {
            int i1 = triangles[3*i + 0];
            int i2 = triangles[3*i + 1];
            int i3 = triangles[3*i + 2];

            polygons[4*i + 0] = 3;
            polygons[4*i + 1] = i1;
            polygons[4*i + 2] = i2;
            polygons[4*i + 3] = i3;

            Eigen::Vector3d v1 = vertices[i1] - vertices[i2];
            Eigen::Vector3d v2 = vertices[i3] - vertices[i2];
            Eigen::Vector3d normal = v2.cross(v1);
            normal.normalize();
            fcl_plane_normals[i] = normal;
            fcl_plane_dis[i] = vertices[i1].dot(normal);
          }
          subshape = new fcl::Convexd(fcl_plane_normals, fcl_plane_dis, triangle_count, fcl_vertices, vertices.size(), polygons);

          return subshape;
        }
        case CollisionObjectType::UseShapeType:
        {
          auto g = new fcl::BVHModel<fcl::OBBRSSd>();
          if (mesh->vertex_count > 0 && mesh->triangle_count > 0)
          {
            std::vector<fcl::Triangle> tri_indices(mesh->triangle_count);
            for (unsigned int i = 0; i < mesh->triangle_count; ++i)
              tri_indices[i] =
                  fcl::Triangle(mesh->triangles[3 * i], mesh->triangles[3 * i + 1], mesh->triangles[3 * i + 2]);

            std::vector<Eigen::Vector3d> points(mesh->vertex_count);
            for (unsigned int i = 0; i < mesh->vertex_count; ++i)
              points[i] = Eigen::Vector3d(mesh->vertices[3 * i], mesh->vertices[3 * i + 1], mesh->vertices[3 * i + 2]);

            g->beginModel();
            g->addSubModel(points, tri_indices);
            g->endModel();
          }

          subshape = g;
          return subshape;
        }
        default:
        {
          ROS_ERROR("This fcl shape type (%d) is not supported for geometry meshes", (int)collision_object_type);
          return nullptr;
        }
      }
      break;
    }
    case shapes::OCTREE:
    {
      assert(collision_object_type == CollisionObjectType::UseShapeType ||
             collision_object_type == CollisionObjectType::ConvexHull ||
             collision_object_type == CollisionObjectType::SDF ||
             collision_object_type == CollisionObjectType::MultiSphere);
      const shapes::OcTree* g = static_cast<const shapes::OcTree*>(geom.get());

      // convert the mesh to the assigned collision object type
      switch (collision_object_type)
      {
        case CollisionObjectType::UseShapeType:
        {
          subshape = new fcl::OcTreed(g->octree);
          return subshape;
        }

        default:
        {
          ROS_ERROR("This fcl shape type (%d) is not supported for geometry octree", (int)collision_object_type);
          return nullptr;
        }
      }
    }
    default:
      ROS_ERROR("This geometric shape type (%d) is not supported using fcl yet", (int)geom->type);
      return nullptr;
  }

  return nullptr;
}

bool collisionCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data)
{
  ContactDistanceData* cdata = reinterpret_cast<ContactDistanceData*>(data);

  if (cdata->done)
    return true;

  const FCLCollisionGeometryWrapper* cd1 = static_cast<const FCLCollisionGeometryWrapper*>(o1->collisionGeometry()->getUserData());
  const FCLCollisionGeometryWrapper* cd2 = static_cast<const FCLCollisionGeometryWrapper*>(o2->collisionGeometry()->getUserData());

  bool needs_collision = (cd1->m_collisionFilterGroup & cd2->m_collisionFilterMask) &&
      (cd2->m_collisionFilterGroup & cd1->m_collisionFilterMask) &&
      !isContactAllowed(cd1->getName(), cd2->getName(), cdata->req->isContactAllowed, false) &&
      (std::find(cdata->req->link_names.begin(), cdata->req->link_names.end(), cd1->getName()) != cdata->req->link_names.end() ||
       std::find(cdata->req->link_names.begin(), cdata->req->link_names.end(), cd2->getName()) != cdata->req->link_names.end());

  if (needs_collision)
    return false;

  fcl::CollisionResultd col_result;

  int num_contacts = fcl::collide(
      o1, o2, fcl::CollisionRequestd(1, true, 1, false),
      col_result);

  if (col_result.isCollision())
  {
    ContactResult contact;
    contact.link_names[0] = cd1->getName();
    contact.link_names[1] = cd2->getName();
    contact.nearest_points[0] = Eigen::Vector3d(-1, -1, -1);
    contact.nearest_points[1] = Eigen::Vector3d(-1, -1, -1);
    contact.type_id[0] = cd1->getTypeID();
    contact.type_id[1] = cd2->getTypeID();
    contact.distance = 0;
    contact.normal = Eigen::Vector3d(-1, -1, -1);

    ObjectPairKey pc = getObjectPairKey(cd1->getName(), cd2->getName());
    const auto& it = cdata->res->find(pc);
    bool found = (it != cdata->res->end());

    processResult(*cdata, contact, pc, found);
  }

  return cdata->done;
}

bool distanceDetailedCallback(fcl::CollisionObjectd* o1, fcl::CollisionObjectd* o2, void* data, double& min_dist)
{
  ContactDistanceData* cdata = reinterpret_cast<ContactDistanceData*>(data);

  if (cdata->done)
    return true;

  const FCLCollisionGeometryWrapper* cd1 = static_cast<const FCLCollisionGeometryWrapper*>(o1->collisionGeometry()->getUserData());
  const FCLCollisionGeometryWrapper* cd2 = static_cast<const FCLCollisionGeometryWrapper*>(o2->collisionGeometry()->getUserData());

  bool needs_collision = (cd1->m_collisionFilterGroup & cd2->m_collisionFilterMask) &&
      (cd2->m_collisionFilterGroup & cd1->m_collisionFilterMask) &&
      !isContactAllowed(cd1->getName(), cd2->getName(), cdata->req->isContactAllowed, false) &&
      (std::find(cdata->req->link_names.begin(), cdata->req->link_names.end(), cd1->getName()) != cdata->req->link_names.end() ||
       std::find(cdata->req->link_names.begin(), cdata->req->link_names.end(), cd2->getName()) != cdata->req->link_names.end());

  if (needs_collision)
    return false;

  fcl::DistanceResultd fcl_result;
  fcl::DistanceRequestd fcl_request(true, true);
  double d = fcl::distance(o1, o2, fcl_request, fcl_result);

  if (d < cdata->req->contact_distance)
  {
    ContactResult contact;
    contact.link_names[0] = cd1->getName();
    contact.link_names[1] = cd2->getName();
    contact.nearest_points[0] = fcl_result.nearest_points[0];
    contact.nearest_points[1] = fcl_result.nearest_points[1];
    contact.type_id[0] = cd1->getTypeID();
    contact.type_id[1] = cd2->getTypeID();
    contact.distance = 0;
    contact.normal =(contact.nearest_points[1] - contact.nearest_points[0]).normalized();

    // TODO: There is an issue with FCL need to track down
    if (std::isnan(contact.nearest_points[0](0)))
    {
      ROS_ERROR("Nearest Points are NAN's");
    }

    ObjectPairKey pc = getObjectPairKey(cd1->getName(), cd2->getName());
    const auto& it = cdata->res->find(pc);
    bool found = (it != cdata->res->end());

    processResult(*cdata, contact, pc, found);
  }

  return cdata->done;
}

void FCLObject::registerTo(fcl::BroadPhaseCollisionManagerd* manager)
{
  std::vector<fcl::CollisionObjectd*> collision_objects(collision_objects_.size());
  for (std::size_t i = 0; i < collision_objects_.size(); ++i)
    collision_objects[i] = collision_objects_[i].get();
  if (!collision_objects.empty())
    manager->registerObjects(collision_objects);
}

void FCLObject::unregisterFrom(fcl::BroadPhaseCollisionManagerd* manager)
{
  for (std::size_t i = 0; i < collision_objects_.size(); ++i)
    manager->unregisterObject(collision_objects_[i].get());
}

void FCLObject::clear()
{
  collision_objects_.clear();
  collision_geometry_.clear();
}
}
