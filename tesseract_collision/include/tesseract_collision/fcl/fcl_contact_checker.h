#ifndef TESSERACT_COLLISION_FCL_CONTACT_CHECKER_H
#define TESSERACT_COLLISION_FCL_CONTACT_CHECKER_H

#include <tesseract_collision/contact_checker_base.h>

namespace tesseract
{
class FCLContactChecker : public ContactCheckerBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  FCLContactChecker();
  void calcDistancesDiscrete(ContactResultMap& contacts) override;

  void calcDistancesDiscrete(const ContactRequest& req,
                             const TransformMap& transforms,
                             ContactResultMap& contacts) const override;

  void calcDistancesContinuous(const ContactRequest& req,
                               const TransformMap& transforms1,
                               const TransformMap& transforms2,
                               ContactResultMap& contacts) const override;

  void calcCollisionsDiscrete(ContactResultMap& contacts) override;

  void calcCollisionsDiscrete(const ContactRequest& req,
                              const TransformMap& transforms,
                              ContactResultMap& contacts) const override;

  void calcCollisionsContinuous(const ContactRequest& req,
                                const TransformMap& transforms1,
                                const TransformMap& transforms2,
                                ContactResultMap& contacts) const override;

  bool addObject(const std::string& name,
                 const int& mask_id,
                 const std::vector<shapes::ShapeConstPtr>& shapes,
                 const EigenSTL::vector_Affine3d& shape_poses,
                 const CollisionObjectTypeVector& collision_object_types,
                 bool enabled = true) override;

  bool removeObject(const std::string& name) override;

  void enableObject(const std::string& name) override;

  void disableObject(const std::string& name) override;

  void setObjectsTransform(const std::string& name, const Eigen::Affine3d& pose) override;

  void setObjectsTransform(const std::vector<std::string>& names, const EigenSTL::vector_Affine3d& poses) override;

  void setObjectsTransform(const TransformMap& transforms) override;

  void setContactRequest(const ContactRequest& req) override;

  const ContactRequest& getContactRequest() const override;

private:
  std::string name_;                        /**< Name of the environment (may be empty) */
  BulletManagerPtr manager_;                /**< Contains the collision objects */
  ContactRequest request_;                  /**< Active request to be used for methods that don't require a request */
  std::vector<std::string> active_objects_; /**< A list of active objects ot check for contact */

  void constructBulletObject(BulletManager& manager,
                             std::vector<std::string>& active_objects,
                             double contact_distance,
                             const TransformMap& transforms,
                             const std::vector<std::string>& active_links,
                             bool continuous = false) const;

  void constructBulletObject(BulletManager& manager,
                             std::vector<std::string>& active_objects,
                             double contact_distance,
                             const TransformMap& transforms1,
                             const TransformMap& transforms2,
                             const std::vector<std::string>& active_links) const;
};
typedef std::shared_ptr<FCLContactChecker> FCLContactCheckerPtr;
typedef std::shared_ptr<const FCLContactChecker> FCLContactCheckerConstPtr;
}
#endif // TESSERACT_COLLISION_FCL_CONTACT_CHECKER_H
