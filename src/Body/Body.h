/**
   \file
   \author Shin'ichiro Nakaoka
*/

#ifndef CNOID_BODY_BODY_H_INCLUDED
#define CNOID_BODY_BODY_H_INCLUDED

#include "LinkTraverse.h"
#include "Link.h"
#include "Device.h"
#include "exportdecl.h"

namespace cnoid {

class Body;
class BodyImpl;
class Mapping;
class SgCloneMap;

struct BodyInterface;
struct BodyCustomizerInterface;
typedef void* BodyCustomizerHandle;

typedef ref_ptr<Body> BodyPtr;
    

class CNOID_EXPORT Body : public Referenced
{
public:
    Body();
    Body(const Body& org);

    virtual Body* clone() const;

    virtual Link* createLink(const Link* org = 0) const;

    virtual ~Body();

    const std::string& name() const;
    void setName(const std::string& name);
    const std::string& modelName() const;
    void setModelName(const std::string& name);
        
    void setRootLink(Link* link);

    /**
       This function must be called when the structure of the link tree is changed.
    */
    void updateLinkTree();

    void initializeState();

    /**
       The number of the links that are actual joints.
       The joints given joint ids are recognized as such joints.
       Note that the acutal value is the maximum joint ID plus one.
       Thus there may be a case where the value does not correspond
       to the actual number of the joints with ids.
       In other words, the value represents the size of the link sequence
       obtained by joint() function.
    */
    int numJoints() const {
        return numActualJoints;
    }

    /**
       The number of the joints without joint ids.
       For example, a joint for simulating a spring is usually handled as such a joint.
       You can get the joints by ginving the index after the last joint id to the joint() function.
    */
    int numVirtualJoints() const {
        return jointIdToLinkArray.size() - numActualJoints;
    }

    /**
       The number of all the joints including both the actual and virtual joints.
    */
    int numAllJoints() const {
        return jointIdToLinkArray.size();
    }

    /**
       This function returns a link that has a given joint ID.
       If there is no link that has a given joint ID,
       the function returns a dummy link object whose ID is minus one.
       If the body has virtual joints, this function returns them by giving the ids
       over the last one.
    */
    Link* joint(int id) const {
        return jointIdToLinkArray[id];
    }

    /**
       The number of all the links the body has.
       The value corresponds to the size of the sequence obtained by link() function.
    */
    int numLinks() const {
        return linkTraverse_.numLinks();
    }

    /**
       This function returns the link of a given index in the whole link sequence.
       The order of the sequence corresponds to a link-tree traverse from the root link.
       The size of the sequence can be obtained by numLinks().
    */
    Link* link(int index) const {
        return linkTraverse_.link(index);
    }

    /**
       LinkTraverse object that traverses all the links from the root link
    */
    const LinkTraverse& linkTraverse() const {
        return linkTraverse_;
    }

    /**
       This function returns a link object whose name of Joint node matches a given name.
       Null is returned when the body has no joint of the given name.
    */
    Link* link(const std::string& name) const;

    /**
       The root link of the body
    */
    Link* rootLink() const {
        return rootLink_;
    }

    int numDevices() const {
        return devices_.size();
    }

    Device* device(int index) { return devices_[index]; }
    const Device* device(int index) const { return devices_[index]; }

    /**
       Example:
       DeviceList<ForceSensor> forceSensors = body->devices();
    */
    const DeviceList<>& devices() const {
        return devices_;
    }

    template<class DeviceType> DeviceList<DeviceType> devices() const {
        return devices_;
    }

    template<class DeviceType> DeviceType* findDevice(const std::string& name) const {
        return dynamic_cast<DeviceType*>(findDeviceSub(name));
    }
        
    void addDevice(Device* device);
    void initializeDeviceStates();
    void clearDevices();
        
    /**
       This function returns true when the whole body is a static, fixed object like a floor.
    */
    bool isStaticModel() const {
        return isStaticModel_;
    }
    bool isFixedRootModel() const {
        return rootLink_->isFixedJoint();
    }

    void resetDefaultPosition(const Position& T);
    const Position& defaultPosition() const { return rootLink_->Tb(); }

    double mass() const;

    const Vector3& calcCenterOfMass();
    const Vector3& centerOfMass() const;

    void calcTotalMomentum(Vector3& out_P, Vector3& out_L);

    void calcForwardKinematics(bool calcVelocity = false, bool calcAcceleration = false) {
        linkTraverse_.calcForwardKinematics(calcVelocity, calcAcceleration);
    }
        
    void clearExternalForces();

    enum ExtraJointType { EJ_PISTON, EJ_BALL };

    struct ExtraJoint {
        ExtraJointType type;
        Vector3 axis;
        Link* link[2];
        Vector3 point[2];
    };

    int numExtraJoints() const { return extraJoints_.size(); }
    ExtraJoint& extraJoint(int index) { return extraJoints_[index]; }
    const ExtraJoint& extraJoint(int index) const {  return extraJoints_[index]; }
    void addExtraJoint(const ExtraJoint& extraJoint) { extraJoints_.push_back(extraJoint); }
    void clearExtraJoints() { extraJoints_.clear(); }

    const Mapping* info() const;
    Mapping* info();
    void resetInfo(Mapping* info);

    void cloneShapes(SgCloneMap& cloneMap);
        
    template<class T> T* findCache(const std::string& name) {
        return dynamic_cast<T*>(findCacheSub(name));
    }

    template<class T> const T* findCache(const std::string& name) const {
        return dynamic_cast<const T*>(findCacheSub(name));
    }

    template<class T> T* getOrCreateCache(const std::string& name) {
        T* cache = findCache<T>(name);
        if(!cache){
            cache = new T();
            insertCache(name, cache);
        }
        return cache;
    }

    bool getCaches(PolymorphicReferencedArrayBase<>& out_caches, std::vector<std::string>& out_names) const;
            
    void removeCache(const std::string& name);

    BodyCustomizerHandle customizerHandle() const;
    BodyCustomizerInterface* customizerInterface() const;

    bool installCustomizer();
    bool installCustomizer(BodyCustomizerInterface* customizerInterface);

    bool hasVirtualJointForces() const;
    void setVirtualJointForces();

    static void addCustomizerDirectory(const std::string& path);
    static BodyInterface* bodyInterface();

protected:
    void copy(const Body& org);

private:
    LinkTraverse linkTraverse_;
    Link* rootLink_;
    bool isStaticModel_;
    std::vector<Link*> jointIdToLinkArray;
    int numActualJoints;
    DeviceList<> devices_;
    std::vector<ExtraJoint> extraJoints_;
    BodyImpl* impl;

    void initialize();
    Link* cloneLinkTree(const Link* orgLink);
    Link* createEmptyJoint(int jointId);
    Device* findDeviceSub(const std::string& name) const;
    Referenced* findCacheSub(const std::string& name);
    const Referenced* findCacheSub(const std::string& name) const;
    void insertCache(const std::string& name, Referenced* cache);
    void setVirtualJointForcesSub();
};

};

#endif
